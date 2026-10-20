"""Nominal feasibility checks for the documented primitive/snap model.

These checks reject known impossible goals; passing is not a reachability or
collision-free execution guarantee. Positions are product-relative metres.
"""
from __future__ import annotations

import math
from .benchmark_core import load_registry, validate_product

LAYER_PITCH = .0192


def footprint(block, registry):
    x, y = block['target']['position'][:2]
    hx, hy = (v / 2 for v in registry[block['type']]['size_m'][:2])
    angle = math.radians(block['target'].get('yaw_deg', 0))
    c, s = math.cos(angle), math.sin(angle)
    return [(x+c*u-s*v, y+s*u+c*v)
            for u, v in ((-hx,-hy), (hx,-hy), (hx,hy), (-hx,hy))]


def intersection_area(first, second):
    """Area of intersection of two counterclockwise convex polygons."""
    polygon = list(first)
    for a, b in zip(second, second[1:] + second[:1]):
        def distance(p):
            return (b[0]-a[0])*(p[1]-a[1]) - (b[1]-a[1])*(p[0]-a[0])
        result = []
        if not polygon:
            return 0.
        previous = polygon[-1]
        dp = distance(previous)
        for current in polygon:
            dc = distance(current)
            if (dc >= 0) != (dp >= 0):
                fraction = dp / (dp-dc)
                result.append(tuple(previous[i] + fraction*(current[i]-previous[i]) for i in (0,1)))
            if dc >= 0:
                result.append(current)
            previous, dp = current, dc
        polygon = result
    return abs(sum(a[0]*b[1]-b[0]*a[1]
                   for a,b in zip(polygon, polygon[1:]+polygon[:1]))) / 2


def audit_product(product, *, z_tolerance=.004, minimum_support_ratio=.25):
    registry = load_registry()
    validate_product(product, registry)
    blocks = sorted(product['blocks'], key=lambda b:b['target']['position'][2])
    polygons = {b['id']:footprint(b, registry) for b in blocks}
    issues, supported = [], set()
    for index, block in enumerate(blocks):
        block_id = block['id']
        z = block['target']['position'][2]
        size = registry[block['type']]['size_m']
        if z < -z_tolerance:
            issues.append({'code':'below_base', 'blocks':[block_id], 'height_m':z})
        nominal_z = round(z / LAYER_PITCH) * LAYER_PITCH
        if abs(z-nominal_z) > z_tolerance + 1e-9:
            issues.append({'code':'layer_height_mismatch', 'blocks':[block_id],
                           'height_m':z, 'nearest_layer_height_m':nominal_z})
        has_support = abs(z) <= z_tolerance
        for lower in blocks[:index]:
            other_size = registry[lower['type']]['size_m']
            gap = z-lower['target']['position'][2]
            area = intersection_area(polygons[block_id], polygons[lower['id']])
            if area > 1e-6 and gap < (size[2]+other_size[2])/2 - .0005:
                issues.append({'code':'solid_overlap', 'blocks':[lower['id'],block_id],
                               'overlap_area_m2':area})
            min_area = min(size[0]*size[1], other_size[0]*other_size[1])
            if (lower['id'] in supported and abs(gap-LAYER_PITCH) <= z_tolerance + 1e-9
                    and area/min_area >= minimum_support_ratio - 1e-6):
                has_support = True
        if has_support:
            supported.add(block_id)
        else:
            issues.append({'code':'insufficient_support', 'blocks':[block_id],
                           'minimum_overlap_ratio':minimum_support_ratio})
    return {'product':product['product']['name'], 'valid_for_snap':not issues,
            'scope':'nominal primitive geometry; not a motion-planning guarantee', 'issues':issues}
