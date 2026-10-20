"""Nominal centre-of-mass margin within the actual overlap support polygon."""
import math
from .benchmark_core import load_registry
from .product_geometry import footprint


def _clip(polygon, boundary):
    for a, b in zip(boundary, boundary[1:] + boundary[:1]):
        if not polygon:
            return []
        def side(point):
            return (b[0]-a[0])*(point[1]-a[1]) - (b[1]-a[1])*(point[0]-a[0])
        output = []
        previous = polygon[-1]
        dp = side(previous)
        for current in polygon:
            dc = side(current)
            if (dc >= 0) != (dp >= 0):
                fraction = dp / (dp-dc)
                output.append(tuple(previous[i] + fraction*(current[i]-previous[i]) for i in (0, 1)))
            if dc >= 0:
                output.append(current)
            previous, dp = current, dc
        polygon = output
    return polygon


def _hull(points):
    points = sorted(set(points))
    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    result = []
    for sequence in (points, points[::-1]):
        chain = []
        for point in sequence:
            while len(chain) >= 2 and cross(chain[-2], chain[-1], point) <= 1e-15:
                chain.pop()
            chain.append(point)
        result.extend(chain[:-1])
    return result


def _support_region(target, placed):
    registry = load_registry()
    def polygon(block):
        return footprint({'type': block['type'], 'target': {
            'position': block['position'], 'yaw_deg': math.degrees(block['yaw_rad'])}}, registry)
    own = polygon(target)
    points = []
    area = 0.
    for lower in placed:
        if abs(target['position'][2] - lower['position'][2] - .0192) < .004:
            overlap = _clip(own, polygon(lower))
            points.extend(overlap)
            area += abs(sum(a[0]*b[1] - a[1]*b[0]
                            for a, b in zip(overlap, overlap[1:] + overlap[:1]))) / 2
    support = _hull(points)
    return support, area


def _edge_margins(target, placed):
    support, _ = _support_region(target, placed)
    if len(support) < 3:
        return []
    x, y = target['position'][:2]
    return [(((b[0]-a[0])*(y-a[1]) - (b[1]-a[1])*(x-a[0])) / math.dist(a, b),
             (-(b[1]-a[1])/math.dist(a, b), (b[0]-a[0])/math.dist(a, b)))
            for a, b in zip(support, support[1:] + support[:1])]


def support_margin_m(target, placed):
    """Nominal COM margin; geometry is not measured load-bearing evidence."""
    edges = _edge_margins(target, placed)
    return min(margin for margin, _ in edges) if edges else None


def jaw_crosses_support_edge(target, placed, yaw):
    """Avoid a held insertion when closing jaws straddle an unsupported edge.

    Closing direction is local hand Y. Opposing jaw loads across the support
    boundary can tip the part as its stud lead-in first engages. Unload those
    jaws before top pressing; allow bounded capture for parallel jaw loading.
    """
    closing = (-math.sin(yaw), math.cos(yaw))
    return any(margin < .001 and abs(sum(a*b for a, b in zip(closing, normal))) > .7
               for margin, normal in _edge_margins(target, placed))


def support_overlap_area_m2(target, placed):
    """Nominal overlap area; lower bricks must have non-overlapping footprints."""
    return _support_region(target, placed)[1]
