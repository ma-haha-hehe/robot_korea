"""Assembly by disassembly, adapted from panda_pick/src/myplanner.py.

Evaluate 90/0 degree gripper approaches in the remaining finished structure,
remove an accessible part, then reverse the removals. Geometry checks are a
planning precondition; the controller still validates actual physical contacts.
"""
import math
import numpy as np
from .benchmark_core import load_registry
from .product_geometry import footprint, intersection_area


class PlanningError(ValueError):
    pass


def _polygon(block, registry):
    return footprint({'type': block['type'], 'target': {
        'position': block['position'], 'yaw_deg': math.degrees(block['yaw_rad'])}}, registry)


def accessible(block, remaining, spin_deg, registry):
    """Check the open/closing fingertip swept footprints during vertical removal."""
    yaw = math.radians(spin_deg)
    angle = block['yaw_rad'] - yaw
    hx, hy = np.array(registry[block['type']]['size_m'][:2]) / 2
    opening = abs(math.sin(angle))*hx + abs(math.cos(angle))*hy
    if opening > .04 - .001:
        return False
    c, s = math.cos(yaw), math.sin(yaw)
    rotation = np.array([[c, -s], [s, c]])
    centre = np.array(block['position'][:2])
    sweeps = []
    for sign in (-1, 1):
        lo, hi = sorted((sign*(opening-.0005), sign*(.04+.0095+.0005)))
        sweeps.append([tuple(centre + rotation @ [x, y])
                       for x, y in ((-.009, lo), (.009, lo), (.009, hi), (-.009, hi))])
    own = _polygon(block, registry)
    for other in remaining:
        if other['id'] == block['id']:
            continue
        dz = other['position'][2] - block['position'][2]
        polygon = _polygon(other, registry)
        # A higher part overlapping the body obstructs upward extraction.
        if dz > .01 and intersection_area(own, polygon) > 1e-8:
            return False
        if dz >= -.012 and any(intersection_area(sweep, polygon) > 1e-8 for sweep in sweeps):
            return False
    return True


def resolve_grasp_yaws(source_yaw_rad, target_yaw_rad, grasp_spin_deg):
    """Map a world-frame planned gripper angle to a source-relative pick angle."""
    if grasp_spin_deg not in (0, 90):
        raise PlanningError('grasp_spin_deg must be 0 or 90')
    if not all(math.isfinite(v) for v in (source_yaw_rad, target_yaw_rad)):
        raise PlanningError('part yaw must be finite radians')
    place_yaw = math.radians(grasp_spin_deg)
    pick_yaw = source_yaw_rad + place_yaw - target_yaw_rad
    return (pick_yaw + math.pi) % (2*math.pi) - math.pi, place_yaw


def plan_assembly(targets):
    registry = load_registry()
    remaining = list(targets)
    if not remaining:
        raise PlanningError('at least one target is required')
    for block in remaining:
        try:
            valid = (isinstance(block['id'], str) and bool(block['id'].strip())
                     and block['type'] in registry and len(block['position']) == 3
                     and all(math.isfinite(v) for v in block['position'])
                     and math.isfinite(block['yaw_rad']))
        except (KeyError, TypeError, ValueError):
            valid = False
        if not valid:
            raise PlanningError('targets require an ID, registered type, finite XYZ and yaw radians')
    if len({b['id'] for b in remaining}) != len(remaining):
        raise PlanningError('duplicate target IDs')
    for index, block in enumerate(remaining):
        for other in remaining[:index]:
            if (abs(block['position'][2] - other['position'][2]) < .0185
                    and intersection_area(_polygon(block, registry), _polygon(other, registry)) > 1e-8):
                raise PlanningError(f"overlapping target solids: {block['id']}, {other['id']}")
    removals = []
    while remaining:
        choice = None
        for block in sorted(remaining, key=lambda b: b['position'][2], reverse=True):
            for spin in (90, 0):
                if accessible(block, remaining, spin, registry):
                    choice = block, spin
                    break
            if choice:
                break
        if choice is None:
            raise PlanningError('no collision-free 0/90 degree removal for remaining parts: '
                                + ', '.join(b['id'] for b in remaining))
        block, spin = choice
        removals.append({'block_id': block['id'], 'grasp_spin_deg': spin,
                         'grasp_offset_rad': math.radians(spin) - block['yaw_rad']})
        remaining = [b for b in remaining if b['id'] != block['id']]
    return list(reversed(removals))
