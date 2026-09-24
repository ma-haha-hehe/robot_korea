"""Assembly by disassembly, adapted from panda_pick/src/myplanner.py.

Evaluate 90/0 degree gripper approaches in the remaining finished structure,
remove an accessible part, then reverse the removals. Geometry checks are a
planning precondition; the controller still validates actual physical contacts.
"""
import math
import numpy as np
from .benchmark_core import load_registry
from .product_geometry import footprint, intersection_area


RELEASE_ABOVE_M = .010
RAISED_GRASP_OFFSET_M = .003
# Accessibility assumes a normal grasp at -3 mm relative to the part origin.
RELEASE_TOOL_CLEARANCE_M = RELEASE_ABOVE_M + RAISED_GRASP_OFFSET_M + .003


class PlanningError(ValueError):
    pass


class GraspAccessError(PlanningError):
    """Valid geometry has no direct 0/90 degree grasp sequence."""


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
            hx, hy = np.array(registry[block['type']]['size_m'][:2]) / 2
            # Prefer the smaller jaw span when both disassembly angles are safe.
            # Square parts keep the historical 90-degree tie preference.
            def span(spin):
                angle = block['yaw_rad'] - math.radians(spin)
                return round(abs(math.sin(angle))*hx + abs(math.cos(angle))*hy, 8)
            for spin in sorted((90, 0), key=span):
                if accessible(block, remaining, spin, registry):
                    choice = block, spin
                    break
            if choice:
                break
        if choice is None:
            raise GraspAccessError('no collision-free 0/90 degree removal for remaining parts: '
                                + ', '.join(b['id'] for b in remaining))
        block, spin = choice
        removals.append({'block_id': block['id'], 'grasp_spin_deg': spin,
                         'grasp_offset_rad': math.radians(spin) - block['yaw_rad']})
        remaining = [b for b in remaining if b['id'] != block['id']]
    original = list(reversed(removals))
    return prefer_narrow_plan(targets, original, registry)


def plan_with_release_above(targets):
    """Use release and top pressing only when the direct grasp planner is blocked.

 This geometric fallback is not an invertible disassembly grasp. Execution
 must confirm a full release and validate all contacts before top pressing.
 """
    try:
        return plan_assembly(targets)
    except GraspAccessError:
        pass
    registry = load_registry()
    remaining = list(targets)
    removals = []

    support_cache = {}
    bottom_z = min(block['position'][2] for block in targets)

    def load_path_support(block):
        """Rank landing support through all lower layers, not just one face.

        This area-weighted score is a planning heuristic, not a force balance.
        A fully covered block on a cantilever does not have full base support.
        """
        if block['id'] in support_cache:
            return support_cache[block['id']]
        if block['position'][2] <= bottom_z + .001:
            return 1.0
        polygon = _polygon(block, registry)
        area = float(np.prod(registry[block['type']]['size_m'][:2]))
        support = sum(
            intersection_area(polygon, _polygon(other, registry))
            * load_path_support(other)
            for other in targets
            if .015 < block['position'][2] - other['position'][2] < .025)
        score = min(1.0, support / area)
        support_cache[block['id']] = score
        return score

    def fallback_priority(block):
        area = float(np.prod(registry[block['type']]['size_m'][:2]))
        return block['position'][2], round(load_path_support(block), 6), -area

    while remaining:
        choice = None
        for mode in ('direct', 'release_above_press'):
            priority = (fallback_priority if mode == 'release_above_press'
                        else lambda b: (b['position'][2],))
            for block in sorted(remaining, key=priority, reverse=True):
                if mode != 'direct' and any((other['position'][2] > block['position'][2] + 0.01 and intersection_area(_polygon(block, registry), _polygon(other, registry)) > 1e-08 for other in remaining if other['id'] != block['id'])):
                    continue
                candidate = block if mode == 'direct' else dict(block, position=[*block['position'][:2], block['position'][2] + RELEASE_TOOL_CLEARANCE_M])
                (hx, hy) = np.array(registry[block['type']]['size_m'][:2]) / 2

                def span(spin):
                    angle = block['yaw_rad'] - math.radians(spin)
                    return round(abs(math.sin(angle)) * hx + abs(math.cos(angle)) * hy, 8)
                for spin in sorted((90, 0), key=span):
                    if accessible(candidate, remaining, spin, registry):
                        choice = (block, spin, mode)
                        break
                if choice:
                    break
            if choice:
                break
        if choice is None:
            raise PlanningError('no safe release-above candidate')
        (b, spin, mode) = choice
        removals.append({'block_id': b['id'], 'grasp_spin_deg': spin, 'placement_mode': mode, 'grasp_offset_rad': math.radians(spin) - b['yaw_rad']})
        remaining = [b2 for b2 in remaining if b2['id'] != b['id']]
    return list(reversed(removals))


def prefer_narrow_plan(targets, original, registry):
    """Minimize wide-side holds for small layouts using reverse removal states.

    Keep the validated greedy plan when every hold is already narrow. Limit
    subset search to twelve parts (the current Workbenchmark maximum), so
    larger user layouts cannot trigger unbounded exponential planning work.
    """
    import functools
    if len(targets) > 12:
        return original
    order = sorted(range(len(targets)), key=lambda i: targets[i]['position'][2], reverse=True)
    def span(block, spin):
        hx, hy = np.asarray(registry[block['type']]['size_m'][:2]) / 2
        angle = block['yaw_rad'] - math.radians(spin)
        return round(abs(math.sin(angle))*hx + abs(math.cos(angle))*hy, 8)
    by_id = {b['id']: b for b in targets}
    original_cost = sum(
        span(by_id[row['block_id']], row['grasp_spin_deg'])
        > min(span(by_id[row['block_id']], 0), span(by_id[row['block_id']], 90)) + .0001
        for row in original)
    if not original_cost:
        return original
    @functools.lru_cache(None)
    def search(mask):
        if not mask: return 0, ()
        remaining = [b for i,b in enumerate(targets) if mask & (1 << i)]
        best = (math.inf, ())
        for i in order:
            if not mask & (1 << i): continue
            block=targets[i]
            for spin in sorted((90,0),key=lambda spin:span(block,spin)):
                if not accessible(block,remaining,spin,registry):continue
                cost = int(span(block,spin)>min(span(block,0),span(block,90))+.0001)
                tail, removals = search(mask ^ (1 << i))
                if cost+tail < best[0]:best=(cost+tail,((i,spin),)+removals)
                if best[0]==0:return best
        return best
    cost,removals=search((1 << len(targets))-1)
    if not math.isfinite(cost) or cost >= original_cost:
        return original
    return [dict(block_id=targets[i]['id'],grasp_spin_deg=spin,
                 grasp_offset_rad=math.radians(spin)-targets[i]['yaw_rad'])
            for i,spin in reversed(removals)]
