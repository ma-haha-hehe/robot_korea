"""Recorded tabletop poses, independent of the model's body-origin height."""
import math
from .benchmark_core import ProductError, load_registry
from .product_geometry import footprint, intersection_area


def validate_layout(product, registry=None):
    layout = product.get('initial_layout')
    if layout is None:
        return None
    if not isinstance(layout, dict) or layout.get('frame') != 'world_tabletop':
        raise ProductError('initial_layout.frame must be world_tabletop')
    rows = layout.get('blocks')
    if not isinstance(rows, list):
        raise ProductError('initial_layout.blocks must be a list')
    targets = {b['id']: b for b in product['blocks']}
    registry = registry or load_registry()
    poses, polygons = {}, {}
    for row in rows:
        if not isinstance(row, dict):
            raise ProductError('initial layout entries must be mappings')
        block_id = row.get('id')
        if not isinstance(block_id, str) or block_id not in targets or block_id in poses:
            raise ProductError(f'unknown or duplicate initial block: {block_id}')
        xy, yaw = row.get('position_xy_m'), row.get('yaw_deg')
        if (not isinstance(xy, list) or len(xy) != 2
                or any(isinstance(v, bool) or not isinstance(v, (int, float))
                       or not math.isfinite(v) for v in [*xy, yaw])):
            raise ProductError(f'invalid initial pose: {block_id}')
        polygon = footprint(dict(targets[block_id], target={'position': [*xy, 0.], 'yaw_deg': yaw}), registry)
        # Keep all loose parts inside the table and outside the assembly baseplate.
        if any(not (.1 <= x <= .7 and -.5 <= y <= .12) for x, y in polygon):
            raise ProductError(f'initial block outside loose-parts table area: {block_id}')
        for other, old in polygons.items():
            if intersection_area(polygon, old) > 1e-10:
                raise ProductError(f'overlapping initial blocks: {other}, {block_id}')
        polygons[block_id] = polygon
        poses[block_id] = row
    if poses.keys() != targets.keys():
        raise ProductError('initial layout must contain exactly the target block IDs')
    return poses
