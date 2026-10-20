#!/usr/bin/env python3
"""Import the owner's local v2 task triples with explicit frame/layer mapping."""
import argparse
import hashlib
import json
import math
from pathlib import Path

import yaml

from mj_bridge.benchmark_core import normalize_product, load_registry
from mj_bridge.product_geometry import footprint


TIERS = {'two_brick_stack': 1, 'simple_multi_brick_stack': 2,
         'simple_shape_assembly': 3, 'balanced_order_sensitive': 4}
MODEL_LAYER_HEIGHT_M = .0192
PARTS_PLATE_CENTER_XY_M = (.4, -.2)


class UniqueLoader(yaml.SafeLoader):
    pass


def unique_mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise ValueError(f'duplicate YAML key: {key}')
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, unique_mapping)


def vector(value, length, label):
    if (not isinstance(value, list) or len(value) != length
            or any(isinstance(x, bool) or not isinstance(x, (int, float))
                   or not math.isfinite(x) for x in value)):
        raise ValueError(f'{label} must contain {length} finite numbers')
    return value


def blocks(raw):
    result = []
    ids = set()
    for row in raw['blocks']:
        name = row['name']
        if not isinstance(name, str) or not name or name in ids:
            raise ValueError('empty or duplicate block name')
        ids.add(name)
        xyz = vector(row['pos'], 3, 'pos')
        rotation = vector(row['rotation'], 3, 'rotation')
        turns = round(rotation[2] / (math.pi/2))
        if (any(abs(v) > 1e-8 for v in rotation[:2])
                or abs(rotation[2] - turns*math.pi/2) > 1e-7):
            raise ValueError('v2 importer requires flat, cardinal radian rotations')
        result.append({'id': name, 'type': row['type'], 'color': row['color'],
                       'target': {'position': xyz.copy(), 'yaw_deg': (turns % 4)*90}})
    if not result:
        raise ValueError('task contains no blocks')
    return result


def import_latest(tasks_root, output):
    root, out = Path(tasks_root).resolve(), Path(output).resolve()
    if out.exists():
        raise ValueError('output already exists; preserve previous imports')
    paths = sorted(root.glob('*/*/final_product.yaml'))
    if not paths:
        raise ValueError('no class/instance/final_product.yaml under tasks root')
    registry = load_registry()
    prepared, sources, rows = {}, {}, []
    for path in paths:
        task, task_class = path.parent.name, path.parent.parent.name
        if task_class not in TIERS or not task.startswith(task_class + '_'):
            raise ValueError(f'unknown class or mismatched instance: {path}')
        documents, hashes = {}, {}
        for name in ('final_product.yaml', 'scene_init.yaml', 'metadata.yaml'):
            source = path.parent/name
            payload = source.read_bytes()
            relative = source.relative_to(root).as_posix()
            sources[relative] = payload
            hashes[name] = hashlib.sha256(payload).hexdigest()
            documents[name] = yaml.load(payload, Loader=UniqueLoader)
        goal, scene, metadata = (documents[name] for name in
                                ('final_product.yaml', 'scene_init.yaml', 'metadata.yaml'))
        geometry = scene['geometry']
        if (geometry.get('length_unit') != 'm' or geometry.get('rotation_unit') != 'rad'
                or geometry.get('pose_origin') != 'xy_center_body_bottom'
                or geometry.get('stud_pitch_m') != .016
                or geometry.get('engaged_layer_height_m') != .02):
            raise ValueError(f'unsupported explicit geometry contract: {task}')
        targets, initial = blocks(goal), blocks(scene)
        inventory = lambda items: {b['id']: (b['type'], b['color']) for b in items}
        if inventory(targets) != inventory(initial):
            raise ValueError(f'initial/goal inventory mismatch: {task}')
        if (metadata.get('instance_id') != task or metadata.get('task_class') != task_class
                or metadata.get('generator_version') != 2
                or metadata.get('num_blocks') != len(targets)):
            raise ValueError(f'metadata mismatch: {task}')
        lowest = min(b['target']['position'][2] for b in targets)
        ground = [b for b in targets if abs(b['target']['position'][2]-lowest) < 1e-8]
        vertices = [point for b in ground for point in footprint(b, registry)]
        origin = [(min(p[i] for p in vertices)+max(p[i] for p in vertices))/2 for i in (0, 1)]
        for block in targets:
            position = block['target']['position']
            level = (position[2]-lowest)/.02
            if abs(level-round(level)) > 1e-7:
                raise ValueError(f'goal does not follow declared 20 mm layers: {task}')
            block['target']['position'] = [round(position[i]-origin[i], 10) for i in (0, 1)] + [
                round(round(level)*MODEL_LAYER_HEIGHT_M, 10)]
        plate = vector(scene['parts_plate']['center'], 3, 'parts_plate.center')
        shift = [PARTS_PLATE_CENTER_XY_M[i]-plate[i] for i in (0, 1)]
        layout = []
        for block in initial:
            p = block['target']['position']
            if abs(p[2]-plate[2]) > 1e-8:
                raise ValueError(f'initial part is not on declared parts plate: {task}')
            layout.append({'id': block['id'],
                           'position_xy_m': [round(p[i]+shift[i], 10) for i in (0, 1)],
                           'yaw_deg': block['target']['yaw_deg']})
        product = normalize_product({'schema_version': 1, 'product': {'name': task},
                                     'blocks': targets,
                                     'initial_layout': {'frame': 'world_tabletop', 'blocks': layout}})
        payload = yaml.safe_dump(product, sort_keys=False).encode()
        prepared[task+'.yaml'] = payload
        rows.append({'product': task+'.yaml', 'tier': TIERS[task_class], 'task_class': task_class,
                     'num_blocks': len(targets), 'source_directory': path.parent.relative_to(root).as_posix(),
                     'source_sha256': hashes, 'normalized_sha256': hashlib.sha256(payload).hexdigest(),
                     'source_ground_envelope_center_xy_m': origin, 'source_ground_z_m': lowest,
                     'initial_translation_xy_m': shift, 'source_metadata': metadata})
    manifest = {'scope': 'Owner-specified local v2 dataset import; not physics acceptance',
                'source_tasks_root': str(root), 'tasks_count': len(rows),
                'source_layer_height_m': .02, 'model_layer_height_m': MODEL_LAYER_HEIGHT_M,
                'transform': 'Goal XY minus ground-envelope center; source Z layer index times 0.0192 m. '
                             'Initial XY translated as one plate to [0.4,-0.2]; initial Z rests on simulator table. '
                             'Radian quarter turns converted explicitly; no XY scaling or per-brick repositioning.',
                'source_tree_sha256': hashlib.sha256(''.join(
                    name+':'+hashlib.sha256(payload).hexdigest()+'\n'
                    for name, payload in sorted(sources.items())).encode()).hexdigest(), 'tasks': rows}
    # Validate all tasks before writing any destination files.
    out.mkdir(parents=True, exist_ok=False)
    for name, payload in prepared.items():
        (out/name).write_bytes(payload)
    for name, payload in sources.items():
        dest = out/'source'/name
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_bytes(payload)
    (out/'import_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return manifest


def verify_imported_task(output, product_name, tasks_root=None):
    """Reject changed normalized, frozen-source, or available live-source inputs."""
    out = Path(output).resolve()
    manifest = json.loads((out/'import_manifest.json').read_text())
    rows = [row for row in manifest['tasks'] if row['product'] == product_name]
    if len(rows) != 1 or Path(product_name).name != product_name:
        raise ValueError('task is missing or ambiguous in import manifest')
    row = rows[0]
    required_sources = {'final_product.yaml', 'scene_init.yaml', 'metadata.yaml'}
    if set(row.get('source_sha256', {})) != required_sources:
        raise ValueError('import manifest must fingerprint all three source files')
    if hashlib.sha256((out/product_name).read_bytes()).hexdigest() != row['normalized_sha256']:
        raise ValueError('imported task changed; import the source again to a new directory')
    live = Path(tasks_root or manifest['source_tasks_root']).resolve()
    relative = Path(row['source_directory'])
    if relative.is_absolute() or '..' in relative.parts:
        raise ValueError('invalid source directory in manifest')
    for name, expected in row['source_sha256'].items():
        if name not in ('final_product.yaml', 'scene_init.yaml', 'metadata.yaml'):
            raise ValueError('invalid source filename in manifest')
        if hashlib.sha256((out/'source'/relative/name).read_bytes()).hexdigest() != expected:
            raise ValueError('frozen source task changed')
        if live.exists() and hashlib.sha256((live/relative/name).read_bytes()).hexdigest() != expected:
            raise ValueError('local Workbenchmark source changed; create a fresh import before running')
    return {'verified': True, 'task': product_name,
            'source': 'live local source and frozen snapshot' if live.exists() else 'frozen source snapshot',
            'source_tasks_root': str(live), 'input_sha256': row['normalized_sha256']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tasks-root', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--verify-task', help='verify an existing imported filename without modifying it')
    args = parser.parse_args()
    if args.verify_task:
        print(json.dumps(verify_imported_task(args.output, args.verify_task, args.tasks_root), indent=2))
        return
    if args.tasks_root is None:
        parser.error('--tasks-root is required when importing')
    result = import_latest(args.tasks_root, args.output)
    print(json.dumps({k: result[k] for k in ('tasks_count', 'source_tasks_root', 'source_tree_sha256')}, indent=2))


if __name__ == '__main__':
    main()
