#!/usr/bin/env python3
"""Place Workbenchmark goals in the assembly frame, preserving relative geometry."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import yaml
from mj_bridge.benchmark_cli import normalized_from_path
from mj_bridge.benchmark_core import load_registry, load_yaml, normalize_product
from mj_bridge.recorded_layout import validate_layout
from mj_bridge.product_geometry import footprint


def import_tasks(repository, output, initial_layout="seeded"):
    if initial_layout not in {"seeded", "recorded"}:
        raise ValueError("initial_layout must be seeded or recorded")
    root, out = Path(repository).resolve(), Path(output).resolve()
    paths = sorted((root/'benchmark_tasks').glob('*.yaml'))
    if not paths:
        raise ValueError('repository contains no benchmark_tasks/*.yaml')
    registry = load_registry()
    out.mkdir(parents=True, exist_ok=True)
    rows = []
    for path in paths:
        product = normalized_from_path(str(path))
        vertices = [xy for block in product['blocks'] for xy in footprint(block,registry)]
        origin = [(min(p[i] for p in vertices)+max(p[i] for p in vertices))/2 for i in (0,1)]
        origin.append(min(b['target']['position'][2] for b in product['blocks']))
        for block in product['blocks']:
            block['target']['position'] = [round(p-o,10) for p,o in zip(block['target']['position'],origin)]
        if initial_layout == 'recorded':
            raw = load_yaml(path)
            initial = raw.get('initial_blocks')
            if not initial:
                raise ValueError(f'missing initial_blocks: {path}')
            for b in initial:
                rotation = b.get('rotation', [0, 0, 0])
                if (len(rotation) != 3 or any(float(v) != 0 for v in rotation[:2])
                        or len(b.get('pos', [])) != 3
                        or not math.isclose(float(b['pos'][2]), .0495, abs_tol=1e-8)):
                    raise ValueError(f'unsupported recorded pose convention: {path}')
            source = normalize_product({'blocks': initial})
            expected = {b['id']: (b['type'], b['color']) for b in product['blocks']}
            actual = {b['id']: (b['type'], b['color']) for b in source['blocks']}
            if actual != expected:
                raise ValueError(f'initial/goal part inventory mismatch: {path}')
            product['initial_layout'] = {'frame': 'world_tabletop', 'blocks': [
                {'id': b['id'], 'position_xy_m': b['target']['position'][:2],
                 'yaw_deg': b['target']['yaw_deg']} for b in source['blocks']]}
            validate_layout(product, registry)
        text = yaml.safe_dump(product,sort_keys=False)
        target = out/path.name
        if target.exists() and target.read_text() != text:
            raise ValueError(f'refusing to overwrite different imported task: {target}')
        target.write_text(text)
        rows.append(dict(source=str(path.relative_to(root)),product=path.name,
                         source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                         normalized_sha256=hashlib.sha256(text.encode()).hexdigest(),
                         source_frame_origin_m=origin))
    commit = subprocess.check_output(['git','-C',str(root),'rev-parse','HEAD'],text=True).strip()
    manifest = dict(source_repository='https://github.com/ma-haha-hehe/Workbenchmark',source_commit=commit,
                    transform='target_relative = original_goal_position - source_frame_origin_m; no rotation or scaling',
                    source_layout=('recorded XY/yaw; legacy center Z=.0495 mapped to model tabletop resting height'
                                   if initial_layout == 'recorded' else 'seeded loose parts; initial_blocks not replayed'),
                    tasks=rows)
    (out/'import_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repository', required=True)
    parser.add_argument('--output', default='runs/workbenchmark-products')
    parser.add_argument('--initial-layout', choices=['seeded', 'recorded'], default='seeded')
    args=parser.parse_args()
    result=import_tasks(args.repository,args.output,args.initial_layout)
    print(f"Imported {len(result['tasks'])} goals to {args.output}; frame transforms recorded in import_manifest.json")


if __name__=='__main__':main()
