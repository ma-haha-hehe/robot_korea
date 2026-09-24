#!/usr/bin/env python3
"""Select a reproducible tier/shape-stratified subset without modifying inputs."""
import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import random
import re
import shutil
from mj_bridge.benchmark_cli import normalized_from_path
from mj_bridge.assembly_planner import plan_assembly, PlanningError
import math


def select(products, per_tier, seed, include):
    groups = defaultdict(lambda: defaultdict(list))
    inventory = {}
    for path in sorted(Path(products).glob('tier*_task_*.yaml')):
        match = re.fullmatch(r'tier([1-4])_task_\d+', path.stem)
        if not match:
            continue
        p = normalized_from_path(str(path))
        blocks = p['blocks']
        targets = [dict(id=b['id'], type=b['type'], position=b['target']['position'],
                        yaw_rad=math.radians(b['target'].get('yaw_deg', 0))) for b in blocks]
        try:
            angles = sorted({step['grasp_spin_deg'] for step in plan_assembly(targets)})
        except PlanningError:
            angles = ['blocked']  # Include failures in the sampling population.
        signature = (len(blocks), sum(b['type']=='brick_4x2' for b in blocks),
                     round(max(b['target']['position'][2] for b in blocks), 5), tuple(angles))
        tier = int(match.group(1))
        groups[tier][signature].append(path.stem)
        inventory[path.stem] = dict(path=path, tier=tier, stratum=signature)
    missing = set(include)-set(inventory)
    if missing:
        raise ValueError(f'requested task IDs not found: {sorted(missing)}')
    rng = random.Random(seed)
    chosen = set(include)
    coverage = {}
    for tier in (1, 2, 3, 4):
        strata = list(groups[tier].values())
        all_ids = sorted(x for group in strata for x in group)
        selected = {x for x in chosen if inventory[x]['tier']==tier}
        if not 1 <= per_tier <= len(all_ids) or len(selected)>per_tier:
            raise ValueError(f'invalid per-tier sample size for tier {tier}')
        rng.shuffle(strata)
        for group in strata:
            if len(selected)>=per_tier:
                break
            if not selected.intersection(group):
                selected.add(rng.choice(sorted(group)))
        selected.update(rng.sample(sorted(set(all_ids)-selected), per_tier-len(selected)))
        chosen.update(selected)
        coverage[tier] = dict(population=len(all_ids), selected=len(selected),
            strata=len(strata), represented=sum(bool(selected.intersection(g)) for g in strata))
    return inventory, sorted(chosen), coverage


def restore_selection(products, selection_manifest, output_dir):
    """Restore exact sampled inputs without consulting the current planner."""
    output = Path(output_dir)
    if output.exists():
        raise ValueError('output directory already exists; preserve previous selections')
    manifest_bytes = Path(selection_manifest).read_bytes()
    manifest = json.loads(manifest_bytes)
    tasks = manifest['tasks']
    if (not tasks or manifest['selected'] != len(tasks)
            or manifest['population'] < len(tasks)):
        raise ValueError('selection counts do not match the recorded tasks')
    names, payloads = set(), []
    for row in tasks:
        name = row['product']
        if not re.fullmatch(r'tier[1-4]_task_\d+', name) or name in names:
            raise ValueError('selection contains an invalid or duplicate task name')
        names.add(name)
        path = Path(products) / (name + '.yaml')
        data = path.read_bytes()
        if hashlib.sha256(data).hexdigest() != row['input_sha256']:
            raise ValueError(f'sampled input checksum mismatch: {name}')
        payloads.append((path.name, data))
    # Validate every file before creating the destination. Keep the original
    # bytes so a later input edit cannot slip between validation and copying.
    output.mkdir(parents=True, exist_ok=False)
    for name, data in payloads:
        (output / name).write_bytes(data)
    (output / 'selection.json').write_bytes(manifest_bytes)
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--products', required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--per-tier', type=int, help='new selection size per tier (default: 60)')
    parser.add_argument('--seed', type=int, help='new selection seed (default: 42)')
    parser.add_argument('--include', nargs='*', default=[])
    parser.add_argument('--selection-manifest', type=Path,
                        help='restore a recorded selection and verify each input hash')
    args = parser.parse_args()
    if args.selection_manifest:
        if args.per_tier is not None or args.seed is not None or args.include:
            parser.error('a recorded selection cannot be combined with new sampling options')
        try:
            manifest = restore_selection(args.products, args.selection_manifest, args.output_dir)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            parser.error(str(exc))
        print(json.dumps(dict(selected=manifest['selected'], population=manifest['population'],
                              restored=True, seed=manifest['seed']), indent=2))
        return
    args.per_tier = 60 if args.per_tier is None else args.per_tier
    args.seed = 42 if args.seed is None else args.seed
    if args.output_dir.exists():
        parser.error('output directory already exists; preserve previous selections')
    inventory, chosen, coverage = select(args.products, args.per_tier, args.seed, args.include)
    args.output_dir.mkdir(parents=True)
    rows=[]
    for name in chosen:
        item=inventory[name]
        shutil.copy2(item['path'], args.output_dir/item['path'].name)
        rows.append(dict(product=name, tier=item['tier'], stratum=item['stratum'],
                         input_sha256=hashlib.sha256(item['path'].read_bytes()).hexdigest()))
    manifest=dict(scope='selected inputs only; not execution or success evidence', seed=args.seed,
        population=len(inventory), selected=len(chosen), tiers=coverage,
        mandatory=args.include, unselected=sorted(set(inventory)-set(chosen)), tasks=rows)
    (args.output_dir/'selection.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps(dict(selected=len(chosen), population=len(inventory), tiers=coverage), indent=2))


if __name__=='__main__':
    main()
