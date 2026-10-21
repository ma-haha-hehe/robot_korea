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


TASK_CLASSES = {'two_brick_stack': 1, 'simple_multi_brick_stack': 2,
                'simple_shape_assembly': 3, 'balanced_order_sensitive': 4}


def task_tier(name):
    """Accept original task names and the version 2 instance names."""
    legacy = re.fullmatch(r'tier([1-4])_task_\d+', name)
    if legacy:
        return int(legacy.group(1))
    for task_class, tier in TASK_CLASSES.items():
        if re.fullmatch(re.escape(task_class) + r'_\d+', name):
            return tier
    return None


def count_group(count, tier):
    if tier >= 3 and count <= 5:
        return '3-5'
    if count <= 3:
        return '2-3'
    if count <= 5:
        return '4-5'
    return '6-8' if count <= 8 else '9-12'


def select(products, per_tier, seed, include, tiers=(1, 2, 3, 4)):
    tiers = tuple(sorted(set(tiers)))
    if not tiers or any(t not in (1, 2, 3, 4) for t in tiers):
        raise ValueError('select at least one tier from 1..4')
    if isinstance(per_tier, bool) or not isinstance(per_tier, int) or per_tier < 1:
        raise ValueError('per-tier sample size must be a positive integer')
    groups = defaultdict(lambda: defaultdict(list))
    inventory = {}
    for path in sorted(Path(products).glob('*.yaml')):
        tier = task_tier(path.stem)
        if tier not in tiers:
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
        groups[tier][signature].append(path.stem)
        inventory[path.stem] = dict(path=path, tier=tier, stratum=signature)
    missing = set(include)-set(inventory)
    if missing:
        raise ValueError(f'requested task IDs not found: {sorted(missing)}')
    rng = random.Random(seed)
    chosen = set(include)
    coverage = {}
    for tier in tiers:
        strata = list(groups[tier].values())
        all_ids = sorted(x for group in strata for x in group)
        selected = {x for x in chosen if inventory[x]['tier']==tier}
        if not 1 <= per_tier <= len(all_ids) or len(selected)>per_tier:
            raise ValueError(f'invalid per-tier sample size for tier {tier}')
        # Include larger, infrequent structures as well as the easy population.
        # The resulting sample is for coverage, not a prevalence estimate.
        count_groups = defaultdict(list)
        for name in all_ids:
            count_groups[count_group(inventory[name]['stratum'][0], tier)].append(name)
        for label, names in sorted(count_groups.items()):
            if len(selected) >= per_tier:
                break
            if not selected.intersection(names):
                selected.add(rng.choice(names))
        rng.shuffle(strata)
        for group in strata:
            if len(selected)>=per_tier:
                break
            if not selected.intersection(group):
                selected.add(rng.choice(sorted(group)))
        selected.update(rng.sample(sorted(set(all_ids)-selected), per_tier-len(selected)))
        chosen.update(selected)
        coverage[tier] = dict(population=len(all_ids), selected=len(selected),
            strata=len(strata), represented=sum(bool(selected.intersection(g)) for g in strata),
            count_groups={label: {'population': len(names),
                                 'selected': len(selected.intersection(names))}
                          for label, names in sorted(count_groups.items())})
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
        if not isinstance(name, str) or task_tier(name) is None or name in names:
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
    parser.add_argument('--tiers', type=int, nargs='+', choices=(1, 2, 3, 4),
                        help='tiers to sample (default: all); e.g. --tiers 1 2 3')
    parser.add_argument('--include', nargs='*', default=[])
    parser.add_argument('--selection-manifest', type=Path,
                        help='restore a recorded selection and verify each input hash')
    args = parser.parse_args()
    if args.selection_manifest:
        if args.per_tier is not None or args.seed is not None or args.include or args.tiers:
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
    inventory, chosen, coverage = select(args.products, args.per_tier, args.seed,
                                         args.include, args.tiers or (1, 2, 3, 4))
    args.output_dir.mkdir(parents=True)
    rows=[]
    for name in chosen:
        item=inventory[name]
        shutil.copy2(item['path'], args.output_dir/item['path'].name)
        rows.append(dict(product=name, tier=item['tier'], stratum=item['stratum'],
                         input_sha256=hashlib.sha256(item['path'].read_bytes()).hexdigest()))
    manifest=dict(scope='coverage sample only; not execution evidence or an unbiased success-rate estimate', seed=args.seed,
        population=len(inventory), selected=len(chosen), tiers=coverage,
        mandatory=args.include, unselected=sorted(set(inventory)-set(chosen)), tasks=rows)
    (args.output_dir/'selection.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps(dict(selected=len(chosen), population=len(inventory), tiers=coverage), indent=2))


if __name__=='__main__':
    main()
