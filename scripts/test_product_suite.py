#!/usr/bin/env python3
"""Run every product in an isolated process and preserve failures and timeouts."""
import argparse
import hashlib
import math
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from mj_bridge.benchmark_cli import normalized_from_path
from mj_bridge.product_geometry import audit_product


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--products', default='examples/products/catalog')
    parser.add_argument('--seeds', type=int, nargs='+', default=[0, 17, 42])
    parser.add_argument('--output-dir', default='runs/product-suite')
    parser.add_argument('--jobs', type=int, default=4)
    parser.add_argument('--timeout', type=float, default=180)
    parser.add_argument('--backend', choices=['oracle', 'groundingdino-sam-foundationpose'],
                        default='oracle')
    parser.add_argument('--connection-mode', choices=['snap', 'physics'], default='snap')
    parser.add_argument('--contact-profile', choices=['loose', 'plastic'], default='loose')
    parser.add_argument('--robot-base-x', type=float, default=0.)
    parser.add_argument('--priority-products', nargs='*', default=[],
                        help='run these selected product names first without changing coverage')
    parser.add_argument('--skip-invalid-products', action='store_true',
                        help='report geometrically invalid products separately; never count them as successes')
    args = parser.parse_args()
    if args.jobs < 1 or args.timeout <= 0:
        parser.error('--jobs and --timeout must be positive')
    if args.backend != 'oracle':
        from mj_bridge.perception import vision_preflight
        status = vision_preflight()
        if not status['ready']:
            print(json.dumps(status, indent=2))
            return 2
    if args.skip_invalid_products and args.connection_mode != 'snap':
        parser.error('--skip-invalid-products applies only to snap geometry')
    if args.contact_profile == 'plastic' and args.connection_mode != 'physics':
        parser.error('plastic contact requires physics connection mode')
    if not math.isfinite(args.robot_base_x):
        parser.error('robot base X must be finite')
    args.seeds = list(dict.fromkeys(args.seeds))
    products = sorted(Path(args.products).glob('*.yaml'))
    if not products:
        parser.error('no product YAML files found')
    priorities = list(dict.fromkeys(args.priority_products))
    missing = set(priorities) - {p.stem for p in products}
    if missing:
        parser.error('priority products not in selection: ' + ', '.join(sorted(missing)))
    rank = {name: i for i, name in enumerate(priorities)}
    products.sort(key=lambda p: (rank.get(p.stem, len(rank)), p.name))
    output = Path(args.output_dir).resolve()
    if output.exists() and any(output.iterdir()):
        parser.error('output directory is not empty; preserve previous evidence')
    output.mkdir(parents=True, exist_ok=True)
    from mj_bridge import benchmark_core
    package = Path(benchmark_core.__file__).parent
    source_files = list(package.glob('*.py')) + [package/name for name in
        ('panda.xml', 'hand.xml', 'scene_template.xml', 'part_registry.yaml', 'benchmark.yaml')]
    def fingerprint():
        return {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in source_files}
    frozen_source = fingerprint()
    input_count = len(products)
    invalid_products = []
    if args.skip_invalid_products:
        accepted = []
        for product in products:
            audit = audit_product(normalized_from_path(str(product)))
            if audit['valid_for_snap']:
                accepted.append(product)
            else:
                invalid_products.append(audit)
                print(f"INVALID {product.stem}: {len(audit['issues'])} geometry issues", flush=True)
        products = accepted
        if not products:
            (output / 'summary.json').write_text(json.dumps({
                'input_products': input_count, 'episodes': 0, 'successes': 0,
                'invalid_products': invalid_products, 'results': []}, indent=2))
            return 2
    rows = []
    expected_episodes = len(products) * len(args.seeds)
    def run_episode(product, seed):
        run = output / product.stem / f'seed-{seed}'
        if fingerprint() != frozen_source:
            return dict(product=product.stem, seed=seed, success=False, not_run=True,
                        completion=0, error='source changed; execution cancelled', run_dir=str(run))
        run.mkdir(parents=True, exist_ok=True)
        # A previous result must never be mistaken for this attempt's output.
        for name in ('result.json', 'actual_state.json', 'progress.json'):
            (run / name).unlink(missing_ok=True)
        started = time.monotonic()
        with (run / 'execution.log').open('w') as log:
            try:
                proc = subprocess.run([sys.executable, '-m', 'mj_bridge.benchmark_cli', 'run',
                    '--product', str(product.resolve()), '--seed', str(seed), '--headless',
                    '--executor', 'baseline', '--backend', args.backend,
                    '--connection-mode', args.connection_mode, '--contact-profile', args.contact_profile,
                    '--robot-base-x', str(args.robot_base_x), '--output-dir', str(run)],
                    stdout=log, stderr=subprocess.STDOUT, timeout=args.timeout,
                    env=dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1'))
                code = proc.returncode
            except subprocess.TimeoutExpired:
                code = 124
        result_error = None
        try:
            result = json.loads((run / 'result.json').read_text()) if (run / 'result.json').is_file() else {}
            if not isinstance(result, dict):
                raise ValueError('result must be an object')
        except (ValueError, OSError) as exc:
            result = {}
            result_error = f'invalid result artifact: {exc}'
        row = {'product': product.stem, 'seed': seed, 'returncode': code,
               'success': code == 0 and result.get('success') is True,
               'completion': result.get('completion', 0),
               'max_gripper_penetration_m': result.get('max_gripper_penetration_m'),
               'error': result_error or result.get('execution_error') or
                        ('gripper penetration limit exceeded' if result.get('gripper_contact_valid') is False else None) or
                        ('physical contact penetration limit exceeded' if result.get('physical_contacts_valid') is False else None) or
                        ('final assembly outside tolerance' if result and not result.get('success') else
                         (f'process exit {code}' if code else None)),
               'failed_blocks': [b for b in result.get('blocks', []) if not b['success']],
               'wall_seconds': round(time.monotonic()-started, 2), 'run_dir': str(run)}
        return row

    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        futures = [pool.submit(run_episode, product, seed) for product in products for seed in args.seeds]
        for future in as_completed(futures):
            row = future.result()
            rows.append(row)
            summary = {'kind': 'physical_execution', 'backend': args.backend,
                       'connection_mode': args.connection_mode, 'contact_profile': args.contact_profile,
                       'robot_base_x': args.robot_base_x, 'source_sha256': frozen_source,
                       'priority_products': priorities,
                       'executed_episodes': sum(not r.get('not_run', False) for r in rows),
                       'input_products': input_count, 'invalid_products': invalid_products,
                       'expected_episodes': expected_episodes,
                       'finished': len(rows) == expected_episodes,
                       'episodes': len(rows), 'successes': sum(r['success'] for r in rows), 'results': rows}
            temporary = output / 'summary.json.tmp'
            temporary.write_text(json.dumps(summary, indent=2))
            temporary.replace(output / 'summary.json')
            print(f"{'PASS' if row['success'] else 'FAIL'} {row['product']} seed={row['seed']}: {row['error'] or row['completion']}", flush=True)
    return 0 if all(row['success'] for row in rows) else 1


if __name__ == '__main__':
    raise SystemExit(main())
