#!/usr/bin/env python3
"""Run every product in an isolated process and preserve failures and timeouts."""
import argparse
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
    args.seeds = list(dict.fromkeys(args.seeds))
    products = sorted(Path(args.products).glob('*.yaml'))
    if not products:
        parser.error('no product YAML files found')
    output = Path(args.output_dir).resolve()
    output.mkdir(parents=True, exist_ok=True)
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
                    '--connection-mode', args.connection_mode, '--output-dir', str(run)],
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
               'error': result_error or result.get('execution_error') or
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
                       'connection_mode': args.connection_mode,
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
