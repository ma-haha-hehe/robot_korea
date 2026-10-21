#!/usr/bin/env python3
"""Run a frozen sample with isolated ROS domains, hard deadlines and audits."""
from __future__ import annotations
import argparse
from collections import Counter, defaultdict
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
from queue import Queue
import runpy
import signal
import subprocess
import sys
import threading
import time


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_selection(products):
    products = Path(products)
    manifest = json.loads((products/'selection.json').read_text())
    rows = manifest['tasks']
    if not rows or len(rows) != manifest['selected']:
        raise ValueError('selection must contain all declared sampled tasks')
    seen = set()
    for row in rows:
        name = row['product']
        if (not isinstance(name, str) or Path(name).name != name
                or '/' in name or '\\' in name or name in seen
                or type(row['tier']) is not int or row['tier'] not in (1, 2, 3, 4)):
            raise ValueError('invalid, ambiguous or duplicate selected task')
        seen.add(name)
        if sha(products/(name+'.yaml')) != row['input_sha256']:
            raise ValueError(f'changed selected input: {name}')
    return manifest


def stop_process_group(proc):
    if proc.poll() is not None:
        return
    # Every child is started in a fresh session; only that owned group is signalled.
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        proc.wait()
        return
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()


def run_child(command, env, log, case_timeout, deadline_epoch, stop_event):
    started = time.monotonic()
    if stop_event.is_set() or time.time() >= deadline_epoch:
        return 124, 'work stopped before process launch', 0.
    proc = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True)
    reason = None
    try:
        while proc.poll() is None:
            if stop_event.is_set():
                reason = 'stop requested'
            elif time.time() >= deadline_epoch:
                reason = 'work deadline reached'
            elif time.monotonic()-started >= case_timeout:
                reason = 'episode process timeout'
            if reason:
                stop_process_group(proc)
                break
            time.sleep(.1)
        return (124 if reason else proc.returncode), reason, time.monotonic()-started
    finally:
        stop_process_group(proc)


def summarize(rows, expected):
    tiers = {}
    for tier in sorted({r['tier'] for r in expected}):
        selected = [r for r in rows if r['tier'] == tier]
        attempted = [r for r in selected if not r.get('not_run')]
        successes = sum(r['success'] for r in attempted)
        planned = sum(r['tier'] == tier for r in expected)
        tiers[str(tier)] = {'expected_episodes': planned, 'attempted': len(attempted),
            'successes': successes, 'failures': len(attempted)-successes,
            'not_run': planned-len(attempted),
            'success_fraction_of_attempted': successes/len(attempted) if attempted else None,
            'failed_phases': dict(Counter(r.get('failed_phase') or 'process_or_audit'
                                         for r in attempted if not r['success']))}
    return {'finished': len(rows) == len(expected),
            'fully_executed': len(rows) == len(expected) and not any(r.get('not_run') for r in rows),
            'expected_episodes': len(expected),
            'executed_episodes': sum(not r.get('not_run') for r in rows),
            'successes': sum(r['success'] for r in rows), 'tiers': tiers, 'results': rows}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--products', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--source-root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--benchmark-config', type=Path,
                        help='explicit configuration override; default is plastic_benchmark.yaml')
    parser.add_argument('--jobs', type=int, default=2)
    parser.add_argument('--seeds', nargs='+', type=int, default=[42])
    parser.add_argument('--episode-timeout', type=float, default=900.)
    parser.add_argument('--deadline-utc', help='absolute ISO-8601 work deadline including timezone')
    parser.add_argument('--ros-domain-base', type=int, default=200)
    args = parser.parse_args(argv)
    if (args.jobs < 1 or args.ros_domain_base < 0 or args.ros_domain_base+args.jobs > 233
            or not math.isfinite(args.episode_timeout) or args.episode_timeout <= 0):
        parser.error('invalid worker count, domain range or episode timeout')
    try:
        deadline = (datetime.fromisoformat(args.deadline_utc.replace('Z', '+00:00'))
                    if args.deadline_utc else datetime.fromtimestamp(time.time()+14400, timezone.utc))
        if deadline.tzinfo is None or deadline.timestamp() <= time.time():
            raise ValueError('deadline must include a timezone and be in the future')
        selection = load_selection(args.products)
    except (ValueError, OSError, KeyError, TypeError) as exc:
        parser.error(str(exc))
    output = args.output_dir.resolve()
    if output.exists():
        parser.error('output directory already exists; preserve prior execution evidence')
    source = args.source_root.resolve()
    package = source/'src/mj_bridge/mj_bridge'
    # enter_sim_env.sh sets a general-purpose environment config. Do not let
    # that silently replace the dedicated dense-plastic execution budget.
    config = (args.benchmark_config or source/'examples/config/plastic_benchmark.yaml').resolve()
    files = [*package.glob('*.py'), *package.glob('*.xml'), *package.glob('*.yaml'), config,
             source/'scripts/write_validation_report.py',
             *(p for p in (package/'assets').rglob('*') if p.is_file())]
    frozen = {str(p): sha(p) for p in files}
    inputs = {str(args.products.resolve()/(r['product']+'.yaml')): r['input_sha256']
              for r in selection['tasks']}
    def intact():
        try:
            return all(sha(p) == digest for p, digest in {**frozen, **inputs}.items())
        except OSError:
            return False
    sys.path.insert(0, str(source/'src/mj_bridge'))
    from mj_bridge.benchmark_core import load_yaml, normalize_product, score_episode
    report_tools = runpy.run_path(str(source/'scripts/write_validation_report.py'))
    audit = report_tools['require_plastic_seating_evidence']
    audit_contacts = report_tools['require_physics_evidence']
    output.mkdir(parents=True)
    (output/'selection.json').write_text(json.dumps(selection, indent=2)+'\n')
    stop = threading.Event()
    previous_handlers = {sig: signal.signal(sig, lambda *_: stop.set())
                         for sig in (signal.SIGTERM, signal.SIGINT)}
    domains = Queue()
    for domain in range(args.ros_domain_base, args.ros_domain_base+args.jobs):
        domains.put(domain)
    seeds = list(dict.fromkeys(args.seeds))
    by_tier = defaultdict(list)
    for row in selection['tasks']:
        by_tier[row['tier']].append(row)
    for rows in by_tier.values():
        rows.sort(key=lambda row: (row.get('stratum', [0])[0], row['product']))
    schedule = [by_tier[tier][index] for index in range(max(map(len, by_tier.values())))
                for tier in sorted(by_tier) if index < len(by_tier[tier])]
    expected = [dict(row, seed=seed) for row in schedule for seed in seeds]
    (output/'execution_schedule.json').write_text(json.dumps(expected, indent=2)+'\n')

    def run(row):
        result = {'product': row['product'], 'tier': row['tier'], 'seed': row['seed'],
                  'success': False, 'completion': 0., 'failed_phase': None}
        if stop.is_set() or time.time() >= deadline.timestamp() or not intact():
            return dict(result, not_run=True, error='stopped, work deadline reached or frozen input changed')
        domain = domains.get()
        try:
            run_dir = output/row['product']/f'seed-{row["seed"]}'
            run_dir.mkdir(parents=True)
            env = dict(os.environ, PYTHONPATH=str(source/'src/mj_bridge')+os.pathsep+os.environ.get('PYTHONPATH', ''),
                       ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1', ROS_LOG_DIR=str(run_dir/'ros-log'),
                       MJ_BRIDGE_BENCHMARK_CONFIG=str(config), OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
            command = [sys.executable, '-B', '-m', 'mj_bridge.benchmark_cli', 'run',
                '--product', str(args.products.resolve()/(row['product']+'.yaml')),
                '--seed', str(row['seed']), '--headless', '--executor', 'oracle-baseline',
                '--connection-mode', 'physics', '--contact-profile', 'plastic',
                '--robot-base-x=-.05', '--output-dir', str(run_dir)]
            with (run_dir/'execution.log').open('w') as log:
                code, stopped, elapsed = run_child(command, env, log, args.episode_timeout,
                                                    deadline.timestamp(), stop)
            result.update(returncode=code, stopped=stopped, wall_seconds=elapsed, run_dir=str(run_dir))
            if stopped and elapsed == 0.:
                return dict(result, not_run=True, error=stopped)
            try:
                actual = json.loads((run_dir/'actual_state.json').read_text())
                scored = json.loads((run_dir/'result.json').read_text())
                manifest = load_yaml(run_dir/'episode_manifest.yaml')
                product = args.products.resolve()/(row['product']+'.yaml')
                if (manifest.get('seed') != row['seed'] or manifest.get('product') !=
                        normalize_product(load_yaml(product), name=row['product'])):
                    raise ValueError('executed seed or product does not match the frozen sample')
                fingerprints = scored.get('executor_source_sha256', {})
                if not {'reference_executor.py', 'mj_bridge3.py', 'benchmark_core.py'} <= fingerprints.keys():
                    raise ValueError('execution is missing controller source fingerprints')
                for name, digest in fingerprints.items():
                    if frozen.get(str(package/name)) != digest:
                        raise ValueError('executed source does not match the frozen controller')
                recheck = score_episode(manifest, actual)
                result.update(completion=scored['completion'], failed_phase=scored.get('failed_phase'),
                              simulation_seconds=scored.get('simulation_time_s'))
                if (code == 0 and not stopped and intact() and scored.get('success') is True
                        and recheck['success'] and scored.get('physical_contacts_valid') is True
                        and scored.get('gripper_contact_valid') is True):
                    audit_contacts(scored)
                    audit(scored, manifest, actual)
                    result['success'] = True
                result['error'] = stopped or scored.get('execution_error') or (None if result['success'] else 'execution or audit failed')
            except (OSError, ValueError, KeyError, TypeError) as exc:
                result['error'] = stopped or f'missing or invalid execution evidence: {exc}'
            return result
        except Exception as exc:
            return dict(result, error=f'episode process could not be evaluated: {exc}')
        finally:
            domains.put(domain)

    rows = []
    def save_summary():
        summary = dict(summarize(rows, expected), source_sha256=frozen,
            execution_inputs_sha256=inputs, deadline_utc=deadline.isoformat(),
            execution_settings={'jobs': args.jobs, 'episode_timeout_s': args.episode_timeout,
                'seeds': seeds, 'ros_domain_base': args.ros_domain_base,
                'source_root': str(source), 'benchmark_config': str(config),
                'connection_mode': 'physics', 'contact_profile': 'plastic',
                'executor': 'oracle-baseline', 'robot_base_x_m': -.05},
            scope='stratified physical sample; does not establish full-dataset success')
        temporary = output/'summary.json.tmp'
        temporary.write_text(json.dumps(summary, indent=2)+'\n')
        temporary.replace(output/'summary.json')
    save_summary()
    try:
        with ThreadPoolExecutor(max_workers=args.jobs) as pool:
            futures = [pool.submit(run, row) for row in expected]
            for future in as_completed(futures):
                row = future.result()
                rows.append(row)
                save_summary()
                print(f'{"PASS" if row["success"] else "FAIL"} {row["product"]}: {row.get("error")}', flush=True)
    finally:
        stop.set()
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)
    return 0 if all(row['success'] for row in rows) and len(rows) == len(expected) else 1


if __name__ == '__main__':
    raise SystemExit(main())
