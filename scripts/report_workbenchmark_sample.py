#!/usr/bin/env python3
"""Audit sampled runs and write a portable report with optional paired baseline."""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
from pathlib import Path
import runpy
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/mj_bridge'))
from mj_bridge.benchmark_core import dump_json, load_yaml, normalize_product, score_episode


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def episode_key(row):
    return row['product'], row['seed']


def paired_changes(current, baseline):
    """Compare only exactly matched executed inputs; unknowns stay unknown."""
    old = {episode_key(row): row for row in baseline}
    changes = []
    for row in current:
        previous = old.get(episode_key(row))
        if previous is None or row.get('not_run') or previous.get('not_run'):
            continue
        limited = bool(row.get('stopped') or previous.get('stopped'))
        changes.append({'product': row['product'], 'seed': row['seed'],
                        'baseline_success': previous['success'], 'current_success': row['success'],
                        'change': ('limited' if limited else
                                   'unchanged' if previous['success'] == row['success'] else
                                   'improved' if row['success'] else 'regressed')})
    return changes


def read_suite(directory, products):
    directory, products = Path(directory).resolve(), Path(products).resolve()
    summary = json.loads((directory/'summary.json').read_text())
    selection = json.loads((directory/'selection.json').read_text())
    schedule = json.loads((directory/'execution_schedule.json').read_text())
    tasks = {row['product']: row for row in selection['tasks']}
    if not tasks or len(tasks) != selection['selected']:
        raise ValueError('selection is incomplete or contains duplicate tasks')
    for name, row in tasks.items():
        if Path(name).name != name or '/' in name or '\\' in name:
            raise ValueError('invalid task path in selection')
        if sha(products/(name+'.yaml')) != row['input_sha256']:
            raise ValueError(f'selected input changed: {name}')
    report_tools = runpy.run_path(str(ROOT/'scripts/write_validation_report.py'))
    expected = {episode_key(row): row for row in schedule}
    if (not schedule or len(expected) != len(schedule)
            or {r['product'] for r in schedule} != tasks.keys()
            or any(row['tier'] != tasks[row['product']]['tier']
                   or row['input_sha256'] != tasks[row['product']]['input_sha256'] for row in schedule)):
        raise ValueError('execution schedule differs from the frozen selection')
    measurements = []
    seen = set()
    for row in summary['results']:
        key = episode_key(row)
        if key in seen or key not in expected or row['tier'] != tasks[row['product']]['tier']:
            raise ValueError('duplicate episode or episode outside the frozen sample')
        seen.add(key)
        measurement = dict(row)
        # Resolve within this suite: archived reports may have moved from Ubuntu.
        run = directory/row['product']/f'seed-{row["seed"]}'
        for name in ('result.json', 'actual_state.json', 'episode_manifest.yaml',
                     'assembly_plan.json', 'phase_trace.json'):
            p = run/name
            if p.exists():
                measurement[name] = load_yaml(p) if name.endswith('.yaml') else json.loads(p.read_text())
        if row.get('not_run'):
            if row['success']:
                raise ValueError('an unrun case cannot be successful')
        elif row['success']:
            result = measurement['result.json']
            manifest = measurement['episode_manifest.yaml']
            actual = measurement['actual_state.json']
            if (row.get('returncode') != 0 or row.get('stopped') or result.get('success') is not True
                    or manifest.get('seed') != row['seed']
                    or manifest.get('product') != normalize_product(
                        load_yaml(products/(row['product']+'.yaml')), name=row['product'])):
                raise ValueError('successful row contradicts its executed input or process')
            fingerprints = result.get('executor_source_sha256', {})
            frozen = {Path(name).name: digest for name, digest in summary['source_sha256'].items()}
            if (not {'reference_executor.py', 'mj_bridge3.py', 'benchmark_core.py'} <= fingerprints.keys()
                    or any(frozen.get(name) != digest for name, digest in fingerprints.items())):
                raise ValueError('controller fingerprints differ from the frozen suite')
            if not score_episode(manifest, actual)['success']:
                raise ValueError('final-state scoring contradicts success')
            report_tools['require_physics_evidence'](result)
            report_tools['require_plastic_seating_evidence'](result, manifest, actual)
        measurements.append(measurement)
    summarize = runpy.run_path(str(ROOT/'scripts/run_workbenchmark_sample.py'))['summarize']
    verified = summarize(measurements, schedule)
    if any(summary[key] != verified[key] for key in verified if key != 'results'):
        raise ValueError('summary counts or coverage contradict episode evidence')
    return {'summary': {k:v for k,v in summary.items() if k != 'results'},
            'selection': selection, 'measurements': measurements}


def markdown(report):
    summary = report['current']['summary']
    lines = ['# Workbenchmark 抽样仿真报告', '',
        '本报告仅覆盖已执行的固定样本。它不证明整个数据集均可完成，也不代表实机性能。', '',
        f"样本状态：{'已结束' if summary['finished'] else '尚未结束'}；"
        f"执行 {summary['executed_episodes']}/{summary['expected_episodes']}，成功 {summary['successes']}。", '',
        '| Tier | 计划 | 已执行 | 成功 | 失败 | 未执行 |',
        '|---|---:|---:|---:|---:|---:|']
    for tier, row in summary['tiers'].items():
        lines.append(f"| {tier} | {row['expected_episodes']} | {row['attempted']} | "
                     f"{row['successes']} | {row['failures']} | {row['not_run']} |")
    changes = report.get('paired_changes')
    if changes is not None:
        lines.extend(['', '相同任务、相同初始输入与 seed 的对照：', '',
            f"改善 {sum(r['change']=='improved' for r in changes)}，"
            f"回归 {sum(r['change']=='regressed' for r in changes)}，"
            f"结果相同 {sum(r['change']=='unchanged' for r in changes)}；"
            f"含超时或中止 {sum(r['change']=='limited' for r in changes)}，不计入控制器改善/回归归因。"])
    lines.extend(['', '| 任务 | seed | 结果 | 完成比例 | 失败阶段 / 原因 |',
                  '|---|---:|---|---:|---|'])
    for row in sorted(report['current']['measurements'], key=episode_key):
        status = '未执行' if row.get('not_run') else '成功' if row['success'] else '失败'
        detail = ' / '.join(str(x) for x in (row.get('failed_phase'), row.get('error')) if x)
        detail = detail.replace('|', '\\|').replace('\n', ' ')
        lines.append(f"| {row['product']} | {row['seed']} | {status} | {row.get('completion',0):.1%} | {detail} |")
    lines.extend(['', '记录的初始布局不会因为更换 seed 自动重排。'
                  '本报告统计固定样本的结果；如果样本经过定向选择，'
                  '其中的成功比例不能直接用作整个数据集的成功率估计。', ''])
    return '\n'.join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--suite', type=Path, required=True)
    parser.add_argument('--products', type=Path, required=True)
    parser.add_argument('--baseline', type=Path)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args(argv)
    if args.output_dir.exists():
        parser.error('report output already exists; use a fresh directory')
    try:
        current = read_suite(args.suite, args.products)
        report = {'scope': 'fixed sample with independently rechecked physical evidence', 'current': current}
        if args.baseline:
            baseline = read_suite(args.baseline, args.products)
            frozen = lambda suite: {r['product']:r['input_sha256'] for r in suite['selection']['tasks']}
            if frozen(current) != frozen(baseline):
                raise ValueError('paired suites have different frozen selections')
            report.update(baseline=baseline, paired_changes=paired_changes(
                current['measurements'], baseline['measurements']))
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.error(str(exc))
    args.output_dir.mkdir(parents=True)
    dump_json(args.output_dir/'evidence.json', report)
    (args.output_dir/'REPORT.md').write_text(markdown(report), encoding='utf-8')
    fields = ('product', 'tier', 'seed', 'success', 'completion', 'failed_phase', 'error',
              'not_run', 'returncode', 'wall_seconds', 'simulation_seconds')
    with (args.output_dir/'episodes.csv').open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(current['measurements'])
    print(args.output_dir/'REPORT.md')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
