#!/usr/bin/env python3
"""Bundle measured results and manifests, keeping invalid/untested cases explicit."""
import argparse
from datetime import datetime, timezone
from importlib.metadata import version
import hashlib
import json
import math
from pathlib import Path
import yaml


def require_complete_summary(summary):
    rows = summary.get('results', [])
    if summary.get('finished') is False:
        raise ValueError('suite is still running')
    if summary.get('episodes') != len(rows):
        raise ValueError('episode count does not match evidence rows')
    if summary.get('expected_episodes', len(rows)) != len(rows):
        raise ValueError('expected episodes are missing')
    if summary.get('successes') != sum(row.get('success') is True for row in rows):
        raise ValueError('success count does not match evidence rows')


def require_physics_evidence(result):
    """A correct final pose alone cannot establish valid physical contact."""
    if result.get('connection_mode') != 'physics':
        raise ValueError('physics evidence has a different connection mode')
    for field in ('max_gripper_penetration_m', 'max_part_penetration_m',
                  'max_robot_environment_penetration_m'):
        value = result.get(field)
        if not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 <= value <= .0005:
            raise ValueError(f'invalid physics contact evidence: {field}')
    if (result.get('attachment_count') != 0 or result.get('timeout') is not False
            or result.get('physical_contacts_valid') is not True
            or result.get('gripper_contact_valid') is not True
            or result.get('execution_error')):
        raise ValueError('physics success contradicts execution evidence')


def require_selected_coverage(selection_dir, measurements):
    """Match fixed sampled inputs, rather than accepting a count of other tasks."""
    from mj_bridge.benchmark_cli import normalized_from_path
    directory = Path(selection_dir)
    selection = json.loads((directory / 'selection.json').read_text())
    tasks = selection['tasks']
    names = {row['product'] for row in tasks}
    if len(names) != len(tasks) or len(names) != selection['selected']:
        raise ValueError('selection has duplicate or missing tasks')
    expected = {(name, selection['seed']) for name in names}
    observed = {(row['product'], row['seed']): row for row in measurements}
    if len(observed) != len(measurements):
        raise ValueError('duplicate execution in selected coverage')
    if expected - observed.keys():
        raise ValueError('fixed selection is missing executed tasks')
    for task in tasks:
        path = directory / (task['product'] + '.yaml')
        if hashlib.sha256(path.read_bytes()).hexdigest() != task['input_sha256']:
            raise ValueError('selected input changed after sampling')
        row = observed[(task['product'], selection['seed'])]
        manifest = row.get('episode_manifest.yaml', {})
        if (row.get('not_run') or manifest.get('seed') != selection['seed']
                or manifest.get('product') != normalized_from_path(str(path))):
            raise ValueError('execution does not match the selected task and seed')
    return {'population': selection['population'], 'selected_tasks': len(expected),
            'selected_successes': sum(observed[key]['success'] is True for key in expected),
            'additional_episodes': len(observed.keys() - expected),
            'selection_sha256': hashlib.sha256((directory / 'selection.json').read_bytes()).hexdigest(),
            'selection': selection}


def require_plastic_seating_evidence(result, manifest, actual):
    """Check every released part again after the complete assembly settles."""
    from mj_bridge.benchmark_core import score_episode, load_registry
    score = score_episode(manifest, actual, xy_tol=.001, z_tol=.0004)
    if not score['success']:
        raise ValueError('plastic final state exceeds seating position limits')
    targets = {b['id'] for b in manifest['target_blocks']}
    observation = result.get('final_contact_observation') or {}
    awake = observation.get('awake_parts', {})
    if (set(awake) != targets or not all(value is True for value in awake.values())
            or observation.get('settle_simulation_s', 0) < 1.):
        raise ValueError('plastic final contact observation is incomplete')
    events = result.get('events', [])
    if (len(events) != len(targets)
            or {event.get('block') for event in events} != targets):
        raise ValueError('plastic success lacks one release record per target')
    for block_id in targets:
        q = actual['blocks'][block_id].get('quaternion_wxyz', [])
        if len(q) != 4 or not all(math.isfinite(v) for v in q):
            raise ValueError('plastic final orientation is missing or invalid')
        norm = sum(v*v for v in q)
        if norm <= 0 or 1 - 2*(q[1]**2 + q[2]**2)/norm < math.cos(math.radians(3)):
            raise ValueError('plastic final state exceeds 3 degree tilt limit')
    for event in events:
        confirmations = [event.get('release_confirmation', {})]
        seating = event.get('seating_confirmation') or {}
        if 'initial_release_confirmation' in seating:
            confirmations.append(seating['initial_release_confirmation'])
            force = seating.get('bottom_support_force_n')
            rigid = isinstance(force, (int, float)) and math.isfinite(force) and force > .01
            clutch = seating.get('clutch_seating') or {}
            gap, upward = clutch.get('bottom_gap_m'), clutch.get('upward_contact_force_n')
            part = next(block for block in manifest['target_blocks'] if block['id'] == event['block'])
            weight = load_registry()[part['type']]['mass_kg'] * 9.81
            supported_fit = (isinstance(gap, (int, float)) and math.isfinite(gap) and 0 < gap < .00002
                             and isinstance(upward, (int, float)) and math.isfinite(upward)
                             and upward >= .8 * weight)
            if not rigid and not supported_fit:
                raise ValueError('top press lacks measured bottom support or a load-bearing seated fit')
        for confirmation in confirmations:
            positions = confirmation.get('finger_positions_m', [])
            if (len(positions) != 2
                    or not all(math.isfinite(v) and v >= .0395 for v in positions)
                    or confirmation.get('target_contact') is not False
                    or confirmation.get('target_contacts_observable') is not True
                    or confirmation.get('wait_simulation_s', 0) < .08):
                raise ValueError('plastic success has invalid measured release evidence')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--suite', required=True)
    parser.add_argument('--reuse-baseline-planner', action='append', default=[],
                        help='archived planner source; only unchanged direct plans can be reused')
    parser.add_argument('--reuse-baseline-executor', action='append', default=[],
                        help='archived executor for proven unused-feedback changes only')
    parser.add_argument('--reuse-equivalent-plans', action='store_true',
                        help='require identical old/new issued steps and unchanged non-planning code')
    parser.add_argument('--additional', nargs='*', default=[])
    parser.add_argument('--expected-episodes', type=int, required=True)
    parser.add_argument('--expected-total-episodes', type=int,
                        help='required unique count across primary and supplementary suites')
    parser.add_argument('--selection-dir', help='fixed sample directory containing selection.json and inputs')
    parser.add_argument('--unit-tests', type=int, required=True)
    parser.add_argument('--output', default='docs/validation')
    parser.add_argument('--invalid-products', help='JSON audit of excluded original designs')
    args = parser.parse_args()
    suite = json.loads((Path(args.suite)/'summary.json').read_text())
    if suite['episodes'] != args.expected_episodes:
        parser.error('suite is not complete; refusing to publish a final report')
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    measurements = []
    mode = suite.get('connection_mode', 'snap')
    profile = suite.get('contact_profile', 'loose')
    seen = set()
    for suite_path in [args.suite] + args.additional:
        summary = json.loads((Path(suite_path)/'summary.json').read_text())
        require_complete_summary(summary)
        if summary.get('connection_mode', 'snap') != mode:
            raise ValueError('cannot combine different connection modes')
        if summary.get('contact_profile', 'loose') != profile:
            raise ValueError('cannot combine different contact profiles')
        for row in summary['results']:
            identity = (row['product'], row['seed'])
            if identity in seen:
                raise ValueError(f'duplicate product and seed: {identity}')
            seen.add(identity)
            directory = Path(row['run_dir'])
            evidence = {k:v for k,v in row.items() if k != 'run_dir'}
            for name in ('result.json','actual_state.json','episode_manifest.yaml','assembly_plan.json'):
                path = directory/name
                if path.exists():
                    evidence[name] = yaml.safe_load(path.read_text())
            if evidence['success']:
                if evidence.get('returncode') != 0:
                    raise ValueError('success lacks a confirmed zero process exit')
                if not all(name in evidence for name in
                           ('result.json', 'actual_state.json', 'episode_manifest.yaml')):
                    raise ValueError(f'success has incomplete evidence: {directory}')
                if mode == 'physics':
                    require_physics_evidence(evidence['result.json'])
                    if evidence['result.json'].get('contact_profile') == 'plastic':
                        require_plastic_seating_evidence(
                            evidence['result.json'], evidence['episode_manifest.yaml'],
                            evidence['actual_state.json'])
                from mj_bridge.benchmark_core import score_episode, load_registry
                if (evidence['result.json'].get('success') is not True
                        or not score_episode(evidence['episode_manifest.yaml'],
                                             evidence['actual_state.json'])['success']):
                    raise ValueError(f'success disagrees with final-state evidence: {directory}')
            evidence['suite'] = Path(suite_path).name
            evidence['runner_source_sha256'] = summary.get('source_sha256', {})
            measurements.append(evidence)
    if (args.expected_total_episodes is not None
            and len(measurements) != args.expected_total_episodes):
        raise ValueError('combined suites do not cover the required unique episode count')
    coverage = require_selected_coverage(args.selection_dir, measurements) if args.selection_dir else None
    baselines = {hashlib.sha256(Path(p).read_bytes()).hexdigest(): p
                 for p in args.reuse_baseline_planner}
    executor_baselines = {hashlib.sha256(Path(p).read_bytes()).hexdigest(): p
                          for p in args.reuse_baseline_executor}
    source = Path('src/mj_bridge/mj_bridge')
    names = ('assembly_planner.py', 'benchmark_core.py', 'benchmark_cli.py', 'reference_executor.py',
             'mj_bridge3.py', 'perception.py', 'product_geometry.py', 'scene_builder.py',
             'scene_template.xml', 'panda.xml', 'hand.xml', 'part_registry.yaml',
             'benchmark.yaml', 'initial_positions.yaml', 'plastic_contact.py', 'recorded_layout.py')
    files = sorted(set(source.glob('*.py')) | {source / name for name in names})
    fingerprints = {str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(files)}
    for measurement in measurements:
        recorded = dict(measurement.get('runner_source_sha256', {}))
        execution_sources = measurement.get('result.json', {}).get('executor_source_sha256', {})
        if any(name in recorded and recorded[name] != digest
               for name, digest in execution_sources.items()):
            raise ValueError('runner and executor recorded different source revisions')
        recorded.update(execution_sources)
        for name, digest in recorded.items():
            path = source / name
            if str(path) in fingerprints and fingerprints[str(path)] != digest:
                if name == 'assembly_planner.py' and digest in baselines:
                    import runpy
                    verifier = runpy.run_path(str(Path(__file__).with_name('direct_plan_reuse.py')))
                    key = 'verify_equivalent_plan_reuse' if args.reuse_equivalent_plans else 'verify_direct_plan_reuse'
                    measurement['plan_reuse'] = verifier[key](
                        baselines[digest], path, recorded,
                        measurement['episode_manifest.yaml'], measurement.get('assembly_plan.json', {}))
                elif name == 'reference_executor.py' and digest in executor_baselines:
                    import runpy
                    verifier = runpy.run_path(str(Path(__file__).with_name('unused_feedback_reuse.py')))
                    measurement['executor_reuse'] = verifier['verify_unused_feedback_reuse'](
                        executor_baselines[digest], path, recorded, measurement['result.json'],
                        measurement['episode_manifest.yaml'])
                else:
                    raise ValueError(f'execution used a different source revision: {name}')
    invalid = (json.loads(Path(args.invalid_products).read_text()) if args.invalid_products
               else suite.get('invalid_products', []))
    report = {
        'created_utc':datetime.now(timezone.utc).isoformat(),
        'scope':f'MuJoCo Oracle baseline, {mode} connections; no real-robot validation',
        'connection_mode':mode, 'contact_profile':profile,
        'unique_episodes':len(measurements),
        'selected_coverage':coverage,
        'unit_tests_passed':args.unit_tests,
        'catalogue_products':suite.get('input_products', 0) + (len(invalid) if args.invalid_products else 0),
        'primary_episodes':suite['episodes'], 'primary_successes':suite['successes'],
        'additional_episodes':len(measurements)-suite['episodes'],
        'additional_successes':sum(bool(m['success']) for m in measurements)-suite['successes'],
        'invalid_products':invalid,
        'vision_status':'NOT_VALIDATED: CUDA runtime, SAM and FoundationPose weights unavailable',
        'runtime':{p:version(p) for p in ('mujoco','numpy','PyYAML','trimesh','pytest')},
        'source_sha256':fingerprints, 'measurements':measurements,
    }
    (output/'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
    lines = ['# 仿真验证报告', '',
             f"记录时间（UTC）：{report['created_utc']}", '',
             f"代码回归：{args.unit_tests} 项通过。主产品测试：{suite['successes']}/{suite['episodes']} 成功。",
             f"补充测试（按下表独立产品名称统计）：{report['additional_successes']}/{report['additional_episodes']} 成功。", '',
             f'范围：Oracle 观测、MuJoCo 接触仿真、{mode} 连接模式。未验证实机。',
             ('plastic 使用空心零件、导入斜面与过盈接触，扣合阻力由柔顺接触和摩擦产生；未经实物材料或拔出力标定。'
              if profile == 'plastic' else
              'physics 使用有间隙的刚体空腔模型，无塑料弹性或真实扣合力标定。')
             + '成功轮的最大接触穿透不得超过0.5mm。' if mode == 'physics' else '历史snap末态评分不能证明物理夹持无穿透。',
             '视觉推理未验证：缺少 CUDA 环境及 SAM/FoundationPose 权重。RGB-D 渲染已单独验证。', '',
             '## 原目标存在问题的历史产品', '',
             '这些产品保留原始目标，未计入装配成功数量。带 `_supported` 后缀的替代设计是独立目标，其成功不代表原目标成功。', '',
             '| 产品 | 几何问题 |', '|---|---|']
    labels = {'solid_overlap':'实体重叠', 'insufficient_support':'支撑不足',
              'layer_height_mismatch':'层高不匹配', 'below_base':'低于底板'}
    if coverage:
        lines[3:3] = [
            f"固定分层样本：{coverage['selected_tasks']}/{coverage['population']} 个任务，"
            f"其中 {coverage['selected_successes']} 个成功；样本外补充 {coverage['additional_episodes']} 轮。",
            '未测试的任务未计入成功数量。原始抽样清单、输入哈希与实际执行清单已逐项核对。', '']
    for item in invalid:
        codes = sorted({issue['code'] for issue in item['issues']})
        lines.append(f"| {item['product']} | {'、'.join(labels.get(c,c) for c in codes)} |")
    lines += ['', '## 实际执行结果', '', '| 产品 | Seed | 结果 |', '|---|---:|---|']
    for item in sorted(measurements, key=lambda r:(r['product'],r['seed'])):
        lines.append(f"| {item['product']} | {item['seed']} | {'成功' if item['success'] else '失败'} |")
    lines += ['', '[完整数据、逐块评分、实际末态及初始 manifest](report.json)', '',
              '这些结果只覆盖记录的产品、种子和连接模式，不构成任意随机布局的成功保证。', '']
    (output/'REPORT_ZH.md').write_text('\n'.join(lines))
    print(output/'REPORT_ZH.md')


if __name__ == '__main__':
    main()
