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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--suite', required=True)
    parser.add_argument('--additional', nargs='*', default=[])
    parser.add_argument('--expected-episodes', type=int, required=True)
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
    for suite_path in [args.suite] + args.additional:
        summary = json.loads((Path(suite_path)/'summary.json').read_text())
        require_complete_summary(summary)
        if summary.get('connection_mode', 'snap') != mode:
            raise ValueError('cannot combine different connection modes')
        for row in summary['results']:
            directory = Path(row['run_dir'])
            evidence = {k:v for k,v in row.items() if k != 'run_dir'}
            for name in ('result.json','actual_state.json','episode_manifest.yaml'):
                path = directory/name
                if path.exists():
                    evidence[name] = yaml.safe_load(path.read_text())
            if evidence['success']:
                if not all(name in evidence for name in
                           ('result.json', 'actual_state.json', 'episode_manifest.yaml')):
                    raise ValueError(f'success has incomplete evidence: {directory}')
                if mode == 'physics':
                    require_physics_evidence(evidence['result.json'])
                from mj_bridge.benchmark_core import score_episode
                if (evidence['result.json'].get('success') is not True
                        or not score_episode(evidence['episode_manifest.yaml'],
                                             evidence['actual_state.json'])['success']):
                    raise ValueError(f'success disagrees with final-state evidence: {directory}')
            evidence['suite'] = Path(suite_path).name
            measurements.append(evidence)
    source = Path('src/mj_bridge/mj_bridge')
    names = ('assembly_planner.py', 'benchmark_core.py', 'benchmark_cli.py', 'reference_executor.py',
             'mj_bridge3.py', 'perception.py', 'product_geometry.py', 'scene_builder.py',
             'scene_template.xml', 'panda.xml', 'hand.xml', 'part_registry.yaml',
             'benchmark.yaml', 'initial_positions.yaml')
    files = [source / name for name in names]
    fingerprints = {str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(files)}
    for measurement in measurements:
        recorded = measurement.get('result.json', {}).get('executor_source_sha256', {})
        for name, digest in recorded.items():
            path = source / name
            if str(path) in fingerprints and fingerprints[str(path)] != digest:
                raise ValueError(f'execution used a different source revision: {name}')
    invalid = (json.loads(Path(args.invalid_products).read_text()) if args.invalid_products
               else suite.get('invalid_products', []))
    report = {
        'created_utc':datetime.now(timezone.utc).isoformat(),
        'scope':f'MuJoCo Oracle baseline, {mode} connections; no real-robot validation',
        'connection_mode':mode,
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
             'physics 使用有间隙的刚体空腔模型，无塑料弹性或真实扣合力标定；成功轮的最大接触穿透不得超过0.5mm。' if mode == 'physics' else '历史snap末态评分不能证明物理夹持无穿透。',
             '视觉推理未验证：缺少 CUDA 环境及 SAM/FoundationPose 权重。RGB-D 渲染已单独验证。', '',
             '## 原目标存在问题的历史产品', '',
             '这些产品保留原始目标，未计入装配成功数量。带 `_supported` 后缀的替代设计是独立目标，其成功不代表原目标成功。', '',
             '| 产品 | 几何问题 |', '|---|---|']
    labels = {'solid_overlap':'实体重叠', 'insufficient_support':'支撑不足',
              'layer_height_mismatch':'层高不匹配', 'below_base':'低于底板'}
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
