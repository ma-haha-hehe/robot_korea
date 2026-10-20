"""Audit one fresh episode; passing this does not validate a whole task suite."""
import argparse
import hashlib
import json
from pathlib import Path
import runpy
import sys
import yaml


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def require(condition, message):
    if not condition:
        raise ValueError(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate-manifest', type=Path, required=True)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--product', type=Path, required=True)
    args = parser.parse_args()
    require(not any((args.run / item).exists() for item in
                    ('DIAGNOSTIC_ONLY.json', 'insertion_snapshots', 'release_snapshots')),
            'instrumented diagnostics are not complete-task acceptance evidence')
    candidate = json.loads(args.candidate_manifest.read_text())
    package = Path(candidate['path'])
    source = package / 'src/mj_bridge/mj_bridge'
    sys.path.insert(0, str(source.parent))
    from mj_bridge.benchmark_cli import normalized_from_path
    from mj_bridge.benchmark_core import score_episode
    from mj_bridge.assembly_planner import plan_with_release_above
    result = json.loads((args.run / 'result.json').read_text())
    require(result['success'] is True and not result['timeout']
            and result['execution_error'] is None, 'episode did not complete successfully')
    require(result['placed'] == result['total'], 'not all parts were placed')
    config_path = package / 'examples/config/plastic_benchmark.yaml'
    require(digest(config_path) == candidate['config_sha256'], 'configuration changed')
    config = yaml.safe_load(config_path.read_text())
    require(result['wall_clock_limit_s'] == config['max_episode_time_s'], 'resource budget differs')
    required_sources = {
        'reference_executor.py', 'mj_bridge3.py', 'benchmark_core.py', 'scene_builder.py',
        'assembly_planner.py', 'plastic_contact.py', 'panda.xml', 'hand.xml',
        'scene_template.xml', 'part_registry.yaml', 'perception.py', 'recorded_layout.py',
    } | ({'execution_phases.py', 'support_geometry.py'} & candidate['runtime_sha256'].keys())
    require(required_sources <= result['executor_source_sha256'].keys(),
            'executed source evidence is incomplete')
    for mapping in (candidate['runtime_sha256'], result['executor_source_sha256']):
        for name, sha in mapping.items():
            require(digest(source / name) == sha, f'source changed: {name}')
    manifest = yaml.safe_load((args.run / 'episode_manifest.yaml').read_text())
    actual = json.loads((args.run / 'actual_state.json').read_text())
    plan = json.loads((args.run / 'assembly_plan.json').read_text())
    require(normalized_from_path(str(args.product)) == yaml.safe_load(
        (args.run / 'product.normalized.yaml').read_text()), 'product input differs')
    require(plan_with_release_above(manifest['target_blocks']) == plan['steps'], 'plan differs')
    require(score_episode(manifest, actual)['success'], 'final state failed independent scoring')
    checks = runpy.run_path(str(package / 'scripts/write_validation_report.py'))
    checks['require_physics_evidence'](result)
    checks['require_plastic_seating_evidence'](result, manifest, actual)
    for name, sha in result['generated_model_sha256'].items():
        require(digest(args.run / name) == sha, f'generated model changed: {name}')
    report = dict(scope='One complete episode only; not a suite or measured-material validation',
                  passed=True, candidate_manifest_sha256=digest(args.candidate_manifest),
                  audit_script_sha256=digest(Path(__file__)),
                  product=str(args.product.resolve()), product_sha256=digest(args.product),
                  config_sha256=digest(config_path),
                  source_path=str(source), result=result,
                  artifact_sha256={name: digest(args.run / name) for name in (
                      'result.json', 'actual_state.json', 'episode_manifest.yaml',
                      'assembly_plan.json', 'product.normalized.yaml')})
    (args.run / 'STRICT_AUDIT.json').write_text(json.dumps(report, indent=2) + '\n')
    print(f'Verified complete episode: {args.run.name} ({result["placed"]}/{result["total"]})')


if __name__ == '__main__':
    main()
