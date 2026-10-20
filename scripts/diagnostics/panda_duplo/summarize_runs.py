"""Summarize complete and incomplete audit episodes without hiding failures."""
import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path


def summarize(directory):
    rows = []
    for manifest in sorted(directory.rglob('episode_manifest.yaml')):
        run = manifest.parent
        name = str(run.relative_to(directory))
        instrumented = any((run / item).exists() for item in
                           ('DIAGNOSTIC_ONLY.json', 'insertion_snapshots', 'release_snapshots'))
        path = run / 'result.json'
        if not path.exists():
            rows.append({'run': name, 'status': 'incomplete',
                         'instrumented_diagnostic': instrumented,
                         'aborted': (run / 'ABORTED.json').exists()})
            continue
        result = json.loads(path.read_text())
        audit = run / 'STRICT_AUDIT.json'
        verified = False
        if audit.exists():
            certificate = json.loads(audit.read_text())
            hashes = certificate.get('artifact_sha256', {})
            verified = (not instrumented and certificate.get('passed') is True
                        and 'result.json' in hashes and all(
                            (run / artifact).is_file() and
                            hashlib.sha256((run / artifact).read_bytes()).hexdigest() == expected
                            for artifact, expected in hashes.items()))
        timing = defaultdict(lambda: {'wall_s': 0., 'simulation_s': 0.})
        for phase in result.get('phase_timings', []):
            for clock in ('wall_s', 'simulation_s'):
                timing[phase['phase']][clock] += phase.get(clock, 0.)
        rows.append({
            'run': name,
            'status': 'passed' if result.get('success') is True else 'failed',
            'independently_audited': verified,
            'instrumented_diagnostic': instrumented,
            'released_count': sum(event.get('status') == 'released'
                                  for event in result.get('events', [])),
            **{key: result.get(key) for key in (
                'placed', 'total', 'seed', 'elapsed_s', 'simulation_time_s',
                'execution_error', 'failed_phase', 'connection_mode', 'contact_profile',
                'max_part_penetration_m', 'max_robot_environment_penetration_m',
                'collision_count', 'attachment_count', 'executor_source_sha256')},
            'phase_totals': dict(timing),
        })
    return {'scope': 'Selected diagnostic episodes across different revisions; '
                     'do not pool them into a suite success rate.', 'episodes': rows}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', type=Path, default=Path(__file__).resolve().parents[3]
                        / 'runs/project-audit-20261013')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    payload = json.dumps(summarize(args.directory), indent=2) + '\n'
    if args.output:
        args.output.write_text(payload)
    else:
        print(payload, end='')
