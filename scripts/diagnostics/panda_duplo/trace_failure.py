"""Instrument a fresh physical episode; diagnostic output is not acceptance evidence."""
import argparse
import functools
import json
import math
from pathlib import Path
import traceback

import numpy as np

from mj_bridge import benchmark_cli, reference_executor


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--product', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--exact-state-block',
                        help='also save full derived simulation data for this diagnostic block')
    parser.add_argument('--motion-trace-block',
                        help='record measured pose after controller steps for one placed block')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output must not exist')
    stack = []
    samples = []
    original_step = reference_executor.OracleExecutor.step

    def traced_step(self, *positional, **kwargs):
        try:
            return original_step(self, *positional, **kwargs)
        finally:
            if args.motion_trace_block and any(
                    row['block'] == args.motion_trace_block for row in stack):
                target = next(b for b in self.node.episode_manifest['target_blocks']
                              if b['id'] == args.motion_trace_block)
                part = self.model.body(target['body_name']).id
                q = self.data.xquat[part]
                samples.append(dict(time_s=float(self.data.time),
                    methods=[row['method'] for row in stack],
                    position_m=self.data.xpos[part].tolist(),
                    error_m=(self.data.xpos[part]-np.asarray(target['position'])).tolist(),
                    tilt_deg=math.degrees(math.acos(float(np.clip(
                        1-2*(q[1]**2+q[2]**2), -1, 1))))))

    if args.motion_trace_block:
        reference_executor.OracleExecutor.step = traced_step

    def instrument(name):
        original = getattr(reference_executor.OracleExecutor, name)

        @functools.wraps(original)
        def wrapped(self, *positional, **kwargs):
            # Delay importing the bridge until the CLI selected its generated model.
            from probe_materials import save_state
            marker = args.output / 'DIAGNOSTIC_ONLY.json'
            if not marker.exists():
                marker.write_text(json.dumps({'scope': 'Instrumented contact trace; not acceptance evidence'}))
            target = positional[0] if positional and isinstance(positional[0], dict) else None
            label = {'method': name, 'block': target.get('id') if target else None,
                     'time_s': float(self.data.time)}
            stack.append(label)
            if name in ('place_physical', 'descend_with_pose_feedback'):
                category = 'insertion_snapshots' if name == 'place_physical' else 'feedback_snapshots'
                directory = args.output / category / target['id']
                directory.mkdir(parents=True, exist_ok=False)
                save_state(self, directory / 'state.npz', exact=target['id'] == args.exact_state_block)
                (directory / 'target.json').write_text(json.dumps(target, indent=2))
            if name == 'release_and_press' and target['id'] == args.exact_state_block:
                directory = args.output / 'release_snapshots' / target['id']
                directory.mkdir(parents=True, exist_ok=False)
                save_state(self, directory / 'state.npz', exact=True)
                (directory / 'release.json').write_text(json.dumps(
                    {'target': target, 'yaw': positional[1], 'placement_mode': positional[2]}, indent=2))
            try:
                return original(self, *positional, **kwargs)
            except reference_executor.ExecutionFailure:
                destination = args.output / 'failure.json'
                if not destination.exists():
                    contacts = []
                    for contact in self.data.contact:
                        if contact.dist >= 0:
                            continue
                        contacts.append({'depth_m': -float(contact.dist),
                            'geoms': [self.model.geom(g).name for g in (contact.geom1, contact.geom2)],
                            'bodies': [self.model.body(self.model.geom_bodyid[g]).name
                                       for g in (contact.geom1, contact.geom2)]})
                    contacts.sort(key=lambda row: row['depth_m'], reverse=True)
                    save_state(self, args.output / 'failure_state.npz')
                    destination.write_text(json.dumps({'scope': 'instrumented diagnostic only',
                        'stack': stack, 'simulation_time_s': float(self.data.time),
                        'contacts': contacts[:30], 'traceback': traceback.format_exc()}, indent=2))
                raise
            finally:
                stack.pop()
        setattr(reference_executor.OracleExecutor, name, wrapped)

    for method in ('place_physical', 'place_with_release_above', 'descend_to_insertion_entry',
                   'release_and_press', 'capture_support_edge', 'lower_with_finger_friction',
                   'open_gripper_before_retreat', '_seat_plastic_part', 'move_vertical',
                   'descend_with_pose_feedback'):
        instrument(method)
    try:
        return benchmark_cli.main(['run', '--product', args.product, '--seed', '42', '--headless',
            '--connection-mode', 'physics', '--contact-profile', 'plastic', '--executor', 'baseline',
            '--robot-base-x=-0.05', '--output-dir', str(args.output)])
    finally:
        if samples:
            (args.output / 'motion_trace.json').write_text(json.dumps(samples, indent=2)+'\n')


if __name__ == '__main__':
    raise SystemExit(main())
