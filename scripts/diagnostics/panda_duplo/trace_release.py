"""Record unmodified controller states at initial releases in a fresh episode."""
import argparse
import json
import math
from pathlib import Path
import numpy as np
from mj_bridge import benchmark_cli, reference_executor


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--product', required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if args.output.exists():
        p.error('output must not exist')
    original = reference_executor.OracleExecutor.release_and_press

    def recorded(self, target, yaw, placement_mode):
        # Import only after benchmark_cli has selected the generated scene.
        # The ROS bridge reads its model/environment paths at module import.
        from probe_materials import save_state
        (args.output / 'DIAGNOSTIC_ONLY.json').write_text(json.dumps(
            {'scope': 'Instrumented release trace; not acceptance evidence'}))
        directory = args.output / 'release_snapshots' / target['id']
        directory.mkdir(parents=True, exist_ok=False)
        save_state(self, directory / 'state.npz')
        (directory / 'release.json').write_text(json.dumps(
            dict(target=target, yaw=yaw, placement_mode=placement_mode),
            default=lambda x: x.tolist() if isinstance(x, np.ndarray) else float(x), indent=2))
        original_step = self.node.step_pid
        samples = []
        part = self.model.body(target['body_name']).id
        def step():
            original_step()
            if getattr(self, '_capture_guard_target', None) is not None:
                q = self.data.xquat[part]
                samples.append(dict(time_s=float(self.data.time),
                    height_m=float(self.data.xpos[part, 2] - target['position'][2]),
                    tilt_deg=math.degrees(math.acos(float(np.clip(1-2*(q[1]**2+q[2]**2), -1, 1)))),
                    max_part_penetration_m=self.node.max_part_penetration_m))
        self.node.step_pid = step
        try:
            return original(self, target, yaw, placement_mode)
        finally:
            self.node.step_pid = original_step
            (directory / 'capture_trace.json').write_text(json.dumps(samples, indent=2))

    reference_executor.OracleExecutor.release_and_press = recorded
    return benchmark_cli.main(['run', '--product', args.product, '--seed', '42', '--headless',
        '--connection-mode', 'physics', '--contact-profile', 'plastic', '--executor', 'baseline',
        '--robot-base-x=-0.05', '--output-dir', str(args.output)])


if __name__ == '__main__':
    raise SystemExit(main())
