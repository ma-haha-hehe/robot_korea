"""Partial saved-state test of guarded edge capture; never suite evidence."""
import argparse
import json
import math
from pathlib import Path
import time
import numpy as np
from probe_materials import OfflineNode, restore, ex


class CaptureLoaded(Exception):
    pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene-dir', type=Path, required=True)
    parser.add_argument('--snapshot', type=Path, required=True)
    parser.add_argument('--tilt-deg', type=float, default=.3)
    parser.add_argument('--penetration-mm', type=float, default=.2)
    parser.add_argument('--tilt-rate-rad-s', type=float)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    node = OfflineNode(args.scene_dir)
    executor = ex.OracleExecutor(node)
    restore(executor, args.snapshot / 'state.npz', args.snapshot / 'state.events.json')
    release = json.loads((args.snapshot / 'release.json').read_text())
    target = release['target']
    target['position'] = np.array(target['position'])
    part = node.model.body(target['body_name']).id
    original_step = executor.step
    original_capture = executor.capture_support_edge
    samples = []

    def step(seconds):
        if getattr(executor, '_capture_guard_target', None) is None:
            return original_step(seconds)
        for _ in range(max(1, int(seconds / node.model.opt.timestep))):
            original_step(node.model.opt.timestep)
            q = node.data.xquat[part]
            tilt = math.degrees(math.acos(np.clip(1 - 2*(q[1]**2 + q[2]**2), -1., 1.)))
            depth = 0.
            for contact in node.data.contact:
                bodies = [int(node.model.geom_bodyid[g]) for g in (contact.geom1, contact.geom2)]
                if part in bodies and all(node.model.body(b).name.startswith('bench__') for b in bodies):
                    depth = max(depth, -float(contact.dist))
            row = dict(time_s=float(node.data.time), tilt_deg=tilt,
                       penetration_m=depth, height_m=float(node.data.xpos[part, 2]-target['position'][2]))
            previous = samples[-1] if samples else row
            tilt_rate = math.radians(tilt - previous['tilt_deg']) / node.model.opt.timestep
            down_speed = (previous['height_m'] - row['height_m']) / node.model.opt.timestep
            samples.append(row)
            tipping = (tilt >= args.tilt_deg if args.tilt_rate_rad_s is None else
                       tilt_rate > args.tilt_rate_rad_s and down_speed > .02)
            if tipping or depth >= args.penetration_mm / 1000:
                node.target_qpos[executor.qadr] = node.data.qpos[executor.qadr].copy()
                raise CaptureLoaded()

    def capture(target, yaw):
        try:
            return original_capture(target, yaw)
        except CaptureLoaded:
            return dict(mode='diagnostic_guard_stop', **samples[-1])

    executor.step = step
    executor.capture_support_edge = capture
    start = time.monotonic()
    error = None
    try:
        executor.release_and_press(target, release['yaw'], release['placement_mode'])
        executor.open_gripper_before_retreat(target)
        executor.move(np.asarray(target['position']) + [0., 0., .157], release['yaw'])
        executor.step(.5)
    except ex.ExecutionFailure as exc:
        error = str(exc)
    report = dict(scope='PARTIAL saved-state diagnostic', error=error,
                  wall_s=time.monotonic()-start, samples=samples,
                  max_part_penetration_m=node.max_part_penetration_m,
                  final_position=node.data.xpos[part].tolist(),
                  seating=getattr(executor, 'last_seating_confirmation', None),
                  local_score=node.benchmark_result())
    (args.output / 'diagnostic.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k: v for k, v in report.items() if k not in ('samples', 'local_score')}, indent=2))


if __name__ == '__main__':
    main()
