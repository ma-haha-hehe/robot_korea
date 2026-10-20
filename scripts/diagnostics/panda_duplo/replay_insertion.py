"""Compare measured-state insertion unloading; never counts as a complete run."""
import argparse
import json
from pathlib import Path
import time
import numpy as np
from probe_materials import OfflineNode, restore, save_state, ex
from replay_release import enable_force_mode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene-dir', type=Path, required=True)
    parser.add_argument('--snapshot', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--policy', choices=['baseline', 'balanced', 'current_width'], required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    node = OfflineNode(args.scene_dir)
    executor = ex.OracleExecutor(node)
    restore(executor, args.snapshot / 'state.npz', args.snapshot / 'state.events.json')
    executor.last_seating_confirmation = None
    target = json.loads((args.snapshot / 'target.json').read_text())
    target['position'] = np.asarray(target['position'])
    original = executor.open_gripper_before_retreat
    first = True
    if args.policy == 'balanced':
        enable_force_mode(node)

    def open_gripper(part, timeout_s=2.):
        nonlocal first
        if first:
            first = False
            save_state(executor, args.output / 'release.npz')
            body = node.model.body(part['body_name']).id
            (args.output / 'release.json').write_text(json.dumps({
                'position': node.data.xpos[body].tolist(),
                'target': part['position'].tolist(),
                'gripper_control': node.gripper_control,
                'arm_qvel': node.data.qvel[executor.dadr].tolist()}, indent=2))
            if args.policy == 'balanced':
                node.target_qpos[executor.qadr] = node.data.qpos[executor.qadr].copy()
                forces = [abs(float(node.data.actuator_force[node.model.actuator(f'finger_actuator{i}').id]))
                          for i in (1, 2)]
                for index in range(21):
                    node._balanced_force = min(5., min(forces)) * (1 - index / 20)
                    executor.step(.01)
                node._balanced_force = None
            if args.policy in ('balanced', 'current_width'):
                qadr = [node.model.joint(f'panda_finger_joint{i}').qposadr[0] for i in (1, 2)]
                node.gripper_control = float(np.mean(node.data.qpos[qadr]))
        return original(part, timeout_s)

    executor.open_gripper_before_retreat = open_gripper
    start = time.monotonic()
    error = None
    try:
        yaw = executor.place_physical(target)
        executor.open_gripper_before_retreat(target)
        executor.move(target['position'] + [0., 0., .157], yaw)
        executor.step(.5)
    except ex.ExecutionFailure as exc:
        error = str(exc)
    body = node.model.body(target['body_name']).id
    report = {'scope': 'Partial restored-state diagnostic; not physical task acceptance',
              'policy': args.policy, 'error': error, 'wall_s': time.monotonic() - start,
              'max_part_penetration_m': node.max_part_penetration_m,
              'final_position': node.data.xpos[body].tolist(),
              'target_position': target['position'].tolist(),
              'final_quaternion': node.data.xquat[body].tolist(),
              'seating': getattr(executor, 'last_seating_confirmation', None)}
    (args.output / 'diagnostic.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
