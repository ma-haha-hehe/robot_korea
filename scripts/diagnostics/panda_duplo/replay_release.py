"""Local release diagnostic from a measured snapshot; never a full task trial."""
import argparse
import ast
import hashlib
import inspect
import json
import math
from pathlib import Path
import textwrap
import time
from types import MethodType
import mujoco
import numpy as np
from probe_materials import OfflineNode, restore, ex, MuJoCoActionServer


def enable_force_mode(node):
    """Change finger actuator setpoints, preserving the real 5 N force limits."""
    original = MuJoCoActionServer.step_pid
    function = ast.parse(textwrap.dedent(inspect.getsource(original))).body[0]
    body = function.body[0].body  # the production lock context
    index = next(i for i, statement in enumerate(body)
                 if isinstance(statement, ast.Expr) and isinstance(statement.value, ast.Call)
                 and isinstance(statement.value.func, ast.Attribute)
                 and statement.value.func.attr == 'mj_step')
    injection = ast.parse('''
if getattr(self, '_balanced_force', None) is not None:
    for finger in (1, 2):
        actuator = self.model.actuator(f'finger_actuator{finger}').id
        qadr = self.model.joint(f'panda_finger_joint{finger}').qposadr[0]
        gain = self.model.actuator_gainprm[actuator, 0]
        self.data.ctrl[actuator] = self.data.qpos[qadr] - self._balanced_force / gain
''').body
    body[index:index] = injection
    namespace = dict(original.__globals__)
    exec(compile(ast.Module(body=[function], type_ignores=[]), __file__, 'exec'), namespace)
    node.step_pid = MethodType(namespace['step_pid'], node)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene-dir', type=Path, required=True)
    parser.add_argument('--snapshot', type=Path, required=True)
    parser.add_argument('--policy', choices=['baseline', 'endpoint', 'balanced', 'balanced_endpoint',
                                           'descend_2mm', 'descend_1mm', 'descend_0.4mm',
                                           'held_2n', 'held', 'held_aligned', 'current_width'], required=True)
    parser.add_argument('--hold-force', type=float, default=2.,
                        help='per-finger force for the held diagnostic, in N (0, 5]')
    parser.add_argument('--trace', action='store_true', help='record release motion every 10 ms')
    parser.add_argument('--integration-only', action='store_true',
                        help='refresh derived state; explicitly not an exact native-data replay')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not math.isfinite(args.hold_force) or not 0 < args.hold_force <= 5:
        parser.error('--hold-force must be finite and in (0, 5] N')
    args.output.mkdir(parents=True, exist_ok=False)
    node = OfflineNode(args.scene_dir)
    executor = ex.OracleExecutor(node)
    restore(executor, args.snapshot / 'state.npz', args.snapshot / 'state.events.json',
            use_exact=not args.integration_only)
    if not hasattr(executor, 'last_seating_confirmation'):
        executor.last_seating_confirmation = None
    release = json.loads((args.snapshot / 'release.json').read_text())
    target = release['target']
    target['position'] = np.array(target['position'])
    original_open = executor.open_gripper_before_retreat
    first_open = True
    if args.policy.startswith('balanced') or args.policy.startswith('held'):
        enable_force_mode(node)

    def open_gripper(target, timeout_s=2.):
        nonlocal first_open
        if not first_open:
            return original_open(target, timeout_s)
        first_open = False
        active = executor._top_press_active
        seating = executor.last_seating_confirmation
        try:
            if args.policy.startswith('balanced'):
                node.target_qpos[executor.qadr] = node.data.qpos[executor.qadr].copy()
                forces = [abs(float(node.data.actuator_force[node.model.actuator(f'finger_actuator{i}').id])) for i in (1, 2)]
                start_force = min(5., min(forces))
                for index in range(21):
                    node._balanced_force = start_force * (1 - index / 20)
                    executor.step(.01)
                qadr = [node.model.joint(f'panda_finger_joint{i}').qposadr[0] for i in (1, 2)]
                node.gripper_control = float(np.mean(node.data.qpos[qadr]))
                node._balanced_force = None
            if 'endpoint' in args.policy:
                executor._top_press_active = False
                executor.last_seating_confirmation = None
            if args.policy == 'current_width':
                qadr = [node.model.joint(f'panda_finger_joint{i}').qposadr[0] for i in (1, 2)]
                node.gripper_control = float(np.mean(node.data.qpos[qadr]))
            return original_open(target, timeout_s)
        finally:
            node._balanced_force = None
            executor._top_press_active = active
            executor.last_seating_confirmation = seating

    executor.open_gripper_before_retreat = open_gripper
    part = node.model.body(target['body_name']).id
    samples = []
    phase = 'release'
    if args.trace:
        import functools
        for method_name in ('capture_support_edge', 'open_gripper_before_retreat',
                            'move_vertical', '_seat_plastic_part'):
            def instrument(name, original):
                @functools.wraps(original)
                def wrapped(*positional, **keywords):
                    nonlocal phase
                    previous, phase = phase, name
                    try:
                        return original(*positional, **keywords)
                    finally:
                        phase = previous
                return wrapped
            setattr(executor, method_name, instrument(method_name, getattr(executor, method_name)))
        original_step = node.step_pid
        last_sample = -math.inf
        def record_step():
            nonlocal last_sample
            original_step()
            if node.data.time - last_sample < .01 - 1e-9:
                return
            last_sample = float(node.data.time)
            q = node.data.xquat[part]
            samples.append({'time_s': last_sample, 'phase': phase,
                            'position_m': node.data.xpos[part].tolist(),
                            'tilt_deg': math.degrees(math.acos(float(np.clip(
                                1 - 2*(q[1]**2+q[2]**2), -1, 1)))),
                            'max_part_penetration_m': node.max_part_penetration_m})
        node.step_pid = record_step
    initial = dict(position=node.data.xpos[part].tolist(), quaternion=node.data.xquat[part].tolist())
    start = time.monotonic()
    error = None
    engine_error = False
    try:
        if args.policy.startswith('held'):
            hold_force = 2. if args.policy == 'held_2n' else args.hold_force
            forces = [abs(float(node.data.actuator_force[node.model.actuator(f'finger_actuator{i}').id]))
                      for i in (1, 2)]
            initial_force = min(5., min(forces))
            try:
                for index in range(1, 11):
                    node._balanced_force = initial_force + (hold_force - initial_force) * index / 10
                    executor.step(.01)
                executor.confirm_held_part(target)
                rotation = node.data.xmat[executor.hand].reshape(3, 3)
                tip = node.data.xpos[executor.hand] + rotation @ executor.tool_offset
                tip[2] += target['position'][2] + .001 - node.data.xpos[part, 2]
                executor.move_vertical(tip, release['yaw'])
                executor._seat_plastic_part(target, release['yaw'])
                if args.policy == 'held_aligned':
                    executor.refine_overhang_pose(target, release['yaw'])
            finally:
                node._balanced_force = None
            qadr = [node.model.joint(f'panda_finger_joint{i}').qposadr[0] for i in (1, 2)]
            node.gripper_control = float(np.mean(node.data.qpos[qadr]))
        if args.policy.startswith('descend_'):
            gap = float(args.policy.removeprefix('descend_').removesuffix('mm')) / 1000
            rotation = node.data.xmat[executor.hand].reshape(3, 3)
            tip = node.data.xpos[executor.hand] + rotation @ executor.tool_offset
            tip[2] += target['position'][2] + gap - node.data.xpos[part, 2]
            previous_speed = executor.speed_scale
            executor.speed_scale = .2
            executor.move_vertical(tip, release['yaw'])
            executor.speed_scale = previous_speed
        executor.release_and_press(target, release['yaw'], release['placement_mode'])
        executor.open_gripper_before_retreat(target)
        executor.move(np.array(target['position']) + [0., 0., .157], release['yaw'])
        executor.step(.5)
    except ex.ExecutionFailure as caught:
        error = str(caught)
    except mujoco.FatalError as caught:
        error = str(caught)
        engine_error = True
    report = dict(scope='PARTIAL saved-state diagnostic; not full-task acceptance', policy=args.policy,
                  hold_force_n=args.hold_force if args.policy in ('held', 'held_aligned') else None,
                  restoration='integration_only' if args.integration_only else 'native_data_if_available',
                  diagnostic_engine_error=engine_error,
                  initial=initial, error=error, wall_s=time.monotonic()-start,
                  max_part_penetration_m=node.max_part_penetration_m,
                  final_position=node.data.xpos[part].tolist(), final_quaternion=node.data.xquat[part].tolist(),
                  seating=getattr(executor, 'last_seating_confirmation', None),
                  snapshot_sha256=hashlib.sha256((args.snapshot/'state.npz').read_bytes()).hexdigest(),
                  local_score=node.benchmark_result())
    (args.output/'diagnostic.json').write_text(json.dumps(report, indent=2)+'\n')
    if args.trace:
        (args.output/'motion_trace.json').write_text(json.dumps(samples, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    if engine_error:
        raise SystemExit(2)


if __name__ == '__main__':
    main()
