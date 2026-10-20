"""Saved-state material diagnostics. Never counts as a complete task run.

Reuse the production controller and stepping methods without ROS initialization.
Restoring a snapshot is ONLY a diagnostic initial condition, not assembly.
"""
import argparse
import ast
import hashlib
import inspect
import json
import gzip
import os
from pathlib import Path
import pickle
import textwrap
import threading
import time
from types import SimpleNamespace

import mujoco
import numpy as np
import yaml
from mj_bridge import reference_executor as ex
from mj_bridge.mj_bridge3 import MuJoCoActionServer

ROOT = Path(__file__).resolve().parents[3]
SOURCE = ROOT / 'runs/contact-unload-3600-full-079'


class OfflineNode:
    step_pid = MuJoCoActionServer.step_pid
    current_block_state = MuJoCoActionServer.current_block_state
    update_safety_metrics = MuJoCoActionServer.update_safety_metrics
    benchmark_result = MuJoCoActionServer.benchmark_result

    def get_logger(self):
        return SimpleNamespace(info=lambda msg: print(msg, flush=True))

    def __init__(self, source=SOURCE):
        self.model = mujoco.MjModel.from_xml_path(str(source / 'scene.xml'))
        self.data = mujoco.MjData(self.model)
        self.mj_lock = threading.RLock()
        self.episode_manifest = yaml.safe_load((source / 'episode_manifest.yaml').read_text())
        self.benchmark_config = {'max_episode_time_s': 3600.}
        self.episode_start_time = time.monotonic()
        self.episode_timeout = False
        self.connection_mode = 'physics'
        self.observation_mode = 'oracle'
        self.fake_welds = []
        self.max_part_penetration_m = 0.
        self.max_robot_environment_penetration_m = 0.
        self.max_gripper_penetration_m = 0.
        self.collision_count = self.stability_violations = 0
        self._active_safety_contacts = set()
        self._unstable_blocks = set()


def save_state(executor, path, exact=False):
    node = executor.node
    spec = mujoco.mjtState.mjSTATE_INTEGRATION
    state = np.empty(mujoco.mj_stateSize(node.model, spec))
    mujoco.mj_getState(node.model, node.data, state, spec)
    np.savez_compressed(path, state=state, state_spec=int(spec),
                        tree_sleep_policy=node.model.tree_sleep_policy.copy(),
                        target_qpos=node.target_qpos.copy(),
                        gripper_target=node.gripper_target, gripper_control=node.gripper_control)
    path.with_suffix('.events.json').write_text(json.dumps(executor.events, indent=2))
    controller = {name: getattr(executor, name) for name in
                  ('speed_scale', 'last_seating_confirmation', '_top_press_active')
                  if hasattr(executor, name)}
    path.with_suffix('.executor.json').write_text(json.dumps(controller, indent=2))
    if exact:
        # Local trusted diagnostics only. Preserve derived observations as well
        # as integration inputs: mj_forward would refresh the controller's
        # observations to a different point in the step sequence.
        with gzip.open(path.with_suffix('.mjdata.pkl.gz'), 'wb') as stream:
            pickle.dump({'mujoco_version': mujoco.__version__, 'data': node.data}, stream,
                        protocol=pickle.HIGHEST_PROTOCOL)


def restore(executor, path, events_path, use_exact=True):
    node = executor.node
    with np.load(path) as saved:
        spec = int(saved['state_spec'])
        assert len(saved['state']) == mujoco.mj_stateSize(node.model, spec)
        node.model.tree_sleep_policy[:] = saved['tree_sleep_policy']
        mujoco.mj_setState(node.model, node.data, saved['state'], spec)
        node.target_qpos = saved['target_qpos'].copy()
        node.gripper_target = float(saved['gripper_target'])
        node.gripper_control = float(saved['gripper_control'])
    executor.events = json.loads(events_path.read_text())
    controller = path.with_suffix('.executor.json')
    if controller.exists():
        for name, value in json.loads(controller.read_text()).items():
            setattr(executor, name, value)
    exact = path.with_suffix('.mjdata.pkl.gz')
    if use_exact and exact.exists():
        # Never load externally supplied pickle files through this diagnostic.
        with gzip.open(exact, 'rb') as stream:
            payload = pickle.load(stream)
        if payload['mujoco_version'] != mujoco.__version__:
            raise ValueError('exact snapshot requires the same MuJoCo version')
        saved_data = payload['data']
        for field in ('qpos', 'qvel', 'ctrl', 'xpos', 'geom_xpos'):
            if getattr(saved_data, field).shape != getattr(node.data, field).shape:
                raise ValueError('exact snapshot dimensions do not match this model')
        mujoco.mj_copyData(node.data, node.model, saved_data)
    else:
        mujoco.mj_forward(node.model, node.data)


def modify(model, variant):
    """Controlled hypotheses, not measured ABS/rubber coefficients."""
    ids = np.arange(model.ngeom)
    active = (model.geom_contype != 0) | (model.geom_conaffinity != 0)
    clutch = ids[active & (model.geom_priority == 2)]
    fingers = {model.body(n).id for n in ('left_finger', 'right_finger')}
    pads = ids[active & np.isin(model.geom_bodyid, list(fingers))]
    changes = {}
    if variant.startswith('clutch_tau_'):
        tau = float(variant.removeprefix('clutch_tau_'))
        model.geom_solref[clutch, 0] = tau
        changes = {'clutch_time_constant_s': tau}
    elif variant.startswith('clutch_mu_'):
        mu = float(variant.removeprefix('clutch_mu_'))
        model.geom_friction[clutch, 0] = mu
        changes = {'clutch_sliding_friction': mu}
    elif variant.startswith('pad_roll_'):
        rolling = float(variant.removeprefix('pad_roll_'))
        model.geom_priority[pads] = 4
        model.geom_condim[pads] = 6
        model.geom_friction[pads, 2] = rolling
        changes = {'pad_rolling_friction_m': rolling, 'pad_condim': 6, 'pad_priority': 4}
    elif variant.startswith('pad_mu_'):
        mu = float(variant.removeprefix('pad_mu_'))
        # Explicitly win over the rigid shell's priority=1. Otherwise reducing
        # a pad coefficient alone has no effect (equal-priority maximum rule).
        model.geom_priority[pads] = 4
        model.geom_friction[pads, 0] = mu
        changes = {'pad_sliding_friction': mu, 'pad_priority': 4}
    elif variant.startswith('shell_mu_'):
        mu = float(variant.removeprefix('shell_mu_'))
        parts = [i for i in range(model.nbody) if model.body(i).name.startswith('bench__')]
        shells = ids[active & np.isin(model.geom_bodyid, parts) & (model.geom_priority == 1)]
        model.geom_friction[shells, 0] = mu
        changes = {'shell_sliding_friction': mu}
    elif variant != 'baseline':
        raise ValueError(variant)
    return changes


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--stage', choices=['pickup', 'release'], required=True)
    parser.add_argument('--variant', default='baseline')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--release-state', type=Path)
    parser.add_argument('--release-gain', type=float, default=2.)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ['LEGO_BENCH_RUN_DIR'] = str(args.output)
    node = OfflineNode()
    executor = ex.OracleExecutor(node)
    if hasattr(executor, 'lower_with_finger_friction'):
        original_lower = executor.lower_with_finger_friction
        executor.lower_with_finger_friction = lambda target: original_lower(target, velocity_gain=args.release_gain)
    target = dict(next(t for t in node.episode_manifest['target_blocks']
                       if t['id'] == '2x2_brick_5'), placement_mode='release_above_press')
    if args.stage == 'pickup':
        snapshot = SOURCE / 'snapshots/2x2_brick_5/before_pick_state.npz'
        events = snapshot.parent / 'before_pick_events.json'
    else:
        snapshot = args.release_state
        events = snapshot.with_suffix('.events.json')
    restore(executor, snapshot, events)
    changes = modify(node.model, args.variant)
    start = time.monotonic()
    rows = []
    original_step = executor.step

    def step(seconds):
        try:
            return original_step(seconds)
        finally:
            body = node.model.body(target['body_name']).id
            if not rows or node.data.time - rows[-1]['time_s'] >= .02:
                rows.append({'time_s': float(node.data.time),
                             'position': node.data.xpos[body].tolist(),
                             'quaternion_wxyz': node.data.xquat[body].tolist(),
                             'max_part_penetration_m': node.max_part_penetration_m,
                             'ncon': node.data.ncon})
    executor.step = step
    error = None
    try:
        if args.stage == 'pickup':
            # Only the eighth iteration of the original production run method.
            tree = ast.parse(textwrap.dedent(inspect.getsource(ex.OracleExecutor.run)))
            function = tree.body[0]
            function.body = function.body[2:]
            for statement in ast.walk(function):
                if isinstance(statement, ast.For) and isinstance(statement.target, ast.Name) and statement.target.id == 'planned':
                    statement.iter = ast.parse('plan[7:8]', mode='eval').body
            ast.fix_missing_locations(tree)
            scope = dict(ex.__dict__)
            exec(compile(tree, '<diagnostic eighth step>', 'exec'), scope)
            original_open = executor.open_gripper_before_retreat
            def capture_open(part, timeout_s=2.):
                path = args.output / 'before_release.npz'
                if not path.exists():
                    save_state(executor, path)
                return original_open(part, timeout_s)
            executor.open_gripper_before_retreat = capture_open
            scope['run'](executor)
        else:
            rot = node.data.xmat[executor.hand].reshape(3, 3)
            yaw = float(np.arctan2(rot[1, 0], rot[0, 0]))
            executor.release_and_press(target, yaw, 'release_above_press')
            executor.open_gripper_before_retreat(target)
            executor.step(.5)
    except (ex.ExecutionFailure, mujoco.FatalError) as exc:
        error = str(exc)
    result = node.benchmark_result()
    result.update(scope='PARTIAL saved-state diagnostic; never complete task acceptance',
                  variant=args.variant, stage=args.stage, changes=changes,
                  error=error, wall_s=time.monotonic()-start,
                  source_scene_sha256=hashlib.sha256((SOURCE/'scene.xml').read_bytes()).hexdigest(),
                  initial_state_sha256=hashlib.sha256(snapshot.read_bytes()).hexdigest(),
                  source_executor_sha256=hashlib.sha256(Path(ex.__file__).read_bytes()).hexdigest(),
                  seating=getattr(executor, 'last_seating_confirmation', None))
    (args.output/'diagnostic.json').write_text(json.dumps(result, indent=2))
    (args.output/'trace.json').write_text(json.dumps(rows))
    print(json.dumps({k:result[k] for k in ('variant','error','placed','total','max_part_penetration_m','wall_s')}, indent=2))


if __name__ == '__main__':
    main()
