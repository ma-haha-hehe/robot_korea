"""Bounded, torque-controlled Oracle baseline; no object teleportation or grasp welds.

This deliberately simple baseline reports failures. It is not a collision-free
planner and is intended for simulation regression, never real robot execution.
"""
from __future__ import annotations
import math
import hashlib
import os
from pathlib import Path
import time
import numpy as np
import mujoco
from .benchmark_core import dump_json, load_registry


def placement_yaw(target, placed_targets):
    """Use square-brick symmetry to keep opening fingers away from neighbours."""
    yaw = target['yaw_rad']
    if target['type'] != 'brick_2x2':
        return yaw
    registry = load_registry()
    neighbours = [b for b in placed_targets
                  if abs(b['position'][2] - target['position'][2]) < .014]
    if not neighbours:
        return yaw

    def clearance(candidate):
        axis = np.array([-math.sin(candidate), math.cos(candidate)])
        result = math.inf
        for neighbour in neighbours:
            angle = neighbour['yaw_rad']
            c, s = math.cos(angle), math.sin(angle)
            rotation = np.array([[c, s], [-s, c]])
            half = np.array(registry[neighbour['type']]['size_m'][:2]) / 2
            for side in (-1, 1):
                # Entire opening sweep, not just the final finger position.
                for opening in np.linspace(.016, .044, 8):
                    finger = np.array(target['position'][:2]) + side * opening * axis
                    local = rotation @ (finger - np.array(neighbour['position'][:2]))
                    result = min(result, float(np.max(np.abs(local) - half)) - .009)
        return result

    return max((yaw, yaw + math.pi/2), key=clearance)


def requires_vertical_approach(target, placed_targets):
    """Avoid sweeping across adjacent parts or a bridge's multiple supports."""
    from .product_geometry import footprint, intersection_area
    registry = load_registry()
    def polygon(block):
        return footprint({'type': block['type'], 'target': {
            'position': block['position'], 'yaw_deg': math.degrees(block['yaw_rad'])}}, registry)
    goal_polygon = polygon(target)
    supports = 0
    for block in placed_targets:
        height = target['position'][2] - block['position'][2]
        distance = np.linalg.norm(np.array(target['position'][:2]) - block['position'][:2])
        if abs(height) < .014 and distance < .10:
            return True
        if abs(height - .0192) < .004 and intersection_area(goal_polygon, polygon(block)) > 1e-6:
            supports += 1
    return supports > 1


class ExecutionFailure(RuntimeError):
    pass


class OracleExecutor:
    def __init__(self, node, perception=None):
        self.node = node
        self.model, self.data = node.model, node.data
        self.ik_data = mujoco.MjData(self.model)
        joints = [self.model.joint(f'panda_joint{i}').id for i in range(1, 8)]
        self.qadr = self.model.jnt_qposadr[joints]
        self.dadr = self.model.jnt_dofadr[joints]
        self.limits = self.model.jnt_range[joints]
        self.hand = self.model.body('hand').id
        self.tool_offset = np.array([0., 0., .1034])
        self.events = []
        self.perception = perception

    def observe_target(self, target):
        if self.perception is None:
            return self.node.current_block_state()['blocks'][target['id']]
        from .perception import capture_frame
        frame = capture_frame(self.node.model, self.node.data, self.node.camera_renderer)
        detections = self.perception.observe(*frame, self.node.episode_manifest['product'])['detections']
        region = self.node.episode_manifest['spawn_region']
        candidates = [d for d in detections if d['type'] == target['type'] and d['color'] == target['color']
                      and region['x'][0] <= d['position'][0] <= region['x'][1]
                      and region['y'][0] <= d['position'][1] <= region['y'][1]]
        if not candidates:
            raise ExecutionFailure(f"vision found no source part for {target['id']}")
        return max(candidates, key=lambda d: d['confidence'])

    def step(self, seconds):
        if time.monotonic() - self.node.episode_start_time >= float(
                self.node.benchmark_config.get('max_episode_time_s', 300)):
            raise ExecutionFailure('episode wall-clock limit exceeded')
        for _ in range(max(1, int(seconds / self.model.opt.timestep))):
            self.node.step_pid()
            if self.node.connection_mode == 'snap':
                self.node.auto_weld_touching_bricks()
                self.node.maintain_fake_welds()
            if not np.isfinite(self.data.qpos).all():
                raise ExecutionFailure('non-finite simulation state')
        self.node.update_safety_metrics()

    def ik(self, position, yaw, seed_q=None):
        d = self.ik_data
        d.qpos[:] = self.data.qpos
        c, s = math.cos(yaw), math.sin(yaw)
        desired = np.array([[c, s, 0], [s, -c, 0], [0, 0, -1]])
        jp, jr = np.zeros((3, self.model.nv)), np.zeros((3, self.model.nv))
        initial = self.data.qpos[self.qadr].copy() if seed_q is None else np.array(seed_q)
        home = np.array([0., -.785, 0., -2.356, 0., 1.571, .785])
        facing = home.copy()
        facing[0] = math.atan2(position[1], position[0])
        for seed in (initial, facing, home):
            d.qpos[self.qadr] = seed
            for _ in range(350):
                mujoco.mj_forward(self.model, d)
                rotation = d.xmat[self.hand].reshape(3, 3)
                tip = d.xpos[self.hand] + rotation @ self.tool_offset
                ep = np.asarray(position) - tip
                er = sum(np.cross(rotation[:, i], desired[:, i]) for i in range(3)) * .5
                if np.linalg.norm(ep) < .001 and np.linalg.norm(er) < .015:
                    return d.qpos[self.qadr].copy()
                mujoco.mj_jac(self.model, d, jp, jr, tip, self.hand)
                jac = np.vstack((jp[:, self.dadr], jr[:, self.dadr]))
                error = np.r_[ep, er]
                delta = jac.T @ np.linalg.solve(jac @ jac.T + .002 * np.eye(6), error)
                d.qpos[self.qadr] = np.clip(d.qpos[self.qadr] + np.clip(delta, -.12, .12),
                                          self.limits[:, 0] + .001, self.limits[:, 1] - .001)
        raise ExecutionFailure(f'IK did not converge at {list(position)}')

    def move(self, position, yaw):
        target = self.ik(position, yaw)
        start = self.data.qpos[self.qadr].copy()
        seconds = max(.6, float(np.max(np.abs(target - start))) / .7)
        count = max(1, int(seconds / .01))
        for i in range(1, count + 1):
            t = i / count
            self.node.target_qpos[self.qadr] = start + (target - start) * (3*t*t - 2*t*t*t)
            self.step(.01)
        self.step(.3)

    def move_vertical(self, position, yaw):
        """Descend through Cartesian waypoints with continuous joint velocity."""
        rotation = self.data.xmat[self.hand].reshape(3, 3)
        start = self.data.xpos[self.hand] + rotation @ self.tool_offset
        initial_yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        angle = (yaw - initial_yaw + math.pi) % (2 * math.pi) - math.pi
        distance = float(np.linalg.norm(np.asarray(position) - start))
        segments = max(1, math.ceil(distance / .01))
        points = [self.data.qpos[self.qadr].copy()]
        for index in range(1, segments + 1):
            fraction = index / segments
            points.append(self.ik(start + fraction * (np.asarray(position) - start),
                                  initial_yaw + fraction * angle, seed_q=points[-1]))
        points = np.array(points)
        slopes = np.zeros_like(points)
        slopes[1:-1] = (points[2:] - points[:-2]) / 2
        travel = float(np.max(np.sum(np.abs(np.diff(points, axis=0)), axis=0)))
        count = max(1, math.ceil(max(.5, distance / .12, travel / .7) / .01))
        for index in range(1, count + 1):
            t = index / count
            along = (3 * t*t - 2 * t*t*t) * segments
            segment = min(int(along), segments - 1)
            u = along - segment
            self.node.target_qpos[self.qadr] = (
                (2*u**3 - 3*u**2 + 1) * points[segment]
                + (u**3 - 2*u**2 + u) * slopes[segment]
                + (-2*u**3 + 3*u**2) * points[segment + 1]
                + (u**3 - u**2) * slopes[segment + 1])
            self.step(.01)
        self.step(.3)

    def run(self):
        self.node.gripper_target = .04
        self.step(.5)
        for target in sorted(self.node.episode_manifest['target_blocks'], key=lambda x: x['position'][2]):
            self.node.get_logger().info(f"[EXECUTE] picking {target['id']}")
            observed = self.observe_target(target)
            pos = np.array(observed['position'])
            yaw = observed['yaw_rad']
            # Close around the collision-box centre, which is 9 mm below body origin.
            grasp = pos + [0, 0, -.009]
            hover = grasp + [0, 0, .16]
            self.move(hover, yaw)
            self.move(grasp, yaw)
            self.node.gripper_target = 0.
            self.step(.7)
            self.move(hover, yaw)
            if self.perception is None:
                lifted = self.node.current_block_state()['blocks'][target['id']]['position'][2]
                if lifted < pos[2] + .06:
                    raise ExecutionFailure(f"grasp failed: {target['id']} (lift {lifted-pos[2]:.4f} m)")
            goal = np.array(target['position']) + [0, 0, -.009]
            placed_ids = {event['block'] for event in self.events}
            placed_targets = [b for b in self.node.episode_manifest['target_blocks']
                              if b['id'] in placed_ids]
            place_yaw = placement_yaw(target, placed_targets)
            self.move(goal + [0, 0, .16], place_yaw)
            # Position tracking can leave a sub-millimetre air gap. Seat the
            # part before opening, otherwise friction with an opening finger
            # can drag the unsupported brick sideways. This commands the arm;
            # the object remains contact-driven and the scored goal is unchanged.
            vertical = requires_vertical_approach(target, placed_targets)
            approach = self.move_vertical if vertical else self.move
            approach(goal - [0, 0, .002], place_yaw)
            self.node.gripper_target = .04
            self.step(.4)
            self.move(goal + [0, 0, .16], place_yaw)
            self.events.append({'block': target['id'], 'status': 'released',
                                'approach': 'vertical' if vertical else 'joint',
                                'placement_yaw_rad': float(place_yaw)})
            run_dir = os.environ.get('LEGO_BENCH_RUN_DIR')
            if run_dir:
                dump_json(Path(run_dir) / 'progress.json', {
                    'finished':False, 'events':self.events,
                    'score_at_checkpoint':self.node.benchmark_result(),
                    'actual_state':self.node.current_block_state()})
            self.node.get_logger().info(f"[EXECUTE] released {target['id']}")
        self.step(1.)


def execute(output_dir, backend='oracle'):
    from pathlib import Path
    import rclpy
    from .mj_bridge3 import MuJoCoActionServer
    # Capture the executed source at episode start, rather than assigning a
    # later working-tree revision to historical results in a report.
    source = Path(__file__).resolve().parent
    fingerprints = {name: hashlib.sha256((source / name).read_bytes()).hexdigest()
                    for name in ('reference_executor.py', 'mj_bridge3.py', 'benchmark_core.py',
                                 'perception.py', 'panda.xml', 'hand.xml', 'part_registry.yaml')}
    rclpy.init()
    node = None
    try:
        node = MuJoCoActionServer()
        perception = None
        if backend != 'oracle':
            from .perception import GroundedPoseBackend
            perception = GroundedPoseBackend()
        executor = OracleExecutor(node, perception)
        error = None
        try:
            executor.run()
        except ExecutionFailure as exc:
            error = str(exc)
        result = node.benchmark_result()
        result.update({'executor': 'torque_baseline', 'perception_backend': backend,
                       'executor_source_sha256': fingerprints,
                       'simulation_time_s': float(node.data.time),
                       'events': executor.events, 'execution_error': error})
        result['success'] = bool(result['success'] and error is None)
        dump_json(Path(output_dir) / 'actual_state.json', node.current_block_state())
        dump_json(Path(output_dir) / 'result.json', result)
        return result
    finally:
        if node is not None:
            if node.camera_renderer is not None:
                node.camera_renderer.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
