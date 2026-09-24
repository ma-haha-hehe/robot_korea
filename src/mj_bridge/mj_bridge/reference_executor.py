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
from .benchmark_core import dump_json, load_registry, score_episode
from .assembly_planner import (plan_assembly, resolve_grasp_yaws, PlanningError,
                               RELEASE_ABOVE_M, RAISED_GRASP_OFFSET_M)


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


class _GripDrift(Exception):
    """Interrupt descent while there is still clearance to restore the grip."""


def free_space_grip_drift(observed, target):
    q = observed['quaternion_wxyz']
    tilt = math.degrees(math.acos(float(np.clip(1 - 2*(q[1]**2 + q[2]**2), -1, 1))))
    return observed['position'][2] > target['position'][2] + .020 and tilt > 1.


def insertion_speed_scale(target, placed_targets, base_height):
    """Use a slower final descent when an upper part has partial support."""
    if target['position'][2] < base_height + .004:
        return .4
    from .product_geometry import footprint, intersection_area
    registry = load_registry()
    def polygon(block):
        return footprint({'type': block['type'], 'target': {
            'position': block['position'], 'yaw_deg': math.degrees(block['yaw_rad'])}}, registry)
    own = polygon(target)
    area = sum(intersection_area(own, polygon(block)) for block in placed_targets
               if abs(target['position'][2] - block['position'][2] - .0192) < .004)
    size = registry[target['type']]['size_m']
    return .2 if area / (size[0]*size[1]) < .75 else .4


class OracleExecutor:
    def __init__(self, node, perception=None, speed_scale=1.5):
        if not math.isfinite(speed_scale) or not .25 <= speed_scale <= 2.:
            raise ValueError('speed_scale must be between 0.25 and 2')
        self.speed_scale = speed_scale
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

    def keep_part_awake(self, target):
        """Retain contact solving for this part until verified release and retreat."""
        body = self.model.body(target['body_name']).id
        tree = int(self.model.body_treeid[body])
        if tree < 0:
            raise ExecutionFailure('assembly part must belong to a dynamic tree')
        self.model.tree_sleep_policy[tree] = mujoco.mjtSleepPolicy.mjSLEEP_NEVER
        # A policy change takes effect through an ordinary physics step. Do not
        # move the part or add a fictitious force to wake it.
        self.step(.01)
        if not self.data.body_awake[body]:
            raise ExecutionFailure('part remained asleep; contact evidence unavailable')

    def wake_supports_before_approach(self):
        """Restore support contacts before the held part enters the assembly area."""
        released = {event['block'] for event in self.events}
        bodies = [self.model.body(block['body_name']).id
                  for block in self.node.episode_manifest['target_blocks']
                  if block['id'] in released]
        for body in bodies:
            tree = int(self.model.body_treeid[body])
            self.model.tree_sleep_policy[tree] = mujoco.mjtSleepPolicy.mjSLEEP_NEVER
        self.step(.01)
        if not all(self.part_contacts_observable(body) for body in bodies):
            raise ExecutionFailure('support contacts unavailable before assembly approach')

    def allow_resting_part_sleep(self, target):
        """Restore ordinary resting sleep only after completed release and retreat."""
        event = self.events[-1] if self.events else {}
        confirmation = event.get('release_confirmation', {})
        if (event.get('block') != target['id'] or event.get('status') != 'released'
                or confirmation.get('target_contacts_observable') is not True
                or confirmation.get('target_contact') is not False):
            raise ExecutionFailure('unverified release cannot enter resting sleep')
        self.verify_released_parts()
        released = {event['block'] for event in self.events}
        for block in self.node.episode_manifest['target_blocks']:
            if block['id'] in released:
                body = self.model.body(block['body_name']).id
                tree = int(self.model.body_treeid[body])
                self.model.tree_sleep_policy[tree] = mujoco.mjtSleepPolicy.mjSLEEP_ALLOWED

    def settle_awake_parts(self, targets):
        """Check the completed assembly with every part actively integrated."""
        for target in targets:
            self.keep_part_awake(target)
        self.step(1.)
        awake = {target['id']: self.part_contacts_observable(
            self.model.body(target['body_name']).id) for target in targets}
        if not all(awake.values()):
            raise ExecutionFailure('final contact observation unavailable')
        self.final_contact_observation = {'awake_parts': awake, 'settle_simulation_s': 1.}
        self.verify_released_parts()

    def observe_target(self, target):
        if self.perception is None:
            if self.node.connection_mode == 'physics':
                self.keep_part_awake(target)
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
            if self.node.connection_mode == 'physics' and max(
                    self.node.max_part_penetration_m,
                    self.node.max_robot_environment_penetration_m) > .0005:
                raise ExecutionFailure('physical contact penetration limit exceeded; motion stopped')
            if self.node.connection_mode == 'snap':
                self.node.auto_weld_touching_bricks()
                self.node.maintain_fake_welds()
            if not np.isfinite(self.data.qpos).all():
                raise ExecutionFailure('non-finite simulation state')
            target = getattr(self, '_descent_guard_target', None)
            if target is not None:
                body = self.model.body(target['body_name']).id
                observed = {'position': self.data.xpos[body],
                            'quaternion_wxyz': self.data.xquat[body]}
                if free_space_grip_drift(observed, target):
                    self.node.target_qpos[self.qadr] = self.data.qpos[self.qadr].copy()
                    raise _GripDrift()
        self.node.update_safety_metrics()

    def ik(self, position, yaw, seed_q=None, target_rotation=None):
        d = self.ik_data
        d.qpos[:] = self.data.qpos
        c, s = math.cos(yaw), math.sin(yaw)
        desired = np.array([[c, s, 0], [s, -c, 0], [0, 0, -1]])
        if target_rotation is not None:
            desired = np.asarray(target_rotation)
        jp, jr = np.zeros((3, self.model.nv)), np.zeros((3, self.model.nv))
        initial = self.data.qpos[self.qadr].copy() if seed_q is None else np.array(seed_q)
        home = np.array([0., -.785, 0., -2.356, 0., 1.571, .785])
        facing = home.copy()
        facing[0] = math.atan2(position[1], position[0])
        for seed in (initial, facing, home):
            d.qpos[self.qadr] = seed
            for _ in range(350):
                # IK needs poses and joint axes, not contact detection or the
                # constraint solver for every hypothetical arm configuration.
                mujoco.mj_kinematics(self.model, d)
                mujoco.mj_comPos(self.model, d)
                rotation = d.xmat[self.hand].reshape(3, 3)
                tip = d.xpos[self.hand] + rotation @ self.tool_offset
                ep = np.asarray(position) - tip
                er = sum(np.cross(rotation[:, i], desired[:, i]) for i in range(3)) * .5
                if np.linalg.norm(ep) < .00015 and np.linalg.norm(er) < .003:
                    return d.qpos[self.qadr].copy()
                mujoco.mj_jac(self.model, d, jp, jr, tip, self.hand)
                jac = np.vstack((jp[:, self.dadr], jr[:, self.dadr]))
                error = np.r_[ep, er]
                delta = jac.T @ np.linalg.solve(jac @ jac.T + .002 * np.eye(6), error)
                d.qpos[self.qadr] = np.clip(d.qpos[self.qadr] + np.clip(delta, -.12, .12),
                                          self.limits[:, 0] + .001, self.limits[:, 1] - .001)
        raise ExecutionFailure(f'IK did not converge at {list(position)}')

    def prepare_grasp_opening(self, part_type, part_yaw, gripper_yaw):
        """Set a measured, part-sized opening above the loose-parts region."""
        hx, hy = np.asarray(load_registry()[part_type]['size_m'][:2]) / 2
        angle = part_yaw - gripper_yaw
        opening = min(.04, abs(math.sin(angle))*hx + abs(math.cos(angle))*hy + .004)
        qadr = [self.model.joint(f'panda_finger_joint{i}').qposadr[0] for i in (1,2)]
        dadr = [self.model.joint(f'panda_finger_joint{i}').dofadr[0] for i in (1,2)]
        self.node.gripper_target = opening
        for _ in range(150):
            self.step(.01)
            if (np.all(np.abs(self.data.qpos[qadr] - opening) < .0005)
                    and np.all(np.abs(self.data.qvel[dadr]) < .002)):
                return
        raise ExecutionFailure('grasp opening did not reach its measured setpoint')

    def confirm_grasp_pose(self, position, yaw, timeout_s=1.):
        """Do not close or lift while the measured tool misses the grasp pose."""
        c, s = math.cos(yaw), math.sin(yaw)
        desired = np.array([[c, s, 0], [s, -c, 0], [0, 0, -1]])
        elapsed = 0.
        while elapsed <= timeout_s:
            rotation = self.data.xmat[self.hand].reshape(3, 3)
            tip = self.data.xpos[self.hand] + rotation @ self.tool_offset
            distance = float(np.linalg.norm(np.asarray(position) - tip))
            angle = math.acos(float(np.clip((np.trace(desired.T @ rotation)-1)/2, -1, 1)))
            if distance <= .001 and angle <= math.radians(2):
                return
            self.step(.02)
            elapsed += .02
        raise ExecutionFailure(f'grasp pose tracking failed: {distance*1000:.2f} mm, {math.degrees(angle):.2f} deg; closure cancelled')

    def move(self, position, yaw):
        target = self.ik(position, yaw)
        start = self.data.qpos[self.qadr].copy()
        seconds = max(.6, float(np.max(np.abs(target - start))) / .7) / self.speed_scale
        count = max(1, int(seconds / .01))
        for i in range(1, count + 1):
            t = i / count
            self.node.target_qpos[self.qadr] = start + (target - start) * (3*t*t - 2*t*t*t)
            self.step(.01)
        self.step(.1)

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
        count = max(1, math.ceil(max(.5, distance / .12, travel / .7) / (.01 * self.speed_scale)))
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
        self.step(.1)

    def align_held_part(self, target):
        """Correct the arm pose from the measured part pose, without moving the part directly."""
        if self.perception is not None:
            raise ExecutionFailure('physics insertion requires validated in-hand pose tracking')
        period = math.radians(load_registry()[target['type']]['yaw_symmetry_deg'])
        for _ in range(6):
            observed = self.node.current_block_state()['blocks'][target['id']]
            position = np.asarray(observed['position'])
            delta = np.asarray(target['position'])[:2] - position[:2]
            angle = (target['yaw_rad'] - observed['yaw_rad'] + period/2) % period - period/2
            rotation = self.data.xmat[self.hand].reshape(3, 3)
            tip = self.data.xpos[self.hand] + rotation @ self.tool_offset
            yaw = math.atan2(rotation[1, 0], rotation[0, 0])
            if np.linalg.norm(delta) < .0002 and abs(angle) < .008:
                return np.asarray(target['position']) + tip - position, yaw
            self.move(tip + np.r_[delta, 0.], yaw + angle)
        raise ExecutionFailure(f"held-part alignment did not converge: {target['id']}")

    def place_physical(self, target):
        if target.get("placement_mode") == "release_above_press":
            return self.place_with_release_above(target)

        endpoint, yaw = self.align_held_part(target)
        self.move_vertical(endpoint + [0, 0, .04], yaw)
        endpoint, yaw = self.align_held_part(target)
        travel_speed = self.speed_scale
        recovered = False
        try:
            # Fast travel, slow final insertion into the real underside cavity.
            targets = self.node.episode_manifest['target_blocks']
            placed = {event['block'] for event in self.events}
            self.speed_scale = insertion_speed_scale(
                target, [b for b in targets if b['id'] in placed],
                min(b['position'][2] for b in targets))
            if (self.perception is None
                    and self.node.episode_manifest.get('contact_profile') == 'plastic'
                    and target['type'] == 'brick_4x2'
                    and abs(math.sin(target['yaw_rad'] - yaw)) > .7):
                self._descent_guard_target = target
            try:
                if self.node.episode_manifest.get('contact_profile') == 'plastic':
                    # Unload the grasp before the final interference fit: jaw
                    # compression can tilt a part as the studs enter its cavity.
                    self.move_vertical(endpoint + [0, 0, .001], yaw)
                    self._descent_guard_target = None
                    self.release_and_press(target, yaw, 'release_at_insertion_entry')
                    recovered = True
                else:
                    self.move_vertical(endpoint, yaw)
            except _GripDrift:
                self._descent_guard_target = None
                self.descend_with_pose_feedback(target, yaw)
                recovered = True
            finally:
                self._descent_guard_target = None
        finally:
            self.speed_scale = travel_speed
        if self.node.episode_manifest.get('contact_profile') == 'plastic':
            if not recovered:
                self.seat_plastic_part(target, yaw)
            if self.perception is None:
                self.refine_overhang_pose(target, yaw)
        return yaw

    def descend_with_pose_feedback(self, target, yaw):
        """Correct a wide-side grasp while lowering, before it touches neighbours."""
        if self.perception is not None:
            raise ExecutionFailure('insertion pose feedback requires validated in-hand tracking')
        c, s = math.cos(target['yaw_rad']), math.sin(target['yaw_rad'])
        desired = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        goal = np.asarray(target['position'])
        maximum_tilt = 0.
        for attempt in range(80):
            observed = self.node.current_block_state()['blocks'][target['id']]
            position = np.asarray(observed['position'])
            q = np.asarray(observed['quaternion_wxyz'])
            tilt = math.acos(float(np.clip(1 - 2*(q[1]**2 + q[2]**2), -1, 1)))
            maximum_tilt = max(maximum_tilt, tilt)
            if tilt > math.radians(3):
                if tilt > math.radians(8) or position[2] - goal[2] <= .020:
                    raise ExecutionFailure('feedback insertion exceeded 3 degree tilt near placement')
                part = self.model.body(target['body_name']).id
                fingers = {self.model.body(n).id for n in ('left_finger', 'right_finger')}
                held = set()
                for contact_index, contact in enumerate(self.data.contact):
                    bodies = [int(self.model.geom_bodyid[g]) for g in (contact.geom1, contact.geom2)]
                    if part not in bodies or contact.dist > .0001:
                        continue
                    other = bodies[1] if bodies[0] == part else bodies[0]
                    if other not in fingers:
                        raise ExecutionFailure('airborne recovery has an external contact')
                    force = np.zeros(6)
                    mujoco.mj_contactForce(self.model, self.data, contact_index, force)
                    if force[0] > .01:
                        held.add(other)
                if held != fingers:
                    raise ExecutionFailure('airborne recovery lost the two-finger grasp')
            if position[2] - goal[2] < .0015:
                self.release_and_press(target, yaw, 'release_before_seating')
                self.last_seating_confirmation['pose_feedback'] = {
                    'steps': attempt, 'max_observed_tilt_deg': math.degrees(maximum_tilt),
                    'max_step_m': .002}
                return
            part_rotation = np.empty(9)
            mujoco.mju_quat2Mat(part_rotation, q)
            correction = desired @ part_rotation.reshape(3, 3).T
            rotation = self.data.xmat[self.hand].reshape(3, 3).copy()
            tip = self.data.xpos[self.hand] + rotation @ self.tool_offset
            stage = goal.copy()
            # Correct measured tilt and lateral error before further descent.
            stage[2] = (max(goal[2] + .001, position[2] - .002)
                        if tilt < math.radians(.3) and np.linalg.norm(position[:2]-goal[:2]) < .0001
                        else position[2])
            start = self.data.qpos[self.qadr].copy()
            end = self.ik(stage + correction @ (tip-position), yaw,
                          target_rotation=correction @ rotation)
            for index in range(1, 11):
                t = index / 10
                self.node.target_qpos[self.qadr] = start + (end-start)*(3*t*t-2*t*t*t)
                self.step(.01)
            self.step(.04)
        raise ExecutionFailure('feedback insertion did not seat within 80 bounded steps')

    def refine_overhang_pose(self, target, yaw):
        """Remove residual insertion tilt before unloading a partial support."""
        if self.perception is not None:
            raise ExecutionFailure('support alignment requires validated in-hand tracking')
        targets = self.node.episode_manifest['target_blocks']
        placed = {event['block'] for event in self.events}
        if insertion_speed_scale(target, [b for b in targets if b['id'] in placed],
                                 min(b['position'][2] for b in targets)) != .2:
            return
        seating = dict(getattr(self, 'last_seating_confirmation', None) or {})
        already_released = 'initial_release_confirmation' in seating
        goal = np.asarray(target['position'])
        c, s = math.cos(target['yaw_rad']), math.sin(target['yaw_rad'])
        desired = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        for attempt in range(20):
            observed = self.node.current_block_state()['blocks'][target['id']]
            position = np.asarray(observed['position'])
            q = np.asarray(observed['quaternion_wxyz'])
            tilt = math.acos(float(np.clip(1 - 2*(q[1]**2 + q[2]**2), -1, 1)))
            lateral = float(np.linalg.norm(position[:2] - goal[:2]))
            if tilt > math.radians(3) or abs(position[2] - goal[2]) > .004:
                raise ExecutionFailure('supported pose alignment exceeded bounds')
            if tilt < math.radians(.2) and lateral < .0001:
                self.seat_plastic_part(target, yaw)
                seating.update(self.last_seating_confirmation)
                self.last_seating_confirmation = seating
                self.last_seating_confirmation['support_alignment_steps'] = attempt
                return
            if already_released:
                raise ExecutionFailure('released overhang requires realignment before further assembly')
            part_rotation = np.empty(9)
            mujoco.mju_quat2Mat(part_rotation, q)
            rotation = self.data.xmat[self.hand].reshape(3, 3).copy()
            tip = self.data.xpos[self.hand] + rotation @ self.tool_offset
            correction = desired @ part_rotation.reshape(3, 3).T
            stage = goal.copy()
            stage[2] = max(position[2], goal[2])
            start = self.data.qpos[self.qadr].copy()
            end = self.ik(stage + correction @ (tip-position), yaw,
                          target_rotation=correction @ rotation)
            for index in range(1, 11):
                t = index / 10
                self.node.target_qpos[self.qadr] = start + (end-start)*(3*t*t-2*t*t*t)
                self.step(.01)
            self.step(.04)
        raise ExecutionFailure('supported pose alignment did not converge')

    def place_with_release_above(self, target):
        """Place a blocked part by releasing above it, then pressing its top."""
        endpoint, yaw = self.align_held_part(target)
        self.move_vertical(endpoint + [0, 0, .06], yaw)
        endpoint, yaw = self.align_held_part(target)
        speed = self.speed_scale
        try:
            self.speed_scale = .2
            self.move_vertical(endpoint + [0, 0, RELEASE_ABOVE_M], yaw)
            self.release_and_press(target, yaw, 'release_above_press')
        finally:
            self.speed_scale = speed
        return yaw

    def release_and_press(self, target, yaw, placement_mode):
        """Unload the jaws before applying pressure through closed fingertips."""
        if (self.perception is not None or self.node.connection_mode != 'physics'
                or self.node.episode_manifest.get('contact_profile') != 'plastic'):
            raise ExecutionFailure('top pressing requires Oracle plastic contact physics')
        if getattr(self, '_top_press_active', False):
            raise ExecutionFailure('top pressing did not seat the part')
        self._top_press_active = True
        speed = self.speed_scale
        try:
            release = self.open_gripper_before_retreat(target)
            rotation = self.data.xmat[self.hand].reshape(3, 3)
            tip = self.data.xpos[self.hand] + rotation @ self.tool_offset
            self.move_vertical(tip + [0, 0, .06], yaw)
            self.node.gripper_target = 0.
            self.step(.7)
            qadr = [self.model.joint(f'panda_finger_joint{i}').qposadr[0]
                    for i in (1, 2)]
            if max(self.data.qpos[qadr]) > .0005:
                raise ExecutionFailure('press fingers did not close above the assembly')
            fingers = {self.model.body(n).id for n in ('left_finger', 'right_finger')}
            bottom = math.inf
            for geom in range(self.model.ngeom):
                if (self.model.geom_bodyid[geom] not in fingers
                        or not (self.model.geom_contype[geom]
                                or self.model.geom_conaffinity[geom])):
                    continue
                rotation = self.data.geom_xmat[geom].reshape(3, 3)
                center = (self.data.geom_xpos[geom]
                          + rotation @ self.model.geom_aabb[geom, :3])
                bottom = min(bottom, center[2]
                             - np.abs(rotation[2]) @ self.model.geom_aabb[geom, 3:])
            if not math.isfinite(bottom):
                raise ExecutionFailure('no collision fingertip available for top pressing')
            rotation = self.data.xmat[self.hand].reshape(3, 3)
            tip = self.data.xpos[self.hand] + rotation @ self.tool_offset
            offset = float(tip[2] - bottom)
            observed = self.node.current_block_state()['blocks'][target['id']]
            if np.linalg.norm(np.asarray(observed['position'])[:2]
                              - target['position'][:2]) > .001:
                raise ExecutionFailure('released part drifted before pressing')
            part = self.model.body(target['body_name']).id
            part_top = -math.inf
            for geom in range(self.model.ngeom):
                if (self.model.geom_bodyid[geom] != part
                        or not (self.model.geom_contype[geom]
                                or self.model.geom_conaffinity[geom])):
                    continue
                rotation = self.data.geom_xmat[geom].reshape(3, 3)
                center = (self.data.geom_xpos[geom]
                          + rotation @ self.model.geom_aabb[geom, :3])
                part_top = max(part_top, center[2]
                               + np.abs(rotation[2]) @ self.model.geom_aabb[geom, 3:])
            if not math.isfinite(part_top):
                raise ExecutionFailure('no physical part surface for top pressing')
            q = observed['quaternion_wxyz']
            if 1 - 2*(q[1]**2 + q[2]**2) < math.cos(math.radians(3)):
                raise ExecutionFailure('released part tilted before top pressing')
            # Approach the measured surface first. Moving straight to nominal
            # seated height can strike a still raised part before bounded pressing.
            press_start = np.asarray(target['position']).copy()
            press_start[2] = part_top + offset + .0001
            self.move_vertical(press_start, yaw)
            self._seat_plastic_part(target, yaw)
            self.last_seating_confirmation.update(
                placement_mode=placement_mode, initial_release_confirmation=release)
        finally:
            self._top_press_active = False
            self.speed_scale = speed

    def seat_plastic_part(self, target, yaw):
        """Retry a captured, aligned fit once with unloaded jaws if it jams."""
        try:
            self._seat_plastic_part(target, yaw)
        except ExecutionFailure as error:
            if (str(error) != 'part not seated within 4 mm bounded insertion travel'
                    or self.perception is not None
                    or getattr(self, '_top_press_active', False)):
                raise
            observed = self.node.current_block_state()['blocks'][target['id']]
            delta = np.asarray(observed['position']) - target['position']
            q = observed['quaternion_wxyz']
            if not (0 < delta[2] < .004 and np.linalg.norm(delta[:2]) < .001
                    and 1 - 2*(q[1]**2 + q[2]**2) > math.cos(math.radians(3))):
                raise
            self.release_and_press(target, yaw, 'release_after_stalled_insertion')

    def _rigid_bottom_support_force(self, target):
        """Measured upward support at the rigid underside, excluding the jaws."""
        part = self.model.body(target['body_name']).id
        fingers = {self.model.body(n).id for n in ('left_finger', 'right_finger')}
        rigid = set()
        bottom = math.inf
        for geom in range(self.model.ngeom):
            if (self.model.geom_bodyid[geom] != part or self.model.geom_priority[geom] == 2
                    or not (self.model.geom_contype[geom] or self.model.geom_conaffinity[geom])):
                continue
            rigid.add(geom)
            rotation = self.data.geom_xmat[geom].reshape(3, 3)
            center = self.data.geom_xpos[geom] + rotation @ self.model.geom_aabb[geom, :3]
            bottom = min(bottom, center[2] - np.abs(rotation[2]) @ self.model.geom_aabb[geom, 3:])
        support_force = 0.
        for i, contact in enumerate(self.data.contact):
            own = contact.geom1 if contact.geom1 in rigid else contact.geom2 if contact.geom2 in rigid else None
            if own is None:
                continue
            other = contact.geom2 if own == contact.geom1 else contact.geom1
            if (self.model.geom_bodyid[other] in fingers or contact.dist > .00002
                    or abs(contact.frame[2]) < .9 or abs(contact.pos[2] - bottom) > .0002):
                continue
            force = np.zeros(6)
            mujoco.mj_contactForce(self.model, self.data, i, force)
            support_force += max(0., force[0])
        return support_force

    def seating_support_evidence(self, target):
        """Measure rigid bearing or clutch load at a nearly closed underside gap."""
        support_force = self._rigid_bottom_support_force(target)
        if support_force > .01:
            return {'bottom_support_force_n': float(support_force)}
        part = self.model.body(target['body_name']).id
        rigid = set()
        bottom = math.inf
        for geom in range(self.model.ngeom):
            if (self.model.geom_bodyid[geom] != part or self.model.geom_priority[geom] == 2
                    or not (self.model.geom_contype[geom] or self.model.geom_conaffinity[geom])):
                continue
            rigid.add(geom)
            rotation = self.data.geom_xmat[geom].reshape(3, 3)
            center = self.data.geom_xpos[geom] + rotation @ self.model.geom_aabb[geom, :3]
            bottom = min(bottom, center[2] - np.abs(rotation[2]) @ self.model.geom_aabb[geom, 3:])
        supports = {0} | {self.model.body(b['body_name']).id for b in self.node.episode_manifest['target_blocks'] if b['id'] != target['id']}
        supports.update(i for i in range(self.model.nbody) if self.model.body(i).name.startswith('assembly_base'))
        upward = 0.
        for i, contact in enumerate(self.data.contact):
            bodies = [int(self.model.geom_bodyid[g]) for g in (contact.geom1, contact.geom2)]
            if part not in bodies:
                continue
            other = bodies[1] if bodies[0] == part else bodies[0]
            if other not in supports:
                continue
            force = np.zeros(6)
            mujoco.mj_contactForce(self.model, self.data, i, force)
            world = contact.frame.reshape(3,3).T @ force[:3]
            upward += float(world[2]) * (1 if bodies[1] == part else -1)
        if upward < .8 * self.model.body_mass[part] * abs(self.model.opt.gravity[2]):
            return None
        support_geoms = []
        for other in range(self.model.ngeom):
            if (self.model.geom_bodyid[other] not in supports or self.model.geom_priority[other] == 2
                    or not (self.model.geom_contype[other] or self.model.geom_conaffinity[other])):
                continue
            rotation = self.data.geom_xmat[other].reshape(3,3)
            center = self.data.geom_xpos[other] + rotation @ self.model.geom_aabb[other,:3]
            top = center[2] + np.abs(rotation[2]) @ self.model.geom_aabb[other,3:]
            if abs(top - bottom) < .0002:
                support_geoms.append(other)
        for own in rigid:
            for other in support_geoms:
                points = np.zeros(6)
                gap = mujoco.mj_geomDistance(self.model, self.data, own, other, .00002, points)
                if (0 < gap < .00002 and abs(points[2] - bottom) < .0002
                        and points[2] - points[5] > .9 * gap):
                    return {'bottom_support_force_n': float(support_force),
                            'clutch_seating': dict(bottom_gap_m=float(gap),
                                                   upward_contact_force_n=upward)}
        return None

    def _requires_precise_support_alignment(self, target):
        targets = self.node.episode_manifest['target_blocks']
        placed = {event['block'] for event in self.events}
        return insertion_speed_scale(
            target, [block for block in targets if block['id'] in placed],
            min(block['position'][2] for block in targets)) == .2

    def _seat_plastic_part(self, target, yaw):
        """Bounded insertion driven by measured part height, never object motion."""
        pressing = getattr(self, '_top_press_active', False)
        partial_support = pressing and self._requires_precise_support_alignment(target)
        travelled = 0.
        insertion_origin = None
        for attempt in range(17):
            observed = self.node.current_block_state()['blocks'][target['id']]
            delta = np.asarray(observed['position']) - np.asarray(target['position'])
            q = observed['quaternion_wxyz']
            upright = 1 - 2 * (q[1]**2 + q[2]**2)
            if delta[2] < -.0004 or upright < math.cos(math.radians(3)):
                raise ExecutionFailure('plastic insertion left height/orientation bounds')
            support = self.seating_support_evidence(target) if pressing else None
            if (abs(delta[2]) < .0004
                    and (not pressing or (support is not None
                         and (not partial_support or (
                             upright > math.cos(math.radians(.2))
                             and np.linalg.norm(delta[:2]) < .0001))))):
                if np.linalg.norm(delta[:2]) > .001:
                    raise ExecutionFailure('plastic insertion lateral alignment exceeded 1 mm')
                self.last_seating_confirmation = {
                    'height_error_m': float(delta[2]),
                    'lateral_error_m': float(np.linalg.norm(delta[:2])),
                    'additional_travel_m': travelled}
                if pressing:
                    self.last_seating_confirmation.update(support)
                return
            if attempt == 16:
                break
            rotation = self.data.xmat[self.hand].reshape(3, 3)
            tip = self.data.xpos[self.hand] + rotation @ self.tool_offset
            start = self.data.qpos[self.qadr].copy()
            if insertion_origin is None:
                insertion_origin = tip.copy()
            # Accumulate commanded travel from a fixed origin. Resetting each
            # correction from the measured tip stalls at a constant load error.
            end = self.ik(insertion_origin + [0, 0, -.00025 * (attempt + 1)], yaw)
            # A 0.25 mm correction does not need the travel trajectory's
            # minimum duration. Smoothly command the arm, then let it settle.
            for index in range(1, 11):
                t = index / 10
                self.node.target_qpos[self.qadr] = start + (end-start)*(3*t*t-2*t*t*t)
                self.step(.01)
            self.step(.1)
            travelled += .00025
        raise ExecutionFailure('part not seated within 4 mm bounded insertion travel')

    def part_contacts_observable(self, body):
        if not self.model.opt.enableflags & int(mujoco.mjtEnableBit.mjENBL_SLEEP):
            return True
        tree = int(self.model.body_treeid[body])
        return bool(tree >= 0 and self.data.body_awake[body]
                    and self.model.tree_sleep_policy[tree] == mujoco.mjtSleepPolicy.mjSLEEP_NEVER)

    def open_gripper_before_retreat(self, target, timeout_s=2.):
        """Hold the arm until both measured fingers are fully open and clear."""
        joints = [self.model.joint(f'panda_finger_joint{i}').id for i in (1, 2)]
        qadr = self.model.jnt_qposadr[joints]
        dadr = self.model.jnt_dofadr[joints]
        fingers = {self.model.body(name).id for name in ('left_finger', 'right_finger')}
        part = self.model.body(target['body_name']).id
        if (not getattr(self, '_top_press_active', False)
                and (getattr(self, 'last_seating_confirmation', None) or {}).get(
                    'initial_release_confirmation')):
            # Remove commanded press overtravel while holding the measured arm
            # posture. Retreat still waits for fully open, contact-free fingers.
            self.node.target_qpos[self.qadr] = self.data.qpos[self.qadr].copy()
        self.node.gripper_target = .04
        stable_time = 0.
        elapsed = 0.
        while elapsed < timeout_s:
            self.step(.01)
            elapsed += .01
            positions = self.data.qpos[qadr]
            velocities = self.data.qvel[dadr]
            touching = False
            for contact in self.data.contact:
                a, b = self.model.geom_bodyid[[contact.geom1, contact.geom2]]
                if contact.dist <= .0001 and ((a in fingers and b == part) or (b in fingers and a == part)):
                    touching = True
                    break
            observable = self.part_contacts_observable(part)
            ready = (observable and np.all(positions >= .0395) and np.all(np.abs(velocities) < .002)
                     and not touching)
            stable_time = stable_time + .01 if ready else 0.
            if stable_time >= .08:
                return {'finger_positions_m': positions.tolist(),
                        'wait_simulation_s': elapsed, 'target_contact': False,
                        'target_contacts_observable': observable}
        raise ExecutionFailure(f"gripper did not fully release {target['id']}; retreat cancelled")

    def verify_released_parts(self):
        """Stop before building on displaced or tilted parts (Oracle only)."""
        if self.perception is not None:
            return
        released = {event['block'] for event in self.events}
        targets = [b for b in self.node.episode_manifest['target_blocks'] if b['id'] in released]
        if not targets:
            return
        manifest = dict(self.node.episode_manifest, target_blocks=targets)
        report = score_episode(manifest, self.node.current_block_state())
        if not report['success']:
            failed = [row['id'] for row in report['blocks'] if not row['success']]
            raise ExecutionFailure('released parts moved or tilted; assembly stopped: ' + ', '.join(failed))

    def run(self):
            self.node.gripper_target = .04
            self.step(.5)
            targets = self.node.episode_manifest['target_blocks']
            try:
                if (self.perception is None and self.node.connection_mode == 'physics'
                        and self.node.episode_manifest.get('contact_profile') == 'plastic'):
                    from .assembly_planner import plan_with_release_above
                    plan = plan_with_release_above(targets)
                else:
                    plan = plan_assembly(targets)
            except PlanningError as exc:
                raise ExecutionFailure(f'assembly planning failed: {exc}') from exc
            run_dir = os.environ.get('LEGO_BENCH_RUN_DIR')
            if run_dir:
                dump_json(Path(run_dir) / 'assembly_plan.json', {
                    'method': ('geometric_reverse_order_with_release_above_fallback'
                               if any(p.get('placement_mode') == 'release_above_press' for p in plan)
                               else 'assembly_by_disassembly'), 'steps': plan})
            by_id = {target['id']: target for target in targets}
            for planned in plan:
                self.verify_released_parts()
                target = dict(by_id[planned['block_id']],
                              placement_mode=planned.get('placement_mode', 'direct'))
                self.last_seating_confirmation = None
                self.node.get_logger().info(f"[EXECUTE] picking {target['id']}")
                observed = self.observe_target(target)
                pos = np.array(observed['position'])
                yaw, place_yaw = resolve_grasp_yaws(
                    observed['yaw_rad'], target['yaw_rad'], planned['grasp_spin_deg'])
                # Grip the upper sidewall, leaving the fingertips clear of supporting studs.
                grasp = pos + [0, 0, RAISED_GRASP_OFFSET_M
                                   if target['placement_mode'] == 'release_above_press' else -.003]
                hover = grasp + [0, 0, .16]
                self.move(hover, yaw)
                self.prepare_grasp_opening(target['type'], observed['yaw_rad'], yaw)
                # A joint-space shortcut can sweep sideways through a loose part.
                # Preserve the measured XY while entering and leaving the grasp.
                self.move_vertical(grasp, yaw)
                self.confirm_grasp_pose(grasp, yaw)
                self.node.gripper_target = 0.
                self.step(.7)
                self.move_vertical(hover, yaw)
                if self.perception is None:
                    lifted = self.node.current_block_state()['blocks'][target['id']]['position'][2]
                    if lifted < pos[2] + .06:
                        raise ExecutionFailure(f"grasp failed: {target['id']} (lift {lifted-pos[2]:.4f} m)")
                goal = np.array(target['position']) + [0, 0, -.003]
                placed_ids = {event['block'] for event in self.events}
                placed_targets = [b for b in self.node.episode_manifest['target_blocks']
                                  if b['id'] in placed_ids]
                if self.perception is None and self.node.connection_mode == 'physics':
                    self.wake_supports_before_approach()
                self.move(goal + [0, 0, .16], place_yaw)
                # Release at the nominal height; pressing down traps the fingertips
                # against supporting bricks. Gravity seats the released part.
                vertical = requires_vertical_approach(target, placed_targets)
                approach = self.move_vertical if vertical else self.move
                if self.node.connection_mode == 'physics':
                    place_yaw = self.place_physical(target)
                    vertical = True
                else:
                    approach(goal, place_yaw)
                release = self.open_gripper_before_retreat(target)
                self.move(goal + [0, 0, .16], place_yaw)
                self.events.append({'block': target['id'], 'status': 'released',
                                    'approach': 'vertical' if vertical else 'joint',
                                    'placement_yaw_rad': float(place_yaw),
                                    'planned_grasp_spin_deg': planned['grasp_spin_deg'],
                                    'release_confirmation': release,
                                    'seating_confirmation': getattr(self, 'last_seating_confirmation', None)})
                if self.perception is None and self.node.connection_mode == 'physics':
                    self.allow_resting_part_sleep(target)
                run_dir = os.environ.get('LEGO_BENCH_RUN_DIR')
                if run_dir:
                    dump_json(Path(run_dir) / 'progress.json', {
                        'finished':False, 'events':self.events,
                        'score_at_checkpoint':self.node.benchmark_result(),
                        'actual_state':self.node.current_block_state()})
                self.node.get_logger().info(f"[EXECUTE] released {target['id']}")
            if self.perception is None and self.node.connection_mode == 'physics':
                self.settle_awake_parts(targets)
            else:
                self.step(1.)
                self.verify_released_parts()


def execute(output_dir, backend='oracle', speed_scale=1.5):
    from pathlib import Path
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from .mj_bridge3 import MuJoCoActionServer
    # Capture the executed source at episode start, rather than assigning a
    # later working-tree revision to historical results in a report.
    source = Path(__file__).resolve().parent
    fingerprints = {name: hashlib.sha256((source / name).read_bytes()).hexdigest()
                    for name in ('reference_executor.py', 'mj_bridge3.py', 'benchmark_core.py',
                                 'perception.py', 'panda.xml', 'hand.xml', 'part_registry.yaml', 'scene_builder.py',
                                 'assembly_planner.py', 'scene_template.xml', 'recorded_layout.py', 'plastic_contact.py')}
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = MuJoCoActionServer()
        perception = None
        if backend != 'oracle':
            from .perception import GroundedPoseBackend
            perception = GroundedPoseBackend()
        executor = OracleExecutor(node, perception, speed_scale=speed_scale)
        error = None
        try:
            executor.run()
        except (ExecutionFailure, mujoco.FatalError) as exc:
            error = str(exc)
        result = node.benchmark_result()
        result.update({'executor': 'torque_baseline', 'perception_backend': backend,
                       'executor_source_sha256': fingerprints,
                       'contact_profile': node.episode_manifest.get('contact_profile', 'loose'),
                       'contact_parameters': node.episode_manifest.get('contact_parameters'),
                       'solver': {'cone': mujoco.mjtCone(node.model.opt.cone).name,
                                  'algorithm': mujoco.mjtSolver(node.model.opt.solver).name,
                                  'timestep_s': float(node.model.opt.timestep),
                                  'iterations': int(node.model.opt.iterations)},
                       'simulation_time_s': float(node.data.time), 'speed_scale': speed_scale,
                       'wall_clock_limit_s': float(node.benchmark_config.get('max_episode_time_s', 300)),
                       'events': executor.events, 'execution_error': error,
                       'final_contact_observation': getattr(executor, 'final_contact_observation', None),
                       'robot_base_position_m': node.episode_manifest.get('robot_base_position_m', [0.,0.,0.]),
                       'generated_model_sha256': {name: hashlib.sha256((Path(output_dir)/name).read_bytes()).hexdigest()
                                                  for name in ('scene.xml','panda.xml')}})
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
