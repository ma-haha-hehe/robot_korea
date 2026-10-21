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
from .execution_phases import ExecutionStateMachine, StateTransitionError
from .benchmark_core import dump_json, load_registry, score_episode
from .assembly_planner import (plan_assembly, resolve_grasp_yaws, PlanningError,
                               RELEASE_ABOVE_M, FALLBACK_GRASP_OFFSET_M)


class _CaptureLoaded(Exception):
    """Preempt a tipping held insertion while there is room to unload."""


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


def requires_dense_pose_feedback(target, placed_targets, base_height):
    """Align an upper square before entering a neighbouring brick's height."""
    if target['type'] != 'brick_2x2' or target['position'][2] <= base_height + .004:
        return False
    from .product_geometry import footprint
    registry = load_registry()
    def polygon(block):
        return np.asarray(footprint({'type': block['type'], 'target': {
            'position': block['position'], 'yaw_deg': math.degrees(block['yaw_rad'])}}, registry))
    own = polygon(target)
    for other in placed_targets:
        if ((target.get('id') is not None and other.get('id') == target['id'])
                or abs(other['position'][2] - target['position'][2]) >= .014):
            continue
        neighbour = polygon(other)
        separation = -math.inf
        for boundary in (own, neighbour):
            for first, second in zip(boundary, np.roll(boundary, -1, axis=0)):
                edge = second - first
                axis = np.array([-edge[1], edge[0]]) / np.linalg.norm(edge)
                a, b = own @ axis, neighbour @ axis
                separation = max(separation, float(a.min()-b.max()), float(b.min()-a.max()))
        if separation <= .002:
            return True
    return False


class ExecutionFailure(RuntimeError):
    pass


class SourceApproachDisturbance(ExecutionFailure):
    """An open approach displaced a loose source before closure."""


def nearest_gripper_yaw(yaw, reference):
    """Choose the shorter turn for an identical opposing-finger jaw axis."""
    deltas = [(candidate-reference+math.pi) % (2*math.pi)-math.pi
              for candidate in (yaw, yaw+math.pi)]
    return reference + min(deltas, key=abs)


class _GripDrift(Exception):
    """Interrupt descent while there is still clearance to restore the grip."""


class _SupportContact(Exception):
    """Stop insertion when a placed support starts loading the held part."""


def free_space_grip_drift(observed, target):
    q = observed['quaternion_wxyz']
    tilt = math.degrees(math.acos(float(np.clip(1 - 2*(q[1]**2 + q[2]**2), -1, 1))))
    # Monitor the final airborne descent too, before reaching the stud lead-in.
    return observed['position'][2] > target['position'][2] + .008 and tilt > 1.


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


def insertion_contact_indices(model, contacts, part, supports, base_supports=()):
    """Select the same ordered support contacts without scanning unrelated pairs."""
    a = model.geom_bodyid[contacts.geom1]
    b = model.geom_bodyid[contacts.geom2]
    support_ids = tuple(supports)
    selected = (((a == part) | (b == part))
                & (np.isin(a, support_ids) | np.isin(b, support_ids))
                & (contacts.dist <= .0001))
    if base_supports:
        base_ids = tuple(base_supports)
        base = np.isin(a, base_ids) | np.isin(b, base_ids)
        selected &= ~base | (contacts.dist <= -.0002)
    # Ascending indices retain the exact force summation order.
    return np.flatnonzero(selected)


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
        self.motion_replans = []
        self.grasp_recoveries = []
        self.perception = perception
        names = [self.model.body(i).name for i in range(self.model.nbody)]
        self.robot_body_mask = np.array([name.startswith(('link', 'panda_')) or
            name in ('hand', 'left_finger', 'right_finger') for name in names])
        self.environment_body_mask = np.array([name in ('world', 'table') or
            name.startswith('assembly_base') for name in names])
        self.fixed_base_mask = np.array([name == 'link0' for name in names])
        run_dir = os.environ.get('LEGO_BENCH_RUN_DIR')
        update = (lambda records: dump_json(Path(run_dir) / 'phase_trace.json', {
            'phases': records, 'state_machine': self.phases.summary()})) if run_dir else None
        self.phases = ExecutionStateMachine(lambda: float(self.data.time), on_update=update,
            on_failure=self.hold_after_failure,
            require_grasp_contact=perception is None and node.connection_mode == 'physics')

    def hold_after_failure(self, error):
        """Latch the measured arm pose; never retreat or open a failed grasp."""
        measured = self.data.qpos[self.qadr]
        if np.isfinite(measured).all():
            self.node.target_qpos[self.qadr] = measured.copy()
        self.node.gripper_unload_force_n = None
        contacts = []
        for contact in self.data.contact:
            if contact.dist >= -.00005:
                continue
            contacts.append({'bodies': [self.model.body(int(self.model.geom_bodyid[g])).name
                                       for g in (contact.geom1, contact.geom2)],
                             'depth_m': -float(contact.dist)})
        self.failure_contacts = sorted(contacts, key=lambda row: row['depth_m'], reverse=True)[:10]

    def close_gripper_for_grasp(self, target, timeout_s=1.2):
        """Finish on stable bilateral load; allow a bounded settling recovery."""
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ExecutionFailure('grasp closure timeout must be finite and positive')
        self.node.gripper_target = 0.
        if (self.perception is not None or self.node.connection_mode != 'physics'
                or self.node.episode_manifest.get('contact_profile') == 'plastic'):
            # Plastic insertion is sensitive to residual arm/part settling.
            # Keep the validated dwell; early finger contact alone regressed
            # full nine- and twelve-part tasks. Lift still verifies the grasp.
            self.step(.7)
            return
        part = self.model.body(target['body_name']).id
        fingers = [self.model.body(name).id for name in ('left_finger', 'right_finger')]
        dofs = [self.model.joint(f'panda_finger_joint{i}').dofadr[0] for i in (1, 2)]
        weight = float(self.model.body_mass[part] * np.linalg.norm(self.model.opt.gravity))
        stable = 0.
        for _ in range(math.ceil(timeout_s / .01)):
            self.step(.01)
            capacity = dict.fromkeys(fingers, 0.)
            for index, contact in enumerate(self.data.contact):
                bodies = [int(self.model.geom_bodyid[g]) for g in (contact.geom1, contact.geom2)]
                if part not in bodies or contact.dist > .0001:
                    continue
                other = bodies[1] if bodies[0] == part else bodies[0]
                if other not in capacity:
                    continue
                force = np.zeros(6)
                mujoco.mj_contactForce(self.model, self.data, index, force)
                capacity[other] += max(0., float(force[0])) * float(contact.friction[0])
            velocity = np.zeros(6)
            mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY, part, velocity, 0)
            ready = (np.isfinite(velocity).all()
                     and np.linalg.norm(velocity[:3]) < .1
                     and np.linalg.norm(velocity[3:]) < .005
                     and all(math.isfinite(load) and load >= max(weight, .01)
                         for load in capacity.values())
                     and np.all(np.abs(self.data.qvel[dofs]) < .002))
            stable = stable + .01 if ready else 0.
            if stable >= .08:
                return
        raise ExecutionFailure(f'grasp closure did not establish stable two-finger contact: {target["id"]}; '
                               f'friction_capacity_n={list(capacity.values())}, finger_velocity={self.data.qvel[dofs].tolist()}, '
                               f'part_velocity={velocity.tolist()}')

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
                    self.node.max_robot_environment_penetration_m,
                    getattr(self.node, 'max_self_penetration_m', 0.)) > .0005:
                raise ExecutionFailure('physical contact penetration limit exceeded; motion stopped')
            if self.node.connection_mode == 'snap':
                self.node.auto_weld_touching_bricks()
                self.node.maintain_fake_welds()
            if not np.isfinite(self.data.qpos).all():
                raise ExecutionFailure('non-finite simulation state')
            airborne = getattr(self, '_airborne_recovery_target', None)
            if airborne is not None:
                self.check_airborne_grasp(airborne, allow_transient=True)
            capture = getattr(self, '_capture_guard_target', None)
            if capture is not None:
                self.check_capture_motion(capture)
            insertion = getattr(self, '_insertion_contact_guard', None)
            if insertion is not None:
                part, supports = insertion
                normal_force = 0.
                indices = insertion_contact_indices(
                    self.model, self.data.contact, part, supports,
                    getattr(self, '_insertion_base_supports', ()))
                for index in indices:
                    force = np.zeros(6)
                    mujoco.mj_contactForce(self.model, self.data, int(index), force)
                    normal_force += max(0., float(force[0]))
                if normal_force > .01:
                    self.node.target_qpos[self.qadr] = self.data.qpos[self.qadr].copy()
                    self._insertion_contact_observation = {
                        'simulation_time_s': float(self.data.time),
                        'normal_force_sum_n': normal_force,
                        'position_m': self.data.xpos[part].tolist(),
                        'quaternion_wxyz': self.data.xquat[part].tolist()}
                    raise _SupportContact()
            target = getattr(self, '_descent_guard_target', None)
            if target is not None:
                body = self.model.body(target['body_name']).id
                observed = {'position': self.data.xpos[body],
                            'quaternion_wxyz': self.data.xquat[body]}
                if free_space_grip_drift(observed, target):
                    self.node.target_qpos[self.qadr] = self.data.qpos[self.qadr].copy()
                    raise _GripDrift()
        self.node.update_safety_metrics()

    def configuration_has_collision(self, data):
        """Reject arm self-contact and unintended arm/environment contact."""
        mujoco.mj_collision(self.model, data)
        contacts = data.contact
        a = self.model.geom_bodyid[contacts.geom1]
        b = self.model.geom_bodyid[contacts.geom2]
        ra, rb = self.robot_body_mask[a], self.robot_body_mask[b]
        blocked = ((ra & rb)
                   | (ra & ~self.fixed_base_mask[a] & self.environment_body_mask[b])
                   | (rb & ~self.fixed_base_mask[b] & self.environment_body_mask[a]))
        return bool(np.any(blocked & (contacts.dist < -.00005)))

    def joint_path_has_collision(self, start, end):
        """Check a joint segment at no more than 0.05 rad increments."""
        data = self.ik_data
        data.qpos[:] = self.data.qpos
        count = max(1, math.ceil(float(np.max(np.abs(end-start))) / .05))
        for fraction in np.linspace(0., 1., count+1):
            data.qpos[self.qadr] = start + fraction*(end-start)
            mujoco.mj_kinematics(self.model, data)
            if self.configuration_has_collision(data):
                return True
        return False

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
        elbows = []
        for offset in (-.8, .8):
            alternative = facing.copy()
            alternative[2] = offset
            elbows.append(alternative)
        collision_rejected = False
        for seed in (initial, facing, home, *elbows):
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
                cosine = (float(np.trace(desired.T @ rotation)) - 1.) / 2.
                # The cross-product error is sin(theta)*axis: it also vanishes
                # at a wrong half turn. Escape that singularity with a world-
                # frame rotation vector, and never accept the opposite pose.
                if cosine < 0. and np.linalg.norm(er) < .1:
                    quaternion = np.empty(4)
                    mujoco.mju_mat2Quat(quaternion, (desired @ rotation.T).reshape(9))
                    if quaternion[0] < 0.:
                        quaternion *= -1.
                    magnitude = float(np.linalg.norm(quaternion[1:]))
                    er = quaternion[1:] * (2*math.atan2(magnitude, quaternion[0]) / magnitude)
                if np.linalg.norm(ep) < .00015 and np.linalg.norm(er) < .003 and cosine > 0.:
                    if self.configuration_has_collision(d):
                        collision_rejected = True
                        break
                    return d.qpos[self.qadr].copy()
                mujoco.mj_jac(self.model, d, jp, jr, tip, self.hand)
                jac = np.vstack((jp[:, self.dadr], jr[:, self.dadr]))
                error = np.r_[ep, er]
                delta = jac.T @ np.linalg.solve(jac @ jac.T + .002 * np.eye(6), error)
                d.qpos[self.qadr] = np.clip(d.qpos[self.qadr] + np.clip(delta, -.12, .12),
                                          self.limits[:, 0] + .001, self.limits[:, 1] - .001)
        reason = 'IK converged only to colliding configurations' if collision_rejected else 'IK did not converge'
        raise ExecutionFailure(f'{reason} at {list(position)}')

    def select_grasp_yaw(self, grasp, hover, yaw):
        """Try the planned jaw axis, then its physically equivalent half turn."""
        if self.node.episode_manifest.get('contact_profile') == 'plastic':
            rotation = self.data.xmat[self.hand].reshape(3, 3)
            yaw = nearest_gripper_yaw(yaw, math.atan2(rotation[1, 0], rotation[0, 0]))
        for candidate in (yaw, (yaw + 2*math.pi) % (2*math.pi) - math.pi):
            try:
                previous = self.ik(hover, candidate)
                for position in np.linspace(hover, grasp, 9)[1:]:
                    current = self.ik(position, candidate, seed_q=previous)
                    if self.joint_path_has_collision(previous, current):
                        raise ExecutionFailure('grasp approach crosses a robot collision')
                    previous = current
            except ExecutionFailure:
                continue
            return candidate
        raise ExecutionFailure('neither symmetric gripper orientation reaches the grasp')

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

    def approach_grasp(self, position, yaw, careful=False):
        """Follow the checked grasp path and confirm the measured tool pose."""
        if not careful:
            self.move_vertical(position, yaw)
            self.confirm_grasp_pose(position, yaw)
            return
        pregrasp = np.asarray(position) + [0., 0., .04]
        self.move_vertical(pregrasp, yaw)
        self.confirm_grasp_pose(pregrasp, yaw)
        speed = self.speed_scale
        try:
            self.speed_scale = min(speed, .35)
            self.move_vertical(position, yaw)
            self.confirm_grasp_pose(position, yaw)
        finally:
            self.speed_scale = speed

    def confirm_source_unchanged(self, target, observed):
        """Do not close on a loose part already tipped by the open approach."""
        if self.perception is not None or self.node.connection_mode != 'physics':
            return
        measured = self.node.current_block_state()['blocks'][target['id']]
        position = np.asarray(measured['position'])
        q = np.asarray(measured['quaternion_wxyz'])
        norm = float(q @ q)
        if (not np.isfinite(position).all() or not np.isfinite(q).all()
                or norm <= 0 or np.linalg.norm(position - observed['position']) > .001
                or 1 - 2*(q[1]**2 + q[2]**2)/norm < math.cos(math.radians(3))):
            raise SourceApproachDisturbance('source part moved during open approach; closure cancelled')

    def recover_source_approach(self, target, planned, original, hover, yaw):
        """Retry once from a measured, upright source; never reposition a part."""
        if self.node.episode_manifest.get('contact_profile') != 'plastic':
            raise SourceApproachDisturbance('source approach recovery requires plastic physics')
        def check_source(observed):
            position = np.asarray(observed['position'])
            q = np.asarray(observed['quaternion_wxyz'])
            norm = float(q @ q)
            region = self.node.episode_manifest['spawn_region']
            valid = (np.isfinite(position).all() and np.isfinite(q).all() and norm > 0
                and np.linalg.norm(position[:2]-np.asarray(original['position'])[:2]) <= .01
                and abs(position[2]-original['position'][2]) <= .001
                and 1-2*(q[1]**2+q[2]**2)/norm >= math.cos(math.radians(3))
                and region['x'][0] <= position[0] <= region['x'][1]
                and region['y'][0] <= position[1] <= region['y'][1])
            if not valid:
                raise SourceApproachDisturbance('source approach cannot safely retry: pose outside bounds')
        measured = self.node.current_block_state()['blocks'][target['id']]
        check_source(measured)
        record = {'block': target['id'], 'status': 'running',
                  'original_position_m': list(original['position']),
                  'disturbed_position_m': list(measured['position'])}
        self.grasp_recoveries.append(record)
        try:
            self.verify_released_parts()
            self.node.target_qpos[self.qadr] = self.data.qpos[self.qadr].copy()
            self.open_gripper_before_retreat(target)
            self.move_vertical(hover, yaw)
            observed = self.observe_target(target)
            check_source(observed)
            position = np.asarray(observed['position'])
            pick_yaw, place_yaw = resolve_grasp_yaws(
                observed['yaw_rad'], target['yaw_rad'], planned['grasp_spin_deg'])
            grasp = position + [0., 0., FALLBACK_GRASP_OFFSET_M
                if target['placement_mode'] == 'release_above_press' else -.003]
            hover = grasp + [0., 0., .16]
            pick_yaw = self.select_grasp_yaw(grasp, hover, pick_yaw)
            self.move(hover, pick_yaw)
            self.prepare_grasp_opening(target['type'], observed['yaw_rad'], pick_yaw)
            self.approach_grasp(grasp, pick_yaw, careful=True)
            self.confirm_source_unchanged(target, observed)
            self.verify_released_parts()
            record.update(status='completed', retry_position_m=position.tolist())
            return observed, position, pick_yaw, place_yaw, grasp, hover
        except BaseException as error:
            record.update(status='failed', error=str(error) or type(error).__name__)
            raise

    def plan_joint_transfer(self, position, yaw):
        """Prefer the direct segment; try bounded, checked alternatives if blocked.

        Each returned segment is checked before the arm starts moving. This is
        a small simulation baseline fallback, not a complete motion planner.
        """
        target = self.ik(position, yaw)
        start = self.data.qpos[self.qadr].copy()
        if not self.joint_path_has_collision(start, target):
            return [target]
        home = np.array([0., -.785, 0., -2.356, 0., 1.571, .785])
        facing = home.copy()
        facing[0] = math.atan2(position[1], position[0])
        candidates = [home, facing]
        for offset in (-.8, .8):
            seed = facing.copy()
            seed[2] = offset
            candidates.append(seed)
        goals = [target]
        for seed in candidates:
            try:
                alternate = self.ik(position, yaw, seed_q=seed)
            except ExecutionFailure:
                continue
            if any(np.max(np.abs(alternate - old)) < .01 for old in goals):
                continue
            goals.append(alternate)
            if not self.joint_path_has_collision(start, alternate):
                self.motion_replans.append({'kind': 'alternate_ik', 'position_m': list(position)})
                return [alternate]
        for waypoint in candidates:
            if self.joint_path_has_collision(start, waypoint):
                continue
            for goal in goals:
                if not self.joint_path_has_collision(waypoint, goal):
                    self.motion_replans.append({'kind': 'joint_waypoint', 'position_m': list(position)})
                    return [waypoint, goal]
        raise ExecutionFailure('no checked joint transfer path around robot collision')

    def move_joint_segment(self, target):
        start = self.data.qpos[self.qadr].copy()
        # Recheck from the measured state after a previous segment has settled.
        if self.joint_path_has_collision(start, target):
            raise ExecutionFailure('measured joint transfer path crosses a robot collision')
        seconds = max(.6, float(np.max(np.abs(target - start))) / .7) / self.speed_scale
        count = max(1, int(seconds / .01))
        for i in range(1, count + 1):
            t = i / count
            self.node.target_qpos[self.qadr] = start + (target - start) * (3*t*t - 2*t*t*t)
            self.step(.01)
        self.step(.1)

    def move(self, position, yaw):
        if self.node.episode_manifest.get('contact_profile') == 'plastic':
            rotation = self.data.xmat[self.hand].reshape(3, 3)
            yaw = nearest_gripper_yaw(yaw, math.atan2(rotation[1, 0], rotation[0, 0]))
        for target in self.plan_joint_transfer(position, yaw):
            self.move_joint_segment(target)

    def move_vertical(self, position, yaw, target_rotation=None):
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
            options = {'target_rotation': target_rotation} if target_rotation is not None else {}
            points.append(self.ik(start + fraction * (np.asarray(position) - start),
                                  initial_yaw + fraction * angle, seed_q=points[-1], **options))
        points = np.array(points)
        if any(self.joint_path_has_collision(a, b) for a, b in zip(points, points[1:])):
            raise ExecutionFailure('vertical approach crosses a robot collision')
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

    def confirm_held_part(self, target):
        """Hold the arm and stop if either fingertip loses its load."""
        part = self.model.body(target['body_name']).id
        fingers = {self.model.body(name).id for name in ('left_finger', 'right_finger')}
        for attempt in range(9):
            loaded = set()
            if self.part_contacts_observable(part):
                for index, contact in enumerate(self.data.contact):
                    bodies = [int(self.model.geom_bodyid[g])
                              for g in (contact.geom1, contact.geom2)]
                    if part not in bodies or contact.dist > .0001:
                        continue
                    other = bodies[1] if bodies[0] == part else bodies[0]
                    if other not in fingers:
                        continue
                    force = np.zeros(6)
                    mujoco.mj_contactForce(self.model, self.data, index, force)
                    if force[0] > .01:
                        loaded.add(other)
            if loaded == fingers:
                return attempt * .01
            # A brief contact transition may recover while the arm holds still.
            # Never chase the measured pose of a falling or resting loose part.
            if attempt == 0:
                self.node.target_qpos[self.qadr] = self.data.qpos[self.qadr]
            if attempt < 8:
                self.step(.01)
        raise ExecutionFailure(f"held-part alignment lost the two-finger grasp: {target['id']}")

    def align_held_part(self, target):
        """Correct the arm pose from the measured part pose, without moving the part directly."""
        if self.perception is not None:
            raise ExecutionFailure('physics insertion requires validated in-hand pose tracking')
        period = math.radians(load_registry()[target['type']]['yaw_symmetry_deg'])
        for _ in range(6):
            self.confirm_held_part(target)
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

    def descend_to_insertion_entry(self, target, endpoint, yaw):
        """Unload at measured support contact, or at the bounded entry height."""
        released = {event['block'] for event in self.events}
        supports = {self.model.body(block['body_name']).id
                    for block in self.node.episode_manifest['target_blocks']
                    if block['id'] in released and abs(
                        target['position'][2] - block['position'][2] - .0192) < .004}
        # The fixed baseplate is also a support, including for the first brick.
        # Waiting for a previously released brick omitted all base-layer contact.
        self._insertion_base_supports = {
            body for body in range(self.model.nbody)
            if self.model.body(body).name.startswith('assembly_base')}
        supports.update(self._insertion_base_supports)
        part = self.model.body(target['body_name']).id
        self._insertion_contact_guard = (part, supports)
        self._insertion_contact_observation = None
        try:
            try:
                self.move_vertical(endpoint, yaw)
            except _SupportContact:
                pass
        finally:
            self._insertion_contact_guard = None
            self._insertion_base_supports = set()
        return self._insertion_contact_observation

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
            # Preserve the original wide-side recovery on the fixed base.
            # Extend pose feedback to upper placements before support contact.
            if (self.perception is None
                    and self.node.episode_manifest.get('contact_profile') == 'plastic'
                    and (target['position'][2] >
                         min(b['position'][2] for b in targets) + .004
                         or (target['type'] == 'brick_4x2'
                             and abs(math.sin(target['yaw_rad'] - yaw)) > .7))):
                self._descent_guard_target = target
            try:
                if (self.perception is None
                        and self.node.episode_manifest.get('contact_profile') == 'plastic'
                        and requires_dense_pose_feedback(
                            target, [b for b in targets if b['id'] in placed],
                            min(b['position'][2] for b in targets))):
                    self._descent_guard_target = None
                    self.descend_with_pose_feedback(target, yaw)
                    self.last_seating_confirmation['pose_feedback']['trigger'] = 'near_same_layer_neighbour'
                    recovered = True
                elif self.node.episode_manifest.get('contact_profile') == 'plastic':
                    # Partial support lets lateral jaw compression tip the part
                    # at the lead-in. Unload earlier there; retain the lower
                    # release for fully supported fits to limit free settling.
                    unload_height = (.002 if self._requires_precise_support_alignment(target)
                                     else .001)
                    contact = self.descend_to_insertion_entry(
                        target, endpoint + [0, 0, unload_height], yaw)
                    self._descent_guard_target = None
                    if self.held_edge_insertion_required(target):
                        self.insert_edge_while_held(target, yaw)
                    else:
                        self.release_and_press(target, yaw, 'release_at_insertion_entry')
                    if contact is not None:
                        self.last_seating_confirmation['insertion_contact'] = contact
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

    def check_airborne_grasp(self, target, allow_transient=False):
        """Permit recovery only while both jaws hold an externally free part."""
        part = self.model.body(target['body_name']).id
        fingers = {self.model.body(n).id for n in ('left_finger', 'right_finger')}
        q = self.data.xquat[part]
        if (not self.part_contacts_observable(part) or not np.isfinite(q).all()
                or not np.isfinite(self.data.xpos[part]).all()
                or np.linalg.norm(self.data.xpos[part, :2] - np.asarray(target['position'])[:2]) > .002
                or 1 - 2*(q[1]**2 + q[2]**2) < math.cos(math.radians(8))):
            raise ExecutionFailure('airborne recovery lost observable pose bounds')
        held = set()
        for index, contact in enumerate(self.data.contact):
            if contact.dist > .0001:
                continue
            bodies = [int(self.model.geom_bodyid[g]) for g in (contact.geom1, contact.geom2)]
            if part in bodies:
                other = bodies[1] if bodies[0] == part else bodies[0]
                if other not in fingers:
                    raise ExecutionFailure('airborne recovery has an external contact')
                force = np.zeros(6)
                mujoco.mj_contactForce(self.model, self.data, index, force)
                if force[0] > .01:
                    held.add(other)
            elif any(body in fingers for body in bodies):
                other = next(body for body in bodies if body not in fingers) if not set(bodies) <= fingers else None
                if other is not None and (other == 0 or self.model.body(other).name.startswith(('bench__', 'assembly_base'))):
                    raise ExecutionFailure('airborne recovery finger touches the environment')
        if held != fingers:
            self.node.target_qpos[self.qadr] = self.data.qpos[self.qadr].copy()
            if allow_transient:
                missing = getattr(self, '_airborne_missing_s', 0.) + self.model.opt.timestep
                self._airborne_missing_s = missing
                if missing <= .01 + 1e-12:
                    return
            raise ExecutionFailure('airborne recovery lost the two-finger grasp')
        self._airborne_missing_s = 0.


    def return_to_airborne_clearance(self, target, yaw):
        """Raise a free, tilted grip without rotating beside the assembly."""
        part = self.model.body(target['body_name']).id
        height = float(self.data.xpos[part, 2] - target['position'][2])
        if not .008 < height <= .020:
            raise ExecutionFailure('airborne return requires a bounded free-space height')
        self.check_airborne_grasp(target)
        rotation = self.data.xmat[self.hand].reshape(3, 3).copy()
        tip = self.data.xpos[self.hand] + rotation @ self.tool_offset
        speed = self.speed_scale
        try:
            self._airborne_missing_s = 0.
            self._airborne_recovery_target = target
            self.speed_scale = min(speed, .5)
            for correction_attempt in range(2):
                self.move_vertical(tip + [0, 0, .030 - height], yaw, target_rotation=rotation)
                self.check_airborne_grasp(target)
                height = float(self.data.xpos[part, 2] - target['position'][2])
                if height >= .025:
                    break
                rotation = self.data.xmat[self.hand].reshape(3, 3).copy()
                tip = self.data.xpos[self.hand] + rotation @ self.tool_offset
        finally:
            self._airborne_recovery_target = None
            self._airborne_missing_s = 0.
            self.speed_scale = speed
        self.check_airborne_grasp(target)
        if self.data.xpos[part, 2] - target['position'][2] < .025:
            raise ExecutionFailure('airborne return did not establish clearance')


    def feedback_support_contact(self, target):
        """Hand over to pressing at a loaded fit, before correcting against it."""
        part = self.model.body(target['body_name']).id
        if not self.part_contacts_observable(part):
            raise ExecutionFailure('feedback support contacts are not observable')
        released = {event['block'] for event in self.events}
        supports = {self.model.body(block['body_name']).id
                    for block in self.node.episode_manifest['target_blocks']
                    if block['id'] in released and abs(
                        target['position'][2] - block['position'][2] - .0192) < .004}
        bases = {body for body in range(self.model.nbody)
                 if self.model.body(body).name.startswith('assembly_base')}
        supports.update(bases)
        normal_force = 0.
        for index in insertion_contact_indices(self.model, self.data.contact, part, supports, bases):
            force = np.zeros(6)
            mujoco.mj_contactForce(self.model, self.data, int(index), force)
            normal_force += max(0., float(force[0]))
        if normal_force <= .01:
            return None
        position, q = self.data.xpos[part], self.data.xquat[part]
        height = float(position[2] - target['position'][2])
        if (not np.isfinite(position).all() or not np.isfinite(q).all()
                or not -.0004 <= height <= .004
                or np.linalg.norm(position[:2] - np.asarray(target['position'])[:2]) > .001
                or 1 - 2*(q[1]**2 + q[2]**2) < math.cos(math.radians(3))):
            raise ExecutionFailure('feedback support contact outside bounded insertion entry')
        return {'normal_force_n': normal_force, 'position_m': position.tolist(),
                'quaternion_wxyz': q.tolist(), 'time_s': float(self.data.time)}

    def prepare_feedback_grip(self, target):
        """Reduce excessive jaw preload while retaining a measured pinch."""
        if (self.perception is not None or self.node.connection_mode != 'physics'
                or self.node.episode_manifest.get('contact_profile') != 'plastic'):
            raise ExecutionFailure('feedback grip requires Oracle plastic physics')
        actuators = [self.model.actuator(f'finger_actuator{i}').id for i in (1, 2)]
        forces = np.abs(self.data.actuator_force[actuators])
        if not np.isfinite(forces).all():
            raise ExecutionFailure('invalid finger force before feedback')
        initial = min(5., float(np.min(forces)))
        holding = 1.5
        self.node.target_qpos[self.qadr] = self.data.qpos[self.qadr].copy()
        self._feedback_force_active = True
        for index in range(1, 11):
            self.node.gripper_unload_force_n = initial + (holding-initial)*index/10
            self.step(.01)
        self.confirm_held_part(target)
        return {'initial_normal_force_command_n': initial,
                'holding_normal_force_command_n': holding, 'transition_simulation_s': .1}

    def descend_with_pose_feedback(self, target, yaw):
        """Correct a wide-side grasp while lowering, before it touches neighbours."""
        if self.perception is not None:
            raise ExecutionFailure('insertion pose feedback requires validated in-hand tracking')
        self.confirm_held_part(target)
        try:
            grip_control = self.prepare_feedback_grip(target)
            period = math.radians(load_registry()[target['type']]['yaw_symmetry_deg'])
            goal = np.asarray(target['position'])
            maximum_tilt = 0.
            airborne_returns = 0
            for attempt in range(80):
                observed = self.node.current_block_state()['blocks'][target['id']]
                position = np.asarray(observed['position'])
                q = np.asarray(observed['quaternion_wxyz'])
                tilt = math.acos(float(np.clip(1 - 2*(q[1]**2 + q[2]**2), -1, 1)))
                maximum_tilt = max(maximum_tilt, tilt)
                if tilt > math.radians(3):
                    if tilt > math.radians(8) or position[2] - goal[2] <= .008:
                        raise ExecutionFailure('feedback insertion exceeded 3 degree tilt near placement')
                    # Hold measured arm position through a bounded contact transition;
                    # do not lift until bilateral load and free space are confirmed.
                    self.confirm_held_part(target)
                    self.check_airborne_grasp(target)
                    if position[2] - goal[2] <= .020:
                        if airborne_returns >= 2:
                            raise ExecutionFailure('airborne return recovery budget exhausted')
                        self.return_to_airborne_clearance(target, yaw)
                        airborne_returns += 1
                        continue
                contact = (self.feedback_support_contact(target)
                           if .0015 <= position[2] - goal[2] <= .004 else None)
                if position[2] - goal[2] < .0015 or contact is not None:
                    if self.held_edge_insertion_required(target):
                        self.insert_edge_while_held(target, yaw)
                    else:
                        self.release_and_press(target, yaw, 'release_before_seating')
                    if contact is not None:
                        self.last_seating_confirmation['insertion_contact'] = contact
                    self.last_seating_confirmation['feedback_grip_control'] = grip_control
                    self.last_seating_confirmation['pose_feedback'] = {
                        'steps': attempt, 'airborne_returns': airborne_returns,
                        'max_observed_tilt_deg': math.degrees(maximum_tilt),
                        'max_step_m': .002}
                    return
                # Refresh the observation after a transient grasp has settled.
                # Never correct the pose of a part that is no longer held.
                if self.confirm_held_part(target):
                    continue
                part_rotation = np.empty(9)
                mujoco.mju_quat2Mat(part_rotation, q)
                # Keep an equivalent observed yaw: a symmetric brick must not spin
                # by 90/180 degrees just because it needs a small roll/pitch correction.
                measured_yaw = math.atan2(part_rotation[3], part_rotation[0])
                desired_yaw = target['yaw_rad'] + round(
                    (measured_yaw-target['yaw_rad'])/period)*period
                c, s = math.cos(desired_yaw), math.sin(desired_yaw)
                desired = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
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
                try:
                    # A new tilt during a lowering segment must interrupt that
                    # segment immediately, just as it does during direct entry.
                    # Apply attitude correction at the measured height next time.
                    if stage[2] < position[2] - 1e-6:
                        self._descent_guard_target = target
                    for index in range(1, 11):
                        t = index / 10
                        self.node.target_qpos[self.qadr] = start + (end-start)*(3*t*t-2*t*t*t)
                        self.step(.01)
                    self.step(.04)
                except _GripDrift:
                    continue
                finally:
                    self._descent_guard_target = None
            raise ExecutionFailure('feedback insertion did not seat within 80 bounded steps')
        finally:
            self.node.gripper_unload_force_n = None
            self._feedback_force_active = False


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

    def lower_with_finger_friction(self, target, velocity_gain=5., handover_height_m=.004):
        """Meter a high release through physical jaw friction before opening.

        Oracle-only candidate: contact coefficients and measured downward motion
        determine bounded actuator setpoints. No part state or material is changed.
        A 10 mm/s reference is a control target, not a guaranteed slip velocity.
        """
        if (self.perception is not None or self.node.connection_mode != 'physics'
                or self.node.episode_manifest.get('contact_profile') != 'plastic'):
            raise ExecutionFailure('controlled release requires Oracle plastic physics')
        if not math.isfinite(velocity_gain) or velocity_gain <= 0:
            raise ExecutionFailure('controlled release requires a finite positive velocity gain')
        if not math.isfinite(handover_height_m) or not .0004 <= handover_height_m <= .004:
            raise ExecutionFailure('controlled release requires bounded handover height')
        body = self.model.body(target['body_name']).id
        fingers = {self.model.body(n).id for n in ('left_finger', 'right_finger')}
        qadr = [self.model.joint(f'panda_finger_joint{i}').qposadr[0] for i in (1, 2)]
        coefficients = []
        for contact in self.data.contact:
            bodies = {int(self.model.geom_bodyid[g]) for g in (contact.geom1, contact.geom2)}
            if body in bodies and bodies & fingers and contact.dist <= .0001:
                coefficients.append(float(contact.friction[0]))
        if not coefficients or not all(math.isfinite(mu) and mu > 0 for mu in coefficients):
            raise ExecutionFailure('controlled release requires frictional finger contact')
        mu = min(coefficients)
        gains = [float(self.model.actuator_gainprm[self.model.actuator(f'finger_actuator{i}').id, 0])
                 for i in (1, 2)]
        if not all(math.isfinite(gain) and gain > 0 for gain in gains) or not math.isclose(*gains):
            raise ExecutionFailure('controlled release requires matched positive finger gains')
        weight = float(self.model.body_mass[body] * abs(self.model.opt.gravity[2]))
        self.node.target_qpos[self.qadr] = self.data.qpos[self.qadr].copy()
        z = float(self.data.xpos[body, 2])
        speed = maximum_speed = 0.
        step_s = .002
        for index in range(1250):
            position = self.data.xpos[body]
            q = self.data.xquat[body]
            if (not self.part_contacts_observable(body)
                    or not np.isfinite(position).all()
                    or not np.isfinite(q).all()
                    or np.linalg.norm(position[:2] - np.asarray(target['position'])[:2]) > .001
                    or 1 - 2*(q[1]**2 + q[2]**2) < math.cos(math.radians(3))):
                raise ExecutionFailure('controlled release left observable alignment bounds')
            height = float(position[2] - target['position'][2])
            if height < -.0004:
                raise ExecutionFailure('controlled release passed below the target')
            if height <= handover_height_m:
                return {'mode': 'friction_metered', 'duration_s': index * step_s,
                        'handover_height_m': height, 'target_speed_m_s': .01,
                        'max_observed_downward_speed_m_s': maximum_speed,
                        'velocity_gain_n_per_m_s': velocity_gain,
                        'contact_sliding_friction': mu,
                        'normal_force_command_bounds_n': [.005, .5]}
            force = float(np.clip(weight / (2 * mu) + velocity_gain * (speed - .01), .005, .5))
            self.node.gripper_target = float(np.clip(
                np.mean(self.data.qpos[qadr]) - force / gains[0], 0., .04))
            self.step(step_s)
            next_z = float(self.data.xpos[body, 2])
            speed = (z - next_z) / step_s
            maximum_speed = max(maximum_speed, speed)
            z = next_z
        raise ExecutionFailure('controlled release did not reach handover height')

    def check_capture_motion(self, target):
        """Unload a fast tipping fit before the global contact stop is reached."""
        body = self.model.body(target['body_name']).id
        position = self.data.xpos[body]
        q = self.data.xquat[body]
        tilt = math.acos(float(np.clip(1 - 2*(q[1]**2 + q[2]**2), -1., 1.)))
        if (not np.isfinite(position).all() or not np.isfinite(q).all()
                or np.linalg.norm(position[:2] - np.asarray(target['position'])[:2]) > .001
                or tilt > math.radians(3)):
            raise ExecutionFailure('support-edge capture lost alignment')
        height = float(position[2] - target['position'][2])
        previous_tilt, previous_height = getattr(self, '_capture_previous', (tilt, height))
        tilt_rate = (tilt - previous_tilt) / self.model.opt.timestep
        downward_speed = (previous_height - height) / self.model.opt.timestep
        self._capture_previous = (tilt, height)
        depth = 0.
        for contact in self.data.contact:
            bodies = [int(self.model.geom_bodyid[g]) for g in (contact.geom1, contact.geom2)]
            if body in bodies and all(self.model.body(b).name.startswith('bench__') for b in bodies):
                depth = max(depth, -float(contact.dist))
        if (tilt_rate > 2. and downward_speed > .02) or depth >= .0002:
            self.node.target_qpos[self.qadr] = self.data.qpos[self.qadr].copy()
            raise _CaptureLoaded({'tilt_rate_rad_s': tilt_rate,
                                  'downward_speed_m_s': downward_speed,
                                  'part_contact_depth_m': depth})

    def capture_support_edge(self, target, yaw):
        """Keep a precariously supported part held until its stud fit engages."""
        from .support_geometry import (support_margin_m, jaw_crosses_support_edge,
                                       support_overlap_area_m2)
        released = {event['block'] for event in self.events}
        placed = [block for block in self.node.episode_manifest['target_blocks']
                  if block['id'] in released]
        margin = support_margin_m(target, placed)
        if margin is None or margin >= .001:
            return None
        # Keep the established unload path when more than two nominal stud
        # cells support the part. Extra held pressing there regressed later
        # layers of full tasks; this capture is for small edge-supported fits.
        if support_overlap_area_m2(target, placed) > 2 * .016**2 + 1e-9:
            return None
        if jaw_crosses_support_edge(target, placed, yaw):
            return {'mode': 'unload_jaws_across_support_edge',
                    'support_margin_m': margin, 'descent_m': 0.}
        body = self.model.body(target['body_name']).id
        height = float(self.data.xpos[body, 2] - target['position'][2])
        if not 0 <= height <= .004:
            raise ExecutionFailure('support-edge capture requires bounded entry height')
        if height <= .001:
            return {'support_margin_m': margin, 'entry_height_m': height, 'descent_m': 0.}
        self.confirm_held_part(target)
        rotation = self.data.xmat[self.hand].reshape(3, 3)
        tip = self.data.xpos[self.hand] + rotation @ self.tool_offset
        tip[2] -= height - .001
        speed = self.speed_scale
        unloaded = None
        try:
            self._capture_guard_target = target
            self.speed_scale = min(speed, .2)
            self.move_vertical(tip, yaw)
        except _CaptureLoaded as contact:
            unloaded = contact.args[0]
        finally:
            self.speed_scale = speed
            self._capture_guard_target = None
            if hasattr(self, '_capture_previous'):
                del self._capture_previous
        final_height = float(self.data.xpos[body, 2] - target['position'][2])
        if unloaded is not None:
            if not -.0004 <= final_height <= .004:
                raise ExecutionFailure('capture unload left bounded entry height')
            return {'support_margin_m': margin, 'entry_height_m': final_height,
                    'descent_m': height - final_height, 'unload_on_motion': unloaded}
        if not -.0004 <= final_height <= .0015:
            raise ExecutionFailure('support-edge capture did not reach the entry height')
        return {'support_margin_m': margin, 'entry_height_m': final_height,
                'descent_m': height - final_height}

    def unload_gripper_for_release(self):
        """Remove opposing jaw preload gradually before commanding separation."""
        qadr = [self.model.joint(f'panda_finger_joint{i}').qposadr[0] for i in (1, 2)]
        actuators = [self.model.actuator(f'finger_actuator{i}').id for i in (1, 2)]
        gains = self.model.actuator_gainprm[actuators, 0]
        forces = np.abs(self.data.actuator_force[actuators])
        if (not np.isfinite(gains).all() or np.any(gains <= 0.)
                or not np.isfinite(forces).all()):
            raise ExecutionFailure('invalid finger actuator state for unloading')
        initial_force = min(5., float(np.min(forces)))
        self.node.target_qpos[self.qadr] = self.data.qpos[self.qadr].copy()
        try:
            for index in range(21):
                self.node.gripper_unload_force_n = initial_force * (1 - index / 20)
                self.step(.01)
        finally:
            self.node.gripper_unload_force_n = None
        # Resume the position ramp from the measured width, without reapplying
        # the stale zero-width command and recreating the released preload.
        self.node.gripper_control = float(np.mean(self.data.qpos[qadr]))
        return {'initial_normal_force_n': initial_force, 'duration_s': .21}

    def release_preload_required(self, target):
        """Unload a tilted fit at the lead-in; preserve level releases."""
        body = self.model.body(target['body_name']).id
        position, q = self.data.xpos[body], self.data.xquat[body]
        if not np.isfinite(position).all() or not np.isfinite(q).all():
            raise ExecutionFailure('invalid measured pose before release')
        threshold = .0025
        targets = self.node.episode_manifest['target_blocks']
        if (target['type'] == 'brick_2x2'
                and target['position'][2] <= min(b['position'][2] for b in targets) + .004):
            threshold = .002
        return (target['type'] in ('brick_4x2', 'brick_2x2')
                and float(position[2] - target['position'][2]) > threshold
                and 1 - 2*(q[1]**2 + q[2]**2) < math.cos(math.radians(1)))

    def held_edge_insertion_required(self, target):
        """Retain a level half-supported square until its stud fit is seated."""
        if target['type'] != 'brick_2x2':
            return False
        targets = self.node.episode_manifest['target_blocks']
        if target['position'][2] <= min(b['position'][2] for b in targets) + .004:
            return False
        body = self.model.body(target['body_name']).id
        position, q = self.data.xpos[body], self.data.xquat[body]
        if not np.isfinite(position).all() or not np.isfinite(q).all():
            raise ExecutionFailure('invalid measured pose before held edge insertion')
        if (not .0004 < position[2] - target['position'][2] <= .003
                or np.linalg.norm(position[:2] - np.asarray(target['position'])[:2]) > .0005
                or 1 - 2*(q[1]**2 + q[2]**2) < math.cos(math.radians(1))):
            return False
        from .support_geometry import support_margin_m, support_overlap_area_m2
        released = {event['block'] for event in self.events}
        placed = [b for b in targets if b['id'] in released]
        margin = support_margin_m(target, placed)
        return (margin is not None and -1e-6 <= margin < .001
                and support_overlap_area_m2(target, placed) >= 2*.016**2 - 1e-9)

    def insert_edge_while_held(self, target, yaw):
        """Use bounded physical insertion before unloading a supported edge."""
        if (self.perception is not None or self.node.connection_mode != 'physics'
                or self.node.episode_manifest.get('contact_profile') != 'plastic'
                or not self.held_edge_insertion_required(target)
                or getattr(self, '_top_press_active', False)):
            raise ExecutionFailure('held edge insertion requires a bounded Oracle plastic fit')
        initial_height = float(self.data.xpos[self.model.body(target['body_name']).id, 2]
                               - target['position'][2])
        self.confirm_held_part(target)
        try:
            grip = self.prepare_feedback_grip(target)
            self._held_edge_insertion_active = True
            try:
                self._seat_plastic_part(target, yaw)
                self.confirm_held_part(target)
            finally:
                self._held_edge_insertion_active = False
            seating = dict(self.last_seating_confirmation)
            unload = self.unload_gripper_for_release()
            self._feedback_force_active = False
            release = self.open_gripper_before_retreat(target)
            support = self.seating_support_evidence(target)
            if support is None:
                raise ExecutionFailure('held edge insertion lacks measured support after release')
            self.last_seating_confirmation = dict(
                seating, **support, placement_mode='held_edge_insertion',
                initial_entry_height_m=initial_height, feedback_grip_control=grip,
                jaw_preload_unloading=unload, initial_release_confirmation=release)
        finally:
            self._held_edge_insertion_active = False
            self.node.gripper_unload_force_n = None
            self._feedback_force_active = False

    def square_release_metering_required(self, target):
        """Avoid dropping a tilted upper square from its first stud contact."""
        if target.get('type') != 'brick_2x2':
            return False
        targets = self.node.episode_manifest['target_blocks']
        if target['position'][2] <= min(b['position'][2] for b in targets) + .004:
            return False
        body = self.model.body(target['body_name']).id
        position, q = self.data.xpos[body], self.data.xquat[body]
        if not np.isfinite(position).all() or not np.isfinite(q).all():
            raise ExecutionFailure('invalid measured pose before square release')
        return (float(position[2] - target['position'][2]) > .0025
                and 1 - 2*(q[1]**2 + q[2]**2) < math.cos(math.radians(1)))


    def prepare_press_gripper(self, timeout_s=1.5):
        """Settle with a small pad gap instead of striking the finger stops."""
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ExecutionFailure('press gripper timeout must be finite and positive')
        if getattr(self.node, 'gripper_unload_force_n', None) is not None:
            raise ExecutionFailure('press gripper requires unloaded force control')
        opening = .0015
        qadr = [self.model.joint(f'panda_finger_joint{i}').qposadr[0] for i in (1, 2)]
        dadr = [self.model.joint(f'panda_finger_joint{i}').dofadr[0] for i in (1, 2)]
        self.node.gripper_target = opening
        stable = 0.
        for index in range(math.ceil(timeout_s/.01)):
            self.step(.01)
            ready = (np.all(np.abs(self.data.qpos[qadr]-opening) <= .0005)
                     and np.all(np.abs(self.data.qvel[dadr]) < .002))
            stable = stable+.01 if ready else 0.
            if stable >= .05:
                return {'commanded_finger_position_m': opening,
                        'finger_positions_m': self.data.qpos[qadr].tolist(),
                        'wait_simulation_s': (index+1)*.01}
        raise ExecutionFailure('press fingers did not settle above the assembly')

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
            feedback_unload = (self.unload_gripper_for_release()
                               if getattr(self, '_feedback_force_active', False) else None)
            if feedback_unload is not None:
                self._feedback_force_active = False
            capture = (self.capture_support_edge(target, yaw)
                       if placement_mode == 'release_at_insertion_entry' else None)
            controlled_release = (self.lower_with_finger_friction(target)
                                  if placement_mode == 'release_above_press' else None)
            if (feedback_unload is None and placement_mode == 'release_at_insertion_entry'
                    and self.square_release_metering_required(target)):
                from .support_geometry import support_margin_m
                placed = {event['block'] for event in self.events}
                margin = support_margin_m(target, [b for b in self.node.episode_manifest['target_blocks']
                                                   if b['id'] in placed])
                if margin is not None and margin >= .001:
                    feedback_unload = self.unload_gripper_for_release()
                    controlled_release = {'mode': 'preload_unload_on_broad_support',
                                          'support_margin_m': margin}
                else:
                    controlled_release = self.lower_with_finger_friction(target, handover_height_m=.001)
            # A tilted fit can retain asymmetric jaw load at the lead-in.
            # Extra unloading on an already level fit regressed dense assembly.
            unload = feedback_unload or (self.unload_gripper_for_release()
                      if self.release_preload_required(target) else None)
            release = self.open_gripper_before_retreat(target)
            rotation = self.data.xmat[self.hand].reshape(3, 3)
            tip = self.data.xpos[self.hand] + rotation @ self.tool_offset
            self.move_vertical(tip + [0, 0, .06], yaw)
            press_gripper = self.prepare_press_gripper()
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
                placement_mode=placement_mode, initial_release_confirmation=release,
                jaw_preload_unloading=unload, press_gripper_confirmation=press_gripper)
            if controlled_release is not None:
                self.last_seating_confirmation['controlled_release'] = controlled_release
            if capture is not None:
                self.last_seating_confirmation['support_edge_capture'] = capture
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
            if getattr(self, '_held_edge_insertion_active', False):
                self.confirm_held_part(target)
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
        if (getattr(self, '_top_press_active', False)
                or (getattr(self, 'last_seating_confirmation', None) or {}).get(
                    'initial_release_confirmation')):
            # Unload residual insertion pressure before opening, including the
            # first release. Retreat still waits for measured, clear fingers.
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
        with self.phases.stage('planning'):
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
            self.phases.configure_plan([item['block_id'] for item in plan])
            self.node.gripper_target = .04
            self.step(.5)
            run_dir = os.environ.get('LEGO_BENCH_RUN_DIR')
            if run_dir:
                dump_json(Path(run_dir) / 'assembly_plan.json', {
                    'method': ('geometric_reverse_order_with_release_above_fallback'
                               if any(p.get('placement_mode') == 'release_above_press' for p in plan)
                               else 'assembly_by_disassembly'), 'steps': plan})
        by_id = {target['id']: target for target in targets}
        for planned in plan:
            target = dict(by_id[planned['block_id']],
                          placement_mode=planned.get('placement_mode', 'direct'))
            self.last_seating_confirmation = None
            self.node.get_logger().info(f"[EXECUTE] picking {target['id']}")
            with self.phases.stage('source_observation', target['id']):
                self.verify_released_parts()
                observed = self.observe_target(target)
                pos = np.array(observed['position'])
                yaw, place_yaw = resolve_grasp_yaws(
                    observed['yaw_rad'], target['yaw_rad'], planned['grasp_spin_deg'])
                grasp = pos + [0, 0, FALLBACK_GRASP_OFFSET_M
                               if target['placement_mode'] == 'release_above_press' else -.003]
                hover = grasp + [0, 0, .16]
            with self.phases.stage('empty_approach', target['id']):
                yaw = self.select_grasp_yaw(grasp, hover, yaw)
                self.move(hover, yaw)
                self.prepare_grasp_opening(target['type'], observed['yaw_rad'], yaw)
            with self.phases.stage('grasp_approach', target['id']):
                self.approach_grasp(grasp, yaw)
                try:
                    self.confirm_source_unchanged(target, observed)
                except SourceApproachDisturbance:
                    if self.node.episode_manifest.get('contact_profile') != 'plastic':
                        raise
                    observed, pos, yaw, place_yaw, grasp, hover = self.recover_source_approach(
                        target, planned, observed, hover, yaw)
                self.phases.evidence(pose_confirmed=True)
            with self.phases.stage('grasp_closure', target['id']):
                self.close_gripper_for_grasp(target)
                checked = self.perception is None and self.node.connection_mode == 'physics'
                if checked:
                    self.confirm_held_part(target)
                self.phases.evidence(closure_complete=True, grasp_confirmed=True if checked else None)
            with self.phases.stage('lift', target['id']):
                self.move_vertical(hover, yaw)
                if self.perception is None:
                    lifted = self.node.current_block_state()['blocks'][target['id']]['position'][2]
                    if lifted < pos[2] + .06:
                        raise ExecutionFailure(f"grasp failed: {target['id']} (lift {lifted-pos[2]:.4f} m)")
                    if self.node.connection_mode == 'physics':
                        self.confirm_held_part(target)
                    self.phases.evidence(measured_lift_m=float(lifted-pos[2]))
            goal = np.array(target['position']) + [0, 0, -.003]
            placed_ids = {event['block'] for event in self.events}
            placed_targets = [b for b in targets if b['id'] in placed_ids]
            with self.phases.stage('loaded_transfer', target['id']):
                if self.perception is None and self.node.connection_mode == 'physics':
                    self.wake_supports_before_approach()
                self.move(goal + [0, 0, .16], place_yaw)
            vertical = requires_vertical_approach(target, placed_targets)
            approach = self.move_vertical if vertical else self.move
            with self.phases.stage('alignment_and_insertion', target['id']):
                if self.node.connection_mode == 'physics':
                    place_yaw = self.place_physical(target)
                    vertical = True
                else:
                    approach(goal, place_yaw)
            with self.phases.stage('release', target['id']):
                release = self.open_gripper_before_retreat(target)
                self.phases.evidence(release_confirmation=release)
            with self.phases.stage('retreat', target['id']):
                self.move(goal + [0, 0, .16], place_yaw)
            with self.phases.stage('placement_verification', target['id']):
                self.events.append({'block': target['id'], 'status': 'released',
                                    'approach': 'vertical' if vertical else 'joint',
                                    'placement_yaw_rad': float(place_yaw),
                                    'planned_grasp_spin_deg': planned['grasp_spin_deg'],
                                    'executed_pick_yaw_rad': float(yaw),
                                    'release_confirmation': release,
                                    'seating_confirmation': getattr(self, 'last_seating_confirmation', None)})
                if self.perception is None and self.node.connection_mode == 'physics':
                    self.allow_resting_part_sleep(target)
                else:
                    self.verify_released_parts()
                self.phases.evidence(placement_verified=True)
            if run_dir:
                dump_json(Path(run_dir) / 'progress.json', {
                    'finished': False, 'events': self.events,
                    'state_machine': self.phases.summary(),
                    'score_at_checkpoint': self.node.benchmark_result(),
                    'actual_state': self.node.current_block_state()})
            self.node.get_logger().info(f"[EXECUTE] released {target['id']}")
        with self.phases.stage('final_stability'):
            if self.perception is None and self.node.connection_mode == 'physics':
                self.settle_awake_parts(targets)
            else:
                self.step(1.)
                self.verify_released_parts()
            self.phases.evidence(placement_verified=True)


def execute(output_dir, backend='oracle', speed_scale=1.5):
    from pathlib import Path
    import signal
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from .mj_bridge3 import MuJoCoActionServer
    # Capture the executed source at episode start, rather than assigning a
    # later working-tree revision to historical results in a report.
    source = Path(__file__).resolve().parent
    fingerprints = {name: hashlib.sha256((source / name).read_bytes()).hexdigest()
                    for name in ('reference_executor.py', 'execution_phases.py', 'support_geometry.py', 'mj_bridge3.py', 'benchmark_core.py',
                                 'perception.py', 'panda.xml', 'hand.xml', 'part_registry.yaml', 'scene_builder.py',
                                 'assembly_planner.py', 'scene_template.xml', 'recorded_layout.py', 'plastic_contact.py')}
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    previous_stop_handler = signal.getsignal(signal.SIGTERM)
    def stop_execution(signum, frame):
        raise ExecutionFailure('execution stopped by process or work deadline')
    signal.signal(signal.SIGTERM, stop_execution)
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
        except (Exception, KeyboardInterrupt) as exc:
            executor.hold_after_failure(exc)
            error = str(exc) or type(exc).__name__
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
                       'motion_replans': executor.motion_replans,
                       'grasp_recoveries': executor.grasp_recoveries,
                       'phase_timings': executor.phases.records,
                       'failed_phase': executor.phases.failed_phase,
                       'state_machine': executor.phases.summary(),
                       'failure_contacts': getattr(executor, 'failure_contacts', []),
                       'final_contact_observation': getattr(executor, 'final_contact_observation', None),
                       'robot_base_position_m': node.episode_manifest.get('robot_base_position_m', [0.,0.,0.]),
                       'generated_model_sha256': {name: hashlib.sha256((Path(output_dir)/name).read_bytes()).hexdigest()
                                                  for name in ('scene.xml','panda.xml')}})
        result['success'] = bool(result['success'] and error is None
                                 and executor.phases.status == 'completed')
        dump_json(Path(output_dir) / 'actual_state.json', node.current_block_state())
        dump_json(Path(output_dir) / 'result.json', result)
        dump_json(Path(output_dir) / 'progress.json', {
            'finished': True, 'success': result['success'], 'events': executor.events,
            'score_at_checkpoint': result, 'actual_state': node.current_block_state()})
        return result
    finally:
        signal.signal(signal.SIGTERM, previous_stop_handler)
        if node is not None:
            if node.camera_renderer is not None:
                node.camera_renderer.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
