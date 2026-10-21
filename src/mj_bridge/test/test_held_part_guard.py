"""A dropped part must not produce another alignment motion."""
from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

from mj_bridge.reference_executor import ExecutionFailure, OracleExecutor


def executor_with_contacts(monkeypatch, forces, observable=True):
    executor = OracleExecutor.__new__(OracleExecutor)
    executor.perception = None
    executor.qadr = np.array([0, 1])
    ids = {'part': 1, 'left_finger': 2, 'right_finger': 3}
    executor.model = SimpleNamespace(
        body=lambda name: SimpleNamespace(id=ids[name]),
        geom_bodyid=np.array([1, 2, 3]))
    executor.data = SimpleNamespace(
        qpos=np.array([.2, -.3]),
        contact=[SimpleNamespace(geom1=0, geom2=i, dist=-.00001)
                 for i in (1, 2)])
    executor.node = SimpleNamespace(target_qpos=np.array([1., 1.]))
    executor.part_contacts_observable = lambda _: observable
    executor.step = lambda _: None
    def contact_force(model, data, index, output):
        output[0] = forces[index]
    monkeypatch.setattr(mujoco, 'mj_contactForce', contact_force)
    return executor


TARGET = dict(id='part', body_name='part', type='brick_2x2',
              position=[.374, .366, .0837], yaw_rad=0.)


@pytest.mark.parametrize('forces,observable', [([0., 0.], True),
                                              ([5., 0.], True),
                                              ([5., 5.], False)])
def test_missing_grasp_stops_before_observation_or_motion(monkeypatch, forces, observable):
    executor = executor_with_contacts(monkeypatch, forces, observable)
    waits = []
    executor.step = waits.append
    # No observation or motion methods: calling either would fail the test.
    with pytest.raises(ExecutionFailure, match='lost the two-finger grasp: part'):
        executor.align_held_part(TARGET)
    assert sum(waits) == pytest.approx(.08)
    np.testing.assert_array_equal(executor.node.target_qpos, executor.data.qpos)


def test_loaded_grasp_adds_no_simulation_steps_or_control_changes(monkeypatch):
    executor = executor_with_contacts(monkeypatch, [5., 5.])
    executor.step = lambda _: pytest.fail('a loaded grasp must not add delay')
    executor.confirm_held_part(TARGET)
    np.testing.assert_array_equal(executor.node.target_qpos, [1., 1.])


def test_contact_transition_can_recover_while_arm_holds(monkeypatch):
    forces = [5., 0.]
    executor = executor_with_contacts(monkeypatch, forces)
    waits = []
    def step(seconds):
        waits.append(seconds)
        forces[1] = 5.
    executor.step = step
    executor.confirm_held_part(TARGET)
    assert waits == [.01]
    np.testing.assert_array_equal(executor.node.target_qpos, executor.data.qpos)


def test_alignment_rechecks_grasp_after_each_correction(monkeypatch):
    forces = [5., 5.]
    executor = executor_with_contacts(monkeypatch, forces)
    executor.hand = 0
    executor.tool_offset = np.zeros(3)
    executor.data.xmat = np.eye(3).reshape(1, 9)
    executor.data.xpos = np.zeros((1, 3))
    executor.node.current_block_state = lambda: {'blocks': {
        'part': {'position': [.37, .366, .0837], 'yaw_rad': 0.}}}
    motions = []
    def move(position, yaw):
        motions.append(position)
        forces[:] = [0., 0.]
    executor.move = move
    with pytest.raises(ExecutionFailure, match='lost the two-finger grasp'):
        executor.align_held_part(TARGET)
    assert len(motions) == 1


@pytest.mark.parametrize('failure', [None, 'hover', 'grasp', 'both'])
def test_symmetric_grasp_fallback_preserves_axis_and_requires_both_poses(failure):
    import math
    executor = OracleExecutor.__new__(OracleExecutor)
    executor.hand = 0
    executor.data = SimpleNamespace(xmat=np.eye(3).reshape(1, 9))
    executor.node = SimpleNamespace(episode_manifest={'contact_profile': 'plastic'})
    grasp, hover = np.array([.46, -.08, .055]), np.array([.46, -.08, .215])
    commands = []
    def ik(position, yaw, **kwargs):
        commands.append((position.copy(), yaw))
        if failure == 'both' or (yaw == 0. and (
                failure == 'hover' and np.array_equal(position, hover)
                or failure == 'grasp' and np.array_equal(position, grasp))):
            raise ExecutionFailure('IK did not converge')
        return np.zeros(7)
    executor.ik = ik
    executor.joint_path_has_collision = lambda start, end: False
    if failure == 'both':
        with pytest.raises(ExecutionFailure, match='neither symmetric'):
            executor.select_grasp_yaw(grasp, hover, 0.)
    else:
        yaw = executor.select_grasp_yaw(grasp, hover, 0.)
        assert yaw == (0. if failure is None else -math.pi)
        assert math.sin(yaw) == pytest.approx(0., abs=1e-12)
        assert np.array_equal(commands[-1][0], grasp)


def airborne_fixture(monkeypatch):
    e = executor_with_contacts(monkeypatch, [5., 5.])
    e.data.xquat = np.tile([1., 0., 0., 0.], (4, 1))
    e.data.xpos = np.zeros((4, 3))
    e.data.xpos[1, 2] = .019
    e.data.xmat = np.tile(np.eye(3).reshape(9), (4, 1))
    e.hand = 0
    e.tool_offset = np.zeros(3)
    e.speed_scale = 1.5
    return e


def test_airborne_return_preserves_orientation_and_restores_motion_guard(monkeypatch):
    e = airborne_fixture(monkeypatch)
    def move(position, yaw, target_rotation):
        assert e._airborne_recovery_target is not None
        np.testing.assert_array_equal(target_rotation, np.eye(3))
        assert position[2] == pytest.approx(.011)
        e.data.xpos[1, 2] = .030
    e.move_vertical = move
    e.return_to_airborne_clearance(dict(TARGET, position=[0., 0., 0.]), 0.)
    assert e._airborne_recovery_target is None
    assert e.speed_scale == 1.5


def test_airborne_return_refuses_near_seated_part_without_motion(monkeypatch):
    e = airborne_fixture(monkeypatch)
    e.data.xpos[1, 2] = .005
    e.move_vertical = lambda *_: pytest.fail('near-seated part must not be lifted by recovery')
    with pytest.raises(ExecutionFailure, match='bounded free-space height'):
        e.return_to_airborne_clearance(dict(TARGET, position=[0., 0., 0.]), 0.)


def test_airborne_return_clears_guard_after_failure(monkeypatch):
    e = airborne_fixture(monkeypatch)
    def fail(*args, **kwargs):
        raise ExecutionFailure('contact stop')
    e.move_vertical = fail
    with pytest.raises(ExecutionFailure, match='contact stop'):
        e.return_to_airborne_clearance(dict(TARGET, position=[0., 0., 0.]), 0.)
    assert e._airborne_recovery_target is None
    assert e.speed_scale == 1.5


def test_airborne_recovery_requires_no_support_contact(monkeypatch):
    e = airborne_fixture(monkeypatch)
    e.model.geom_bodyid = np.array([1, 2, 3, 0])
    e.data.contact.append(SimpleNamespace(geom1=0, geom2=3, dist=0.))
    with pytest.raises(ExecutionFailure, match='external contact'):
        e.check_airborne_grasp(dict(TARGET, position=[0., 0., 0.]))


def test_airborne_transient_holds_arm_and_expires_after_ten_ms(monkeypatch):
    e = airborne_fixture(monkeypatch)
    e.model.opt = SimpleNamespace(timestep=.0005)
    e.data.contact = e.data.contact[:1]
    target = dict(TARGET, position=[0., 0., 0.])
    for _ in range(20):
        e.check_airborne_grasp(target, allow_transient=True)
    np.testing.assert_array_equal(e.node.target_qpos, e.data.qpos)
    with pytest.raises(ExecutionFailure, match='two-finger grasp'):
        e.check_airborne_grasp(target, allow_transient=True)


def test_airborne_contact_recovery_resets_transient_budget(monkeypatch):
    e = airborne_fixture(monkeypatch)
    e.model.opt = SimpleNamespace(timestep=.0005)
    contacts = e.data.contact[:]
    e.data.contact = contacts[:1]
    target = dict(TARGET, position=[0., 0., 0.])
    e.check_airborne_grasp(target, allow_transient=True)
    e.data.contact = contacts
    e.check_airborne_grasp(target, allow_transient=True)
    assert e._airborne_missing_s == 0.


def test_airborne_transient_never_bypasses_lateral_guard(monkeypatch):
    e = airborne_fixture(monkeypatch)
    e.model.opt = SimpleNamespace(timestep=.0005)
    e.data.xpos[1, 0] = .003
    with pytest.raises(ExecutionFailure, match='observable pose bounds'):
        e.check_airborne_grasp(dict(TARGET, position=[0., 0., 0.]), allow_transient=True)


@pytest.mark.parametrize('recover', [True, False])
def test_airborne_lift_uses_measured_clearance_with_bounded_retry(monkeypatch, recover):
    e = airborne_fixture(monkeypatch)
    commands=[]
    def move(position, yaw, target_rotation):
        commands.append(position.copy())
        e.data.xpos[1,2] = .030 if recover and len(commands)==2 else .02494
    e.move_vertical=move
    target=dict(TARGET,position=[0.,0.,0.])
    if recover:
        e.return_to_airborne_clearance(target,0.)
    else:
        with pytest.raises(ExecutionFailure,match='did not establish clearance'):
            e.return_to_airborne_clearance(target,0.)
    assert len(commands)==2
    assert e._airborne_recovery_target is None
    assert e.speed_scale==1.5
def test_feedback_cannot_chase_a_dropped_part():
    executor = OracleExecutor.__new__(OracleExecutor)
    executor.perception = None
    executor.node = SimpleNamespace(current_block_state=lambda: {'blocks': {'part': {
        'position': [0., 0., .03], 'quaternion_wxyz': [1., 0., 0., 0.]}}})
    def lost(_):
        raise ExecutionFailure('two-finger grasp lost')
    executor.confirm_held_part = lost
    executor.ik = lambda *args, **kwargs: pytest.fail('a dropped part cannot issue another IK goal')
    with pytest.raises(ExecutionFailure, match='grasp lost'):
        executor.descend_with_pose_feedback(dict(TARGET, position=[0., 0., 0.]), 0.)


@pytest.mark.parametrize('fault', ['new_tilt', 'failure'])
def test_feedback_interrupts_lowering_and_clears_the_tilt_guard(fault):
    from mj_bridge.reference_executor import _GripDrift
    executor = OracleExecutor.__new__(OracleExecutor)
    executor.perception = None
    executor.prepare_feedback_grip = lambda _: None
    executor.hand = 0
    executor.qadr = np.array([0])
    executor.tool_offset = np.zeros(3)
    state = {'position': [0., 0., .03], 'quaternion_wxyz': [1., 0., 0., 0.]}
    executor.data = SimpleNamespace(qpos=np.zeros(1), xpos=np.array([[0., 0., .03]]),
                                    xmat=np.eye(3).reshape(1, 9))
    executor.node = SimpleNamespace(target_qpos=np.zeros(1),
                                    current_block_state=lambda: {'blocks': {'part': state}})
    executor.confirm_held_part = lambda _: 0.
    executor.ik = lambda *args, **kwargs: np.zeros(1)
    target = dict(TARGET, position=[0., 0., 0.])
    pauses = []
    def step(_):
        if getattr(executor, '_descent_guard_target', None) is target:
            pauses.append(True)
            if fault == 'failure':
                raise ExecutionFailure('contact stop')
            state['quaternion_wxyz'] = [np.cos(.01), np.sin(.01), 0., 0.]
            raise _GripDrift()
        state.update(position=[0., 0., .001], quaternion_wxyz=[1., 0., 0., 0.])
    executor.step = step
    executor.held_edge_insertion_required = lambda _: False
    executor.release_and_press = lambda *args: setattr(executor, 'last_seating_confirmation', {})
    if fault == 'failure':
        with pytest.raises(ExecutionFailure, match='contact stop'):
            executor.descend_with_pose_feedback(target, 0.)
        assert not hasattr(executor, 'last_seating_confirmation')
    else:
        executor.descend_with_pose_feedback(target, 0.)
        assert executor.last_seating_confirmation['pose_feedback']['steps'] == 2
    assert len(pauses) == 1 and executor._descent_guard_target is None
