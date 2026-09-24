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
