from types import SimpleNamespace
import numpy as np
import pytest
from mj_bridge.execution_phases import PhaseRecorder
from mj_bridge.reference_executor import OracleExecutor, ExecutionFailure


def test_failed_stage_records_reason_and_preserves_exception():
    snapshots = []
    recorder = PhaseRecorder(lambda: 0., on_update=lambda records: snapshots.append(dict(records[-1])))
    with pytest.raises(ValueError, match='contact lost'):
        with recorder.stage('lift', 'brick'):
            raise ValueError('contact lost')
    assert recorder.failed_phase == 'lift'
    assert recorder.records[0]['status'] == 'failed'
    assert recorder.records[0]['block'] == 'brick'
    assert recorder.records[0]['wall_s'] >= 0
    assert [item['status'] for item in snapshots] == ['running', 'failed']


def fixture(monkeypatch, both=True, velocity=0., part_velocity=None):
    e = OracleExecutor.__new__(OracleExecutor)
    e.perception = None
    e.node = SimpleNamespace(connection_mode='physics', gripper_target=.04, episode_manifest={})
    e.model = SimpleNamespace(
        body=lambda name: SimpleNamespace(id={'part': 0, 'left_finger': 1, 'right_finger': 2}[name]),
        joint=lambda name: SimpleNamespace(dofadr=[int(name[-1])-1]),
        body_mass=np.array([.012]), opt=SimpleNamespace(gravity=np.array([0., 0., -9.81])),
        geom_bodyid=np.array([0, 1, 2]))
    e.data = SimpleNamespace(qvel=np.array([velocity, velocity]), contact=[
        SimpleNamespace(geom1=0, geom2=i, dist=0., friction=np.array([1.]))
        for i in ((1, 2) if both else (1,))])
    steps = []
    e.step = lambda seconds: steps.append(seconds)
    monkeypatch.setattr('mj_bridge.reference_executor.mujoco.mj_contactForce',
                        lambda model, data, index, force: force.__setitem__(0, 1.))
    monkeypatch.setattr('mj_bridge.reference_executor.mujoco.mj_objectVelocity',
                        lambda model, data, kind, body, result, local: result.__setitem__(
                            slice(None), np.zeros(6) if part_velocity is None else part_velocity))
    return e, steps


def test_grasp_advances_early_only_after_stable_bilateral_load(monkeypatch):
    e, steps = fixture(monkeypatch)
    e.close_gripper_for_grasp({'id': 'brick', 'body_name': 'part'})
    assert .08 <= sum(steps) < .7
    assert e.node.gripper_target == 0.


@pytest.mark.parametrize('both,velocity', [(False, 0.), (True, .01)])
def test_unstable_or_unilateral_grasp_stops_before_lifting(monkeypatch, both, velocity):
    e, steps = fixture(monkeypatch, both, velocity)
    with pytest.raises(ExecutionFailure, match='stable two-finger contact'):
        e.close_gripper_for_grasp({'id': 'brick', 'body_name': 'part'})
    assert sum(steps) == pytest.approx(1.2)


def test_stable_window_resets_after_contact_loss(monkeypatch):
    e, steps = fixture(monkeypatch)
    contacts = e.data.contact
    def step(seconds):
        steps.append(seconds)
        e.data.contact = [] if len(steps) == 5 else contacts
    e.step = step
    e.close_gripper_for_grasp({'id': 'brick', 'body_name': 'part'})
    assert len(steps) >= 13


@pytest.mark.parametrize('velocity', [[.2, 0., 0., 0., 0., 0.], [0., 0., 0., .01, 0., 0.]])
def test_moving_part_is_not_ready_even_when_fingers_are_stationary(monkeypatch, velocity):
    e, _ = fixture(monkeypatch, part_velocity=velocity)
    with pytest.raises(ExecutionFailure, match='stable two-finger contact'):
        e.close_gripper_for_grasp({'id': 'brick', 'body_name': 'part'})


def test_plastic_retains_validated_settling_dwell(monkeypatch):
    e, steps = fixture(monkeypatch)
    e.node.episode_manifest = {"contact_profile": "plastic"}
    e.close_gripper_for_grasp({"id": "brick", "body_name": "part"})
    assert steps == [.7]
