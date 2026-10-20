"""Safety contracts for the optional high-placement friction controller."""
from types import SimpleNamespace
import numpy as np
import pytest
from mj_bridge.reference_executor import OracleExecutor, ExecutionFailure


TARGET = {'id': 'test', 'body_name': 'test', 'position': [0., 0., 0.]}


def fixture():
    executor = OracleExecutor.__new__(OracleExecutor)
    executor.perception = None
    executor.qadr = np.array([0])
    executor.model = SimpleNamespace(
        body=lambda name: SimpleNamespace(id={'test': 0, 'left_finger': 1, 'right_finger': 2}[name]),
        joint=lambda name: SimpleNamespace(qposadr=[int(name[-1])]),
        actuator=lambda name: SimpleNamespace(id=int(name[-1])-1),
        actuator_gainprm=np.array([[500.], [500.]]),
        geom_bodyid=np.array([0, 1, 2]), body_mass=np.array([.012, .1, .1]),
        opt=SimpleNamespace(gravity=np.array([0., 0., -9.81])))
    executor.data = SimpleNamespace(
        contact=[SimpleNamespace(geom1=0, geom2=i, dist=0., friction=np.array([1.])) for i in (1, 2)],
        qpos=np.array([.2, .016, .016]), xpos=np.array([[0., 0., .016]]),
        xquat=np.array([[1., 0., 0., 0.]]))
    executor.node = SimpleNamespace(connection_mode='physics', episode_manifest={'contact_profile': 'plastic'},
                                    target_qpos=np.zeros(3), gripper_target=0.)
    executor.part_contacts_observable = lambda _: True
    executor.step = lambda _: None
    return executor


@pytest.mark.parametrize('failure', ['vision', 'snap', 'loose', 'no_contact', 'bad_gain', 'nan_mu'])
def test_release_rejects_unsupported_or_invalid_contact_before_motion(failure):
    e = fixture()
    if failure == 'vision': e.perception = object()
    if failure == 'snap': e.node.connection_mode = 'snap'
    if failure == 'loose': e.node.episode_manifest['contact_profile'] = 'loose'
    if failure == 'no_contact': e.data.contact = []
    if failure == 'bad_gain': e.model.actuator_gainprm[0, 0] = 0.
    if failure == 'nan_mu': e.data.contact[0].friction[0] = np.nan
    e.step = lambda _: pytest.fail('invalid setup must not issue a physics step')
    with pytest.raises(ExecutionFailure): e.lower_with_finger_friction(TARGET)


@pytest.mark.parametrize('bad_state', ['xy', 'tilt', 'nan', 'asleep', 'below'])
def test_release_rejects_misalignment_and_unobservable_state(bad_state):
    e = fixture()
    if bad_state == 'xy': e.data.xpos[0, 0] = .002
    if bad_state == 'tilt': e.data.xquat[0] = [np.cos(.1), np.sin(.1), 0., 0.]
    if bad_state == 'nan': e.data.xpos[0, 0] = np.nan
    if bad_state == 'asleep': e.part_contacts_observable = lambda _: False
    if bad_state == 'below': e.data.xpos[0, 2] = -.001
    e.step = lambda _: pytest.fail('bad state must not be advanced')
    with pytest.raises(ExecutionFailure): e.lower_with_finger_friction(TARGET)


def test_release_only_commands_bounded_setpoints_and_reports_handover():
    e = fixture()
    before = e.data.qpos.copy()
    commands = []
    def step(seconds):
        commands.append(e.node.gripper_target)
        e.data.xpos[0, 2] -= .01 * seconds  # measured slip provided by the test
    e.step = step
    report = e.lower_with_finger_friction(TARGET)
    assert 0 <= report['handover_height_m'] <= .004
    assert report['duration_s'] <= 2.5
    assert all(0 <= command <= .04 for command in commands)
    np.testing.assert_array_equal(e.data.qpos, before)
    np.testing.assert_array_equal(e.node.target_qpos[e.qadr], before[e.qadr])


def test_stuck_part_times_out_without_increasing_force_limit():
    e = fixture()
    commands = []
    e.step = lambda _: commands.append(e.node.gripper_target)
    with pytest.raises(ExecutionFailure, match='handover height'):
        e.lower_with_finger_friction(TARGET)
    assert len(commands) == 1250
    assert all(.016 - .5/500 <= command <= .016 for command in commands)


def test_failed_metering_prevents_opening_retreat_or_press():
    e = fixture()
    e.speed_scale = 1.5
    e.lower_with_finger_friction = lambda _: (_ for _ in ()).throw(ExecutionFailure('metering failed'))
    e.open_gripper_before_retreat = lambda _: pytest.fail('must not open after metering failure')
    with pytest.raises(ExecutionFailure, match='metering failed'):
        e.release_and_press(TARGET, 0., 'release_above_press')
    assert e._top_press_active is False


def unload_fixture():
    e = OracleExecutor.__new__(OracleExecutor)
    e.qadr = np.array([0])
    e.node = SimpleNamespace(target_qpos=np.zeros(3), gripper_control=0.)
    e.model = SimpleNamespace(
        joint=lambda name: SimpleNamespace(qposadr=[int(name[-1])]),
        actuator=lambda name: SimpleNamespace(id=int(name[-1])-1),
        actuator_gainprm=np.array([[100.], [100.]]))
    e.data = SimpleNamespace(qpos=np.array([.2, .015, .016]), actuator_force=np.array([-5., -4.]))
    return e


def test_unload_preserves_measured_state_and_does_not_reapply_closed_command():
    e = unload_fixture()
    before = e.data.qpos.copy()
    forces = []
    e.step = lambda dt: forces.append(e.node.gripper_unload_force_n)
    report = e.unload_gripper_for_release()
    assert all(0 <= b <= a <= 5 for a, b in zip(forces, forces[1:]))
    assert forces[-1] == 0
    assert e.node.gripper_unload_force_n is None
    assert e.node.gripper_control == pytest.approx(.0155)
    np.testing.assert_array_equal(e.data.qpos, before)
    assert report['duration_s'] <= .25


def test_unload_clears_force_override_after_contact_stop():
    e = unload_fixture()
    def fail(_):
        raise ExecutionFailure('contact stop')
    e.step = fail
    with pytest.raises(ExecutionFailure, match='contact stop'):
        e.unload_gripper_for_release()
    assert e.node.gripper_unload_force_n is None


@pytest.mark.parametrize('field', ['force', 'gain'])
def test_unload_rejects_invalid_actuator_state_without_stepping(field):
    e = unload_fixture()
    if field == 'force':
        e.data.actuator_force[0] = np.nan
    else:
        e.model.actuator_gainprm[0, 0] = 0.
    e.step = lambda dt: pytest.fail('invalid actuator state must not advance')
    with pytest.raises(ExecutionFailure, match='invalid finger'):
        e.unload_gripper_for_release()


@pytest.mark.parametrize('kind,height,tilt,expected', [
    ('brick_4x2', .004, 2., True),
    ('brick_4x2', .004, .85, False),
    ('brick_4x2', .001, 2., False),
    ('brick_2x2', .004, 2., False),
])
def test_preload_unloading_preserves_level_seated_and_square_releases(kind, height, tilt, expected):
    e = OracleExecutor.__new__(OracleExecutor)
    e.model = SimpleNamespace(body=lambda _: SimpleNamespace(id=0))
    angle = np.radians(tilt) / 2
    e.data = SimpleNamespace(xpos=np.array([[0., 0., height]]),
                             xquat=np.array([[np.cos(angle), np.sin(angle), 0., 0.]]))
    target = {'body_name': 'part', 'type': kind, 'position': [0., 0., 0.]}
    assert bool(e.release_preload_required(target)) is expected


def test_preload_unloading_rejects_nonfinite_pose():
    e = OracleExecutor.__new__(OracleExecutor)
    e.model = SimpleNamespace(body=lambda _: SimpleNamespace(id=0))
    e.data = SimpleNamespace(xpos=np.array([[0., 0., np.nan]]), xquat=np.array([[1., 0., 0., 0.]]))
    with pytest.raises(ExecutionFailure, match='invalid measured pose'):
        e.release_preload_required({'body_name': 'part', 'type': 'brick_4x2', 'position': [0., 0., 0.]})
