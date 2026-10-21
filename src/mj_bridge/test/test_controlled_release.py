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
    ('brick_2x2', .004, 2., True),
])
def test_preload_unloading_is_limited_to_tilted_high_entries(kind, height, tilt, expected):
    e = OracleExecutor.__new__(OracleExecutor)
    e.model = SimpleNamespace(body=lambda _: SimpleNamespace(id=0))
    angle = np.radians(tilt) / 2
    e.data = SimpleNamespace(xpos=np.array([[0., 0., height]]),
                             xquat=np.array([[np.cos(angle), np.sin(angle), 0., 0.]]))
    target = {'body_name': 'part', 'type': kind, 'position': [0., 0., 0.]}
    e.node = SimpleNamespace(episode_manifest={'target_blocks': [target]})
    assert bool(e.release_preload_required(target)) is expected


@pytest.mark.parametrize('kind,layer,height,tilt,expected', [
    ('brick_2x2', 0., .00232, 1.44, True),
    ('brick_4x2', 0., .00232, 1.44, False),
    ('brick_2x2', .0192, .00232, 1.44, False),
    ('brick_2x2', 0., .00232, .8, False),
    ('brick_2x2', 0., .0019, 1.44, False),
    ('brick_2x2', .0192, .0027, 1.44, True),
])
def test_early_unload_is_limited_to_a_tilted_base_square(kind, layer, height, tilt, expected):
    e = OracleExecutor.__new__(OracleExecutor)
    target = dict(body_name='part', type=kind, position=[0., 0., .0646+layer])
    lower = dict(position=[0., 0., .0646])
    e.node = SimpleNamespace(episode_manifest={'target_blocks': [lower, target]})
    e.model = SimpleNamespace(body=lambda _: SimpleNamespace(id=0))
    angle = np.radians(tilt)/2
    e.data = SimpleNamespace(xpos=np.array([[0., 0., target['position'][2]+height]]),
        xquat=np.array([[np.cos(angle), np.sin(angle), 0., 0.]]))
    assert bool(e.release_preload_required(target)) is expected


def test_preload_unloading_rejects_nonfinite_pose():
    e = OracleExecutor.__new__(OracleExecutor)
    e.model = SimpleNamespace(body=lambda _: SimpleNamespace(id=0))
    e.data = SimpleNamespace(xpos=np.array([[0., 0., np.nan]]), xquat=np.array([[1., 0., 0., 0.]]))
    with pytest.raises(ExecutionFailure, match='invalid measured pose'):
        e.release_preload_required({'body_name': 'part', 'type': 'brick_4x2', 'position': [0., 0., 0.]})


@pytest.mark.parametrize('handover', [.001, .004])
def test_metering_reaches_requested_measured_height_without_changing_state_directly(handover):
    e = fixture()
    before = e.data.qpos.copy()
    def step(seconds):
        e.data.xpos[0, 2] -= .01 * seconds
    e.step = step
    result = e.lower_with_finger_friction(TARGET, handover_height_m=handover)
    assert handover - .000021 <= result['handover_height_m'] <= handover
    np.testing.assert_array_equal(e.data.qpos, before)


@pytest.mark.parametrize('handover', [0., .005, float('nan')])
def test_metering_rejects_invalid_handover_before_motion(handover):
    e = fixture()
    e.step = lambda _: pytest.fail('invalid handover must not command motion')
    with pytest.raises(ExecutionFailure, match='bounded handover'):
        e.lower_with_finger_friction(TARGET, handover_height_m=handover)


@pytest.mark.parametrize('kind,layer,height,tilt,expected', [
    ('brick_2x2', .0192, .0035, 2.9, True),
    ('brick_2x2', 0., .0035, 2.9, False),
    ('brick_2x2', .0192, .001, 2.9, False),
    ('brick_2x2', .0192, .0035, .5, False),
    ('brick_4x2', .0192, .0035, 2.9, False),
])
def test_square_metering_is_limited_to_tilted_high_upper_release(kind, layer, height, tilt, expected):
    e = fixture()
    target = dict(TARGET, type=kind, position=[0., 0., layer], yaw_rad=0.)
    e.events = [dict(block='lower')]
    e.node.episode_manifest['target_blocks'] = [
        dict(TARGET, id='lower', type='brick_2x2', position=[.016, 0., 0.], yaw_rad=0.), target]
    e.data.xpos[0, 2] = layer + height
    angle = np.radians(tilt)/2
    e.data.xquat[0] = [np.cos(angle), np.sin(angle), 0., 0.]
    assert e.square_release_metering_required(target) == expected
def test_feedback_grip_reduces_motor_preload_gradually_without_moving_the_part():
    e = fixture()
    e.data.actuator_force = np.array([-5., -5.])
    e.confirm_held_part = lambda _: 0.
    commands = []
    e.step = lambda seconds: commands.append((seconds, e.node.gripper_unload_force_n))
    before = e.data.qpos.copy()
    result = e.prepare_feedback_grip(TARGET)
    assert sum(row[0] for row in commands) == pytest.approx(.1)
    assert all(a[1] > b[1] for a,b in zip(commands, commands[1:]))
    assert commands[-1][1] == 1.5
    assert result['holding_normal_force_command_n'] == 1.5
    np.testing.assert_array_equal(before, e.data.qpos)


def test_invalid_feedback_motor_force_stops_before_any_command():
    e = fixture()
    e.data.actuator_force = np.array([float('nan'), -5.])
    e.step = lambda _: pytest.fail('invalid actuator state must stop before stepping')
    with pytest.raises(ExecutionFailure, match='invalid finger force'):
        e.prepare_feedback_grip(TARGET)
    assert not getattr(e, '_feedback_force_active', False)
def test_feedback_force_is_unloaded_before_release_and_no_retreat_on_failure():
    e = fixture()
    e.speed_scale = 1.5
    e._feedback_force_active = True
    e.node.gripper_unload_force_n = 1.5
    events = []
    def unload():
        events.append('unload')
        e.node.gripper_unload_force_n = None
        return {'initial_normal_force_n': 1.5, 'duration_s': .21}
    def fail_release(_):
        events.append('release')
        raise ExecutionFailure('release not confirmed')
    e.unload_gripper_for_release = unload
    e.open_gripper_before_retreat = fail_release
    e.move_vertical = lambda *args: pytest.fail('a failed release cannot retreat')
    with pytest.raises(ExecutionFailure, match='not confirmed'):
        e.release_and_press(TARGET, 0., 'release_before_seating')
    assert events == ['unload', 'release']
    assert not e._feedback_force_active and not e._top_press_active
    assert e.node.gripper_unload_force_n is None and e.speed_scale == 1.5
@pytest.mark.parametrize('fault', ['none','far','moving','force_override'])
def test_press_gripper_requires_a_stable_measured_gap(fault):
    e = OracleExecutor.__new__(OracleExecutor)
    e.node = SimpleNamespace(gripper_target=.04,
                            gripper_unload_force_n=1.5 if fault=='force_override' else None)
    e.model = SimpleNamespace(joint=lambda name: SimpleNamespace(
        qposadr=[int(name[-1])-1], dofadr=[int(name[-1])-1]))
    e.data = SimpleNamespace(qpos=np.array([.04,.04]),qvel=np.zeros(2))
    steps = []
    def step(dt):
        steps.append(dt)
        e.data.qpos[:] = .02 if fault=='far' else .0016
        e.data.qvel[:] = .003 if fault=='moving' else 0.
    e.step = step
    if fault=='none':
        result = e.prepare_press_gripper(timeout_s=.06)
        assert result['commanded_finger_position_m']==.0015
        assert result['finger_positions_m']==[.0016,.0016]
        assert result['wait_simulation_s']>=.05
    else:
        with pytest.raises(ExecutionFailure): e.prepare_press_gripper(timeout_s=.06)
        if fault=='force_override': assert not steps and e.node.gripper_target==.04
        else: assert sum(steps)<=.061


@pytest.mark.parametrize('offset,expected', [(0.,'unload'),(.016,'meter')])
def test_upper_square_handover_uses_the_measured_support_region(offset, expected):
    e = fixture()
    e.speed_scale = 1.5
    target = dict(TARGET,type='brick_2x2',position=[0.,0.,.0192],yaw_rad=0.)
    lower = dict(target,id='lower',position=[offset,0.,0.])
    e.events=[dict(block='lower')]
    e.node.episode_manifest['target_blocks']=[lower,target]
    e.data.xpos[0,2]=.0227
    angle=np.radians(2.)/2
    e.data.xquat[0]=[np.cos(angle),np.sin(angle),0.,0.]
    e.capture_support_edge=lambda *args:None
    actions=[]
    def meter(*args,**kwargs):
        actions.append('meter');e.data.xpos[0,2]=.0202
        return {'mode':'friction_metered'}
    def unload():
        actions.append('unload');return {'initial_normal_force_n':5.,'duration_s':.21}
    e.lower_with_finger_friction=meter
    e.unload_gripper_for_release=unload
    def stop(_):raise ExecutionFailure('after handover')
    e.open_gripper_before_retreat=stop
    with pytest.raises(ExecutionFailure,match='after handover'):
        e.release_and_press(target,0.,'release_at_insertion_entry')
    assert actions==[expected]
