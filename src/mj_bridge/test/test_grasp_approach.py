"""An open approach must not disturb a source part before closure."""
from types import SimpleNamespace
import math
import numpy as np
import pytest
from mj_bridge.reference_executor import OracleExecutor, ExecutionFailure


def pose_executor():
    executor = OracleExecutor.__new__(OracleExecutor)
    executor.speed_scale = 1.5
    executor.hand = 0
    executor.tool_offset = np.zeros(3)
    executor.data = SimpleNamespace(xmat=np.diag([1., -1., -1.]).reshape(1, 9),
                                    xpos=np.array([[.202, -.1, .06]]))
    # An issued goal is not a measured arrival.
    executor.move_vertical = lambda *args: None
    return executor


def test_approach_waits_for_measured_arrival_before_confirming():
    executor = pose_executor()
    waits = []
    def settle(seconds):
        waits.append(seconds)
        executor.data.xpos[0] = [.2, -.1, .06]
    executor.step = settle
    executor.approach_grasp([.2, -.1, .06], 0.)
    assert waits == [.02]


def test_issued_grasp_goal_does_not_bypass_failed_measured_tracking():
    executor = pose_executor()
    waits = []
    executor.step = waits.append
    with pytest.raises(ExecutionFailure, match='closure cancelled'):
        executor.approach_grasp([.2, -.1, .06], 0.)
    assert 1. <= sum(waits) <= 1.03
    assert executor.speed_scale == 1.5


@pytest.mark.parametrize('delta,tilt,valid', [
    ([0., 0., 0.], 0., True), ([.0002, 0., 0.], .1, True),
    ([.002, 0., 0.], 0., False), ([0., 0., 0.], 35., False),
    ([0., 0., float('nan')], 0., False),
])
def test_closure_requires_an_undisturbed_observable_source(delta, tilt, valid):
    executor = OracleExecutor.__new__(OracleExecutor)
    executor.perception = None
    angle = math.radians(tilt)/2
    state = {'position': np.asarray([.2, -.1, .06])+delta,
             'quaternion_wxyz': [math.cos(angle), math.sin(angle), 0., 0.]}
    executor.node = SimpleNamespace(connection_mode='physics',
        current_block_state=lambda: {'blocks': {'part': state}})
    observed = {'position': [.2, -.1, .06]}
    if valid:
        executor.confirm_source_unchanged({'id': 'part'}, observed)
    else:
        with pytest.raises(ExecutionFailure, match='closure cancelled'):
            executor.confirm_source_unchanged({'id': 'part'}, observed)


def recovery_executor():
    executor = OracleExecutor.__new__(OracleExecutor)
    executor.qadr = np.array([0])
    executor.data = SimpleNamespace(qpos=np.array([.2]))
    observed = {'position': [.305, -.1, .06], 'quaternion_wxyz': [1., 0., 0., 0.], 'yaw_rad': 0.}
    executor.node = SimpleNamespace(target_qpos=np.zeros(1),
        episode_manifest={'contact_profile': 'plastic', 'spawn_region': {'x': [.2,.7], 'y': [-.4,.1]}},
        current_block_state=lambda: {'blocks': {'part': observed}})
    executor.grasp_recoveries = []
    actions = []
    executor.verify_released_parts = lambda: None
    executor.open_gripper_before_retreat = lambda _: actions.append('open')
    executor.move_vertical = lambda *args: actions.append('retreat')
    executor.observe_target = lambda _: dict(observed)
    executor.select_grasp_yaw = lambda grasp, hover, yaw: yaw
    executor.move = lambda *args: actions.append('hover')
    executor.prepare_grasp_opening = lambda *args: actions.append('opening')
    executor.approach_grasp = lambda *args, **kwargs: actions.append(('approach',kwargs['careful']))
    executor.confirm_source_unchanged = lambda *args: None
    return executor, observed, actions


@pytest.mark.parametrize('fault', ['far', 'airborne', 'tilted', 'nan'])
def test_unsafe_disturbed_source_is_rejected_before_opening_or_retreat(fault):
    executor, observed, actions = recovery_executor()
    if fault == 'far': observed['position'][0] = .32
    if fault == 'airborne': observed['position'][2] = .065
    if fault == 'tilted': observed['quaternion_wxyz'] = [np.cos(.1), np.sin(.1), 0., 0.]
    if fault == 'nan': observed['position'][0] = float('nan')
    with pytest.raises(ExecutionFailure, match='cannot safely retry'):
        executor.recover_source_approach({'id':'part'}, {}, {'position':[.3,-.1,.06]}, [0.,0.,.2], 0.)
    assert not actions and not executor.grasp_recoveries


@pytest.mark.parametrize('retry_fails', [False, True])
def test_source_retry_uses_measured_pose_and_preserves_the_assembly_target(retry_fails):
    executor, observed, actions = recovery_executor()
    target = {'id':'part', 'type':'brick_2x2', 'position':[.35,.35,.08],
              'yaw_rad':0., 'placement_mode':'direct'}
    before = dict(target, position=list(target['position']))
    def fail(*args):
        raise ExecutionFailure('retry disturbed source again')
    if retry_fails: executor.confirm_source_unchanged = fail
    args=(target, {'grasp_spin_deg':90}, {'position':[.3,-.1,.06]}, [.3,-.1,.2], 0.)
    if retry_fails:
        with pytest.raises(ExecutionFailure, match='again'): executor.recover_source_approach(*args)
    else:
        result = executor.recover_source_approach(*args)
        assert np.array_equal(result[1], observed['position'])
    assert target == before
    assert actions[:2] == ['open', 'retreat']
    assert actions.count(('approach',True)) == 1
    assert executor.grasp_recoveries[0]['status'] == ('failed' if retry_fails else 'completed')


def test_careful_retry_restores_speed_when_final_approach_fails():
    executor = pose_executor()
    moves = []
    def move(position,yaw):
        moves.append(list(position))
        if len(moves)==2: raise ExecutionFailure('motion stopped')
        executor.data.xpos[0] = position
    executor.move_vertical = move
    with pytest.raises(ExecutionFailure,match='motion stopped'):
        executor.approach_grasp([.2,-.1,.06],0.,careful=True)
    assert len(moves)==2 and executor.speed_scale==1.5
