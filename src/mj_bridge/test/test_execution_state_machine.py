"""An execution state cannot advance by skipping measured preconditions."""
import pytest
from mj_bridge.execution_phases import ExecutionStateMachine, StateTransitionError


RELEASE = {'finger_positions_m': [.04, .04], 'target_contact': False,
           'target_contacts_observable': True, 'wait_simulation_s': .08}


def planned(blocks=('a',), **kwargs):
    machine = ExecutionStateMachine(lambda: 0., **kwargs)
    with machine.stage('planning'):
        machine.configure_plan(blocks)
    return machine


def advance(machine, block, stop_before=None):
    for phase in machine.BLOCK_PHASES:
        if phase == stop_before:
            return
        with machine.stage(phase, block):
            if phase == 'grasp_approach':
                machine.evidence(pose_confirmed=True)
            if phase == 'grasp_closure':
                machine.evidence(closure_complete=True, grasp_confirmed=True)
            if phase == 'lift':
                machine.evidence(measured_lift_m=.16)
            if phase == 'release':
                machine.evidence(release_confirmation=RELEASE)
            if phase == 'placement_verification':
                machine.evidence(placement_verified=True)


def test_complete_order_requires_every_block_and_final_stability():
    machine = planned(('a', 'b'))
    advance(machine, 'a')
    assert machine.expected() == ('source_observation', 'b')
    advance(machine, 'b')
    assert machine.summary()['status'] == 'running'
    assert machine.expected() == ('final_stability', None)
    with machine.stage('final_stability'):
        machine.evidence(placement_verified=True)
    assert machine.summary()['status'] == 'completed'
    assert machine.summary()['verified_blocks'] == 2


@pytest.mark.parametrize('phase,block', [('lift', 'a'), ('source_observation', 'b'),
                                       ('final_stability', None), ('retreat', 'a')])
def test_skip_wrong_block_or_early_final_check_cannot_issue_a_motion(phase, block):
    holds = []
    machine = planned(on_failure=lambda error: holds.append(str(error)))
    ran = []
    with pytest.raises(StateTransitionError, match='illegal transition'):
        with machine.stage(phase, block):
            ran.append(True)
    assert not ran and len(holds) == 1
    assert machine.summary()['status'] == 'failed'


@pytest.mark.parametrize('fault', ['one_finger', 'touching', 'asleep', 'nan'])
def test_invalid_release_latches_failure_before_retreat(fault):
    machine = planned()
    advance(machine, 'a', stop_before='release')
    release = dict(RELEASE)
    if fault == 'one_finger': release['finger_positions_m'] = [.04, .02]
    if fault == 'touching': release['target_contact'] = True
    if fault == 'asleep': release['target_contacts_observable'] = False
    if fault == 'nan': release['finger_positions_m'] = [.04, float('nan')]
    with pytest.raises(StateTransitionError, match='release lacks'):
        with machine.stage('release', 'a'):
            machine.evidence(release_confirmation=release)
    assert machine.failed_phase == 'release'
    with pytest.raises(StateTransitionError):
        with machine.stage('retreat', 'a'):
            pytest.fail('failed release cannot retreat')


def test_failure_retains_original_reason_and_never_resumes():
    machine = planned()
    with pytest.raises(RuntimeError, match='timeout'):
        with machine.stage('source_observation', 'a'):
            raise RuntimeError('timeout')
    assert machine.summary()['error'] == 'timeout'
    with pytest.raises(StateTransitionError):
        with machine.stage('source_observation', 'a'):
            pytest.fail('failed episode must be reset')
    assert machine.summary()['error'] == 'timeout'


def test_duplicate_or_missing_plan_cannot_start_an_episode():
    machine = ExecutionStateMachine(lambda: 0.)
    with pytest.raises(StateTransitionError):
        with machine.stage('planning'):
            machine.configure_plan(['a', 'a'])
    assert machine.summary()['status'] == 'failed'


def test_grasp_cannot_advance_on_dwell_time_without_contact_confirmation():
    machine = planned()
    advance(machine, 'a', stop_before='grasp_closure')
    with pytest.raises(StateTransitionError, match='holding contact'):
        with machine.stage('grasp_closure', 'a'):
            machine.evidence(closure_complete=True)
    assert machine.failed_phase == 'grasp_closure'


@pytest.mark.parametrize('lift', [None, .02, float('nan'), True])
def test_lift_cannot_advance_without_measured_clearance(lift):
    machine = planned()
    advance(machine, 'a', stop_before='lift')
    with pytest.raises(StateTransitionError, match='measured clearance'):
        with machine.stage('lift', 'a'):
            machine.evidence(measured_lift_m=lift)
    assert machine.failed_phase == 'lift'


@pytest.mark.parametrize('terminal', ['failed', 'completed'])
def test_terminal_state_cannot_restart_through_none_transition(terminal):
    machine = planned()
    if terminal == 'failed':
        with pytest.raises(RuntimeError):
            with machine.stage('source_observation', 'a'):
                raise RuntimeError('stopped')
    else:
        advance(machine, 'a')
        with machine.stage('final_stability'):
            machine.evidence(placement_verified=True)
    original = machine.summary().copy()
    with pytest.raises(StateTransitionError):
        with machine.stage(None, None):
            pytest.fail('a terminal state cannot issue another action')
    assert machine.summary() == original
