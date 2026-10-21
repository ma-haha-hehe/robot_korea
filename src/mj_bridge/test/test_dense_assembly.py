"""Dense layouts must preserve targets and release before raising the tool."""
import copy
import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from mj_bridge.assembly_planner import (
    GraspAccessError, PlanningError, plan_assembly, plan_with_release_above,
)
from mj_bridge.reference_executor import ExecutionFailure, OracleExecutor


@pytest.mark.parametrize('gap,height,kind,expected', [
    (0., .0384, 'brick_2x2', True),
    (.001, .0384, 'brick_2x2', True),
    (.005, .0384, 'brick_2x2', False),
    (0., .0192, 'brick_2x2', False),
    (0., .0384, 'brick_4x2', False),
])
@pytest.mark.parametrize('rotation', [0., math.pi/2, math.pi/4])
def test_dense_feedback_requires_an_upper_square_near_a_same_layer_neighbour(
        gap, height, kind, expected, rotation):
    from mj_bridge.reference_executor import requires_dense_pose_feedback
    target = dict(id='target', type=kind, position=[.3, .2, .0384], yaw_rad=rotation)
    neighbour = dict(id='neighbour', type='brick_2x2',
                     position=[.3-(.032+gap)*math.sin(rotation),
                               .2+(.032+gap)*math.cos(rotation), height], yaw_rad=rotation)
    assert requires_dense_pose_feedback(target, [neighbour], 0.) is expected
    assert not requires_dense_pose_feedback(dict(target, position=[.3, .2, 0.]), [neighbour], 0.)
    assert not requires_dense_pose_feedback(target, [target], 0.)


def test_dense_feedback_runs_before_the_ordinary_entry_and_records_its_trigger(monkeypatch):
    monkeypatch.setattr('mj_bridge.reference_executor.requires_dense_pose_feedback', lambda *_: True)
    target = dict(id='part', type='brick_2x2', position=[0., 0., .0192], yaw_rad=0.)
    support = dict(target, id='support', position=[0., 0., 0.])
    executor = OracleExecutor.__new__(OracleExecutor)
    executor.perception = None
    executor.speed_scale = 1.5
    executor.events = [dict(block='support')]
    executor.node = SimpleNamespace(episode_manifest={
        'contact_profile': 'plastic', 'target_blocks': [support, target]})
    executor.align_held_part = lambda _: (np.array(target['position']), 0.)
    calls = []
    executor.move_vertical = lambda position, yaw: calls.append('clearance')
    def feedback(*_):
        assert executor._descent_guard_target is None
        calls.append('feedback')
        executor.last_seating_confirmation = {'pose_feedback': {'steps': 1}}
    executor.descend_with_pose_feedback = feedback
    executor.descend_to_insertion_entry = lambda *_: pytest.fail('feedback must precede ordinary entry')
    executor.seat_plastic_part = lambda *_: pytest.fail('completed feedback must not insert twice')
    executor.refine_overhang_pose = lambda *_: calls.append('verify_support')
    assert executor.place_physical(target) == 0.
    assert calls == ['clearance', 'feedback', 'verify_support']
    assert executor.last_seating_confirmation['pose_feedback']['trigger'] == 'near_same_layer_neighbour'
    assert executor.speed_scale == 1.5
    assert executor._descent_guard_target is None


@pytest.mark.parametrize('name', ['tier3_task_079', 'tier4_task_051'])
def test_dense_originals_get_explicit_fallback_without_moving_targets(name):
    targets = json.loads((Path(__file__).parent / 'fixtures' / f'{name}_targets.json').read_text())
    before = copy.deepcopy(targets)
    with pytest.raises(GraspAccessError):
        plan_assembly(targets)
    plan = plan_with_release_above(targets)
    assert targets == before
    assert {row['block_id'] for row in plan} == {b['id'] for b in targets}
    assert len(plan) == len(targets)
    assert any(row['placement_mode'] == 'release_above_press' for row in plan)
    for row in plan:
        target = next(b for b in targets if b['id'] == row['block_id'])
        assert target['yaw_rad'] + row['grasp_offset_rad'] == pytest.approx(
            math.radians(row['grasp_spin_deg']))


def test_fallback_does_not_repair_invalid_solids_or_change_simple_plan():
    part = dict(id='a', type='brick_2x2', position=[0, 0, 0], yaw_rad=0.)
    assert plan_with_release_above([part]) == plan_assembly([part])
    with pytest.raises(PlanningError, match='overlapping target'):
        plan_with_release_above([part, dict(part, id='b')])


def test_failed_release_prevents_retreat_or_press():
    executor = OracleExecutor.__new__(OracleExecutor)
    executor.perception = None
    executor.speed_scale = 1.5
    executor.node = SimpleNamespace(gripper_target=.02, connection_mode="physics",
                                    episode_manifest={"contact_profile": "plastic"})
    executor.node.episode_manifest['target_blocks'] = [dict(position=[0., 0., 0.])]
    executor.align_held_part = lambda _: (np.zeros(3), 0.)
    moves = []
    executor.move_vertical = lambda position, yaw: moves.append(position.copy())
    def fail_release(_):
        raise ExecutionFailure('fingers not open')
    executor.open_gripper_before_retreat = fail_release
    executor.lower_with_finger_friction = lambda _: {'mode': 'test_already_lowered'}
    executor.unload_gripper_for_release = lambda: {}
    executor.model = SimpleNamespace(body=lambda _: SimpleNamespace(id=0))
    executor.data = SimpleNamespace(xpos=np.array([[0., 0., .003]]),
                                    xquat=np.array([[1., 0., 0., 0.]]))
    with pytest.raises(ExecutionFailure, match='fingers not open'):
        executor.place_with_release_above({'body_name': 'part', 'position': np.zeros(3), 'type': 'brick_4x2'})
    assert [p[2] for p in moves] == [.06, .016]
    assert executor.node.gripper_target == .02
    assert executor.speed_scale == 1.5


@pytest.mark.parametrize('height,support_x,release_height', [
    (0., .1, .001), (.0192, 0., .001), (.0192, .02, .002),
])
@pytest.mark.parametrize('kind,yaw', [
    ('brick_2x2', 0.), ('brick_4x2', 0.), ('brick_4x2', math.pi/2),
])
def test_direct_release_uses_support_geometry_and_stops_on_release_failure(
        height, support_x, release_height, kind, yaw):
    target = dict(id='part', type=kind, position=[0., 0., height], yaw_rad=yaw)
    support = dict(target, id='support', position=[support_x, 0., 0.])
    executor = OracleExecutor.__new__(OracleExecutor)
    executor.perception = None
    executor.speed_scale = 1.5
    executor.events = [dict(block='support')]
    executor.node = SimpleNamespace(episode_manifest={
        'contact_profile': 'plastic', 'target_blocks': [support, target]})
    executor.align_held_part = lambda _: (np.array(target['position']), 0.)
    moves = []
    executor.move_vertical = lambda position, yaw: moves.append(position.copy())
    def failed_release(*_):
        raise ExecutionFailure('fingers not open')
    executor.release_and_press = failed_release
    executor.held_edge_insertion_required = lambda _: False
    def insertion_entry(target, endpoint, gripper_yaw):
        expected_guard = (height > .004 or (kind == 'brick_4x2'
                          and abs(math.sin(target['yaw_rad'] - gripper_yaw)) > .7))
        assert getattr(executor, '_descent_guard_target', None) == (target if expected_guard else None)
        return executor.move_vertical(endpoint, gripper_yaw)
    executor.descend_to_insertion_entry = insertion_entry
    with pytest.raises(ExecutionFailure, match='fingers not open'):
        executor.place_physical(target)
    assert [p[2] - height for p in moves] == pytest.approx([.04, release_height])
    assert executor.speed_scale == 1.5
    assert executor._descent_guard_target is None


@pytest.mark.parametrize('position,expected', [
    ([0., 0., .002], True), ([0., 0., .006], False),
    ([.002, 0., .002], False), ([0., 0., -.001], False),
])
def test_top_press_retry_requires_captured_aligned_fit(position, expected):
    executor = OracleExecutor.__new__(OracleExecutor)
    executor.perception = None
    executor.node = SimpleNamespace(current_block_state=lambda: {'blocks': {'p': {
        'position': position, 'quaternion_wxyz': [1., 0., 0., 0.]}}})
    def stalled(*_):
        raise ExecutionFailure('part not seated within 4 mm bounded insertion travel')
    executor._seat_plastic_part = stalled
    retries = []
    executor.release_and_press = lambda *args: retries.append(args)
    target = {'id': 'p', 'position': [0., 0., 0.]}
    if expected:
        executor.seat_plastic_part(target, 0.)
        assert len(retries) == 1
    else:
        with pytest.raises(ExecutionFailure, match='4 mm'):
            executor.seat_plastic_part(target, 0.)
        assert not retries


def test_top_press_cannot_recursively_retry_a_jammed_press():
    executor = OracleExecutor.__new__(OracleExecutor)
    executor.perception = None
    executor.node = SimpleNamespace(connection_mode='physics',
                                    episode_manifest={'contact_profile': 'plastic'})
    executor._top_press_active = True
    with pytest.raises(ExecutionFailure, match='did not seat'):
        executor.release_and_press({}, 0., 'release_before_seating')


@pytest.mark.parametrize('fault', ['height', 'tilt', 'finger', 'release', 'sleeping', 'missing_contacts', 'sleeping_final', 'missing_final', 'short_final', 'none'])
def test_public_report_rechecks_settled_pose_and_measured_release(fault):
    import runpy
    audit = runpy.run_path(str(Path(__file__).resolve().parents[3]
                              / 'scripts' / 'write_validation_report.py'))
    manifest = {'target_blocks': [dict(id='p', type='brick_2x2',
                                      position=[0., 0., 0.], yaw_rad=0.)]}
    state = {'blocks': {'p': dict(position=[0., 0., .0001],
                                 yaw_rad=0., quaternion_wxyz=[1., 0., 0., 0.])}}
    release = dict(finger_positions_m=[.04, .04], target_contact=False, target_contacts_observable=True,
                   wait_simulation_s=.8)
    result = {'events': [dict(block='p', release_confirmation=release)],
              'final_contact_observation': {'awake_parts': {'p': True}, 'settle_simulation_s': 1.}}
    if fault == 'sleeping_final':
        result['final_contact_observation']['awake_parts']['p'] = False
    elif fault == 'missing_final':
        result.pop('final_contact_observation')
    elif fault == 'short_final':
        result['final_contact_observation']['settle_simulation_s'] = .5
    elif fault == 'height':
        state['blocks']['p']['position'][2] = .002
    elif fault == 'tilt':
        angle = math.radians(4.) / 2
        state['blocks']['p']['quaternion_wxyz'] = [math.cos(angle), math.sin(angle), 0., 0.]
    elif fault == 'finger':
        release['finger_positions_m'][1] = .025
    elif fault == 'sleeping':
        release['target_contacts_observable'] = False
    elif fault == 'missing_contacts':
        release.pop('target_contacts_observable')
    elif fault == 'release':
        result['events'] = []
    check = audit['require_plastic_seating_evidence']
    if fault == 'none':
        check(result, manifest, state)
    else:
        with pytest.raises(ValueError):
            check(result, manifest, state)


@pytest.mark.parametrize('force', [None, 0., -.1, .01, float('nan'), .2])
def test_report_requires_bottom_support_for_top_press(force):
    import runpy
    audit = runpy.run_path(str(Path(__file__).resolve().parents[3]
                              / 'scripts' / 'write_validation_report.py'))
    release = dict(finger_positions_m=[.04, .04], target_contact=False, target_contacts_observable=True, wait_simulation_s=.8)
    result = {'events': [dict(block='p', release_confirmation=release,
        seating_confirmation=dict(initial_release_confirmation=release, bottom_support_force_n=force))],
        'final_contact_observation': {'awake_parts': {'p': True}, 'settle_simulation_s': 1.}}
    manifest = {'target_blocks': [dict(id='p', type='brick_2x2', position=[0., 0., 0.], yaw_rad=0.)]}
    state = {'blocks': {'p': dict(position=[0., 0., 0.], yaw_rad=0.,
                                 quaternion_wxyz=[1., 0., 0., 0.])}}
    if force == .2:
        audit['require_plastic_seating_evidence'](result, manifest, state)
    else:
        with pytest.raises(ValueError, match='bottom support'):
            audit['require_plastic_seating_evidence'](result, manifest, state)


def test_release_fallback_prefers_supported_parts():
    targets = json.loads((Path(__file__).parent / 'fixtures' / 'tier3_task_079_targets.json').read_text())
    plan = plan_with_release_above(targets)
    released = [row['block_id'] for row in plan if row.get('placement_mode') == 'release_above_press']
    assert released == ['2x2_brick_5']
    assert next(row for row in plan if row['block_id'] == '4x2_brick_4')['placement_mode'] == 'direct'


def test_reuse_proof_rejects_changed_direct_code_and_fallback_runs(tmp_path):
    import hashlib
    import runpy
    from mj_bridge import assembly_planner
    proof = runpy.run_path(str(Path(__file__).resolve().parents[3] / 'scripts' / 'direct_plan_reuse.py'))
    verify = proof['verify_direct_plan_reuse']
    current = Path(assembly_planner.__file__)
    baseline = tmp_path / 'planner.py'
    baseline.write_text(current.read_text().replace('round(load_path_support(block), 6)', '-round(load_path_support(block), 6)'))
    recorded = {'assembly_planner.py': hashlib.sha256(baseline.read_bytes()).hexdigest()}
    manifest = {'target_blocks': [dict(id='a', type='brick_2x2', position=[0., 0., 0.], yaw_rad=0.)]}
    plan = {'method': 'assembly_by_disassembly', 'steps': plan_assembly(manifest['target_blocks'])}
    assert verify(baseline, current, recorded, manifest, plan)['executed_planner_sha256'] == recorded['assembly_planner.py']
    with pytest.raises(ValueError, match='fallback execution'):
        verify(baseline, current, recorded, manifest, dict(plan, method='geometric_reverse_order_with_release_above_fallback'))
    baseline.write_text(baseline.read_text().replace('if not remaining:', 'if remaining:', 1))
    recorded['assembly_planner.py'] = hashlib.sha256(baseline.read_bytes()).hexdigest()
    with pytest.raises(ValueError, match='changes extend'):
        verify(baseline, current, recorded, manifest, plan)


def test_supported_landing_avoids_releasing_the_half_supported_edge():
    from mj_bridge.assembly_planner import accessible, load_registry, RELEASE_TOOL_CLEARANCE_M
    targets = json.loads((Path(__file__).parent / 'fixtures' / 'tier4_task_051_targets.json').read_text())
    before = copy.deepcopy(targets)
    plan = plan_with_release_above(targets)
    released = [row['block_id'] for row in plan if row.get('placement_mode') == 'release_above_press']
    # The old greedy sequence released part 7 onto a half-supported edge and
    # it fell. Part 10 has full face coverage; dynamic stability needs a run.
    assert released == ['2x2_brick_10']
    assert next(row for row in plan if row['block_id'] == '2x2_brick_7')['placement_mode'] == 'direct'
    remaining = list(targets)
    for row in reversed(plan):
        block = next(b for b in remaining if b['id'] == row['block_id'])
        candidate = block
        if row['placement_mode'] == 'release_above_press':
            candidate = dict(block, position=[*block['position'][:2],
                                             block['position'][2] + RELEASE_TOOL_CLEARANCE_M])
        assert accessible(candidate, remaining, row['grasp_spin_deg'], load_registry())
        remaining.remove(block)
    assert not remaining
    assert targets == before


def test_direct_search_avoids_wide_holds_without_unsafe_removals():
    from mj_bridge.assembly_planner import accessible, load_registry
    targets = json.loads((Path(__file__).parent / 'fixtures' / 'tier3_task_005_targets.json').read_text())
    plan = plan_assembly(targets)
    assert {p['block_id']: p['grasp_spin_deg'] for p in plan}['4x2_brick_5'] == 90
    assert {p['block_id']: p['grasp_spin_deg'] for p in plan}['4x2_brick_1'] == 90
    remaining = list(targets)
    registry = load_registry()
    for step in reversed(plan):
        part = next(b for b in remaining if b['id'] == step['block_id'])
        assert accessible(part, remaining, step['grasp_spin_deg'], registry)
        remaining.remove(part)
    assert not remaining


def test_equivalent_plan_proof_rejects_angle_changes_and_shared_code_edits(tmp_path):
    import hashlib
    import runpy
    from mj_bridge import assembly_planner
    proof = runpy.run_path(str(Path(__file__).resolve().parents[3] / 'scripts' / 'direct_plan_reuse.py'))
    verify = proof['verify_equivalent_plan_reuse']
    current = Path(assembly_planner.__file__)
    baseline = tmp_path / 'archived.py'
    baseline.write_text(current.read_text().replace('round(load_path_support(block), 6)', '-round(load_path_support(block), 6)'))
    recorded = {'assembly_planner.py': hashlib.sha256(baseline.read_bytes()).hexdigest()}
    manifest = {'target_blocks': [dict(id='a', type='brick_2x2', position=[0., 0., 0.], yaw_rad=0.)]}
    plan = {'method': 'assembly_by_disassembly', 'steps': plan_assembly(manifest['target_blocks'])}
    assert verify(baseline, current, recorded, manifest, plan)['executed_planner_sha256'] == recorded['assembly_planner.py']
    changed = copy.deepcopy(plan)
    changed['steps'][0]['grasp_spin_deg'] = 0
    with pytest.raises(ValueError, match='steps differ'):
        verify(baseline, current, recorded, manifest, changed)
    baseline.write_text(baseline.read_text().replace('opening > .04 - .001', 'opening > .04 - .002'))
    recorded['assembly_planner.py'] = hashlib.sha256(baseline.read_bytes()).hexdigest()
    with pytest.raises(ValueError, match='beyond discrete planning'):
        verify(baseline, current, recorded, manifest, plan)


@pytest.mark.parametrize('fault', ['none', 'near_goal', 'lost_finger', 'external_contact', 'large_tilt'])
def test_airborne_recovery_requires_clear_space_and_two_loaded_fingers(monkeypatch, fault):
    import mujoco
    executor = OracleExecutor.__new__(OracleExecutor)
    executor.perception = None
    executor.prepare_feedback_grip = lambda _: None
    executor.hand = 0
    executor.tool_offset = np.zeros(3)
    executor.qadr = np.array([0])
    angle = math.radians(9 if fault == 'large_tilt' else 3.5) / 2
    height = .005 if fault == 'near_goal' else .04
    observed = {'position': [0., 0., height],
                'quaternion_wxyz': [math.cos(angle), math.sin(angle), 0., 0.]}
    ids = {'part': 1, 'left_finger': 2, 'right_finger': 3}
    executor.model = SimpleNamespace(
        body=lambda name: SimpleNamespace(id=ids[name]),
        geom_bodyid=np.array([1, 2, 1, 4 if fault == 'external_contact' else 3]))
    contacts = [SimpleNamespace(geom1=0, geom2=1, dist=-.00001),
                SimpleNamespace(geom1=2, geom2=3, dist=-.00001)]
    executor.data = SimpleNamespace(xmat=np.eye(3).reshape(1,9),
                                   xpos=np.array([[0., 0., height], [0., 0., height]]),
                                   xquat=np.array([[1., 0., 0., 0.], observed['quaternion_wxyz']]),
                                   qpos=np.zeros(1), contact=contacts)
    executor.part_contacts_observable = lambda _: True
    executor.node = SimpleNamespace(target_qpos=np.zeros(1),
        current_block_state=lambda: {'blocks': {'part': observed}})
    def force(model, data, index, output):
        output[0] = 0 if fault == 'lost_finger' and index == 1 else 1.
    monkeypatch.setattr(mujoco, 'mj_contactForce', force)
    commands = []
    def ik(position, yaw, **kwargs):
        commands.append(position.copy())
        return np.zeros(1)
    executor.ik = ik
    def step(_):
        observed.update(position=[0., 0., .0013], quaternion_wxyz=[1., 0., 0., 0.])
    executor.step = step
    def press(*_):
        executor.last_seating_confirmation = {}
    executor.release_and_press = press
    target = dict(id='part', type='brick_4x2', body_name='part',
                  position=[0., 0., 0.], yaw_rad=0.)
    if fault == 'none':
        executor.descend_with_pose_feedback(target, 0.)
        assert commands[0][2] == pytest.approx(height)
        assert executor.last_seating_confirmation['pose_feedback']['max_observed_tilt_deg'] == pytest.approx(3.5)
    else:
        with pytest.raises(ExecutionFailure):
            executor.descend_with_pose_feedback(target, 0.)
        assert not commands


def test_unused_feedback_reuse_rejects_executed_feedback_and_other_control_changes(tmp_path):
    import hashlib
    import runpy
    from mj_bridge import reference_executor
    proof = runpy.run_path(str(Path(__file__).resolve().parents[3] / 'scripts' / 'unused_feedback_reuse.py'))
    verify = proof['verify_unused_feedback_reuse']
    current = Path(reference_executor.__file__)
    baseline = tmp_path / 'reference_executor.py'
    baseline.write_text(current.read_text().replace('if tilt > math.radians(8) or', 'if tilt > math.radians(7) or'))
    recorded = {'reference_executor.py': hashlib.sha256(baseline.read_bytes()).hexdigest()}
    manifest = {'target_blocks': [dict(id='p')]}
    result = {'success': True, 'events': [dict(block='p', seating_confirmation={'height_error_m': .0001})]}
    assert verify(baseline, current, recorded, result, manifest)['executed_executor_sha256'] == recorded['reference_executor.py']
    result['events'][0]['seating_confirmation']['pose_feedback'] = {'steps': 1}
    with pytest.raises(ValueError, match='rerun required'):
        verify(baseline, current, recorded, result, manifest)
    result['events'][0]['seating_confirmation'].pop('pose_feedback')
    baseline.write_text(baseline.read_text().replace('positions >= .0395', 'positions >= .0300'))
    recorded['reference_executor.py'] = hashlib.sha256(baseline.read_bytes()).hexdigest()
    with pytest.raises(ValueError, match='beyond pose feedback'):
        verify(baseline, current, recorded, result, manifest)


@pytest.mark.parametrize('gap,upward,kind,accepted', [
    (.00001, .012*9.81, 'brick_2x2', True),
    (.00001, .022*9.81, 'brick_4x2', True),
    (.00001, .012*9.81, 'brick_4x2', False),
    (.00001, .01, 'brick_2x2', False),
    (.00002, .2, 'brick_2x2', False),
    (.0002, .2, 'brick_2x2', False),
    (0., .2, 'brick_2x2', False),
    (float('nan'), .2, 'brick_2x2', False),
    (.00001, float('nan'), 'brick_2x2', False),
])
def test_report_requires_near_seating_and_actual_clutch_load(gap, upward, kind, accepted):
    import runpy
    check = runpy.run_path(str(Path(__file__).resolve().parents[3]
                              / 'scripts/write_validation_report.py'))['require_plastic_seating_evidence']
    release = dict(finger_positions_m=[.04, .04], target_contact=False,
                   target_contacts_observable=True, wait_simulation_s=.8)
    result = {'events': [dict(block='p', release_confirmation=release,
        seating_confirmation=dict(initial_release_confirmation=release, bottom_support_force_n=0.,
            clutch_seating=dict(bottom_gap_m=gap, upward_contact_force_n=upward)))],
        'final_contact_observation': {'awake_parts': {'p': True}, 'settle_simulation_s': 1.}}
    manifest = {'target_blocks': [dict(id='p', type=kind, position=[0., 0., 0.], yaw_rad=0.)]}
    state = {'blocks': {'p': dict(position=[0., 0., 0.], yaw_rad=0., quaternion_wxyz=[1., 0., 0., 0.])}}
    if accepted:
        check(result, manifest, state)
    else:
        with pytest.raises(ValueError, match='support'):
            check(result, manifest, state)


@pytest.mark.parametrize('kind,yaw', [('brick_2x2', math.pi/2), ('brick_4x2', math.pi)])
def test_feedback_corrects_tilt_without_rotating_symmetric_yaw(kind, yaw):
    executor = OracleExecutor.__new__(OracleExecutor)
    executor.perception = None
    executor.prepare_feedback_grip = lambda _: None
    executor.hand = 0
    executor.qadr = np.array([0])
    executor.tool_offset = np.zeros(3)
    tilt = math.radians(.5)
    q = [math.cos(yaw/2)*math.cos(tilt/2), math.cos(yaw/2)*math.sin(tilt/2),
         math.sin(yaw/2)*math.sin(tilt/2), math.sin(yaw/2)*math.cos(tilt/2)]
    observed = {'position': [0., 0., .03], 'quaternion_wxyz': q}
    executor.data = SimpleNamespace(qpos=np.zeros(1),xpos=np.array([[0.,0.,.03]]),
                                    xmat=np.eye(3).reshape(1,9))
    executor.node = SimpleNamespace(target_qpos=np.zeros(1),
        current_block_state=lambda: {'blocks': {'p': observed}})
    rotations=[]
    def ik(position, angle, **kwargs):
        rotations.append(kwargs['target_rotation'].copy())
        return np.zeros(1)
    executor.ik=ik
    executor.confirm_held_part=lambda _: 0.
    executor.step=lambda _: observed.update(position=[0.,0.,.001])
    executor.held_edge_insertion_required=lambda _: False
    executor.release_and_press=lambda *args: setattr(executor,'last_seating_confirmation',{})
    executor.descend_with_pose_feedback(dict(id='p',type=kind,position=[0.,0.,0.],yaw_rad=0.),0.)
    assert len(rotations)==1
    assert np.linalg.norm(rotations[0]-np.eye(3)) < .02
    assert np.linalg.det(rotations[0]) == pytest.approx(1.)


@pytest.mark.parametrize('case', ['loaded', 'unreleased', 'weak', 'asleep', 'xy', 'height', 'tilt'])
def test_feedback_handover_requires_observed_placed_support_and_bounded_pose(monkeypatch, case):
    import mujoco
    executor = OracleExecutor.__new__(OracleExecutor)
    target = dict(id='part', body_name='part', type='brick_2x2', position=[0., 0., .0192], yaw_rad=0.)
    lower = dict(id='lower', body_name='lower', type='brick_2x2', position=[0., 0., 0.], yaw_rad=0.)
    names = ['world', 'part', 'lower']
    def body(key):
        index = names.index(key) if isinstance(key, str) else key
        return SimpleNamespace(id=index, name=names[index])
    executor.model = SimpleNamespace(nbody=3, body=body, geom_bodyid=np.arange(3))
    contacts = np.array([(1, 2, -.0001)], dtype=[('geom1', 'i4'), ('geom2', 'i4'), ('dist', 'f8')]).view(np.recarray)
    executor.data = SimpleNamespace(contact=contacts, time=1., xpos=np.array([[0., 0., 0.], [0., 0., .021], [0., 0., 0.]]),
                                    xquat=np.tile([1., 0., 0., 0.], (3, 1)))
    executor.events = [] if case == 'unreleased' else [dict(block='lower')]
    executor.node = SimpleNamespace(episode_manifest={'target_blocks': [target, lower]})
    executor.part_contacts_observable = lambda _: case != 'asleep'
    def force(model, data, index, output):
        output[0] = .001 if case == 'weak' else .02
    monkeypatch.setattr(mujoco, 'mj_contactForce', force)
    if case == 'xy': executor.data.xpos[1, 0] = .002
    if case == 'height': executor.data.xpos[1, 2] = .0242
    if case == 'tilt': executor.data.xquat[1] = [math.cos(.04), math.sin(.04), 0., 0.]
    if case in ('asleep', 'xy', 'height', 'tilt'):
        with pytest.raises(ExecutionFailure): executor.feedback_support_contact(target)
    else:
        contact = executor.feedback_support_contact(target)
        if case == 'loaded': assert contact['normal_force_n'] == pytest.approx(.02)
        else: assert contact is None
