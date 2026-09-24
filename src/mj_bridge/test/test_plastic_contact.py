"""Contact profile integration and strict seating regressions."""
from pathlib import Path
from types import SimpleNamespace
import math

import mujoco
import pytest

from mj_bridge.benchmark_cli import generate, normalized_from_path
from mj_bridge.benchmark_core import generate_episode, ProductError
from mj_bridge.reference_executor import OracleExecutor, ExecutionFailure
from mj_bridge.reference_executor import insertion_speed_scale

ROOT = Path(__file__).resolve().parents[3]


def test_plastic_requires_physics():
    product = normalized_from_path(str(ROOT / 'examples/products/traffic_light.yaml'))
    with pytest.raises(ProductError, match='requires physics'):
        generate_episode(product, seed=42, contact_profile='plastic')


def test_generated_plastic_assets_preserve_task_layout(tmp_path):
    product = str(ROOT / 'examples/products/traffic_light.yaml')
    loose = generate_episode(normalized_from_path(product), seed=42, connection_mode='physics')
    episode, scene = generate(product, 42, str(tmp_path), connection_mode='physics', contact_profile='plastic')
    model = mujoco.MjModel.from_xml_path(str(scene))
    assert episode['target_blocks'] == loose['target_blocks']
    assert [b['position'] for b in episode['spawned_blocks']] == [b['position'] for b in loose['spawned_blocks']]
    assert episode['geometry'] == 'hollow_plastic_clutch_v2'
    assert episode['contact_parameters']['sliding_friction'] == .3
    assert model.mesh('clutch_rib').id >= 0
    assert model.opt.cone == mujoco.mjtCone.mjCONE_PYRAMIDAL
    # Each compliant rib or tube must have one continuous collision hull.
    compliant = [i for i in range(model.ngeom) if model.geom_priority[i] == 2]
    assert compliant
    assert all(model.geom_type[i] == mujoco.mjtGeom.mjGEOM_MESH for i in compliant)
    assert model.geom('table_geom').priority == 3
    assert model.geom('assembly_base_stud_0_0').priority < 3
    assert episode['contact_parameters']['contact_time_constant_s'] == .03
    assert model.actuator('finger_actuator1').forcerange.tolist() == [-5., 5.]
    # Added contact geometry must not create attachment constraints.
    assert all(model.eq_type[i] != mujoco.mjtEq.mjEQ_WELD for i in range(model.neq))


@pytest.mark.parametrize('position,quaternion,reason', [
    ([0., 0., 0.], [math.cos(.1), math.sin(.1), 0., 0.], 'orientation'),
    ([.002, 0., 0.], [1., 0., 0., 0.], 'lateral'),
])
def test_nominal_height_cannot_hide_bad_plastic_seating(position, quaternion, reason):
    executor = OracleExecutor.__new__(OracleExecutor)
    executor.node = SimpleNamespace(current_block_state=lambda: {'blocks': {
        'part': {'position': position, 'quaternion_wxyz': quaternion}}})
    with pytest.raises(ExecutionFailure, match=reason):
        executor.seat_plastic_part({'id': 'part', 'position': [0., 0., 0.]}, 0.)


def test_stalled_insertion_accumulates_bounded_command_travel():
    import numpy as np
    executor = OracleExecutor.__new__(OracleExecutor)
    executor.hand = 0
    executor.tool_offset = np.zeros(3)
    executor.qadr = np.array([0])
    executor.data = SimpleNamespace(xpos=np.array([[0., 0., .001]]),
        xmat=np.eye(3).reshape(1, 9), qpos=np.zeros(1))
    executor.node = SimpleNamespace(target_qpos=np.zeros(1),
        current_block_state=lambda: {'blocks': {'part': {
            'position': [0., 0., .001], 'quaternion_wxyz': [1., 0., 0., 0.]}}})
    commands = []
    def ik(position, yaw):
        commands.append(position.copy())
        return np.zeros(1)
    executor.ik = ik
    executor.step = lambda seconds: None  # Actuator stalled under an external load.
    with pytest.raises(ExecutionFailure, match='4 mm'):
        executor.seat_plastic_part({'id': 'part', 'position': [0., 0., 0.]}, 0.)
    assert len(commands) == 16
    assert np.allclose(np.diff(np.array(commands)[:, 2]), -.00025)
    assert commands[-1][2] == pytest.approx(.001 - .004)


def test_coplanar_plastic_mesh_does_not_report_table_bottom_penetration(tmp_path):
    import yaml
    product = {'schema_version': 1, 'product': {'name': 'table_contact'}, 'blocks': [
        {'id': 'part', 'type': 'brick_4x2', 'color': 'green',
         'target': {'position': [0., 0., 0.], 'yaw_deg': 90.}}],
        'initial_layout': {'frame': 'world_tabletop', 'blocks': [
            {'id': 'part', 'position_xy_m': [.2521, -.1295], 'yaw_deg': 0.}]}}
    path = tmp_path / 'product.yaml'
    path.write_text(yaml.safe_dump(product))
    episode, scene = generate(str(path), 42, str(tmp_path / 'run'),
                              connection_mode='physics', contact_profile='plastic')
    model = mujoco.MjModel.from_xml_path(str(scene))
    table = model.geom('table_geom').id
    part = model.body(episode['spawned_blocks'][0]['body_name']).id
    qadr = model.jnt_qposadr[model.body_jntadr[part]]
    assert model.geom_margin[table] == pytest.approx(1e-6)
    for x, y in ((.2521, -.1295), (.25, -.3), (.51, -.21), (.61, -.31)):
        data = mujoco.MjData(model)
        data.qpos[qadr:qadr+2] = [x, y]
        mujoco.mj_forward(model, data)
        contacts = [c for c in data.contact if table in (c.geom1, c.geom2)
                    and part in (model.geom_bodyid[c.geom1], model.geom_bodyid[c.geom2])]
        assert contacts, 'contact must remain enabled'
        assert min(c.dist for c in contacts) > -1e-5


def test_partial_support_slows_only_final_upper_insertion():
    lower = {'type': 'brick_2x2', 'position': [.016, 0., .0191], 'yaw_rad': math.pi/2}
    upper = {'type': 'brick_2x2', 'position': [0., 0., .0382], 'yaw_rad': 0.}
    assert insertion_speed_scale(upper, [lower], 0.) == .2
    aligned = dict(upper, position=[.016, 0., .0382])
    assert insertion_speed_scale(aligned, [lower], 0.) == .4
    assert insertion_speed_scale(dict(upper, position=[0., 0., 0.]), [], 0.) == .4


@pytest.mark.parametrize('tilt_deg,height,trigger', [
    (.1, .04, False), (2., .04, True), (2., .01, False), (2., .02, False)])
def test_grip_recovery_requires_observed_tilt_and_free_space(tilt_deg, height, trigger):
    from mj_bridge.reference_executor import free_space_grip_drift
    angle = math.radians(tilt_deg) / 2
    state = {'position': [0., 0., height],
             'quaternion_wxyz': [math.cos(angle), math.sin(angle), 0., 0.]}
    assert free_space_grip_drift(state, {'position': [0., 0., 0.]}) == trigger


def test_pose_recovery_rejects_unvalidated_visual_tracking():
    executor = OracleExecutor.__new__(OracleExecutor)
    executor.perception = object()
    with pytest.raises(ExecutionFailure, match='validated in-hand tracking'):
        executor.descend_with_pose_feedback({}, 0.)


def test_mounting_fixture_matches_half_pitch_product_without_moving_parts(tmp_path):
    import yaml
    import numpy as np
    from mj_bridge.benchmark_core import plastic_fixture_offset
    product = {'schema_version': 1, 'product': {'name': 'half_pitch'}, 'blocks': [
        {'id': 'part', 'type': 'brick_4x2', 'color': 'green',
         'target': {'position': [-.016, .008, 0.], 'yaw_deg': 90.}}]}
    path = tmp_path / 'product.yaml'
    path.write_text(yaml.safe_dump(product))
    original = generate_episode(product, seed=42, connection_mode='physics')
    episode, scene = generate(str(path), 42, str(tmp_path / 'run'),
                              connection_mode='physics', contact_profile='plastic')
    assert episode['target_blocks'] == original['target_blocks']
    assert episode['product'] == product
    assert plastic_fixture_offset(product) == [0., -.008]
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    studs = np.array([data.geom_xpos[i, :2] for i in range(model.ngeom)
                     if model.geom(i).name.startswith('assembly_base_stud_')])
    target = episode['target_blocks'][0]
    # Every rotated part stud must lie on a fixture stud, not between studs.
    for x in (-.008, .008):
        for y in (-.024, -.008, .008, .024):
            expected = np.array(target['position'][:2]) + [x, y]
            assert np.min(np.linalg.norm(studs - expected, axis=1)) < 1e-7
    incompatible = dict(product, blocks=product['blocks'] + [
        {'id': 'other', 'type': 'brick_2x2', 'color': 'red',
         'target': {'position': [.05, 0., 0.], 'yaw_deg': 0.}}])
    with pytest.raises(ProductError, match='one plastic stud grid'):
        plastic_fixture_offset(incompatible)


def test_fast_grip_drift_interrupts_within_the_physics_step():
    import numpy as np
    import time
    from mj_bridge.reference_executor import _GripDrift
    executor = OracleExecutor.__new__(OracleExecutor)
    target = {'id': 'part', 'body_name': 'part', 'position': [0., 0., 0.]}
    executor._descent_guard_target = target
    executor.qadr = np.array([0])
    executor.model = SimpleNamespace(opt=SimpleNamespace(timestep=.0005),
                                     body=lambda name: SimpleNamespace(id=0))
    executor.data = SimpleNamespace(qpos=np.array([.2]),
        xpos=np.array([[0., 0., .03]]), xquat=np.array([[1., 0., 0., 0.]]))
    steps = []
    def physical_step():
        steps.append(1)
        angle = math.radians(1.2) / 2
        executor.data.xquat[0] = [math.cos(angle), math.sin(angle), 0., 0.]
    executor.node = SimpleNamespace(episode_start_time=time.monotonic(), benchmark_config={},
        step_pid=physical_step, connection_mode='physics', max_part_penetration_m=0.,
        max_robot_environment_penetration_m=0., target_qpos=np.zeros(1),
        update_safety_metrics=lambda: None)
    with pytest.raises(_GripDrift):
        executor.step(.02)
    assert len(steps) == 1  # Do not continue 39 more steps before noticing drift.
    assert executor.node.target_qpos.tolist() == [.2]
