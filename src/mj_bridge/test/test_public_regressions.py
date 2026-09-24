import math
from pathlib import Path
import numpy as np
import pytest
from mj_bridge.benchmark_core import ProductError, normalize_product, generate_episode, score_episode
from mj_bridge.perception import validate_frame


def product():
    return normalize_product({'blocks': [{'name': 'red_2x2', 'pos': [0, 0, 0]}]})


def test_oracle_doctor_reports_missing_ros_dependency(monkeypatch, capsys):
    from mj_bridge import benchmark_cli
    monkeypatch.setattr(benchmark_cli.importlib.util, 'find_spec',
                        lambda name: None if name == 'control_msgs' else object())
    assert benchmark_cli.main(['doctor', '--backend', 'oracle']) == 2
    assert 'missing Python module: control_msgs' in capsys.readouterr().out


@pytest.mark.parametrize('summary', [
    {'finished':False,'episodes':1,'successes':1,'results':[{'success':True}]},
    {'finished':True,'episodes':1,'successes':1,'results':[{'success':False}]},
])
def test_report_rejects_partial_or_inconsistent_suite(summary):
    import runpy
    script = Path(__file__).resolve().parents[3] / 'scripts/write_validation_report.py'
    validate = runpy.run_path(str(script))['require_complete_summary']
    with pytest.raises(ValueError):
        validate(summary)


def test_square_placement_avoids_adjacent_finger_collision():
    from mj_bridge.reference_executor import placement_yaw
    target = {'type':'brick_2x2', 'position':[0, .016, .0384], 'yaw_rad':0.}
    neighbour = {'type':'brick_2x2', 'position':[0, -.016, .0384], 'yaw_rad':0.}
    assert placement_yaw(target, [neighbour]) == pytest.approx(math.pi/2)
    assert placement_yaw(target, []) == 0.


def test_vertical_approach_for_neighbours_and_bridge_supports():
    from mj_bridge.reference_executor import requires_vertical_approach
    target = {'type':'brick_2x2', 'position':[0,0,.0384], 'yaw_rad':0.}
    neighbour = {'type':'brick_2x2', 'position':[0,.032,.0384], 'yaw_rad':0.}
    supports = [{'type':'brick_2x2', 'position':[0,y,.0192], 'yaw_rad':0.}
                for y in (-.016,.016)]
    assert requires_vertical_approach(target, [neighbour])
    assert requires_vertical_approach(target, supports)
    assert not requires_vertical_approach(target, supports[:1])
    assert not requires_vertical_approach(target, [])


def test_geometry_rejects_overlapping_rectangular_bricks():
    from mj_bridge.product_geometry import audit_product
    product = normalize_product({'blocks':[
        {'name':'a_4x2', 'pos':[0,0,0]}, {'name':'b_4x2', 'pos':[.032,0,0]}]})
    report = audit_product(product)
    assert not report['valid_for_snap']
    assert any(i['code'] == 'solid_overlap' for i in report['issues'])


def test_geometry_rejects_floating_goal_and_accepts_stacked_goal():
    from mj_bridge.product_geometry import audit_product
    floating = normalize_product({'blocks':[{'name':'a_2x2','pos':[0,0,.0384]}]})
    assert not audit_product(floating)['valid_for_snap']
    stack = normalize_product({'blocks':[{'name':f'a_2x2_{i}','pos':[0,0,.0192*i]} for i in range(3)]})
    assert audit_product(stack)['valid_for_snap']


def test_invalid_goal_fails_before_scene_generation(monkeypatch, capsys, tmp_path):
    from mj_bridge import benchmark_cli
    monkeypatch.setattr(benchmark_cli, 'generate', lambda *a:pytest.fail('invalid goal generated a scene'))
    code = benchmark_cli.main(['run', '--product', str(CATALOG/'final_product_t.yaml'),
                               '--output-dir', str(tmp_path/'unused')])
    assert code == 2
    assert 'solid_overlap' in capsys.readouterr().out
    assert not (tmp_path/'unused').exists()


@pytest.mark.parametrize("gripped", [False, True])
def test_contact_snapshot_survives_contact_array_rebuild(gripped):
    import ast
    import mujoco
    import threading
    from types import SimpleNamespace
    source = Path(__file__).resolve().parents[1] / 'mj_bridge/mj_bridge3.py'
    cls = next(n for n in ast.parse(source.read_text()).body
               if isinstance(n, ast.ClassDef) and n.name == 'MuJoCoActionServer')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef)
                  and n.name == 'auto_weld_touching_bricks')
    namespace = {'np':np, 'mujoco':mujoco, 'ASSEMBLY_BASE_NAME':'assembly_base_plate'}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), namespace)
    finger = '<body name="left_finger" pos="-.06 0 .025"><geom type="sphere" size=".006"/></body>' if gripped else ''
    model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <body name="assembly_base_plate"><geom type="box" size=".2 .2 .01"/></body>
      <body name="brick_a" pos="-.04 0 .015"><freejoint/><geom type="box" size=".016 .016 .01"/></body>
      <body name="brick_b" pos=".04 0 .015"><freejoint/><geom type="box" size=".016 .016 .01"/></body>
      ''' + finger + '</worldbody></mujoco>')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    original_ncon = data.ncon
    called = []
    def snap(name):
        called.append(name)
        data.qpos[2] = 1.
        data.qpos[9] = 1.
        mujoco.mj_forward(model, data)
    node = SimpleNamespace(model=model, data=data, mj_lock=threading.RLock(),
        welded_pairs=set(), snap_brick_to_base_plate=snap,
        should_weld_bottom_to_top=lambda *args:False)
    namespace['auto_weld_touching_bricks'](node)
    assert original_ncon > 0 and data.ncon == 0
    assert set(called) == ({'brick_b'} if gripped else {'brick_a','brick_b'})


@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf')])
def test_nonfinite_product_rejected(value):
    with pytest.raises(ProductError, match='finite'):
        normalize_product({'blocks': [{'name': 'red_2x2', 'pos': [value, 0, 0]}]})


def test_future_schema_not_silently_reinterpreted_as_legacy():
    with pytest.raises(ProductError, match='schema_version'):
        normalize_product({'schema_version': 2, 'blocks': [{'name': 'red_2x2', 'pos': [0, 0, 0]}]})


def test_upside_down_brick_is_not_successful():
    episode = generate_episode(product(), seed=42)
    target = episode['target_blocks'][0]
    actual = {'blocks': {target['id']: dict(target, quaternion_wxyz=[0, 1, 0, 0])}}
    assert not score_episode(episode, actual)['success']


def test_empty_episode_is_rejected():
    with pytest.raises(ProductError):
        score_episode({'target_blocks': []}, {})


def test_invalid_camera_extrinsics_rejected():
    transform = np.eye(4)
    transform[0, 0] = 2
    with pytest.raises(ValueError, match='rigid'):
        validate_frame(np.zeros((4, 5, 3), dtype=np.uint8), np.ones((4, 5)), np.eye(3), transform)


CATALOG = Path(__file__).resolve().parents[3] / 'examples/products/catalog'
VARIANTS = CATALOG.parent / 'supported_variants'
@pytest.mark.parametrize('path', sorted(CATALOG.glob('*.yaml')) + sorted(VARIANTS.glob('*.yaml')), ids=lambda p: p.stem)
def test_catalog_generates_loadable_scene(path, tmp_path):
    import mujoco
    from mj_bridge.benchmark_cli import generate
    if path.parent == VARIANTS:
        from mj_bridge.benchmark_cli import normalized_from_path
        from mj_bridge.product_geometry import audit_product
        assert audit_product(normalized_from_path(str(path)))['valid_for_snap']
    manifest, scene = generate(str(path), 42, str(tmp_path))
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert len(manifest['spawned_blocks']) == len(manifest['target_blocks'])
    for block in manifest['spawned_blocks']:
        assert model.body(block['body_name']).id > 0
    assert not score_episode(manifest, {'blocks': {b['id']: b for b in manifest['spawned_blocks']}})['success']


def test_rgbd_publisher_does_not_expose_oracle():
    # Exercise the actual bridge method with a minimal transport, without ROS init.
    import ast
    import json
    from types import SimpleNamespace
    source = Path(__file__).resolve().parents[1] / 'mj_bridge/mj_bridge3.py'
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'MuJoCoActionServer')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'publish_public_state')
    namespace = {'String': SimpleNamespace, 'json': json}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), namespace)
    oracle_messages = []
    node = SimpleNamespace(observation_mode='rgbd',
        oracle_pub=SimpleNamespace(publish=oracle_messages.append),
        goal_pub=SimpleNamespace(publish=lambda msg: None),
        benchmark_pub=SimpleNamespace(publish=lambda msg: None),
        episode_manifest={'episode_id': 'test', 'product': {}, 'target_blocks': []},
        current_block_state=lambda: (_ for _ in ()).throw(AssertionError('Oracle accessed')))
    namespace['publish_public_state'](node, {'success': False})
    assert oracle_messages == []


def test_base_snap_uses_collision_frame_offset():
    import ast
    import mujoco
    from types import SimpleNamespace
    source = Path(__file__).resolve().parents[1] / 'mj_bridge/mj_bridge3.py'
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'MuJoCoActionServer')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'snap_brick_to_base_plate')
    namespace = dict(np=np, mujoco=mujoco, ASSEMBLY_BASE_NAME='assembly_base_plate',
        ASSEMBLY_BASE_CENTER_X=.35, ASSEMBLY_BASE_CENTER_Y=.35, BASE_PLATE_HALF_X=.096,
        BASE_PLATE_HALF_Y=.096, BRICK_BODY_HALF_HEIGHT=.0095, COLLISION_Z_OFFSET=-.009,
        BASE_STUD_TOP_Z=.05, BASE_SNAP_VERTICAL_TOL=.012, BRICK_ON_BASE_CENTER_Z=.0685,
        TABLE_TOP_Z=.04, snap_xy_to_stud_grid=lambda x,y:(x,y),
        yaw_from_quat_wxyz=lambda q:0., snap_yaw_to_90=lambda yaw:0.,
        quat_wxyz_from_yaw=lambda yaw:np.array([1.,0.,0.,0.]))
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), namespace)
    model = mujoco.MjModel.from_xml_string('<mujoco><worldbody><body name="brick" pos=".35 .35 .075"><freejoint/><geom type="box" size=".016 .016 .0095"/></body></worldbody></mujoco>')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    node = SimpleNamespace(model=model, data=data, welded_pairs=set(), fake_welds=[],
        get_logger=lambda:SimpleNamespace(info=lambda msg:None))
    assert namespace['snap_brick_to_base_plate'](node, 'brick')
    assert data.qpos[2] == pytest.approx(.0685)


def test_torque_controller_converts_affine_actuator_units():
    import ast
    import mujoco
    import threading
    from types import SimpleNamespace
    source = Path(__file__).resolve().parents[1] / 'mj_bridge/mj_bridge3.py'
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'MuJoCoActionServer')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'step_pid')
    namespace = dict(np=np, mujoco=mujoco, KP=1500., KD=120., MAX_TORQUE=150.)
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), namespace)
    model = mujoco.MjModel.from_xml_string('''<mujoco><option gravity="0 0 0"/>
      <worldbody><body><joint name="joint"/><geom type="sphere" size=".1"/></body></worldbody>
      <actuator><general name="actuator1" joint="joint" biastype="affine"
      gainprm="4500" biasprm="0 -4500 -450"/></actuator></mujoco>''')
    data = mujoco.MjData(model)
    data.qpos[0] = .4
    mujoco.mj_forward(model, data)
    node = SimpleNamespace(model=model, data=data, target_qpos=np.array([.401]), episode_manifest=None, mj_lock=threading.RLock())
    # mj_step leaves actuator_length/velocity at the pre-integration state.
    # Repeated control updates must use the current joint state, without an
    # extra mj_forward between steps masking stale derived quantities.
    for _ in range(3):
        expected = np.clip(1500. * (.401 - data.qpos[0])
                           - 120. * data.qvel[0] + data.qfrc_bias[0], -150., 150.)
        namespace['step_pid'](node)
        assert data.actuator_force[0] == pytest.approx(expected, abs=1e-8)


@pytest.mark.parametrize("elapsed,penetration", [(301., 0.), (0., .0006)])
def test_invalid_perfect_assembly_is_not_episode_success(elapsed, penetration):
    import ast
    import threading
    from types import SimpleNamespace
    source = Path(__file__).resolve().parents[1] / 'mj_bridge/mj_bridge3.py'
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'MuJoCoActionServer')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'benchmark_result')
    namespace = {'time': SimpleNamespace(monotonic=lambda:elapsed), 'score_episode':score_episode}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), namespace)
    episode = generate_episode(product(), seed=42)
    node = SimpleNamespace(mj_lock=threading.RLock(), episode_manifest=episode,
        episode_start_time=0., episode_timeout=False, benchmark_config={'max_episode_time_s':300},
        current_block_state=lambda:{'blocks':{b['id']:b for b in episode['target_blocks']}},
        max_gripper_penetration_m=penetration, max_part_penetration_m=0.,
        max_robot_environment_penetration_m=0., fake_welds=[], collision_count=0, stability_violations=0, observation_mode='oracle', connection_mode='snap')
    result = namespace['benchmark_result'](node)
    assert result['completion'] == 1.
    assert result['timeout'] is (elapsed >= 300.)
    assert result['gripper_contact_valid'] is (penetration <= .0005)
    assert result['success'] is False


def test_fingers_hold_surface_with_bounded_force():
    """Exercise the actual Panda finger model against a free rigid brick."""
    import copy
    import xml.etree.ElementTree as ET
    import mujoco
    source = Path(__file__).resolve().parents[1] / 'mj_bridge'
    panda = ET.parse(source / 'panda.xml').getroot()
    model_xml = ET.Element('mujoco')
    ET.SubElement(model_xml, 'compiler', angle='radian', autolimits='true',
                  meshdir=str(source / 'assets'))
    ET.SubElement(model_xml, 'option', timestep='.001', gravity='0 0 0',
                  integrator='implicitfast')
    model_xml.append(copy.deepcopy(panda.find('default')))
    model_xml.append(copy.deepcopy(panda.find('asset')))
    world = ET.SubElement(model_xml, 'worldbody')
    hand = copy.deepcopy(panda.find(".//body[@name='hand']"))
    hand.set('pos', '0 0 0')
    hand.set('quat', '1 0 0 0')
    hand.set('childclass', 'panda')
    world.append(hand)
    contacts = ET.SubElement(model_xml, 'contact')
    for finger_name in ('left_finger', 'right_finger'):
        ET.SubElement(contacts, 'exclude', body1='hand', body2=finger_name)
    brick = ET.SubElement(world, 'body', name='test_brick', pos='0 0 .1034')
    ET.SubElement(brick, 'freejoint')
    ET.SubElement(brick, 'geom', type='box', size='.016 .016 .0095', mass='.012',
                  solref='.004 1', solimp='.90 .98 .002', friction='5 .2 .02')
    actuators = ET.SubElement(model_xml, 'actuator')
    for actuator in panda.find('actuator'):
        if actuator.get('name', '').startswith('finger_actuator'):
            actuators.append(copy.deepcopy(actuator))
    model = mujoco.MjModel.from_xml_string(ET.tostring(model_xml, encoding='unicode'))
    data = mujoco.MjData(model)
    for name in ('panda_finger_joint1', 'panda_finger_joint2'):
        data.qpos[model.jnt_qposadr[model.joint(name).id]] = .04
    peak = 0.
    for _ in range(1200):
        mujoco.mj_step(model, data)
        assert np.max(np.abs(data.actuator_force)) <= 5.00001
        if data.ncon:
            peak = max(peak, float(-np.min(data.contact.dist)))
    opening = sum(data.qpos[model.jnt_qposadr[model.joint(name).id]]
                  for name in ('panda_finger_joint1', 'panda_finger_joint2'))
    assert .030 < opening < .033
    assert peak < .0005
    assert data.ncon > 0


@pytest.mark.parametrize('part', ['brick_2x2', 'brick_4x2'])
def test_hollow_physics_targets_have_no_solid_overlap(tmp_path, part):
    import mujoco
    from mj_bridge.benchmark_cli import generate
    from mj_bridge.benchmark_core import dump_yaml
    definition = {'schema_version':1, 'product':{'name':'two_layers'}, 'blocks':[
        {'id':str(i), 'type':part, 'color':'red',
         'target':{'position':[0,0,i*.0192], 'yaw_deg':0}} for i in range(2)]}
    path = tmp_path/'product.yaml'
    dump_yaml(path, definition)
    episode, scene = generate(str(path), 42, str(tmp_path/'run'), connection_mode='physics')
    assert episode['geometry'] == 'hollow_primitives_v1'
    assert episode['target_blocks'][0]['position'][2] == pytest.approx(.0646)
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    bodies = set()
    for block in episode['target_blocks']:
        body = model.body(block['body_name']).id
        bodies.add(body)
        q = model.jnt_qposadr[model.body_jntadr[body]]
        data.qpos[q:q+3] = block['position']
        data.qpos[q+3:q+7] = [1,0,0,0]
    mujoco.mj_forward(model, data)
    for contact in data.contact:
        a,b = model.geom_bodyid[contact.geom1], model.geom_bodyid[contact.geom2]
        if a in bodies and b in bodies:
            assert contact.dist >= -1e-6


def test_ik_kinematics_matches_full_forward(tmp_path):
    import mujoco
    from mj_bridge.benchmark_cli import generate
    _, scene = generate(str(CATALOG/'final_product_hammer.yaml'), 42,
                        str(tmp_path/'episode'), connection_mode='physics')
    model = mujoco.MjModel.from_xml_path(str(scene))
    full, kinematic = mujoco.MjData(model), mujoco.MjData(model)
    body = model.body('hand').id
    for angle in (0., .4, -.6):
        full.qpos[0] = angle
        kinematic.qpos[:] = full.qpos
        mujoco.mj_forward(model, full)
        mujoco.mj_kinematics(model, kinematic)
        mujoco.mj_comPos(model, kinematic)
        np.testing.assert_allclose(kinematic.xpos[body], full.xpos[body], atol=1e-12)
        np.testing.assert_allclose(kinematic.xmat[body], full.xmat[body], atol=1e-12)
        a, b = np.zeros((3, model.nv)), np.zeros((3, model.nv))
        c, d = np.zeros_like(a), np.zeros_like(b)
        point = full.xpos[body] + full.xmat[body].reshape(3,3) @ [0,0,.1034]
        mujoco.mj_jac(model, full, a, b, point, body)
        mujoco.mj_jac(model, kinematic, c, d, point, body)
        np.testing.assert_allclose(a, c, atol=1e-12)
        np.testing.assert_allclose(b, d, atol=1e-12)


@pytest.mark.parametrize('patch', [
    {'max_part_penetration_m': .00051},
    {'max_part_penetration_m': float('nan')},
    {'max_robot_environment_penetration_m': None},
    {'attachment_count': 1}, {'timeout': True},
    {'physical_contacts_valid': False}, {'connection_mode': 'snap'},
])
def test_physics_report_rejects_invalid_contact_evidence(patch):
    import runpy
    script = Path(__file__).resolve().parents[3] / 'scripts/write_validation_report.py'
    validate = runpy.run_path(str(script))['require_physics_evidence']
    result = dict(connection_mode='physics', max_part_penetration_m=.0001,
                  max_gripper_penetration_m=.0001, max_robot_environment_penetration_m=0.,
                  attachment_count=0, timeout=False, physical_contacts_valid=True,
                  gripper_contact_valid=True, execution_error=None)
    validate(result)
    result.update(patch)
    with pytest.raises(ValueError):
        validate(result)


def test_physics_executor_stops_on_first_excessive_contact():
    from types import SimpleNamespace
    import time
    from mj_bridge.reference_executor import OracleExecutor, ExecutionFailure
    calls = []
    node = SimpleNamespace(episode_start_time=time.monotonic(), benchmark_config={},
                           connection_mode='physics', max_part_penetration_m=0.,
                           max_robot_environment_penetration_m=0.)
    def step_pid():
        calls.append(True)
        node.max_robot_environment_penetration_m = .0006
    node.step_pid = step_pid
    executor = OracleExecutor.__new__(OracleExecutor)
    executor.node = node
    executor.model = SimpleNamespace(opt=SimpleNamespace(timestep=.0005))
    with pytest.raises(ExecutionFailure, match='motion stopped'):
        executor.step(.1)
    assert len(calls) == 1


def test_disassembly_plan_reverses_removals_and_keeps_grasp_frame():
    from mj_bridge.assembly_planner import plan_assembly, accessible
    from mj_bridge.benchmark_core import load_registry
    targets = [
        dict(id='left', type='brick_4x2', position=[-.016, 0, 0], yaw_rad=math.pi/2),
        dict(id='right', type='brick_4x2', position=[.016, 0, 0], yaw_rad=math.pi/2),
        dict(id='top', type='brick_2x2', position=[0, 0, .0192], yaw_rad=0.),
    ]
    plan = plan_assembly(targets)
    assert plan[-1]['block_id'] == 'top'
    remaining = list(targets)
    for step in reversed(plan):
        target = next(b for b in remaining if b['id'] == step['block_id'])
        assert accessible(target, remaining, step['grasp_spin_deg'], load_registry())
        assert target['yaw_rad'] + step['grasp_offset_rad'] == pytest.approx(
            math.radians(step['grasp_spin_deg']))
        remaining.remove(target)
    # Opposing neighbouring rectangles require the alternative world-axis grip.
    assert next(s for s in plan if s['block_id'] == 'left')['grasp_spin_deg'] == 0


def test_disassembly_plan_rejects_overlapping_goals():
    from mj_bridge.assembly_planner import plan_assembly, PlanningError
    targets = [dict(id=str(i), type='brick_4x2', position=[0, 0, 0], yaw_rad=0.)
               for i in range(2)]
    with pytest.raises(PlanningError, match='overlapping target'):
        plan_assembly(targets)


def test_legacy_planner_and_simulation_share_the_same_plan(tmp_path):
    import runpy, yaml
    from mj_bridge.assembly_planner import plan_assembly
    script = Path(__file__).resolve().parents[2] / 'panda_pick/src/myplanner.py'
    if not script.exists():
        pytest.skip('legacy research adapter is outside the public package')
    process = runpy.run_path(str(script))['process_blueprint']
    src, dest = tmp_path/'input.yaml', tmp_path/'plan.yaml'
    src.write_text(yaml.safe_dump({'blocks': [
        {'name': 'red_2x4', 'pos': [0, 0, 0], 'rotation': [0,0,90]}]}))
    process(src, dest)
    plan = plan_assembly([dict(id='red_2x4',type='brick_4x2',position=[0,0,0],yaw_rad=math.pi/2)])
    task = yaml.safe_load(dest.read_text())['tasksh'][0]
    assert task['name'] == plan[0]['block_id']
    assert task['grasp_spin'] == plan[0]['grasp_spin_deg']
    assert task['grasp_offset_rad'] == pytest.approx(plan[0]['grasp_offset_rad'])


def test_disassembly_plan_refuses_when_clearance_check_fails(monkeypatch):
    from mj_bridge import assembly_planner as planner
    monkeypatch.setattr(planner, 'accessible', lambda *args: False)
    with pytest.raises(planner.PlanningError, match='no collision-free'):
        planner.plan_assembly([dict(id='part', type='brick_2x2', position=[0,0,0], yaw_rad=0.)])


def test_grasp_clearance_checks_off_axis_neighbour_footprint():
    from mj_bridge.assembly_planner import accessible
    from mj_bridge.benchmark_core import load_registry
    target = dict(id='target', type='brick_2x2', position=[0,0,0], yaw_rad=0.)
    neighbour = dict(id='diagonal', type='brick_2x2', position=[.023,.035,0], yaw_rad=0.)
    assert not accessible(target, [target, neighbour], 0, load_registry())
    assert accessible(target, [target, neighbour], 90, load_registry())


def test_disassembly_checks_covering_footprint_not_only_centres():
    from mj_bridge.assembly_planner import plan_assembly
    base = dict(id='base', type='brick_4x2', position=[0,0,0], yaw_rad=0.)
    top = dict(id='top', type='brick_2x2', position=[.024,0,.0192], yaw_rad=0.)
    assert [step['block_id'] for step in plan_assembly([base, top])] == ['base','top']


@pytest.mark.parametrize('source,target,spin', [(0,0,0),(30,90,0),(-170,90,90),(45,0,90)])
def test_grasp_angles_preserve_the_product_orientation(source, target, spin):
    from mj_bridge.assembly_planner import resolve_grasp_yaws
    pick, place = resolve_grasp_yaws(math.radians(source), math.radians(target), spin)
    resulting_part_yaw = math.radians(source) + place - pick
    error = (resulting_part_yaw - math.radians(target) + math.pi) % (2*math.pi) - math.pi
    assert error == pytest.approx(0.)
    assert place == pytest.approx(math.radians(spin))


@pytest.mark.parametrize('initial_opening,blocked', [(.016,False),(.032,False),(.032,True)])
def test_release_waits_for_both_measured_fingers(initial_opening, blocked):
    import ast
    import threading
    import time
    from types import SimpleNamespace, MethodType
    import mujoco
    from mj_bridge.reference_executor import OracleExecutor, ExecutionFailure
    right_limit = .032 if blocked else .04
    model = mujoco.MjModel.from_xml_string(f'''<mujoco>
      <compiler autolimits="true"/>
      <option timestep=".0005" gravity="0 0 0" integrator="implicitfast"/>
      <worldbody>
        <body name="left_finger" pos="-.1 0 0"><joint name="panda_finger_joint1" type="slide" axis="0 1 0" range="0 .04"/><geom type="sphere" size=".004" mass=".03"/></body>
        <body name="right_finger" pos=".1 0 0"><joint name="panda_finger_joint2" type="slide" axis="0 1 0" range="0 {right_limit}"/><geom type="sphere" size=".004" mass=".03"/></body>
        <body name="part" pos="1 0 0"><geom type="sphere" size=".004"/></body>
      </worldbody>
      <actuator>
        <position name="finger_actuator1" joint="panda_finger_joint1" kp="5000" kv="80" forcerange="-5 5"/>
        <position name="finger_actuator2" joint="panda_finger_joint2" kp="5000" kv="80" forcerange="-5 5"/>
      </actuator></mujoco>''')
    data = mujoco.MjData(model)
    data.qpos[:] = initial_opening
    mujoco.mj_forward(model,data)
    source = Path(__file__).resolve().parents[1]/'mj_bridge/mj_bridge3.py'
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='MuJoCoActionServer')
    method = next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='step_pid')
    namespace = {'np':np,'mujoco':mujoco}
    exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),namespace)
    node=SimpleNamespace(model=model,data=data,target_qpos=data.qpos.copy(),mj_lock=threading.RLock(),
                         gripper_control=0.,gripper_target=0.,episode_manifest=None,
                         episode_start_time=time.monotonic(),benchmark_config={},connection_mode='physics',
                         max_part_penetration_m=0.,max_robot_environment_penetration_m=0.,update_safety_metrics=lambda:None)
    node.step_pid=MethodType(namespace['step_pid'],node)
    executor=OracleExecutor.__new__(OracleExecutor)
    executor.node,executor.model,executor.data=node,model,data
    target=dict(id='brick',body_name='part')
    if blocked:
        with pytest.raises(ExecutionFailure,match='retreat cancelled'):
            executor.open_gripper_before_retreat(target)
        assert data.qpos[0] >= .0395 and data.qpos[1] < .0395
    else:
        result=executor.open_gripper_before_retreat(target)
        assert data.time > .67
        assert min(result['finger_positions_m']) >= .0395
        assert result['target_contact'] is False
        executor.prepare_grasp_opening('brick_4x2', 0., 0.)
        assert data.qpos == pytest.approx([.02, .02], abs=.0005)
        executor.prepare_grasp_opening('brick_4x2', math.pi/2, 0.)
        assert data.qpos == pytest.approx([.036, .036], abs=.0005)


@pytest.mark.parametrize('failure', ['none', 'shift', 'tilt', 'missing'])
def test_released_part_gate_stops_before_building_on_failed_placement(failure):
    from types import SimpleNamespace
    from mj_bridge.reference_executor import OracleExecutor, ExecutionFailure
    target = dict(id='released', type='brick_2x2', position=[.35, .35, .0646], yaw_rad=0.)
    observed = dict(position=target['position'][:], yaw_rad=0., quaternion_wxyz=[1., 0., 0., 0.])
    if failure == 'shift': observed['position'][0] += .02
    if failure == 'tilt': observed['quaternion_wxyz'] = [math.cos(.1), math.sin(.1), 0., 0.]
    actual = {'blocks': {} if failure == 'missing' else {'released': observed}}
    executor = OracleExecutor.__new__(OracleExecutor)
    executor.perception = None
    executor.events = [{'block': 'released'}]
    # Pending blocks deliberately have no observation; the gate only checks released ones.
    executor.node = SimpleNamespace(episode_manifest={'target_blocks': [target, dict(target, id='pending')]},
                                    current_block_state=lambda: actual)
    if failure == 'none':
        executor.verify_released_parts()
    else:
        with pytest.raises(ExecutionFailure, match='released parts moved or tilted'):
            executor.verify_released_parts()


@pytest.mark.parametrize('part,yaw', [('brick_2x2', 0.), ('brick_4x2', math.pi/2)])
def test_adjacent_drop_has_clearance_for_small_placement_error(part, yaw):
    import xml.etree.ElementTree as ET
    import mujoco
    from mj_bridge.scene_builder import create_episode_brick_body
    root = ET.Element('mujoco')
    ET.SubElement(root, 'option', timestep='.0005', integrator='implicitfast')
    world = ET.SubElement(root, 'worldbody')
    ET.SubElement(world, 'geom', type='plane', size='.2 .2 .01')
    fixed = create_episode_brick_body(dict(type='brick_2x2', body_name='placed',
        geometry='hollow_primitives_v1', position=[.00005, 0., .0186]))
    fixed.remove(fixed.find('freejoint'))
    world.append(fixed)
    world.append(create_episode_brick_body(dict(type=part, body_name='falling',
        geometry='hollow_primitives_v1', position=[.032, 0., .05], yaw_rad=yaw)))
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    data = mujoco.MjData(model)
    pair = {model.body('placed').id, model.body('falling').id}
    for _ in range(1200):
        mujoco.mj_step(model, data)
        assert not any({int(model.geom_bodyid[c.geom1]), int(model.geom_bodyid[c.geom2])} == pair
                       for c in data.contact)
    body = model.body('falling').id
    assert data.xpos[body] == pytest.approx([.032, 0., .0186], abs=.0001)
    assert data.xmat[body, 8] > math.cos(math.radians(1.))


@pytest.mark.parametrize('part,yaw,spin', [('brick_4x2',0.,0), ('brick_4x2',math.pi/2,90), ('brick_2x2',0.,90)])
def test_planner_prefers_small_safe_jaw_span(part,yaw,spin):
    from mj_bridge.assembly_planner import plan_assembly
    target=dict(id='part',type=part,position=[0,0,0],yaw_rad=yaw)
    assert plan_assembly([target])[0]['grasp_spin_deg']==spin


def test_small_jaw_span_never_overrides_clearance():
    from mj_bridge.assembly_planner import plan_assembly
    target=dict(id='long',type='brick_4x2',position=[0,0,0],yaw_rad=0.)
    neighbour=dict(id='adjacent',type='brick_2x2',position=[0,.032,0],yaw_rad=0.)
    step=next(s for s in plan_assembly([target,neighbour]) if s['block_id']=='long')
    assert step['grasp_spin_deg']==90


@pytest.mark.parametrize('problem', ['position','flipped_orientation','settles'])
def test_grasp_pose_confirmation_uses_measured_pose(problem):
    from types import SimpleNamespace
    from mj_bridge.reference_executor import OracleExecutor,ExecutionFailure
    executor=OracleExecutor.__new__(OracleExecutor)
    executor.hand=0
    executor.tool_offset=np.zeros(3)
    desired=np.diag([1.,-1.,-1.])
    executor.data=SimpleNamespace(xmat=desired.reshape(1,9).copy(),xpos=np.zeros((1,3)))
    elapsed=[]
    if problem=='flipped_orientation':executor.data.xmat[:]=np.eye(3).reshape(1,9)
    else:executor.data.xpos[0,0]=.01
    def step(seconds):
        elapsed.append(seconds)
        if problem=='settles' and sum(elapsed)>.1:executor.data.xpos[:]=0.
    executor.step=step
    if problem=='settles':
        executor.confirm_grasp_pose([0,0,0],0.)
        assert sum(elapsed)>.1
    else:
        with pytest.raises(ExecutionFailure,match='closure cancelled'):
            executor.confirm_grasp_pose([0,0,0],0.,timeout_s=.2)
