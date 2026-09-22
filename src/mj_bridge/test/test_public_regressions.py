import math
from pathlib import Path
import numpy as np
import pytest
from mj_bridge.benchmark_core import ProductError, normalize_product, generate_episode, score_episode
from mj_bridge.perception import validate_frame


def product():
    return normalize_product({'blocks': [{'name': 'red_2x2', 'pos': [0, 0, 0]}]})


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


def test_contact_snapshot_survives_contact_array_rebuild():
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
    model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <body name="assembly_base_plate"><geom type="box" size=".2 .2 .01"/></body>
      <body name="brick_a" pos="-.04 0 .015"><freejoint/><geom type="box" size=".016 .016 .01"/></body>
      <body name="brick_b" pos=".04 0 .015"><freejoint/><geom type="box" size=".016 .016 .01"/></body>
      </worldbody></mujoco>''')
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
    assert set(called) == {'brick_a','brick_b'}


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
    node = SimpleNamespace(model=model, data=data, target_qpos=np.array([.401]), mj_lock=threading.RLock())
    namespace['step_pid'](node)
    assert data.actuator_force[0] == pytest.approx(1.5, abs=1e-8)


def test_timed_out_perfect_assembly_is_not_episode_success():
    import ast
    import threading
    from types import SimpleNamespace
    source = Path(__file__).resolve().parents[1] / 'mj_bridge/mj_bridge3.py'
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'MuJoCoActionServer')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'benchmark_result')
    namespace = {'time': SimpleNamespace(monotonic=lambda:301.), 'score_episode':score_episode}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), namespace)
    episode = generate_episode(product(), seed=42)
    node = SimpleNamespace(mj_lock=threading.RLock(), episode_manifest=episode,
        episode_start_time=0., episode_timeout=False, benchmark_config={'max_episode_time_s':300},
        current_block_state=lambda:{'blocks':{b['id']:b for b in episode['target_blocks']}},
        collision_count=0, stability_violations=0, observation_mode='oracle', connection_mode='snap')
    result = namespace['benchmark_result'](node)
    assert result['completion'] == 1.
    assert result['timeout'] is True
    assert result['success'] is False
