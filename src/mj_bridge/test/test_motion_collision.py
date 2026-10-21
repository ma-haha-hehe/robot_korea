"""Robot motion must reject collisions between otherwise reachable poses."""
from pathlib import Path
from types import SimpleNamespace
import math
import mujoco
import numpy as np
import pytest
from mj_bridge.benchmark_cli import generate
from mj_bridge.reference_executor import OracleExecutor, ExecutionFailure


def test_joint_path_detects_self_collision_between_clear_endpoints():
    model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <body name="link5"><geom type="sphere" size=".02"/></body>
      <body name="link1"><joint type="slide" axis="1 0 0"/>
        <geom type="sphere" size=".02" mass="1"/></body>
    </worldbody></mujoco>''')
    executor = OracleExecutor.__new__(OracleExecutor)
    executor.model = model
    executor.data = mujoco.MjData(model)
    executor.ik_data = mujoco.MjData(model)
    executor.qadr = np.array([0])
    executor.robot_body_mask = np.array([False, True, True])
    executor.environment_body_mask = np.array([True, False, False])
    executor.fixed_base_mask = np.zeros(3, dtype=bool)
    for q in (-.1, .1):
        executor.data.qpos[0] = q
        mujoco.mj_kinematics(model, executor.data)
        assert not executor.configuration_has_collision(executor.data)
    assert executor.joint_path_has_collision(np.array([-.1]), np.array([.1]))
    assert not executor.joint_path_has_collision(np.array([-.1]), np.array([-.07]))


@pytest.fixture(scope='module')
def panda_model(tmp_path_factory):
    root = Path(__file__).resolve().parents[3]
    output = tmp_path_factory.mktemp('panda_collision')
    _, scene = generate(str(root/'examples/products/catalog/final_product_blue_single.yaml'),
        42, str(output), connection_mode='physics', contact_profile='plastic', robot_base_x=-.05)
    return mujoco.MjModel.from_xml_path(str(scene))


def executor_for_model(model):
    data = mujoco.MjData(model)
    for index, value in enumerate((0., -.785, 0., -2.356, 0., 1.571, .785), 1):
        data.qpos[model.joint(f'panda_joint{index}').qposadr[0]] = value
    for index in (1, 2):
        data.qpos[model.joint(f'panda_finger_joint{index}').qposadr[0]] = .04
    mujoco.mj_forward(model, data)
    node = SimpleNamespace(model=model, data=data, connection_mode='physics')
    return OracleExecutor(node)


def test_real_panda_rejects_the_observed_base_grasp_self_collision(panda_model):
    executor = executor_for_model(panda_model)
    assert not executor.configuration_has_collision(executor.data)
    executor.data.qpos[executor.qadr] = [
        .842385513316143, -.8064528671141549, -1.5542339114600412,
        -2.9362443311414563, -1.3621347967086919, 2.311876298157459,
        -.2230189004759997]
    mujoco.mj_kinematics(panda_model, executor.data)
    assert executor.configuration_has_collision(executor.data)


def test_real_panda_rejects_blocked_orientation_and_accepts_equivalent_half_turn(panda_model):
    executor = executor_for_model(panda_model)
    position = np.array([.2272734, -.088566, .0956021])
    with pytest.raises(ExecutionFailure, match='colliding configurations'):
        executor.ik(position, math.pi/2)
    q = executor.ik(position, -math.pi/2)
    executor.ik_data.qpos[executor.qadr] = q
    mujoco.mj_kinematics(panda_model, executor.ik_data)
    assert not executor.configuration_has_collision(executor.ik_data)
    rotation = executor.ik_data.xmat[executor.hand].reshape(3, 3)
    tip = executor.ik_data.xpos[executor.hand] + rotation @ executor.tool_offset
    assert np.linalg.norm(tip-position) < .00015


def test_ik_must_not_accept_a_pose_rotated_by_exactly_180_degrees(panda_model):
    executor = executor_for_model(panda_model)
    rotation = executor.data.xmat[executor.hand].reshape(3, 3).copy()
    position = executor.data.xpos[executor.hand] + rotation @ executor.tool_offset
    desired = rotation @ np.diag([-1., -1., 1.])
    q = executor.ik(position, 0., target_rotation=desired)
    executor.ik_data.qpos[executor.qadr] = q
    mujoco.mj_kinematics(panda_model, executor.ik_data)
    actual = executor.ik_data.xmat[executor.hand].reshape(3, 3)
    angle = math.acos(float(np.clip((np.trace(desired.T@actual)-1)/2, -1., 1.)))
    assert angle < .003


def test_blocked_transfer_finds_checked_waypoint_before_any_motion():
    executor = OracleExecutor.__new__(OracleExecutor)
    start, goal = np.zeros(7), np.ones(7)
    executor.data = SimpleNamespace(qpos=start)
    executor.qadr = np.arange(7)
    executor.motion_replans = []
    executor.ik = lambda *args, **kwargs: goal.copy()
    checked = []
    def blocked(a, b):
        checked.append((a.copy(), b.copy()))
        return np.array_equal(a, start) and np.array_equal(b, goal)
    executor.joint_path_has_collision = blocked
    route = executor.plan_joint_transfer([.4, .2, .3], 0.)
    assert len(route) == 2
    assert np.array_equal(route[-1], goal)
    assert all(not blocked(a, b) for a, b in zip([start, *route], route))
    assert executor.motion_replans[0]['kind'] == 'joint_waypoint'


def test_blocked_transfer_fails_without_executing_any_segment():
    executor = OracleExecutor.__new__(OracleExecutor)
    executor.data = SimpleNamespace(qpos=np.zeros(7), xmat=np.eye(3).reshape(1, 9))
    executor.hand = 0
    executor.node = SimpleNamespace(episode_manifest={'contact_profile': 'plastic'})
    executor.qadr = np.arange(7)
    executor.motion_replans = []
    executor.ik = lambda *args, **kwargs: np.ones(7)
    executor.joint_path_has_collision = lambda *args: True
    executed = []
    executor.move_joint_segment = lambda target: executed.append(target)
    with pytest.raises(ExecutionFailure, match='no checked joint transfer'):
        executor.move([.4, .2, .3], 0.)
    assert not executed


@pytest.mark.parametrize('yaw,reference', [(math.pi,0.),(-math.pi/2,math.pi/2),
                                          (0.,2.8),(math.pi/2,-2.7),(8*math.pi,.1)])
def test_symmetric_jaw_goal_avoids_unnecessary_half_turn(yaw, reference):
    from mj_bridge.reference_executor import nearest_gripper_yaw
    result = nearest_gripper_yaw(yaw, reference)
    assert abs(result-reference) <= math.pi/2 + 1e-12
    assert abs(math.sin(result-yaw)) < 1e-12


@pytest.mark.parametrize('profile,expected', [('plastic',0.),('loose',math.pi)])
def test_only_plastic_motion_uses_the_validated_equivalent_jaw_policy(profile, expected):
    executor = OracleExecutor.__new__(OracleExecutor)
    executor.hand = 0
    executor.data = SimpleNamespace(xmat=np.eye(3).reshape(1, 9))
    executor.node = SimpleNamespace(episode_manifest={'contact_profile': profile})
    goals = []
    executor.plan_joint_transfer = lambda position,yaw: goals.append(yaw) or []
    executor.move([.4, .3, .2], math.pi)
    assert goals == [expected]
def test_press_gap_avoids_the_finger_contact_seen_after_empty_closure(panda_model):
    executor=executor_for_model(panda_model)
    fingers=[panda_model.joint(f'panda_finger_joint{i}').qposadr[0] for i in (1,2)]
    executor.data.qpos[fingers]=-.0005
    mujoco.mj_kinematics(panda_model,executor.data)
    assert executor.configuration_has_collision(executor.data)
    executor.data.qpos[fingers]=.0015
    mujoco.mj_kinematics(panda_model,executor.data)
    assert not executor.configuration_has_collision(executor.data)
