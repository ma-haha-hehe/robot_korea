#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import yaml
import threading
import numpy as np

import mujoco
import mujoco.viewer

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState

BASE = os.path.dirname(__file__)
sys.path.append(BASE)

from scene_builder import build

def yaw_from_quat_wxyz(q: np.ndarray) -> float:
    w, x, y, z = q
    return np.arctan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def quat_wxyz_from_yaw(yaw: float) -> np.ndarray:
    return np.array(
        [
            np.cos(yaw / 2.0),
            0.0,
            0.0,
            np.sin(yaw / 2.0),
        ],
        dtype=float,
    )


def snap_yaw_to_90(yaw: float) -> float:
    return round(yaw / (np.pi / 2.0)) * (np.pi / 2.0)


def snap_xy_to_stud_grid(x: float, y: float) -> tuple[float, float]:
    local_x = x - ASSEMBLY_BASE_CENTER_X
    local_y = y - ASSEMBLY_BASE_CENTER_Y

    snapped_local_x = round(local_x / STUD_PITCH) * STUD_PITCH
    snapped_local_y = round(local_y / STUD_PITCH) * STUD_PITCH

    return (
        ASSEMBLY_BASE_CENTER_X + snapped_local_x,
        ASSEMBLY_BASE_CENTER_Y + snapped_local_y,
    )

def infer_brick_type_from_name(name: str):
    if "2x2" in name:
        return "brick_2x2"
    if "4x2" in name or "2x4" in name:
        return "brick_4x2"
    return None


def yaw_from_quat_wxyz(q: np.ndarray) -> float:
    """
    q 是 MuJoCo 顺序: w x y z
    """
    w, x, y, z = q
    return np.arctan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def get_world_half_size(body_name: str, quat_wxyz: np.ndarray):
    """
    根据积木类型和 yaw，得到世界坐标下的 XY 半尺寸。
    适合你的 0 / 90 / 180 / 270 度积木放置。
    """
    brick_type = infer_brick_type_from_name(body_name)

    if brick_type not in BRICK_HALF_SIZE:
        return None

    hx, hy, hz = BRICK_HALF_SIZE[brick_type]

    yaw = yaw_from_quat_wxyz(quat_wxyz)

    c = abs(np.cos(yaw))
    s = abs(np.sin(yaw))

    world_hx = c * hx + s * hy
    world_hy = s * hx + c * hy

    return world_hx, world_hy, hz


def xy_overlap_area(pos_a, half_a, pos_b, half_b) -> float:
    ax1 = pos_a[0] - half_a[0]
    ax2 = pos_a[0] + half_a[0]
    ay1 = pos_a[1] - half_a[1]
    ay2 = pos_a[1] + half_a[1]

    bx1 = pos_b[0] - half_b[0]
    bx2 = pos_b[0] + half_b[0]
    by1 = pos_b[1] - half_b[1]
    by2 = pos_b[1] + half_b[1]

    overlap_x = max(0.0, min(ax2, bx2) - max(ax1, bx1))
    overlap_y = max(0.0, min(ay2, by2) - max(ay1, by1))

    return overlap_x * overlap_y

# ============================================================
# 路径配置
# ============================================================

SOURCE_DIR = "/home/i6user/Desktop/robot_lego/src/mj_bridge/mj_bridge"
MODEL_XML_PATH = os.path.join(SOURCE_DIR, "scene.xml")
YAML_CONFIG_PATH = os.path.join(SOURCE_DIR, "initial_positions.yaml")


# ============================================================
# 手臂控制参数
# ============================================================

KP = 1500.0
KD = 120.0
MAX_TORQUE = 150.0


# ============================================================
# 仿真参数
# ============================================================

SIM_SUBSTEPS = 5
LOOP_DT = 0.016


# ============================================================
# 夹爪参数
# ============================================================

GRIPPER_OPEN_VALUE = 0.04

# 这里可以接近 0，让夹爪尽量合紧
GRIPPER_CLOSE_VALUE = 0.0

# 夹爪 action 返回前等待时间
# 注意：真正的平滑运动由 update_gripper_target() 控制
GRIPPER_ACTION_EXTRA_SLEEP = 0.1

# ============================================================
# 积木 fake weld 判定参数
# ============================================================

BRICK_HALF_SIZE = {
    "brick_2x2": (0.016, 0.016, 0.0095),
    "brick_4x2": (0.032, 0.016, 0.0095),
}

MIN_OVERLAP_RATIO = 0.25

# 上面积木底面和下面积木顶面的 z 方向容差
VERTICAL_CONTACT_TOL = 0.006

# ============================================================
# assembly base plate snap 参数
# ============================================================

ASSEMBLY_BASE_NAME = "assembly_base_plate"

ASSEMBLY_BASE_CENTER_X = 0.25
ASSEMBLY_BASE_CENTER_Y = 0.0

STUD_PITCH = 0.016

BRICK_BODY_HALF_HEIGHT = 0.0095

TABLE_TOP_Z = 0.04
BASE_PLATE_THICKNESS = 0.006
BASE_STUD_HEIGHT = 0.004

BASE_STUD_TOP_Z = TABLE_TOP_Z + BASE_PLATE_THICKNESS + BASE_STUD_HEIGHT
BRICK_ON_BASE_CENTER_Z = BASE_STUD_TOP_Z + BRICK_BODY_HALF_HEIGHT

BASE_PLATE_HALF_X = 12 * STUD_PITCH / 2.0
BASE_PLATE_HALF_Y = 12 * STUD_PITCH / 2.0

BASE_SNAP_VERTICAL_TOL = 0.012


class MuJoCoActionServer(Node):
    def __init__(self):
        super().__init__("mj_action_server_node")
        self.get_logger().info("正在启动 MuJoCo Action Server...")

        # ====================================================
        # 1. 生成并加载 MuJoCo 模型
        # ====================================================

        build()

        self.model = mujoco.MjModel.from_xml_path(MODEL_XML_PATH)
        self.data = mujoco.MjData(self.model)
        self.mj_lock = threading.RLock()

        # ====================================================
        # 2. fake weld：接触后固定积木
        # ====================================================

        self.welded_pairs = set()
        self.fake_welds = []

        # ====================================================
        # 3. 关节名
        # ====================================================

        self.arm_joint_names = [
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",
        ]

        self.finger_joint_names = [
            "panda_finger_joint1",
            "panda_finger_joint2",
        ]

        self.all_joint_names = self.arm_joint_names + self.finger_joint_names

        self.joint_qpos_addr = {}
        self.joint_qvel_addr = {}

        for name in self.all_joint_names:
            jid = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_JOINT,
                name,
            )

            if jid == -1:
                self.get_logger().warn(f"MuJoCo 模型里找不到 joint: {name}")
                continue

            self.joint_qpos_addr[name] = self.model.jnt_qposadr[jid]
            self.joint_qvel_addr[name] = self.model.jnt_dofadr[jid]

        # ====================================================
        # 4. 手臂目标
        # ====================================================

        self.target_qpos = np.zeros(self.model.nq)
        self.target_qvel = np.zeros(self.model.nv)

        self.current_goal_handle = None
        self.trajectory = None
        self.active_joint_names = []

        self.traj_start_wall_time = 0.0
        self.traj_duration = 0.0
        self.is_executing = False
        self.cancel_requested = False

        # ====================================================
        # 5. 夹爪目标：平滑插值版本
        # ====================================================

        # 当前 actuator 目标
        self.gripper_target = GRIPPER_OPEN_VALUE

        # 夹爪插值起点
        self.gripper_start_value = GRIPPER_OPEN_VALUE

        # 夹爪插值终点
        self.gripper_goal_value = GRIPPER_OPEN_VALUE

        # 插值开始时间
        self.gripper_start_time = 0.0

        # 插值持续时间
        self.gripper_duration = 1.0

        # 是否正在插值
        self.gripper_moving = False

        # open / close_hold
        self.gripper_mode = "open"

        # ====================================================
        # 6. 初始化位姿
        # ====================================================

        self.load_initial_positions()

        with self.mj_lock:
            self.target_qpos[:] = self.data.qpos[:]
            self.target_qvel[:] = 0.0

        # ====================================================
        # 7. 发布 joint_states，给 MoveIt / RViz 用
        # ====================================================

        self.joint_state_pub = self.create_publisher(
            JointState,
            "/joint_states",
            20,
        )

        self.joint_state_timer = self.create_timer(
            0.02,
            self.publish_joint_states,
        )

        # ====================================================
        # 8. Action Server
        # ====================================================

        self._arm_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/mj_panda_arm_controller/follow_joint_trajectory",
            execute_callback=self.execute_arm_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_arm_accepted_callback,
        )

        self._hand_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/mj_panda_hand_controller/follow_joint_trajectory",
            execute_callback=self.execute_hand_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info("手臂与夹爪 Action Server 均已就绪")
        self.get_logger().info("Arm action: /mj_panda_arm_controller/follow_joint_trajectory")
        self.get_logger().info("Hand action: /mj_panda_hand_controller/follow_joint_trajectory")

    # =========================================================
    # 初始位姿
    # =========================================================

    def load_initial_positions(self):
        if not os.path.exists(YAML_CONFIG_PATH):
            self.get_logger().warn(f"没有找到 initial_positions.yaml: {YAML_CONFIG_PATH}")
            return

        with open(YAML_CONFIG_PATH, "r", encoding="utf-8") as f:
            config = yaml.safe_load(f)

        positions = config.get("initial_positions", {})

        with self.mj_lock:
            for j_name, j_val in positions.items():
                jid = mujoco.mj_name2id(
                    self.model,
                    mujoco.mjtObj.mjOBJ_JOINT,
                    j_name,
                )

                if jid == -1:
                    self.get_logger().warn(f"初始位姿里有未知 joint: {j_name}")
                    continue

                qadr = self.model.jnt_qposadr[jid]
                self.data.qpos[qadr] = float(j_val)
                self.target_qpos[qadr] = float(j_val)

            # 默认夹爪张开
            for name in self.finger_joint_names:
                if name in self.joint_qpos_addr:
                    qadr = self.joint_qpos_addr[name]
                    self.data.qpos[qadr] = GRIPPER_OPEN_VALUE
                    self.target_qpos[qadr] = GRIPPER_OPEN_VALUE

            mujoco.mj_forward(self.model, self.data)

        self.get_logger().info("初始关节位姿已加载")

    # =========================================================
    # Action 通用回调
    # =========================================================

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn("收到 trajectory cancel request")

        with self.mj_lock:
            self.cancel_requested = True
            self.is_executing = False

        return CancelResponse.ACCEPT

    # =========================================================
    # 手臂 Action
    # =========================================================

    def handle_arm_accepted_callback(self, goal_handle):
        with self.mj_lock:
            if self.current_goal_handle is not None and self.current_goal_handle.is_active:
                self.current_goal_handle.abort()

            self.current_goal_handle = goal_handle

        goal_handle.execute()

    def execute_arm_callback(self, goal_handle):
        traj = goal_handle.request.trajectory

        if not traj.points:
            self.get_logger().error("手臂轨迹为空")
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        joint_names = list(traj.joint_names)

        for name in joint_names:
            if name not in self.joint_qpos_addr:
                self.get_logger().error(f"MuJoCo 模型里没有 joint: {name}")
                goal_handle.abort()
                return FollowJointTrajectory.Result()

        last_point = traj.points[-1]

        duration = (
            last_point.time_from_start.sec
            + last_point.time_from_start.nanosec * 1e-9
        )

        if duration <= 0.0:
            duration = 0.1

        with self.mj_lock:
            self.trajectory = traj
            self.active_joint_names = joint_names
            self.traj_start_wall_time = time.time()
            self.traj_duration = duration
            self.is_executing = True
            self.cancel_requested = False

        self.get_logger().info(
            f"收到手臂轨迹请求，joint数={len(joint_names)}, "
            f"point数={len(traj.points)}, duration={duration:.3f}s"
        )

        timeout = duration * 3.0 + 5.0
        start_wait = time.time()

        while rclpy.ok():
            with self.mj_lock:
                executing = self.is_executing
                cancelled = self.cancel_requested

            if cancelled:
                goal_handle.canceled()
                return FollowJointTrajectory.Result()

            if not executing:
                break

            if time.time() - start_wait > timeout:
                self.get_logger().error(f"手臂轨迹执行超时: timeout={timeout:.3f}s")
                with self.mj_lock:
                    self.is_executing = False
                goal_handle.abort()
                return FollowJointTrajectory.Result()

            time.sleep(0.01)

        if goal_handle.is_active:
            goal_handle.succeed()
            self.get_logger().info("MuJoCo arm trajectory 执行完成，返回 success")

        return FollowJointTrajectory.Result()

    # =========================================================
    # 夹爪 Action：平滑开合
    # =========================================================

    def execute_hand_callback(self, goal_handle):
        """
        旧问题：
        之前这里直接 self.gripper_target = cmd_val。
        这样目标从 0.04 瞬间跳到 0.0，所以夹爪会突然猛合。

        现在：
        记录起点、终点、持续时间。
        update_gripper_target() 在主循环里慢慢插值。
        """
        traj = goal_handle.request.trajectory

        if not traj.points:
            self.get_logger().error("夹爪轨迹为空")
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        point = traj.points[-1]

        if not point.positions:
            self.get_logger().error("夹爪轨迹 positions 为空")
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        cmd_val = float(point.positions[0])
        cmd_val = max(0.0, min(0.04, cmd_val))

        duration = (
            point.time_from_start.sec
            + point.time_from_start.nanosec * 1e-9
        )

        if duration <= 0.0:
            duration = 1.0

        with self.mj_lock:
            # 从当前实际目标开始插值，避免跳变
            self.gripper_start_value = self.gripper_target
            self.gripper_goal_value = cmd_val
            self.gripper_start_time = time.time()
            self.gripper_duration = duration
            self.gripper_moving = True

            if cmd_val >= 0.03:
                self.gripper_mode = "open"
                self.get_logger().info(
                    f"夹爪开始平滑 OPEN: {self.gripper_start_value:.4f} -> "
                    f"{self.gripper_goal_value:.4f}, duration={duration:.2f}s"
                )
            else:
                self.gripper_mode = "close_hold"
                self.get_logger().info(
                    f"夹爪开始平滑 CLOSE_HOLD: {self.gripper_start_value:.4f} -> "
                    f"{self.gripper_goal_value:.4f}, duration={duration:.2f}s"
                )

        # 等待插值完成，再返回 action success
        # 这样 C++ 里的 driveGripperAction() 会真的等夹爪合完
        start_wait = time.time()
        timeout = duration + 3.0

        while rclpy.ok():
            with self.mj_lock:
                moving = self.gripper_moving

            if not moving:
                break

            if time.time() - start_wait > timeout:
                self.get_logger().error("夹爪 action 等待超时")
                goal_handle.abort()
                return FollowJointTrajectory.Result()

            time.sleep(0.01)

        time.sleep(GRIPPER_ACTION_EXTRA_SLEEP)

        if goal_handle.is_active:
            goal_handle.succeed()

        return FollowJointTrajectory.Result()

    def update_gripper_target(self):
        """
        每一帧更新夹爪目标值。

        gripper_target 会从 start_value 平滑变化到 goal_value。
        MuJoCo position actuator 每帧追这个 gripper_target。
        """
        with self.mj_lock:
            if not self.gripper_moving:
                return

            elapsed = time.time() - self.gripper_start_time
            alpha = elapsed / self.gripper_duration
            alpha = max(0.0, min(1.0, alpha))

            self.gripper_target = (
                self.gripper_start_value
                + alpha * (self.gripper_goal_value - self.gripper_start_value)
            )

            if alpha >= 1.0:
                self.gripper_target = self.gripper_goal_value
                self.gripper_moving = False

    # =========================================================
    # 手臂轨迹插值
    # =========================================================

    def update_action_state(self):
        with self.mj_lock:
            if not self.is_executing or self.trajectory is None:
                return

            t_rel = time.time() - self.traj_start_wall_time
            points = self.trajectory.points

            if not points:
                self.is_executing = False
                return

            if t_rel >= self.traj_duration:
                self.set_target_direct_locked(points[-1])
                self.is_executing = False
                return

            idx = 0

            for i in range(len(points) - 1):
                t_next = (
                    points[i + 1].time_from_start.sec
                    + points[i + 1].time_from_start.nanosec * 1e-9
                )

                if t_rel <= t_next:
                    idx = i
                    break

            p0 = points[idx]
            p1 = points[min(idx + 1, len(points) - 1)]

            t0 = p0.time_from_start.sec + p0.time_from_start.nanosec * 1e-9
            t1 = p1.time_from_start.sec + p1.time_from_start.nanosec * 1e-9

            if abs(t1 - t0) < 1e-9:
                alpha = 0.0
            else:
                alpha = (t_rel - t0) / (t1 - t0)
                alpha = max(0.0, min(1.0, alpha))

            for i, name in enumerate(self.active_joint_names):
                if name not in self.joint_qpos_addr:
                    continue

                if i >= len(p0.positions) or i >= len(p1.positions):
                    continue

                qadr = self.joint_qpos_addr[name]

                self.target_qpos[qadr] = (
                    p0.positions[i]
                    + alpha * (p1.positions[i] - p0.positions[i])
                )

    def set_target_direct_locked(self, point):
        for i, name in enumerate(self.active_joint_names):
            if name not in self.joint_qpos_addr:
                continue

            if i >= len(point.positions):
                continue

            qadr = self.joint_qpos_addr[name]
            self.target_qpos[qadr] = point.positions[i]

    # =========================================================
    # MuJoCo 控制
    # =========================================================

    def step_pid(self):
        with self.mj_lock:
            self.data.ctrl[:] = 0.0

            for i in range(self.model.nu):
                actuator_name = mujoco.mj_id2name(
                    self.model,
                    mujoco.mjtObj.mjOBJ_ACTUATOR,
                    i,
                )

                if actuator_name is None:
                    continue

                jid = self.model.actuator_trnid[i, 0]
                qadr = self.model.jnt_qposadr[jid]
                vadr = self.model.jnt_dofadr[jid]

                # 手臂 torque PD
                if actuator_name.startswith("actuator"):
                    error_p = self.target_qpos[qadr] - self.data.qpos[qadr]
                    error_v = -self.data.qvel[vadr]

                    torque = KP * error_p + KD * error_v
                    torque = np.clip(torque, -MAX_TORQUE, MAX_TORQUE)

                    self.data.ctrl[i] = torque

                # 夹爪 position actuator
                elif actuator_name == "finger_actuator1":
                    self.data.ctrl[i] = self.gripper_target

                elif actuator_name == "finger_actuator2":
                    self.data.ctrl[i] = self.gripper_target

            mujoco.mj_step(self.model, self.data)

    def should_weld_bottom_to_top(self, upper_body_name: str, lower_body_name: str) -> bool:
        """
        只允许 upper 的底面和 lower 的顶面连接。
        条件：
        1. upper 的中心 z 高于 lower
        2. upper bottom_z 接近 lower top_z
        3. XY 投影重叠面积达到阈值
        """

        upper_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            upper_body_name,
        )

        lower_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            lower_body_name,
        )

        if upper_id == -1 or lower_id == -1:
            return False

        upper_jnt = self.model.body_jntadr[upper_id]
        lower_jnt = self.model.body_jntadr[lower_id]

        if upper_jnt < 0 or lower_jnt < 0:
            return False

        upper_qadr = self.model.jnt_qposadr[upper_jnt]
        lower_qadr = self.model.jnt_qposadr[lower_jnt]

        upper_pos = self.data.qpos[upper_qadr:upper_qadr + 3].copy()
        lower_pos = self.data.qpos[lower_qadr:lower_qadr + 3].copy()

        upper_quat = self.data.qpos[upper_qadr + 3:upper_qadr + 7].copy()
        lower_quat = self.data.qpos[lower_qadr + 3:lower_qadr + 7].copy()

        upper_half = get_world_half_size(upper_body_name, upper_quat)
        lower_half = get_world_half_size(lower_body_name, lower_quat)

        if upper_half is None or lower_half is None:
            return False

        upper_hx, upper_hy, upper_hz = upper_half
        lower_hx, lower_hy, lower_hz = lower_half

        # 必须是 upper 在上方
        if upper_pos[2] <= lower_pos[2]:
            return False

        upper_bottom_z = upper_pos[2] - upper_hz
        lower_top_z = lower_pos[2] + lower_hz

        vertical_gap = abs(upper_bottom_z - lower_top_z)

        if vertical_gap > VERTICAL_CONTACT_TOL:
            return False

        overlap = xy_overlap_area(
            upper_pos,
            (upper_hx, upper_hy),
            lower_pos,
            (lower_hx, lower_hy),
        )

        upper_area = 4.0 * upper_hx * upper_hy
        lower_area = 4.0 * lower_hx * lower_hy
        min_area = min(upper_area, lower_area)

        if overlap < MIN_OVERLAP_RATIO * min_area:
            return False

        return True
    
    def snap_brick_to_base_plate(self, brick_name: str) -> bool:
        """
        积木落到 assembly_base_plate 上时：
        1. XY 吸附到最近 stud 网格
        2. yaw 对齐到 0/90/180/270
        3. roll/pitch 归零
        4. z 放到底板 stud 顶部
        5. fake weld 固定到底板
        """

        pair = (ASSEMBLY_BASE_NAME, brick_name)

        if pair in self.welded_pairs:
            return False

        brick_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            brick_name,
        )

        if brick_id == -1:
            return False

        brick_jnt = self.model.body_jntadr[brick_id]

        if brick_jnt < 0:
            return False

        qadr = self.model.jnt_qposadr[brick_jnt]
        dofadr = self.model.jnt_dofadr[brick_jnt]

        pos = self.data.qpos[qadr:qadr + 3].copy()
        quat = self.data.qpos[qadr + 3:qadr + 7].copy()

        # 必须在 12x12 底板范围内
        dx = abs(pos[0] - ASSEMBLY_BASE_CENTER_X)
        dy = abs(pos[1] - ASSEMBLY_BASE_CENTER_Y)

        if dx > BASE_PLATE_HALF_X or dy > BASE_PLATE_HALF_Y:
            return False

        # 必须接近底板 stud 顶部
        brick_bottom_z = pos[2] - BRICK_BODY_HALF_HEIGHT
        vertical_gap = abs(brick_bottom_z - BASE_STUD_TOP_Z)

        if vertical_gap > BASE_SNAP_VERTICAL_TOL:
            return False

        # XY 吸附到最近 stud
        snapped_x, snapped_y = snap_xy_to_stud_grid(pos[0], pos[1])

        # yaw 吸附到 90 度倍数，roll/pitch 清零
        yaw = yaw_from_quat_wxyz(quat)
        snapped_yaw = snap_yaw_to_90(yaw)
        snapped_quat = quat_wxyz_from_yaw(snapped_yaw)

        # 写回 MuJoCo qpos
        self.data.qpos[qadr + 0] = snapped_x
        self.data.qpos[qadr + 1] = snapped_y
        self.data.qpos[qadr + 2] = BRICK_ON_BASE_CENTER_Z
        self.data.qpos[qadr + 3:qadr + 7] = snapped_quat

        # 清空速度，避免弹飞
        self.data.qvel[dofadr:dofadr + 6] = 0.0

        mujoco.mj_forward(self.model, self.data)

        # 固定到底板
        self.fake_welds.append(
            {
                "parent": ASSEMBLY_BASE_NAME,
                "child": brick_name,
                "rel_pos": np.array(
                    [
                        snapped_x - ASSEMBLY_BASE_CENTER_X,
                        snapped_y - ASSEMBLY_BASE_CENTER_Y,
                        BRICK_ON_BASE_CENTER_Z - TABLE_TOP_Z,
                    ],
                    dtype=float,
                ),
                "child_quat": snapped_quat.copy(),
            }
        )

        self.welded_pairs.add(pair)

        self.get_logger().info(
            f"[BASE_SNAP] {brick_name} -> "
            f"x={snapped_x:.4f}, y={snapped_y:.4f}, "
            f"z={BRICK_ON_BASE_CENTER_Z:.4f}, yaw={snapped_yaw:.3f}"
        )

        return True
    # =========================================================
    # 一接触就固定：fake weld
    # =========================================================

    def auto_weld_touching_bricks(self):
        """
        只在上面积木底面接触下方面积木顶面，
        并且 XY 重叠面积足够时，才创建 fake weld。
        """

        with self.mj_lock:
            for i in range(self.data.ncon):
                contact = self.data.contact[i]

                body1_id = self.model.geom_bodyid[contact.geom1]
                body2_id = self.model.geom_bodyid[contact.geom2]

                body1_name = mujoco.mj_id2name(
                    self.model,
                    mujoco.mjtObj.mjOBJ_BODY,
                    body1_id,
                )

                body2_name = mujoco.mj_id2name(
                    self.model,
                    mujoco.mjtObj.mjOBJ_BODY,
                    body2_id,
                )

                if body1_name is None or body2_name is None:
                    continue

                # brick 接触装配底板：自动吸附 + 固定
                if body1_name == ASSEMBLY_BASE_NAME and "brick" in body2_name:
                    self.snap_brick_to_base_plate(body2_name)
                    continue

                if body2_name == ASSEMBLY_BASE_NAME and "brick" in body1_name:
                    self.snap_brick_to_base_plate(body1_name)
                    continue

                if "brick" not in body1_name or "brick" not in body2_name:
                    continue

                if body1_name == body2_name:
                    continue

                pair = tuple(sorted([body1_name, body2_name]))

                if pair in self.welded_pairs:
                    continue

                id1 = mujoco.mj_name2id(
                    self.model,
                    mujoco.mjtObj.mjOBJ_BODY,
                    body1_name,
                )

                id2 = mujoco.mj_name2id(
                    self.model,
                    mujoco.mjtObj.mjOBJ_BODY,
                    body2_name,
                )

                if id1 == -1 or id2 == -1:
                    continue

                z1 = self.data.xpos[id1][2]
                z2 = self.data.xpos[id2][2]

                if z1 > z2:
                    upper_name = body1_name
                    lower_name = body2_name
                else:
                    upper_name = body2_name
                    lower_name = body1_name

                if not self.should_weld_bottom_to_top(upper_name, lower_name):
                    continue

                # 下方积木作为 parent，上方积木作为 child
                self.create_fake_weld(lower_name, upper_name)
                self.welded_pairs.add(pair)

                self.get_logger().info(
                    f"[AUTO_WELD] lower={lower_name}, upper={upper_name}, "
                    f"condition=bottom_to_top_overlap"
                )

    def create_fake_weld(self, parent_name: str, child_name: str):
        parent_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            parent_name,
        )

        child_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            child_name,
        )

        if parent_id == -1 or child_id == -1:
            return

        child_jnt = self.model.body_jntadr[child_id]

        if child_jnt < 0:
            return

        qadr = self.model.jnt_qposadr[child_jnt]

        rel_pos = self.data.xpos[child_id].copy() - self.data.xpos[parent_id].copy()
        child_quat = self.data.qpos[qadr + 3:qadr + 7].copy()

        self.fake_welds.append(
            {
                "parent": parent_name,
                "child": child_name,
                "rel_pos": rel_pos,
                "child_quat": child_quat,
            }
        )

    def maintain_fake_welds(self):
        """
        每一帧把 child brick 拉回 parent brick 的相对位置。
        这让“插上去”的积木保持固定。
        """
        with self.mj_lock:
            for weld in self.fake_welds:
                parent_id = mujoco.mj_name2id(
                    self.model,
                    mujoco.mjtObj.mjOBJ_BODY,
                    weld["parent"],
                )

                child_id = mujoco.mj_name2id(
                    self.model,
                    mujoco.mjtObj.mjOBJ_BODY,
                    weld["child"],
                )

                if parent_id == -1 or child_id == -1:
                    continue

                child_jnt = self.model.body_jntadr[child_id]

                if child_jnt < 0:
                    continue

                qadr = self.model.jnt_qposadr[child_jnt]
                dofadr = self.model.jnt_dofadr[child_jnt]

                if weld["parent"] == ASSEMBLY_BASE_NAME:
                    target_pos = np.array(
                        [
                            ASSEMBLY_BASE_CENTER_X + weld["rel_pos"][0],
                            ASSEMBLY_BASE_CENTER_Y + weld["rel_pos"][1],
                            BRICK_ON_BASE_CENTER_Z,
                        ],
                        dtype=float,
                    )
                else:
                    target_pos = self.data.xpos[parent_id] + weld["rel_pos"]

                self.data.qpos[qadr + 0] = target_pos[0]
                self.data.qpos[qadr + 1] = target_pos[1]
                self.data.qpos[qadr + 2] = target_pos[2]
                self.data.qpos[qadr + 3:qadr + 7] = weld["child_quat"]

                self.data.qvel[dofadr:dofadr + 6] = 0.0

            if self.fake_welds:
                mujoco.mj_forward(self.model, self.data)

    # =========================================================
    # JointState 发布
    # =========================================================

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        with self.mj_lock:
            for name in self.all_joint_names:
                if name not in self.joint_qpos_addr:
                    continue

                qadr = self.joint_qpos_addr[name]
                vadr = self.joint_qvel_addr[name]

                msg.name.append(name)
                msg.position.append(float(self.data.qpos[qadr]))
                msg.velocity.append(float(self.data.qvel[vadr]))
                msg.effort.append(0.0)

        self.joint_state_pub.publish(msg)


def main():
    rclpy.init()

    node = None
    executor = None

    try:
        node = MuJoCoActionServer()

        executor = MultiThreadedExecutor()
        executor.add_node(node)

        ros_thread = threading.Thread(target=executor.spin, daemon=True)
        ros_thread.start()

        with mujoco.viewer.launch_passive(node.model, node.data) as viewer:
            while viewer.is_running() and rclpy.ok():
                loop_start = time.time()

                # 更新手臂目标
                node.update_action_state()

                # 更新夹爪目标：这一步让夹爪匀速开合
                node.update_gripper_target()

                for _ in range(SIM_SUBSTEPS):
                    node.step_pid()

                    # 积木接触后固定
                    node.auto_weld_touching_bricks()
                    node.maintain_fake_welds()

                viewer.sync()

                elapsed = time.time() - loop_start
                sleep_time = LOOP_DT - elapsed

                if sleep_time > 0:
                    time.sleep(sleep_time)

    except KeyboardInterrupt:
        pass

    finally:
        if executor is not None:
            executor.shutdown()

        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()