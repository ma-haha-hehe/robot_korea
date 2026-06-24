import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # =========================================================
    # 1. 声明参数
    # =========================================================
    # 这里保留这个参数，只是为了兼容 panda.urdf.xacro。
    # 但本 launch 里不再启动 ros2_control 的 fake arm controller。
    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="mock_components",
    )

    # =========================================================
    # 2. 构建 MoveIt 配置
    # =========================================================
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(
            file_path="config/panda.urdf.xacro",
            mappings={"ros2_control_hardware_type": "mock_components"},
        )
        .robot_description_semantic(
            file_path="config/panda.srdf"
        )
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml"
        )
        .planning_pipelines(
            pipelines=[
                "ompl",
                "chomp",
                "pilz_industrial_motion_planner",
            ]
        )
        .to_moveit_configs()
    )

    # =========================================================
    # 3. MoveIt Controller 配置
    # =========================================================
    # 这里非常重要：
    #
    # MoveIt 不再发给：
    #   /panda_arm_controller/follow_joint_trajectory
    #   /panda_hand_controller/follow_joint_trajectory
    #
    # 而是发给你自己在 mj_bridge3.py 里开的 Action Server：
    #   /mj_panda_arm_controller/follow_joint_trajectory
    #   /mj_panda_hand_controller/follow_joint_trajectory
    #
    moveit_controllers_overrides = {
        "moveit_controller_manager": (
            "moveit_simple_controller_manager/MoveItSimpleControllerManager"
        ),
        "moveit_simple_controller_manager": {
            "controller_names": [
                "mj_panda_arm_controller",
                "mj_panda_hand_controller",
            ],

            "mj_panda_arm_controller": {
                "type": "FollowJointTrajectory",
                "action_ns": "follow_joint_trajectory",
                "default": True,
                "joints": [
                    "panda_joint1",
                    "panda_joint2",
                    "panda_joint3",
                    "panda_joint4",
                    "panda_joint5",
                    "panda_joint6",
                    "panda_joint7",
                ],
            },

            "mj_panda_hand_controller": {
                "type": "FollowJointTrajectory",
                "action_ns": "follow_joint_trajectory",
                "default": True,
                "joints": [
                    "panda_finger_joint1",
                    "panda_finger_joint2",
                ],
            },
        },
    }

    # =========================================================
    # 4. move_group
    # =========================================================
    # move_group 负责：
    #   - 接收 cpp_pick_node 的规划请求
    #   - 做 OMPL / Cartesian path 规划
    #   - 把轨迹发送给 mj_bridge 的 action server
    #
    # moveit_manage_controllers=False：
    #   因为 mj_bridge 不是 ros2_control controller，
    #   MoveIt 不应该尝试启动/停止 controller。
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            moveit_controllers_overrides,
            {
                "moveit_manage_controllers": False,
                "trajectory_execution.allowed_execution_duration_scaling": 3.0,
                "trajectory_execution.allowed_goal_duration_margin": 5.0,
                "trajectory_execution.allowed_start_tolerance": 0.05,
                "robot_description_kinematics.panda_arm.kinematics_solver_timeout": 0.05,
            },
        ],
    )

    # =========================================================
    # 5. robot_state_publisher
    # =========================================================
    # robot_state_publisher 读取 /joint_states，
    # 然后发布 panda_link0 -> panda_link1 -> ... 的 TF。
    #
    # 注意：
    # /joint_states 应该由 mj_bridge 发布。
    # 如果 mj_bridge 不发布 joint_states，RViz 不会跟着 MuJoCo 动。
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
        ],
    )

    # =========================================================
    # 6. world 到 panda_link0 的静态 TF
    # =========================================================
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_panda_link0_tf",
        output="screen",
        arguments=[
            "0", "0", "0",
            "0", "0", "0",
            "world",
            "panda_link0",
        ],
    )

    # =========================================================
    # 7. MuJoCo Bridge
    # =========================================================
    # 这个节点必须提供：
    #
    #   /mj_panda_arm_controller/follow_joint_trajectory
    #   /mj_panda_hand_controller/follow_joint_trajectory
    #
    # 并且最好发布：
    #
    #   /joint_states
    #
    bridge = Node(
        package="mj_bridge",
        executable="mj_bridge",
        output="screen",
    )

    # =========================================================
    # 8. RViz
    # =========================================================
    rviz_base = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "launch",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(rviz_base, "moveit.rviz"),
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {
                "robot_description_kinematics.panda_arm.kinematics_solver_timeout": 0.05,
            },
        ],
    )

    # =========================================================
    # 9. 你的 C++ 执行节点
    # =========================================================
    # 延迟启动，保证：
    #   - move_group 已经起来
    #   - mj_bridge 的 action server 已经起来
    #   - RViz / TF 已经基本就绪
    pick_node = Node(
        package="panda_pick",
        executable="cpp_pick_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            moveit_controllers_overrides,
        ],
    )

    delayed_pick_node = TimerAction(
        period=8.0,
        actions=[pick_node],
    )

    # =========================================================
    # 10. 返回 LaunchDescription
    # =========================================================
    # 注意：
    # 这里没有 ros2_control_node。
    # 这里也没有 spawner。
    #
    # 原因：
    # 你的真实执行端是 mj_bridge，
    # 不是 ros2_control fake controller。
    return LaunchDescription([
        ros2_control_hardware_type,
        static_tf,
        rsp,
        move_group_node,
        rviz_node,
        bridge,
        delayed_pick_node,
    ])