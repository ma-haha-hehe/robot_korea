from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ur_type = "ur5"
    robot_name = "ur"
    gripper_control_mode = LaunchConfiguration("gripper_control_mode")
    gripper_action_name = LaunchConfiguration("gripper_action_name")
    gripper_open_position = LaunchConfiguration("gripper_open_position")
    gripper_close_position = LaunchConfiguration("gripper_close_position")
    gripper_max_effort = LaunchConfiguration("gripper_max_effort")
    gripper_io_service_name = LaunchConfiguration("gripper_io_service_name")
    gripper_io_open_pin = LaunchConfiguration("gripper_io_open_pin")
    gripper_io_close_pin = LaunchConfiguration("gripper_io_close_pin")
    gripper_io_pulse_seconds = LaunchConfiguration("gripper_io_pulse_seconds")
    gripper_io_latching = LaunchConfiguration("gripper_io_latching")
    gripper_urscript_topic = LaunchConfiguration("gripper_urscript_topic")
    gripper_urscript_wait_seconds = LaunchConfiguration("gripper_urscript_wait_seconds")
    gripper_socket_open_wait_seconds = LaunchConfiguration("gripper_socket_open_wait_seconds")
    gripper_socket_speed = LaunchConfiguration("gripper_socket_speed")
    gripper_socket_force = LaunchConfiguration("gripper_socket_force")
    gripper_socket_release_position = LaunchConfiguration("gripper_socket_release_position")
    gripper_socket_release_wait_seconds = LaunchConfiguration("gripper_socket_release_wait_seconds")
    motion_control_mode = LaunchConfiguration("motion_control_mode")
    urscript_movej_acceleration = LaunchConfiguration("urscript_movej_acceleration")
    urscript_movej_velocity = LaunchConfiguration("urscript_movej_velocity")
    urscript_pick_approach_movej_velocity = LaunchConfiguration("urscript_pick_approach_movej_velocity")
    urscript_movel_acceleration = LaunchConfiguration("urscript_movel_acceleration")
    urscript_movel_velocity = LaunchConfiguration("urscript_movel_velocity")
    urscript_pick_approach_movel_velocity = LaunchConfiguration("urscript_pick_approach_movel_velocity")
    urscript_pick_lift_velocity = LaunchConfiguration("urscript_pick_lift_velocity")
    urscript_descend_acceleration = LaunchConfiguration("urscript_descend_acceleration")
    urscript_descend_velocity = LaunchConfiguration("urscript_descend_velocity")
    urscript_place_descend_velocity = LaunchConfiguration("urscript_place_descend_velocity")
    urscript_place_wiggle_enabled = LaunchConfiguration("urscript_place_wiggle_enabled")
    urscript_place_wiggle_xy_amplitude = LaunchConfiguration("urscript_place_wiggle_xy_amplitude")
    urscript_place_wiggle_z_amplitude = LaunchConfiguration("urscript_place_wiggle_z_amplitude")
    urscript_place_wiggle_velocity = LaunchConfiguration("urscript_place_wiggle_velocity")
    urscript_place_wiggle_steps = LaunchConfiguration("urscript_place_wiggle_steps")
    urscript_disassemble_extra_lift = LaunchConfiguration("urscript_disassemble_extra_lift")
    urscript_disassemble_place_offset_x = LaunchConfiguration("urscript_disassemble_place_offset_x")
    urscript_disassemble_place_offset_y = LaunchConfiguration("urscript_disassemble_place_offset_y")
    urscript_disassemble_place_offset_z = LaunchConfiguration("urscript_disassemble_place_offset_z")
    urscript_place_lift_after_release = LaunchConfiguration("urscript_place_lift_after_release")
    urscript_place_slow_final_descend = LaunchConfiguration("urscript_place_slow_final_descend")
    urscript_place_settle_wait_seconds = LaunchConfiguration("urscript_place_settle_wait_seconds")
    urscript_pipeline_wait_seconds = LaunchConfiguration("urscript_pipeline_wait_seconds")
    task_file = LaunchConfiguration("task_file")
    task_max_xy_offset = LaunchConfiguration("task_max_xy_offset")
    task_max_z_offset = LaunchConfiguration("task_max_z_offset")
    task_max_descend = LaunchConfiguration("task_max_descend")
    urscript_pregrasp_joints = LaunchConfiguration("urscript_pregrasp_joints")
    urscript_preplace_joints = LaunchConfiguration("urscript_preplace_joints")
    urscript_home_joints = LaunchConfiguration("urscript_home_joints")
    joint_ptp_return_home = LaunchConfiguration("joint_ptp_return_home")
    joint_ptp_dwell_seconds = LaunchConfiguration("joint_ptp_dwell_seconds")
    fixed_grasp_x = LaunchConfiguration("fixed_grasp_x")
    fixed_grasp_y = LaunchConfiguration("fixed_grasp_y")
    fixed_grasp_z = LaunchConfiguration("fixed_grasp_z")
    fixed_grasp_hover = LaunchConfiguration("fixed_grasp_hover")
    fixed_place_x = LaunchConfiguration("fixed_place_x")
    fixed_place_y = LaunchConfiguration("fixed_place_y")
    fixed_place_z = LaunchConfiguration("fixed_place_z")
    fixed_place_hover = LaunchConfiguration("fixed_place_hover")
    fixed_grasp_qx = LaunchConfiguration("fixed_grasp_qx")
    fixed_grasp_qy = LaunchConfiguration("fixed_grasp_qy")
    fixed_grasp_qz = LaunchConfiguration("fixed_grasp_qz")
    fixed_grasp_qw = LaunchConfiguration("fixed_grasp_qw")
    use_current_pose_as_pregrasp = LaunchConfiguration("use_current_pose_as_pregrasp")
    use_cartesian_descent = LaunchConfiguration("use_cartesian_descent")

    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "visual_parameters.yaml"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]
            ),
            " ",
            "robot_ip:=xxx.yyy.zzz.www",
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "safety_limits:=true",
            " ",
            "safety_pos_margin:=0.15",
            " ",
            "safety_k_position:=20",
            " ",
            "name:=",
            robot_name,
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "script_filename:=ros_control.urscript",
            " ",
            "input_recipe_filename:=rtde_input_recipe.txt",
            " ",
            "output_recipe_filename:=rtde_output_recipe.txt",
            " ",
            "prefix:=",
            "",
            " ",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]
            ),
            " ",
            "name:=",
            robot_name,
            " ",
            "prefix:=",
            "",
            " ",
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            robot_description_semantic_content, value_type=str
        )
    }

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare("ur_moveit_config"), "config", "kinematics.yaml"]
    )

    run_node = Node(
        package="panda_pick",
        executable="cpp_pick_node_ur5",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {
                "use_sim_time": False,
                "gripper_control_mode": gripper_control_mode,
                "gripper_action_name": gripper_action_name,
                "gripper_open_position": ParameterValue(gripper_open_position, value_type=float),
                "gripper_close_position": ParameterValue(gripper_close_position, value_type=float),
                "gripper_max_effort": ParameterValue(gripper_max_effort, value_type=float),
                "gripper_io_service_name": gripper_io_service_name,
                "gripper_io_open_pin": ParameterValue(gripper_io_open_pin, value_type=int),
                "gripper_io_close_pin": ParameterValue(gripper_io_close_pin, value_type=int),
                "gripper_io_pulse_seconds": ParameterValue(gripper_io_pulse_seconds, value_type=float),
                "gripper_io_latching": ParameterValue(gripper_io_latching, value_type=bool),
                "gripper_urscript_topic": gripper_urscript_topic,
                "gripper_urscript_wait_seconds": ParameterValue(
                    gripper_urscript_wait_seconds, value_type=float
                ),
                "gripper_socket_open_wait_seconds": ParameterValue(
                    gripper_socket_open_wait_seconds, value_type=float
                ),
                "gripper_socket_speed": ParameterValue(gripper_socket_speed, value_type=int),
                "gripper_socket_force": ParameterValue(gripper_socket_force, value_type=int),
                "gripper_socket_pregrasp_position": ParameterValue(
                    LaunchConfiguration("gripper_socket_pregrasp_position"), value_type=int
                ),
                "gripper_socket_pregrasp_wait_seconds": ParameterValue(
                    LaunchConfiguration("gripper_socket_pregrasp_wait_seconds"), value_type=float
                ),
                "urscript_done_still_seconds": ParameterValue(
                    LaunchConfiguration("urscript_done_still_seconds"), value_type=float
                ),
                "gripper_socket_release_position": ParameterValue(
                    gripper_socket_release_position, value_type=int
                ),
                "gripper_socket_release_wait_seconds": ParameterValue(
                    gripper_socket_release_wait_seconds, value_type=float
                ),
                "motion_control_mode": motion_control_mode,
                "urscript_movej_acceleration": ParameterValue(
                    urscript_movej_acceleration, value_type=float
                ),
                "urscript_movej_velocity": ParameterValue(
                    urscript_movej_velocity, value_type=float
                ),
                "urscript_pick_approach_movej_velocity": ParameterValue(
                    urscript_pick_approach_movej_velocity, value_type=float
                ),
                "urscript_movel_acceleration": ParameterValue(
                    urscript_movel_acceleration, value_type=float
                ),
                "urscript_movel_velocity": ParameterValue(
                    urscript_movel_velocity, value_type=float
                ),
                "urscript_pick_approach_movel_velocity": ParameterValue(
                    urscript_pick_approach_movel_velocity, value_type=float
                ),
                "urscript_pick_lift_velocity": ParameterValue(
                    urscript_pick_lift_velocity, value_type=float
                ),
                "urscript_descend_acceleration": ParameterValue(
                    urscript_descend_acceleration, value_type=float
                ),
                "urscript_descend_velocity": ParameterValue(
                    urscript_descend_velocity, value_type=float
                ),
                "urscript_place_descend_velocity": ParameterValue(
                    urscript_place_descend_velocity, value_type=float
                ),
                "urscript_place_wiggle_enabled": ParameterValue(
                    urscript_place_wiggle_enabled, value_type=bool
                ),
                "urscript_place_wiggle_xy_amplitude": ParameterValue(
                    urscript_place_wiggle_xy_amplitude, value_type=float
                ),
                "urscript_place_wiggle_z_amplitude": ParameterValue(
                    urscript_place_wiggle_z_amplitude, value_type=float
                ),
                "urscript_place_wiggle_velocity": ParameterValue(
                    urscript_place_wiggle_velocity, value_type=float
                ),
                "urscript_place_wiggle_steps": ParameterValue(
                    urscript_place_wiggle_steps, value_type=int
                ),
                "urscript_disassemble_extra_lift": ParameterValue(
                    urscript_disassemble_extra_lift, value_type=float
                ),
                "urscript_disassemble_place_offset_x": ParameterValue(
                    urscript_disassemble_place_offset_x, value_type=float
                ),
                "urscript_disassemble_place_offset_y": ParameterValue(
                    urscript_disassemble_place_offset_y, value_type=float
                ),
                "urscript_disassemble_place_offset_z": ParameterValue(
                    urscript_disassemble_place_offset_z, value_type=float
                ),
                "urscript_place_lift_after_release": ParameterValue(
                    urscript_place_lift_after_release, value_type=float
                ),
                "urscript_place_slow_final_descend": ParameterValue(
                    urscript_place_slow_final_descend, value_type=float
                ),
                "urscript_place_settle_wait_seconds": ParameterValue(
                    urscript_place_settle_wait_seconds, value_type=float
                ),
                "urscript_pipeline_wait_seconds": ParameterValue(
                    urscript_pipeline_wait_seconds, value_type=float
                ),
                "task_file": task_file,
                "task_max_xy_offset": ParameterValue(task_max_xy_offset, value_type=float),
                "task_max_z_offset": ParameterValue(task_max_z_offset, value_type=float),
                "task_max_descend": ParameterValue(task_max_descend, value_type=float),
                "urscript_pregrasp_joints": urscript_pregrasp_joints,
                "urscript_preplace_joints": urscript_preplace_joints,
                "urscript_home_joints": urscript_home_joints,
                "joint_ptp_return_home": ParameterValue(joint_ptp_return_home, value_type=bool),
                "joint_ptp_dwell_seconds": ParameterValue(joint_ptp_dwell_seconds, value_type=float),
                "fixed_grasp_x": ParameterValue(fixed_grasp_x, value_type=float),
                "fixed_grasp_y": ParameterValue(fixed_grasp_y, value_type=float),
                "fixed_grasp_z": ParameterValue(fixed_grasp_z, value_type=float),
                "fixed_grasp_hover": ParameterValue(fixed_grasp_hover, value_type=float),
                "fixed_place_x": ParameterValue(fixed_place_x, value_type=float),
                "fixed_place_y": ParameterValue(fixed_place_y, value_type=float),
                "fixed_place_z": ParameterValue(fixed_place_z, value_type=float),
                "fixed_place_hover": ParameterValue(fixed_place_hover, value_type=float),
                "fixed_grasp_qx": ParameterValue(fixed_grasp_qx, value_type=float),
                "fixed_grasp_qy": ParameterValue(fixed_grasp_qy, value_type=float),
                "fixed_grasp_qz": ParameterValue(fixed_grasp_qz, value_type=float),
                "fixed_grasp_qw": ParameterValue(fixed_grasp_qw, value_type=float),
                "use_current_pose_as_pregrasp": ParameterValue(
                    use_current_pose_as_pregrasp, value_type=bool
                ),
                "use_cartesian_descent": ParameterValue(use_cartesian_descent, value_type=bool),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("gripper_control_mode", default_value="none"),
        DeclareLaunchArgument("gripper_action_name", default_value="/robotiq_gripper_controller/gripper_cmd"),
        DeclareLaunchArgument("gripper_open_position", default_value="0.0"),
        DeclareLaunchArgument("gripper_close_position", default_value="0.7"),
        DeclareLaunchArgument("gripper_max_effort", default_value="80.0"),
        DeclareLaunchArgument("gripper_io_service_name", default_value="/io_and_status_controller/set_io"),
        DeclareLaunchArgument("gripper_io_open_pin", default_value="16"),
        DeclareLaunchArgument("gripper_io_close_pin", default_value="17"),
        DeclareLaunchArgument("gripper_io_pulse_seconds", default_value="0.5"),
        DeclareLaunchArgument("gripper_io_latching", default_value="false"),
        DeclareLaunchArgument("gripper_urscript_topic", default_value="/urscript_interface/script_command"),
        DeclareLaunchArgument("gripper_urscript_wait_seconds", default_value="4.0"),
        DeclareLaunchArgument("gripper_socket_open_wait_seconds", default_value="4.0"),
        DeclareLaunchArgument("gripper_socket_speed", default_value="180"),
        DeclareLaunchArgument("gripper_socket_force", default_value="80"),
        DeclareLaunchArgument("gripper_socket_pregrasp_position", default_value="0"),
        DeclareLaunchArgument("gripper_socket_pregrasp_wait_seconds", default_value="1.0"),
        DeclareLaunchArgument("urscript_done_still_seconds", default_value="5.0"),
        DeclareLaunchArgument("gripper_socket_release_position", default_value="140"),
        DeclareLaunchArgument("gripper_socket_release_wait_seconds", default_value="0.5"),
        DeclareLaunchArgument("motion_control_mode", default_value="joint_pick_place"),
        DeclareLaunchArgument("urscript_movej_acceleration", default_value="2.0"),
        DeclareLaunchArgument("urscript_movej_velocity", default_value="1.2"),
        DeclareLaunchArgument("urscript_pick_approach_movej_velocity", default_value="0.35"),
        DeclareLaunchArgument("urscript_movel_acceleration", default_value="1.0"),
        DeclareLaunchArgument("urscript_movel_velocity", default_value="0.4"),
        DeclareLaunchArgument("urscript_pick_approach_movel_velocity", default_value="0.18"),
        DeclareLaunchArgument("urscript_pick_lift_velocity", default_value="0.14"),
        DeclareLaunchArgument("urscript_descend_acceleration", default_value="0.15"),
        DeclareLaunchArgument("urscript_descend_velocity", default_value="0.10"),
        DeclareLaunchArgument("urscript_place_descend_velocity", default_value="0.002"),
        DeclareLaunchArgument("urscript_place_wiggle_enabled", default_value="true"),
        DeclareLaunchArgument("urscript_place_wiggle_xy_amplitude", default_value="0.0006"),
        DeclareLaunchArgument("urscript_place_wiggle_z_amplitude", default_value="0.0"),
        DeclareLaunchArgument("urscript_place_wiggle_velocity", default_value="0.012"),
        DeclareLaunchArgument("urscript_place_wiggle_steps", default_value="12"),
        DeclareLaunchArgument("urscript_disassemble_extra_lift", default_value="0.0"),
        DeclareLaunchArgument("urscript_disassemble_place_offset_x", default_value="0.0"),
        DeclareLaunchArgument("urscript_disassemble_place_offset_y", default_value="0.0"),
        DeclareLaunchArgument("urscript_disassemble_place_offset_z", default_value="0.0"),
        DeclareLaunchArgument("urscript_place_lift_after_release", default_value="0.0"),
        DeclareLaunchArgument("urscript_place_slow_final_descend", default_value="0.020"),
        DeclareLaunchArgument("urscript_place_settle_wait_seconds", default_value="0.5"),
        DeclareLaunchArgument("urscript_pipeline_wait_seconds", default_value="50.0"),
        DeclareLaunchArgument("task_file", default_value=""),
        DeclareLaunchArgument("task_max_xy_offset", default_value="0.15"),
        DeclareLaunchArgument("task_max_z_offset", default_value="0.15"),
        DeclareLaunchArgument("task_max_descend", default_value="0.30"),
        DeclareLaunchArgument("urscript_pregrasp_joints", default_value=""),
        DeclareLaunchArgument("urscript_preplace_joints", default_value=""),
        DeclareLaunchArgument("urscript_home_joints", default_value=""),
        DeclareLaunchArgument("joint_ptp_return_home", default_value="true"),
        DeclareLaunchArgument("joint_ptp_dwell_seconds", default_value="1.0"),
        DeclareLaunchArgument("fixed_grasp_x", default_value="0.03"),
        DeclareLaunchArgument("fixed_grasp_y", default_value="0.3529"),
        DeclareLaunchArgument("fixed_grasp_z", default_value="0.90"),
        DeclareLaunchArgument("fixed_grasp_hover", default_value="0.15"),
        DeclareLaunchArgument("fixed_place_x", default_value="0.20"),
        DeclareLaunchArgument("fixed_place_y", default_value="0.3529"),
        DeclareLaunchArgument("fixed_place_z", default_value="0.95"),
        DeclareLaunchArgument("fixed_place_hover", default_value="0.15"),
        DeclareLaunchArgument("fixed_grasp_qx", default_value="-0.1779"),
        DeclareLaunchArgument("fixed_grasp_qy", default_value="0.7564"),
        DeclareLaunchArgument("fixed_grasp_qz", default_value="0.6179"),
        DeclareLaunchArgument("fixed_grasp_qw", default_value="-0.1200"),
        DeclareLaunchArgument("use_current_pose_as_pregrasp", default_value="false"),
        DeclareLaunchArgument("use_cartesian_descent", default_value="false"),
        run_node,
    ])
