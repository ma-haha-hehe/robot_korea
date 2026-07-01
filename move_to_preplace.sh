#!/bin/bash
# 把机械臂移动到放置悬停姿态(UR5_PREPLACE_JOINTS)，用来检查 preplace 位姿。
cd /home/i6user/Desktop/robot_lego
source /opt/ros/humble/setup.bash
source install/setup.bash
source src/panda_pick/config/ur5_taught_joints.env
ros2 launch panda_pick run_ur5.launch.py \
  motion_control_mode:=joint_ptp \
  gripper_control_mode:=none \
  urscript_pregrasp_joints:="$UR5_PREPLACE_JOINTS" \
  urscript_home_joints:="$UR5_PREPLACE_JOINTS" \
  joint_ptp_return_home:=false \
  joint_ptp_dwell_seconds:=0.1 \
  urscript_pipeline_wait_seconds:=8.0 \
  urscript_movej_velocity:=0.90 \
  urscript_movej_acceleration:=2.0
