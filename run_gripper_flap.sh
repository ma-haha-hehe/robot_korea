#!/bin/bash
# 夹爪原地快速开合(OnRobot RG2), 机械臂完全不动。
# 用法: bash run_gripper_flap.sh [次数] [--wait 秒]   例: bash run_gripper_flap.sh 30 --wait 0.25
cd /home/i6user/Desktop/robot_lego
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/i6user/Desktop/robot_lego/fastdds_udp_only.xml
source /opt/ros/humble/setup.bash
source install/setup.bash
# 用系统 python3.10(rclpy 编译目标), 避开 conda base 的 python3.13
/usr/bin/python3 gripper_flap.py "$@"
