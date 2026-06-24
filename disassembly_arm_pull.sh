#!/bin/bash
# 机械臂【拆卸】拔取一块积木: 抓 -> 提 -> 放到一边 -> 松。
# 用和装配同款的 joint_pick_place + robotiq_socket(共享的机器人接口), 但用【拆卸自己的】
# 示教关节角 + 独立任务文件(disassembly_task.yaml), 完全不碰装配的 pipeline/配置。
#
# 用法: disassembly_arm_pull.sh <pose_name>
#   pose_name 决定这一块的【抓取关节角】(每块高度不同); 放置位是统一的 DIS_DROP_JOINTS。
#
# 先用示教器(freedrive)+print_pose 录好关节角, 填进 ur5_taught_joints.env:
#   export DIS_PULL_TOP_JOINTS="j0,j1,j2,j3,j4,j5"
#   export DIS_PULL_3_JOINTS="..."
#   export DIS_PULL_2_JOINTS="..."
#   export DIS_DROP_JOINTS="..."        # 统一的放置(扔到一边)位
set -e
POSE="$1"
cd /home/i6user/Desktop/robot_lego
source /opt/ros/humble/setup.bash
source install/setup.bash
source src/panda_pick/config/ur5_taught_joints.env

case "$POSE" in
  pull_top) GRASP_JOINTS="$DIS_PULL_TOP_JOINTS" ;;
  pull_3)   GRASP_JOINTS="$DIS_PULL_3_JOINTS" ;;
  pull_2)   GRASP_JOINTS="$DIS_PULL_2_JOINTS" ;;
  *) echo "未知姿态: $POSE"; exit 1 ;;
esac

if [ -z "$GRASP_JOINTS" ] || [ -z "$DIS_DROP_JOINTS" ]; then
  echo "❌ 姿态 $POSE 或 DIS_DROP_JOINTS 还没示教(在 ur5_taught_joints.env 里填 DIS_* 变量)。"
  exit 1
fi

# 抓提放松一条龙(robotiq_socket 半合->下降->全合的时序, 数值沿用装配 run_pick.sh 里验证过的)
ros2 launch panda_pick run_ur5.launch.py \
  motion_control_mode:=joint_pick_place \
  gripper_control_mode:=robotiq_socket \
  task_file:="$PWD/disassembly_task.yaml" \
  urscript_pregrasp_joints:="$GRASP_JOINTS" \
  urscript_preplace_joints:="$DIS_DROP_JOINTS" \
  urscript_home_joints:="$UR5_OBSERVE_JOINTS" \
  gripper_socket_speed:=155 \
  gripper_socket_force:=145 \
  gripper_socket_pregrasp_position:=128 \
  gripper_socket_release_position:=120 \
  gripper_socket_release_wait_seconds:=0.5 \
  urscript_movej_velocity:=1.10 \
  urscript_movej_acceleration:=2.0 \
  urscript_pick_approach_movej_velocity:=0.60 \
  urscript_pick_approach_movel_velocity:=0.18 \
  urscript_pick_lift_velocity:=0.32 \
  urscript_movel_velocity:=0.40 \
  urscript_movel_acceleration:=1.2 \
  urscript_pipeline_wait_seconds:=50.0
