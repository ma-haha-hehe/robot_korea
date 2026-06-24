#!/bin/bash
# 机械臂【拆卸】拔取动作 —— 完全独立于装配流程, 不要调用 run_auto_pick_pipeline.py。
# 用法: disassembly_arm_pull.sh <pose_name>
#
# 思路: 用你现有的 joint_ptp 点到点机制(和 move_to_observe.sh 一样)。
#       一次拔取 = 移到抓取位 -> 闭合夹爪 -> 竖直提起 -> 移到放置位 -> 松开。
#       先把每个姿态用示教器(freedrive) + print_pose 录好关节角, 填进
#       src/panda_pick/config/ur5_taught_joints.env, 例如:
#           DIS_PULL_TOP_JOINTS="[j0,j1,j2,j3,j4,j5]"
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

if [ -z "$GRASP_JOINTS" ]; then
  echo "❌ 姿态 $POSE 的关节角还没示教(在 ur5_taught_joints.env 里填 DIS_*_JOINTS)。"
  exit 1
fi

# ── 最小骨架: 先只做"移动到抓取关节角"。其余(闭合/提起/放置/松开)按你拆卸需要补全。
#    可参考装配里 joint_pick_place 的参数, 但请新建独立的姿态变量, 别复用装配的。
ros2 launch panda_pick run_ur5.launch.py \
  motion_control_mode:=joint_ptp \
  gripper_control_mode:=none \
  urscript_pregrasp_joints:="$GRASP_JOINTS" \
  urscript_home_joints:="$GRASP_JOINTS" \
  joint_ptp_return_home:=false \
  joint_ptp_dwell_seconds:=0.2 \
  urscript_movej_velocity:=0.60 \
  urscript_movej_acceleration:=1.5
