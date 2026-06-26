#!/bin/bash
# 机械臂【拆卸】拔取一块积木: 抓 -> 提 -> 放到一边 -> 松。
# 用拆卸专用的 disassemble_pick_place + robotiq_socket(共享的机器人接口), 但用【拆卸自己的】
# 示教关节角 + 独立任务文件(disassembly_task.yaml), 不碰装配的 assembly 动作流。
#
# 用法: disassembly_arm_pull.sh <pose_name>
#   pose_name 决定这一块的【抓取关节角】(每块高度不同); 放置位是统一的 DIS_DROP_JOINTS。
#
# 先用示教器(freedrive)+print_pose 录好关节角, 填进 ur5_taught_joints.env:
#   export DIS_PULL_TOP_JOINTS="j0,j1,j2,j3,j4,j5"
#   export DIS_PULL_3_JOINTS="..."
#   export DIS_PULL_2_JOINTS="..."
#   export DIS_DROP_JOINTS="..."        # 统一的放置(扔到一边)位
#   export DIS_HOME_JOINTS="..."        # 放完后回到的拆卸安全 home, 再去下一块 pregrasp
set -e
POSE="$1"
REPEAT="${2:-4}"
DROP_STEP_X="${DROP_STEP_X:--0.6}"
DROP_STEP_Y="${DROP_STEP_Y:-0.0}"
DROP_STEP_Z="${DROP_STEP_Z:-0.0}"
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

if [ -z "$GRASP_JOINTS" ] || [ -z "$DIS_DROP_JOINTS" ] || [ -z "$DIS_HOME_JOINTS" ]; then
  echo "❌ 姿态 $POSE、DIS_DROP_JOINTS 或 DIS_HOME_JOINTS 还没示教(在 ur5_taught_joints.env 里填 DIS_* 变量)。"
  exit 1
fi
if ! [[ "$REPEAT" =~ ^[0-9]+$ ]] || [ "$REPEAT" -lt 1 ]; then
  echo "❌ repeat 必须是正整数, 当前: $REPEAT"
  exit 1
fi

# 抓提放松一条龙: 到拔取姿态 -> 下降/闭合 -> 抬起 -> 到丢放姿态 -> 慢速下降 -> 松开 -> 回拆卸 home。
for i in $(seq 1 "$REPEAT"); do
  PLACE_OFFSET_X=$(awk -v n="$i" -v step="$DROP_STEP_X" 'BEGIN { printf "%.6f", (n - 1) * step }')
  PLACE_OFFSET_Y=$(awk -v n="$i" -v step="$DROP_STEP_Y" 'BEGIN { printf "%.6f", (n - 1) * step }')
  PLACE_OFFSET_Z=$(awk -v n="$i" -v step="$DROP_STEP_Z" 'BEGIN { printf "%.6f", (n - 1) * step }')
  echo "========== disassembly arm pull $i/$REPEAT: $POSE, drop_offset=($PLACE_OFFSET_X,$PLACE_OFFSET_Y,$PLACE_OFFSET_Z) =========="
  ros2 launch panda_pick run_ur5.launch.py \
    motion_control_mode:=disassemble_pick_place \
    gripper_control_mode:=robotiq_socket \
    task_file:="$PWD/disassembly_task.yaml" \
    urscript_pregrasp_joints:="$GRASP_JOINTS" \
    urscript_preplace_joints:="$DIS_DROP_JOINTS" \
    urscript_home_joints:="$DIS_HOME_JOINTS" \
    joint_ptp_return_home:=true \
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
    urscript_descend_velocity:=0.08 \
    urscript_disassemble_extra_lift:=0.06 \
    urscript_disassemble_place_offset_x:="$PLACE_OFFSET_X" \
    urscript_disassemble_place_offset_y:="$PLACE_OFFSET_Y" \
    urscript_disassemble_place_offset_z:="$PLACE_OFFSET_Z" \
    urscript_place_descend_velocity:=0.02 \
    urscript_place_settle_wait_seconds:=0.3 \
    urscript_pipeline_wait_seconds:=50.0
done
