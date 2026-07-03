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
# 2026-07-02: 固定抓取点抓 3 次, 放到 3 个点: 放置示教点(observe)往下10cm(见 disassembly_task.yaml
#   place.descend=0.10), 水平沿 base +X 逐块 +8cm (第1块0 / 第2块+8 / 第3块+16)。
POSE="$1"
REPEAT="${2:-3}"
DROP_STEP_X="${DROP_STEP_X:-0.08}"
DROP_STEP_Y="${DROP_STEP_Y:-0.0}"
DROP_STEP_Z="${DROP_STEP_Z:-0.0}"
PICK_OFFSET_X="${PICK_OFFSET_X:-0.0}"   # base系抓取x偏移(estop=0.01 抓取x+1cm); carrot 默认0 不受影响
PRESTEP_DESCEND="${PRESTEP_DESCEND:-0.143}"   # 前置动作【下降抓】距离; carrot 默认0.143; estop 传更大值下探更深
PULL_OFFSET_X="${PULL_OFFSET_X:-0.005}"       # 拔取点(前置横移落地点+每块抓取点)在 base x 整体平移, 默认+5mm; 两产品共用
# 抓/放 descend 等来自 task 文件。默认 disassembly_task.yaml(=carrot 3块);
#   estop 脚本传 TASK_FILE=disassembly_task_estop.yaml 用自己的一份, 与 carrot 完全分开。
TASK_FILE="${TASK_FILE:-$PWD/disassembly_task.yaml}"
# 夹爪模式: disassemble_pick_place 支持 none / robotiq_socket / onrobot_rg / onrobot_io。
# 默认 onrobot_io: 与装配一致(工具口 pin16, False开/True关)。首验轨迹可传 GRIPPER_MODE=none。
GRIPPER_MODE="${GRIPPER_MODE:-onrobot_io}"
cd /home/i6user/Desktop/robot_lego
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/i6user/Desktop/robot_lego/fastdds_udp_only.xml  # 禁SHM走UDP修sequence-size崩溃
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
  PRESTEP=$([ "$i" -eq 1 ] && echo true || echo false)   # 前置动作(goto固定起点+开爪下降闭合, 及第一块的释放)只在第一块执行一次
  ros2 launch panda_pick run_ur5.launch.py \
    motion_control_mode:=disassemble_pick_place \
    urscript_disassemble_prestep:="$PRESTEP" \
    gripper_control_mode:="$GRIPPER_MODE" \
    task_file:="$TASK_FILE" \
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
    urscript_disassemble_pick_offset_x:="$PICK_OFFSET_X" \
    urscript_disassemble_prestep_descend:="$PRESTEP_DESCEND" \
    urscript_disassemble_pull_offset_x:="$PULL_OFFSET_X" \
    urscript_disassemble_place_offset_x:="$PLACE_OFFSET_X" \
    urscript_disassemble_place_offset_y:="$PLACE_OFFSET_Y" \
    urscript_disassemble_place_offset_z:="$PLACE_OFFSET_Z" \
    urscript_place_descend_velocity:=0.02 \
    urscript_place_settle_wait_seconds:=0.3 \
    urscript_pipeline_wait_seconds:=50.0
done
