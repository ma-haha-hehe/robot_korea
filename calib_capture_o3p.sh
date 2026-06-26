#!/bin/bash
# O3P 高清彩色 手眼标定采样(纯 host, 不用容器/RealSense):
#   从 grabber 写的 color_hi.png 抓帧+检测棋盘 + 读机器人 T_base_tool + 配对写 samples.yaml。
# 每摆一个新姿态(示教器 freedrive)就跑一次。够 15-20 个后跑 calib_run_o3p.sh。
#
# 前提: o3p_grabber.py 在跑(有新帧), 棋盘固定不动, 相机刚性锁在夹爪上。
set -e
cd /home/i6user/Desktop/robot_lego
source /opt/ros/humble/setup.bash
source install/setup.bash
source src/panda_pick/config/ur5_taught_joints.env

HE=src/my_robot_vision/my_robot_vision/handeye
BOARD_COLS=4          # 内角点: 横向交叉点数 (板是 5x7 格子 = 4x6 内角点)
BOARD_ROWS=6          # 内角点: 纵向交叉点数
SQUARE=0.030          # 格子边长(米) = 30mm
mkdir -p "$HE"
N=$(ls "$HE"/img_*.jpg 2>/dev/null | grep -v _viz | wc -l)
ID=$(printf "%03d" $((N + 1)))
echo "==================== 采集样本 #$ID ===================="

echo "[1/3] 抓高清彩色 + 检测棋盘 (${BOARD_COLS}x${BOARD_ROWS} 内角点) ..."
if ! /usr/bin/python3 src/my_robot_vision/my_robot_vision/calib_grab_o3p.py \
     "$HE/img_$ID.jpg" $BOARD_COLS $BOARD_ROWS; then
  echo "❌ 没检测到棋盘! 让【整块板清晰完整可见】再重跑。本次不计入。"
  echo "   检查检测图: $HE/img_${ID}_viz.jpg"
  rm -f "$HE/img_$ID.jpg" "$HE/img_${ID}_viz.jpg"
  exit 1
fi

echo "[2/3] 读取机器人 T_base_tool (保持机械臂别动) ..."
timeout 25 ros2 launch panda_pick run_ur5.launch.py motion_control_mode:=print_pose \
  > /tmp/pp_o3p_$ID.log 2>&1 || true

echo "[3/3] 配对写入 samples.yaml ..."
/usr/bin/python3 src/my_robot_vision/calib_append_o3p.py \
  --id "$ID" --pp /tmp/pp_o3p_$ID.log --handeye "$HE" \
  --image "$(pwd)/$HE/img_$ID.jpg" \
  --board-size $BOARD_COLS $BOARD_ROWS --square-size $SQUARE

echo "✅ 完成 #$ID。换个【明显不同的姿态(多转腕、远近、板在画面不同区域、带倾角)】再跑一次。"
echo "   检测可视化: $HE/img_${ID}_viz.jpg"
