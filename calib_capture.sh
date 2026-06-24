#!/bin/bash
# 采集一个手眼标定样本: 自动 拍照+检测棋盘 + 读机器人位姿 + 配对写入 samples.yaml。
# 每摆一个新姿态(示教器freedrive)就跑一次。够 15-20 个后跑 calib_run.sh。
set -e
cd /home/i6user/Desktop/robot_lego
source /opt/ros/humble/setup.bash; source install/setup.bash
source src/panda_pick/config/ur5_taught_joints.env
HE=src/my_robot_vision/my_robot_vision/handeye
mkdir -p "$HE"
N=$(ls "$HE"/img_*.jpg 2>/dev/null | wc -l); ID=$(printf "%03d" $((N+1)))
echo "==================== 采集样本 #$ID ===================="

echo "[1/3] 拍照 + 检测棋盘 ..."
if ! sudo docker exec vision_node_final /opt/conda/envs/my/bin/python \
     /vision_code/calib_grab.py /vision_code/handeye/img_$ID.jpg 4 6; then
  echo "❌ 没检测到棋盘! 让【整块板清晰可见】再重跑。本次不计入。"
  rm -f "$HE/img_$ID.jpg"
  exit 1
fi

echo "[2/3] 读取机器人 T_base_tool (保持机械臂别动) ..."
timeout 25 ros2 launch panda_pick run_ur5.launch.py motion_control_mode:=print_pose \
  > /tmp/pp_$ID.log 2>&1 || true

echo "[3/3] 配对写入 samples.yaml ..."
/usr/bin/python3 src/my_robot_vision/calib_append.py --id "$ID" --pp /tmp/pp_$ID.log --handeye "$HE"

echo "✅ 完成 #$ID。换个【明显不同的姿态(多转腕、远近、板子在画面不同区域)】再跑一次。"
