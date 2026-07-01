#!/bin/bash
# 斜抓实验 (tilt) —— 与主线完全分离, 不影响 run_pick.sh。
# 沿物体 z 轴方向接近并下探抓取(仅 pick, 抓完回 home, 不做 place)。
# pre 位姿默认落在【观察高度】: 先在高处把夹爪角度对齐积木 z 轴, 再沿斜轴下探。
#
# 前提: 先让 vision_output.yaml 是【当前这块积木】的结果(先跑一次 dry-run 视觉):
#   ros2 run my_robot_vision run_auto_pick_pipeline.py \
#     --product-yaml src/panda_pick/src/final_product_white_orange_blue_stack.yaml \
#     --start-index 0 --max-tasks 1 --dry-run --gripper-mode robotiq_socket --vision-timeout 120
#   (报 dx/dy 超限没关系, 只要 vision_output.yaml 更新了)
#
# 用法:
#   bash run_pick_tilt.sh pre     # 只到"观察高度+倾斜姿态"就停, 肉眼确认方向(不下探/不闭爪) ★第一次先这个
#   bash run_pick_tilt.sh         # 完整: 高处对齐 -> 沿斜轴下探 -> 闭爪 -> 抬回 -> 回 home
set -e
cd /home/i6user/Desktop/robot_lego
source /opt/ros/humble/setup.bash
source install/setup.bash
source src/panda_pick/config/ur5_taught_joints.env

VIS=FoundationPose/root/FoundationPose/vision_output.yaml
STOP_AT_PRE=false
[ "$1" = "pre" ] && STOP_AT_PRE=true

# 1) 换算: vision_output -> pick_place_task_tilt.yaml (pre 落在观察高度)
/usr/bin/python3 src/my_robot_vision/my_robot_vision/tilt/vision_to_execution_tilt.py "$VIS"

# 2) 执行斜抓 (慢速; 确认方向对了再提速)
ros2 launch panda_pick run_ur5.launch.py \
  motion_control_mode:=joint_pick_place_tilt \
  gripper_control_mode:=robotiq_socket \
  gripper_socket_speed:=80 gripper_socket_force:=145 \
  gripper_socket_open_wait_seconds:=1.2 \
  task_file:=src/panda_pick/config/pick_place_task_tilt.yaml \
  urscript_home_joints:="$UR5_HOME_JOINTS" \
  tilt_stop_at_pre:=$STOP_AT_PRE \
  urscript_movej_velocity:=0.5 urscript_movej_acceleration:=1.2 \
  urscript_movel_velocity:=0.12 urscript_movel_acceleration:=0.6 \
  urscript_pipeline_wait_seconds:=40.0
