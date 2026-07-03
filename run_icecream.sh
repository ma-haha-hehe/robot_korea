#!/bin/bash
# icecream: 黄2x2底 -> 黄2x4中(长轴沿 y) -> 红2x2+蓝2x2并列 -> 绿2x2顶 (5块4层)。
# 放置不做 wiggle 抖动(--disable-place-wiggle); 其余动作与带 wiggle 版完全一致。
cd /home/i6user/Desktop/robot_lego
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run my_robot_vision run_auto_pick_pipeline.py \
  --product-yaml src/panda_pick/src/final_product_icecream.yaml \
  --async-next-vision --start-index 0 --max-tasks 5 \
  --gripper-mode robotiq_socket --gripper-speed 50 --gripper-force 145 --gripper-wait 2.0 \
  --gripper-pregrasp-position 128 \
  --gripper-release-position 120 --gripper-release-wait 0.5 \
  --movej-velocity 1.10 --movej-acceleration 2.0 \
  --movel-velocity 0.40 --movel-acceleration 1.2 \
  --pick-lift-velocity 0.32 \
  --pick-approach-movej-velocity 0.60 --pick-approach-movel-velocity 0.18 \
  --pipeline-wait 50.0 --done-still-seconds 3.5 --place-descend-velocity 0.002 \
  --disable-place-wiggle \
  --vision-timeout 180
