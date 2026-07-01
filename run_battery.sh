#!/bin/bash
# battery 产品: 蓝2x2底 + 黄2x2顶。半合抓取 + wiggle + 闭环等待 + 视觉窗口。
cd /home/i6user/Desktop/robot_lego
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run my_robot_vision run_auto_pick_pipeline.py \
  --product-yaml src/panda_pick/src/final_product_battery.yaml \
  --async-next-vision --start-index 0 --max-tasks 2 \
  --show-windows \
  --gripper-mode robotiq_socket --gripper-speed 50 --gripper-force 145 --gripper-wait 2.0 \
  --movej-velocity 1.10 --movej-acceleration 2.0 \
  --movel-velocity 0.40 --movel-acceleration 1.2 \
  --pick-lift-velocity 0.32 \
  --pick-approach-movej-velocity 0.60 --pick-approach-movel-velocity 0.18 \
  --pipeline-wait 50.0 --done-still-seconds 3.5 --place-descend-velocity 0.002 \
  --place-wiggle-xy-amplitude 0.0006 --place-wiggle-velocity 0.012 --place-wiggle-steps 12 \
  --vision-timeout 180
