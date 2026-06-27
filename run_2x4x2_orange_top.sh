#!/bin/bash
# 两块2x4堆叠 + 顶上橙色2x2：白(底) -> 蓝(中) -> 橙2x2(顶)
cd /home/i6user/Desktop/robot_lego
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run my_robot_vision run_auto_pick_pipeline.py \
  --product-yaml src/panda_pick/src/final_product_2x4x2_orange_top.yaml \
  --async-next-vision --start-index 0 --max-tasks 3 \
  --gripper-mode robotiq_socket --gripper-speed 50 --gripper-force 145 --gripper-wait 2.0 \
  --gripper-pregrasp-position 128 \
  --gripper-release-position 120 --gripper-release-wait 0.5 \
  --movej-velocity 1.10 --movej-acceleration 2.0 \
  --movel-velocity 0.40 --movel-acceleration 1.2 \
  --pick-lift-velocity 0.32 \
  --pick-approach-movej-velocity 0.60 --pick-approach-movel-velocity 0.18 \
  --pipeline-wait 50.0 --done-still-seconds 3.5 --place-descend-velocity 0.002 \
  --place-wiggle-xy-amplitude 0.0006 --place-wiggle-velocity 0.012 --place-wiggle-steps 12 \
  --vision-timeout 180
