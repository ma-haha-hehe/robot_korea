#!/bin/bash
# trafficlight 装配 (OnRobot, onrobot_io = 工具口 pin16 开合: 0.0开/1.0关)。
# 产品: 绿2x2底 + 黄2x2中 + 红2x2顶 (3块), 装配点=装配区原点。
# 布局见 src/panda_pick/src/final_product_trafficlight.yaml。
# 前提: UR 远程模式、已上电无保护停止、ur_robot_driver 在跑(不开 rviz)。
# ⚠️ descend 现为无夹爪深值(抓0.199/放0.3671), 首跑手放急停旁, 或让我先调保守。
cd /home/i6user/Desktop/robot_lego
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/i6user/Desktop/robot_lego/fastdds_udp_only.xml  # 禁SHM走UDP修sequence-size崩溃
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run my_robot_vision run_auto_pick_pipeline.py \
  --product-yaml src/panda_pick/src/final_product_trafficlight.yaml \
  --async-next-vision --start-index 0 --max-tasks 3 \
  --show-windows \
  --gripper-mode onrobot_io \
  --movej-velocity 1.25 --movej-acceleration 2.5 \
  --movel-velocity 0.40 --movel-acceleration 1.25 \
  --pick-lift-velocity 0.16 \
  --pick-approach-movej-velocity 1.25 --pick-approach-movel-velocity 0.09 \
  --pipeline-wait 70.0 --done-still-seconds 12.0 --place-descend-velocity 0.002 \
  --place-wiggle-xy-amplitude 0.0006 --place-wiggle-velocity 0.012 --place-wiggle-steps 12 \
  --vision-timeout 180
