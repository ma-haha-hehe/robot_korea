#!/bin/bash
# OnRobot RG2 版 (2026-07-02): 完全照昨晚 run_pick.sh 的参数, 只把夹爪从 none 换成 onrobot_rg。
# RG2 单线缆插 UR 工具口(无 Compute Box), 走工具口 I/O 控制(cpp 里 onrobot_rg 模式)。
# 前提: UR 远程模式、已上电无保护停止、ur_robot_driver 在跑。
# 先单独测夹爪能动: python3 gripper.py activate && python3 gripper.py open && python3 gripper.py close
#
# ⚠️ descend 提醒: 当前 vision_execution_bridge.yaml 的 descend 还是"无夹爪"深值(抓0.25/放0.325)。
#    装上 RG2(末端变长)后这个深度会偏深、有撞台面风险。首跑请手放急停旁,
#    或让我先把 descend 调保守(停在上方)看准 RG2 停位再逐步下探。
cd /home/i6user/Desktop/robot_lego
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/i6user/Desktop/robot_lego/fastdds_udp_only.xml  # 禁SHM走UDP修sequence-size崩溃
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run my_robot_vision run_auto_pick_pipeline.py \
  --product-yaml src/panda_pick/src/final_product_white_blue_orange.yaml \
  --async-next-vision --start-index 0 --max-tasks 3 \
  --show-windows \
  --gripper-mode onrobot_io \
  --movej-velocity 0.55 --movej-acceleration 1.0 \
  --movel-velocity 0.20 --movel-acceleration 0.6 \
  --pick-lift-velocity 0.16 \
  --pick-approach-movej-velocity 0.30 --pick-approach-movel-velocity 0.09 \
  --pipeline-wait 70.0 --done-still-seconds 12.0 --place-descend-velocity 0.002 \
  --place-wiggle-xy-amplitude 0.0006 --place-wiggle-velocity 0.012 --place-wiggle-steps 12 \
  --vision-timeout 180
