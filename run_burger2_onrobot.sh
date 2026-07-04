#!/bin/bash
# burger2 新夹爪版 (OnRobot, onrobot_io = 工具口 pin16 开合: 0.0开/1.0关)。
# 产品: 黄2x4底 + 红2x4/红2x2内联(两端各悬1) + 黄2x4顶 (4块3层), 装配点=装配区原点。
# 布局见 src/panda_pick/src/final_product_burger2.yaml。顶/底黄2x4同中心。
# 前提: UR 远程模式、已上电无保护停止、ur_robot_driver 在跑(不开 rviz)。
# ⚠️ descend 为无夹爪深值(抓/放来自 config bridge yaml), 首跑手放急停旁。
cd /home/i6user/Desktop/robot_lego
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/i6user/Desktop/robot_lego/fastdds_udp_only.xml  # 禁SHM走UDP修sequence-size崩溃
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run my_robot_vision run_auto_pick_pipeline.py \
  --product-yaml src/panda_pick/src/final_product_burger2.yaml \
  --async-next-vision --start-index 0 --max-tasks 4 \
  --show-windows \
  --gripper-mode onrobot_io \
  --movej-velocity 2.4 --movej-acceleration 2.5 \
  --movel-velocity 0.40 --movel-acceleration 1.25 \
  --pick-lift-velocity 0.16 \
  --pick-approach-movej-velocity 3.0 --pick-approach-movel-velocity 0.14 \
  --pipeline-wait 70.0 --done-still-seconds 12.0 --place-descend-velocity 0.002 \
  --place-wiggle-xy-amplitude 0.0006 --place-wiggle-velocity 0.012 --place-wiggle-steps 12 \
  --finish-grab --finish-place-descend 0.10 \
  --vision-timeout 180
# 收尾: 装配完抓起成品搬到右前放置位(UR5_FINISH_DROP_JOINTS)再回observe。
# 收尾降深自动=最后一块落点(成品顶, cpp内算=末块place.descend-place.dz); 太紧/太松用 --finish-pick-descend-offset 微调(+更深/-更浅,米)。
