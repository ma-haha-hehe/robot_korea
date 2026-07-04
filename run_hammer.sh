#!/bin/bash
# hammer: 2x2 红(底) -> 2x2 红(中) -> 2x4 蓝(顶, 长轴沿 base X 轴), 三块堆叠。
cd /home/i6user/Desktop/robot_lego
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/i6user/Desktop/robot_lego/fastdds_udp_only.xml  # 禁SHM走UDP修sequence-size崩溃
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run my_robot_vision run_auto_pick_pipeline.py \
  --product-yaml src/panda_pick/src/final_product_hammer.yaml \
  --async-next-vision --start-index 0 --max-tasks 3 \
  --gripper-mode onrobot_io \
  --movej-velocity 2.4 --movej-acceleration 2.0 \
  --movel-velocity 0.40 --movel-acceleration 1.2 \
  --pick-lift-velocity 0.32 \
  --pick-approach-movej-velocity 3.0 --pick-approach-movel-velocity 0.28 \
  --pipeline-wait 50.0 --done-still-seconds 3.5 --place-descend-velocity 0.002 \
  --place-wiggle-xy-amplitude 0.0006 --place-wiggle-velocity 0.012 --place-wiggle-steps 12 \
  --finish-grab --finish-place-descend 0.10 \
  --vision-timeout 180
# 收尾: 装配完抓起成品搬到右前放置位(UR5_FINISH_DROP_JOINTS)再回observe。
# 收尾降深自动=最后一块落点(成品顶, cpp内算=末块place.descend-place.dz); 太紧/太松用 --finish-pick-descend-offset 微调(+更深/-更浅,米)。
