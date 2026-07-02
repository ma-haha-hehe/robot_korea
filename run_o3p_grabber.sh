#!/bin/bash
# 启动 O3P 相机帧 grabber: 持续把【SDK对齐彩色240x180 + 高清彩色1344x1008 + 深度 + 内参/外参】
# 写到 /shared_data/o3p(= src/panda_pick/src/o3p), 供容器内视觉桥读取。
#
# ★ 跑 pipeline 前必须先在一个独立终端跑这个并【保持运行】。视觉帧全靠它。
# 前提: ifm o3p ROS 节点(ros2 launch o3p_node camera_node.launch.py)已在跑且在出数据。
cd /home/i6user/Desktop/robot_lego
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/i6user/Desktop/robot_lego/fastdds_udp_only.xml
source /opt/ros/humble/setup.bash
exec /usr/bin/python3 src/my_robot_vision/my_robot_vision/o3p_grabber.py \
  --out /home/i6user/Desktop/robot_lego/src/panda_pick/src/o3p --rate 15
