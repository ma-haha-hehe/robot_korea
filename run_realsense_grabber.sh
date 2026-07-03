#!/bin/bash
# RealSense(D435i) 帧 grabber: 640x480 对齐彩色+深度 -> 和 O3P 相同的文件格式,
# 写到 src/panda_pick/src/o3p(容器内视觉桥挂载读取)。替代 O3P 相机节点+o3p_grabber。
# 直接用 pyrealsense2, 不走 ROS/DDS。跑 pipeline 前在独立终端跑这个并【保持运行】。
cd /home/i6user/Desktop/robot_lego
exec /usr/bin/python3 src/my_robot_vision/my_robot_vision/realsense_grabber.py \
  --out /home/i6user/Desktop/robot_lego/src/panda_pick/src/o3p --rate 15
