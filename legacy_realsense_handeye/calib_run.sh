#!/bin/bash
# 用采好的 samples.yaml 跑手眼标定(强制用正确实时内参), 结果写到暂存区。
cd /home/i6user/Desktop/robot_lego
sudo docker exec vision_node_final /opt/conda/envs/my/bin/python \
  /vision_code/handeye_calibrate.py \
  --samples /vision_code/handeye/samples.yaml \
  --output  /vision_code/handeye/camera_to_tool.yaml \
  --annotated-dir /vision_code/handeye/debug \
  --method tsai
echo
echo "↑ 看最后一行 'base target translation std [m]' 三个值:"
echo "   都 < 0.005~0.010(5-10mm) = 合格 -> 执行  bash ~/Desktop/robot_lego/calib_apply.sh"
echo "   偏大 -> 再补几个【旋转差异大】的姿态(calib_capture.sh)重跑本脚本。"
echo "   角点检测图: src/my_robot_vision/my_robot_vision/handeye/debug/"
