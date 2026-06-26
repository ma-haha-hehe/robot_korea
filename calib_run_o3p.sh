#!/bin/bash
# O3P: 用采好的 samples.yaml 跑高清彩色(鱼眼)手眼标定, 结果写到暂存区(handeye/)。
# 合格后用 calib_apply.sh 应用到正式配置。
cd /home/i6user/Desktop/robot_lego
HE=src/my_robot_vision/my_robot_vision/handeye

/usr/bin/python3 src/my_robot_vision/my_robot_vision/handeye_calibrate_o3p.py \
  --samples "$HE/samples.yaml" \
  --output  "$HE/camera_to_tool.yaml" \
  --debug-dir "$HE/debug" \
  --method tsai

echo
echo "↑ 看 'base target translation std [m]' 三个值:"
echo "   都 < 0.005~0.010(5-10mm) = 合格 -> 跑  bash calib_apply.sh"
echo "   偏大 -> 再补几个【旋转差异大】的姿态(calib_capture_o3p.sh)重跑本脚本。"
echo "   角点检测图: $HE/debug/"
