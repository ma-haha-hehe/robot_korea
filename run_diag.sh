#!/bin/bash
# 方法A诊断：棋盘角点 -> 相机系 + 基座系坐标(容器内跑)。
# 默认 5x7方格(=4x6内角点) 30mm。检测不到就 --board-size 5 7 再试。
set -e
SRC=/home/i6user/Desktop/robot_lego/src/my_robot_vision
VC=$SRC/my_robot_vision     # 容器内挂载为 /vision_code
# 把运行时配置拷进 /vision_code 供脚本读取
cp -f "$SRC/config/camera_to_tool.yaml"          "$VC/camera_to_tool.yaml"
cp -f "$SRC/config/vision_execution_bridge.yaml" "$VC/vision_execution_bridge.yaml"
sudo docker exec vision_node_final /opt/conda/envs/my/bin/python \
  /vision_code/diag_chessboard_to_base.py --board-size 4 6 --square-size 0.030 "$@"
echo "标注图: $VC/diag_corner.jpg"
