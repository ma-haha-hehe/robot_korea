#!/bin/bash
# 标定合格后, 把新结果应用到正式配置(自动备份旧的)。
cd /home/i6user/Desktop/robot_lego
HE=src/my_robot_vision/my_robot_vision/handeye
CFG=src/my_robot_vision/config/camera_to_tool.yaml
if [ ! -s "$HE/camera_to_tool.yaml" ]; then
  echo "没有标定结果, 先跑 calib_run.sh"; exit 1
fi
cp -f "$CFG" "$CFG.bak.$(date +%Y%m%d_%H%M%S)" 2>/dev/null || true
cp -f "$HE/camera_to_tool.yaml" "$CFG"
echo "✅ 新标定已应用 -> $CFG (旧的已备份为 .bak.*)"
echo "下次跑 pipeline 即生效。建议先用 run_diag.sh 复测一下角点基座坐标是否对上了。"
