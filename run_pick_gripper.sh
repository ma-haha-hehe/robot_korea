#!/bin/bash
# 半合抓取(128) + wiggle + 小幅提速 + 闭环/静止等待(不空等) + 诊断日志
# 2026-06-30: 取消两段开合夹爪(见文件末尾注释), 并加 --show-windows 弹视觉窗口
cd /home/i6user/Desktop/robot_lego
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run my_robot_vision run_auto_pick_pipeline.py \
  --product-yaml src/panda_pick/src/final_product_white_blue_orange.yaml \
  --async-next-vision --start-index 0 --max-tasks 3 \
  --show-windows \
  --gripper-mode robotiq_socket --gripper-speed 50 --gripper-force 145 --gripper-wait 2.0 \
  --movej-velocity 1.10 --movej-acceleration 2.0 \
  --movel-velocity 0.40 --movel-acceleration 1.2 \
  --pick-lift-velocity 0.32 \
  --pick-approach-movej-velocity 0.60 --pick-approach-movel-velocity 0.18 \
  --pipeline-wait 50.0 --done-still-seconds 3.5 --place-descend-velocity 0.002 \
  --place-wiggle-xy-amplitude 0.0006 --place-wiggle-velocity 0.012 --place-wiggle-steps 12 \
  --vision-timeout 180

# ---- 恢复"两段开合夹爪"用 ----
# 2026-06-30 取消了下面两行(pregrasp 半合预合 + place 分段释放)。
# 想恢复: 把这两行插回上面命令里 --gripper-wait 那行之后(都以 \ 结尾):
#   --gripper-pregrasp-position 128 \
#   --gripper-release-position 120 --gripper-release-wait 0.5 \
# 当前(取消后)行为: gripper-pregrasp-position 默认 0(下降前完全张开),
#                   gripper-release-position 默认 140。
