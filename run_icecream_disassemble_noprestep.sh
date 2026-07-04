#!/bin/bash
# icecream 【拆卸 · 无前置版】: 5 块, 执行逻辑与 run_carrot_disassemble_noprestep.sh 完全一致
# (NO_PRESTEP=1: 每块直接 movej 拔取姿态 -> 下降 pick.descend 抓 -> 抬 -> 丢放 -> 松), 区别只有:
#   1) 多两轮(5 块 -> 抓 5 次, 丢放点沿 base +X 逐块 +DROP_STEP_X);
#   2) 第2/3轮(红/蓝 2x2 并排层)抓取下降起点 x 各偏 ±半个 2x2 Duplo 边长(0.016m):
#        轮1 绿2x2顶(居中0) / 轮2 x+0.016 / 轮3 x-0.016 / 轮4 黄2x4(0) / 轮5 黄2x2底(0)。
# 其余(拔取姿态/descend/速度/夹爪 onrobot_io)全部同 carrot 版, 用同一 disassembly_task.yaml。
# 2026-07-05 丢放改为: x 固定(DROP_STEP_X=0), y 以 observe 为原点来回摆, 间隔6cm:
#   第1块 y=0 / 第2块 -0.06 / 第3块 +0.06 / 第4块 -0.12 / 第5块 +0.12。
# 首验轨迹用 GRIPPER_MODE=none bash run_icecream_disassemble_noprestep.sh。
set -e
cd /home/i6user/Desktop/robot_lego
NO_PRESTEP=1 PICK_OFFSET_X_LIST="0 0.016 -0.016 0 0" \
DROP_STEP_X=0 PLACE_OFFSET_Y_LIST="0 -0.06 0.06 -0.12 0.12" \
bash disassembly_arm_pull.sh pull_top 5
