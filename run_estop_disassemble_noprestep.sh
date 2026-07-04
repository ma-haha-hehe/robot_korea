#!/bin/bash
# estop 【拆卸 · 无前置版】: 去掉前置动作(不先抓预置物到初始位), 每块直接在拔取姿态下降抓。
#   实现 = NO_PRESTEP=1, 每块: movej 拔取姿态 -> 下降 pick.descend 抓 -> 抬 -> 丢放 -> 松。
#   其余与 run_estop_disassemble.sh 一致(2块, disassembly_task_estop.yaml, PICK_OFFSET_X=0.01)。
#   注: PRESTEP_DESCEND 在无前置时不生效(只前置段用), 故不传。
# 2026-07-05 丢放改为: x 固定(DROP_STEP_X=0), y 以 observe 为原点来回摆, 间隔6cm:
#   第1块 y=0 / 第2块 y=-0.06。
# 首验轨迹用 GRIPPER_MODE=none bash run_estop_disassemble_noprestep.sh。
set -e
cd /home/i6user/Desktop/robot_lego
TASK_FILE="$PWD/disassembly_task_estop.yaml" \
PICK_OFFSET_X=0.01 \
NO_PRESTEP=1 DROP_STEP_X=0 PLACE_OFFSET_Y_LIST="0 -0.06" \
bash disassembly_arm_pull.sh pull_top 2
