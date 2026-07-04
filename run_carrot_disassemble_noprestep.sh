#!/bin/bash
# carrot 【拆卸 · 无前置版】: 去掉前置动作(不先抓预置物到初始位), 每块直接在拔取姿态下降抓。
#   实现 = NO_PRESTEP=1, 每块: movej 拔取姿态 -> 下降 pick.descend 抓 -> 抬 -> 丢放 -> 松。
#   其余与 run_carrot_disassemble.sh 完全一致(3块, disassembly_task.yaml, onrobot_io)。
# 首验轨迹用 GRIPPER_MODE=none bash run_carrot_disassemble_noprestep.sh。
set -e
cd /home/i6user/Desktop/robot_lego
NO_PRESTEP=1 bash disassembly_arm_pull.sh pull_top 3
