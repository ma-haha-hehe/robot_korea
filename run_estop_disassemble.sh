#!/bin/bash
# estop 【拆卸】: 直接调用共享的 disassembly_arm_pull.sh(取+放一条龙, 用 disassembly_task.yaml)。
# 产品 estop = 2 块 -> 固定抓取点抓 2 次, 放到 2 个点:
#   放置示教点 = 装配 observe 点, 每块 = 往下10cm + base +X 逐块 +11cm (第1块0 / 第2块+11)。
# 独立于装配, 不碰 run_estop_onrobot.sh。
# 夹爪默认 onrobot_io(与装配一致, pin16 开合)。首验轨迹用 GRIPPER_MODE=none bash run_estop_disassemble.sh。
# 2026-07-03: 抓取位置 base 系 x +1cm (PICK_OFFSET_X=0.01): 前置后移动到的抓取点/后续每块抓取点都 +1cm;
#   放置(丢放)位置不变。carrot 不传该值(默认0)故不受影响。
set -e
cd /home/i6user/Desktop/robot_lego
# 2026-07-03: PRESTEP_DESCEND=0.165 前置下降抓比默认(0.143)低2.2cm; carrot 不传故不受影响。
# 2026-07-03: TASK_FILE 指向 estop 专属 task, 抓/放 descend 与 carrot 完全分开(改那个文件只影响 estop)。
TASK_FILE="$PWD/disassembly_task_estop.yaml" \
PICK_OFFSET_X=0.01 PRESTEP_DESCEND=0.165 \
bash disassembly_arm_pull.sh pull_top 2
