#!/bin/bash
# carrot 【拆卸】: 直接调用共享的 disassembly_arm_pull.sh(取+放一条龙, 用 disassembly_task.yaml)。
# 产品 carrot = 3 块 -> 固定抓取点抓 3 次, 放到 3 个点:
#   放置示教点 = 装配 observe 点, 每块 = 往下10cm + base +X 逐块 +11cm (第1块0 / 第2块+11 / 第3块+22)。
# 独立于装配, 不碰 run_carrot_onrobot.sh。
# 夹爪默认 onrobot_io(与装配一致, pin16 开合)。首验轨迹用 GRIPPER_MODE=none bash run_carrot_disassemble.sh。
set -e
cd /home/i6user/Desktop/robot_lego
bash disassembly_arm_pull.sh pull_top 3
