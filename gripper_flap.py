#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""夹爪【原地快速开合】(OnRobot RG2)。机械臂完全不动, 只有夹爪反复张开/闭合。

复用 gripper.py 的 URScript RG 协议(工具口 DO8/9 + 上电脉冲), 但只发【一条】
URScript: 上电一次, 然后在机器人端 while 循环里反复 合->开, 所以很快, 不用每次重上电。

前提(同 gripper.py): UR 远程模式、已上电无保护停止、ur_robot_driver 在跑。
用法:
  source /opt/ros/humble/setup.bash && source install/setup.bash
  python3 gripper_flap.py            # 默认开合 8 次
  python3 gripper_flap.py 20         # 开合 20 次
  可选: --open-width mm --close-width mm --force N --wait s(每次开/合等待)
"""
import argparse
import sys

import gripper as g  # 同目录, 复用 HELPERS / command_value / publish / MODELS


def build_flap(cycles, open_mm, close_mm, force, wait, m):
    close_cmd = g.command_value(close_mm, force, m)
    open_cmd = g.command_value(open_mm, force, m)
    w = "%.3f" % max(0.05, wait)
    lines = [
        "def onrobot_rg_flap():",
        g.HELPERS.rstrip("\n"),
        "  onrobot_rg_ready = onrobot_rg_powerup()",
        "  if (onrobot_rg_ready):",
        "    local i = 0",
        "    while i < %d:" % max(1, cycles),
        "      onrobot_rg_bit(%d)" % close_cmd,   # 合
        "      onrobot_rg_wait_motion(%s)" % w,
        "      onrobot_rg_bit(%d)" % open_cmd,    # 开
        "      onrobot_rg_wait_motion(%s)" % w,
        "      i = i + 1",
        "    end",
        "  end",
        "end",
    ]
    return "\n".join(lines) + "\n"


def main():
    ap = argparse.ArgumentParser(description="OnRobot RG2 夹爪原地快速开合(机械臂不动)")
    ap.add_argument("cycles", nargs="?", type=int, default=8, help="开合次数(默认8)")
    ap.add_argument("--open-width", type=float, default=100, help="张开宽度 mm")
    ap.add_argument("--close-width", type=float, default=0, help="闭合宽度 mm")
    ap.add_argument("--force", type=float, default=20, help="夹持力 N (rg2:0-40)")
    ap.add_argument("--wait", type=float, default=0.6, help="每次开/合等待秒数(越小越快)")
    ap.add_argument("--model", default="rg2", choices=list(g.MODELS))
    ap.add_argument("--dry-run", action="store_true", help="只打印 URScript 不发送")
    a = ap.parse_args()
    m = g.MODELS[a.model]

    script = build_flap(a.cycles, a.open_width, a.close_width, a.force, a.wait, m)
    print("---- URScript(开合 %d 次) ----" % max(1, a.cycles))
    print(script)
    print("------------------")
    if a.dry_run:
        return
    ok = g.publish(script)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
