#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""控制 fawkes-robotino 夹持器：拆卸时夹住"要拔的积木的下面那块"，
   让机械臂往上拔的时候不会把整摞都带起来。

硬件命令（在 fawkes-robotino 容器里跑）：
    ./bin/skillet "gripper_commands{command='MOVEABS', x=<高度>, y=<开合>, z=0.0}"

轴含义（来自手动测试）：
    x = 夹持器高度，从下到上 4 档：0.0 / 0.04 / 0.08 / 0.115  [m]
    y = 夹爪开合：闭合(夹住)=0.001，张开(松开)=0.003
    z = 没用到，固定 0.0

铁律：改变高度(x)之前夹爪必须先【张开】，否则夹着的积木会被拖着一起动。
      所以"换到某一档并夹住"= 在当前高度张开 -> 移到目标高度(保持张开) -> 闭合。

典型拆卸配合：
    1) clamp_control.py grip --level 1   # 夹住目标积木下面那块
    2) (机械臂把目标积木往上拔、放到一边)
    3) clamp_control.py grip --level 0   # 夹住新的"下面那块"(换档自动先松开)
    4) ... 重复 ...
    5) clamp_control.py open             # 全部拆完，松开
"""
import argparse
import json
import os
import subprocess
import sys
import time

# ── 改这里即可适配你的硬件 ────────────────────────────────────────────
CONTAINER = "fawkes-robotino-fawkes-robotino-1"
SKILLET = "./bin/skillet"
WORKDIR = os.path.expanduser("~/fawkes-robotino")  # skillet 要在这个目录下跑

JAW_CLOSE = -0.001   # 闭合/夹住
JAW_OPEN = -0.003    # 张开/松开   （与测试命令里的 y=-0.003 一致）
LEVELS = [0.0, 0.04, 0.08, 0.115]   # index 0=最低 ... 3=最高
Z_FIXED = 0.0
SETTLE = 1.0        # 每条命令后等待运动完成的秒数
USE_SUDO = True
# ──────────────────────────────────────────────────────────────────────

STATE_FILE = "/tmp/fawkes_clamp_state.json"


def load_state():
    try:
        with open(STATE_FILE) as f:
            return json.load(f)
    except Exception:
        # 不知道当前位置时, x=None: 第一次操作不尝试在"上一个高度"松开
        return {"x": None, "y": JAW_OPEN}


def save_state(x, y):
    with open(STATE_FILE, "w") as f:
        json.dump({"x": x, "y": y}, f)


def send(x, y, z=Z_FIXED, dry_run=False):
    """发一条 MOVEABS 给夹持器。"""
    cmd = "gripper_commands{command='MOVEABS', x=%s, y=%s, z=%s}" % (x, y, z)
    full = (["sudo"] if USE_SUDO else []) + \
           ["docker", "exec", CONTAINER, SKILLET, cmd]
    tag = "DRY" if dry_run else "->"
    print("[clamp %s] x=%-6s y=%-7s z=%s" % (tag, x, y, z))
    if not dry_run:
        subprocess.run(full, check=True, cwd=WORKDIR)
        time.sleep(SETTLE)
    save_state(x, y)


def open_jaw(dry_run=False):
    """在当前高度张开(松开积木)。不知道当前高度就在最低档松开。"""
    x = load_state()["x"]
    if x is None:
        x = LEVELS[0]
    send(x, JAW_OPEN, dry_run=dry_run)


def grip_level(i, dry_run=False):
    """安全地换到第 i 档并夹住: 先在当前高度松开 -> 移到目标高度 -> 闭合。"""
    if not 0 <= i < len(LEVELS):
        raise SystemExit("level 必须是 0..%d" % (len(LEVELS) - 1))
    x = LEVELS[i]
    st = load_state()
    # 1) 若当前夹着东西且要换高度, 先在【当前】高度松开, 别把它拖走
    if st["x"] is not None and st["x"] != x and st["y"] == JAW_CLOSE:
        send(st["x"], JAW_OPEN, dry_run=dry_run)
    # 2) 保持张开移动到目标高度
    send(x, JAW_OPEN, dry_run=dry_run)
    # 3) 闭合夹住
    send(x, JAW_CLOSE, dry_run=dry_run)
    print("[clamp] 已夹住第 %d 档 (x=%s)" % (i, x))


def goto_level(i, dry_run=False):
    """只移动到第 i 档(保持张开, 不夹)。"""
    if not 0 <= i < len(LEVELS):
        raise SystemExit("level 必须是 0..%d" % (len(LEVELS) - 1))
    open_jaw(dry_run=dry_run)              # 先确保松开再移动
    send(LEVELS[i], JAW_OPEN, dry_run=dry_run)


def main():
    ap = argparse.ArgumentParser(description="fawkes-robotino 夹持器控制")
    sub = ap.add_subparsers(dest="action", required=True)

    p = sub.add_parser("grip", help="换到某一档并夹住(自动先松开再移动)")
    p.add_argument("--level", type=int, required=True, help="0..%d" % (len(LEVELS) - 1))

    p = sub.add_parser("goto", help="只移动到某一档, 保持张开不夹")
    p.add_argument("--level", type=int, required=True)

    sub.add_parser("open", help="在当前高度张开/松开")
    sub.add_parser("close", help="在当前高度闭合/夹住(不移动)")

    p = sub.add_parser("raw", help="直接发任意 x/y/z")
    p.add_argument("--x", type=float, required=True)
    p.add_argument("--y", type=float, required=True)
    p.add_argument("--z", type=float, default=Z_FIXED)

    for sp in sub.choices.values():
        sp.add_argument("--dry-run", action="store_true", help="只打印不真正发送")

    a = ap.parse_args()
    dry = getattr(a, "dry_run", False)

    if a.action == "grip":
        grip_level(a.level, dry_run=dry)
    elif a.action == "goto":
        goto_level(a.level, dry_run=dry)
    elif a.action == "open":
        open_jaw(dry_run=dry)
    elif a.action == "close":
        x = load_state()["x"]
        send(x if x is not None else LEVELS[0], JAW_CLOSE, dry_run=dry)
    elif a.action == "raw":
        send(a.x, a.y, a.z, dry_run=dry)


if __name__ == "__main__":
    main()
