#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""控制 fawkes-robotino 夹持器：拆卸时夹住"要拔的积木的下面那块"，
   让机械臂往上拔的时候不会把整摞都带起来。

硬件命令（在 fawkes-robotino 容器里跑）：
    ./bin/skillet "gripper_commands{command='MOVEABS', x=<高度>, y=<开合>, z=0.0}"

轴含义（来自手动测试）：
    x = 夹持器高度，【x 越小夹得越高】。4 档(从高到低)：0.0 / 0.04 / 0.08 / 0.115  [m]
    y = 夹爪开合：闭合(夹住)=0.001，张开(松开)=0.003
    z = 没用到，固定 0.0

铁律：改变高度(x)之前夹爪必须先【张开】，否则夹着的积木会被拖着一起动。
      所以"换到某一档并夹住"= 在当前高度张开 -> 移到目标高度(保持张开) -> 闭合。

典型拆卸配合（从上往下拆, 夹持器越往后越往下 = level 越大）：
    1) clamp_control.py grip --level 1   # 夹住目标积木下面那块(较高)
    2) (机械臂把目标积木往上拔、放到一边)
    3) clamp_control.py grip --level 2   # 塔变矮, 夹更低那块(换档自动先松开)
    4) ... 重复(level 递增) ...
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
LEVELS = [0.0, 0.04, 0.08, 0.115]   # x越小夹得越高 -> index 0=最高, 3=最低
Z_FIXED = 0.0
SETTLE = 1.0        # 夹爪开/合(不改 x)后等待秒数
MOVE_SETTLE = 3.0   # 改变 x(升降)后等待秒数, 要够长保证 x 真正到位再闭合
USE_SUDO = True
# ──────────────────────────────────────────────────────────────────────
# 上面是默认值; 若同目录存在 clamp_config.json 则用它覆盖(改 json 即可, 不用动代码)。

STATE_FILE = "/tmp/fawkes_clamp_state.json"
CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "clamp_config.json")


def apply_config():
    """用同目录 clamp_config.json 覆盖默认参数(存在才覆盖)。"""
    global JAW_CLOSE, JAW_OPEN, LEVELS, Z_FIXED, SETTLE, MOVE_SETTLE, USE_SUDO, CONTAINER, SKILLET, WORKDIR
    try:
        with open(CONFIG_FILE) as f:
            c = json.load(f)
    except FileNotFoundError:
        return
    JAW_CLOSE = c.get("jaw_close", JAW_CLOSE)
    JAW_OPEN = c.get("jaw_open", JAW_OPEN)
    LEVELS = c.get("levels", LEVELS)
    Z_FIXED = c.get("z", Z_FIXED)
    SETTLE = c.get("settle", SETTLE)
    MOVE_SETTLE = c.get("move_settle", MOVE_SETTLE)
    USE_SUDO = c.get("use_sudo", USE_SUDO)
    CONTAINER = c.get("container", CONTAINER)
    SKILLET = c.get("skillet", SKILLET)
    WORKDIR = os.path.expanduser(c.get("workdir", WORKDIR))


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


def send(x, y, z=None, dry_run=False, settle=None):
    """发一条 MOVEABS 给夹持器。settle=本条命令后等待秒数(不传则用 SETTLE)。"""
    if z is None:
        z = Z_FIXED
    if settle is None:
        settle = SETTLE
    cmd = "gripper_commands{command='MOVEABS', x=%s, y=%s, z=%s}" % (x, y, z)
    full = (["sudo"] if USE_SUDO else []) + \
           ["docker", "exec", CONTAINER, SKILLET, cmd]
    tag = "DRY" if dry_run else "->"
    print("[clamp %s] x=%-6s y=%-7s z=%s  (等待 %.1fs)" % (tag, x, y, z, settle))
    if not dry_run:
        subprocess.run(full, check=True, cwd=WORKDIR)
        time.sleep(settle)
    save_state(x, y)


def open_jaw(dry_run=False):
    """在当前高度张开(松开积木)。不知道当前高度就在最低档松开。"""
    x = load_state()["x"]
    if x is None:
        x = LEVELS[0]
    send(x, JAW_OPEN, dry_run=dry_run)


def grip_level(i, dry_run=False):
    """换到第 i 档并夹住, 严格三步且互相等待:
       1) x 变化【前】先把夹爪张开(在当前高度, 单独一条, 等它完成);
       2) 保持张开升降到目标高度, 等 MOVE_SETTLE 保证 x 真正到位;
       3) x 到位【后】才闭合。"""
    if not 0 <= i < len(LEVELS):
        raise SystemExit("level 必须是 0..%d" % (len(LEVELS) - 1))
    x = LEVELS[i]
    st = load_state()
    # 1) x 变化前: 只要夹爪不是张开状态, 就先在当前高度张开(单独完成)
    if st["x"] is not None and st["y"] != JAW_OPEN:
        send(st["x"], JAW_OPEN, dry_run=dry_run)
    # 2) 升降到目标高度(保持张开), 用更长的 MOVE_SETTLE 等 x 真正到位。
    #    已经在该高度且张开(如刚归零)则不重复移动。
    if st["x"] != x or st["y"] != JAW_OPEN:
        send(x, JAW_OPEN, dry_run=dry_run, settle=MOVE_SETTLE)
    # 3) x 到位后才闭合
    send(x, JAW_CLOSE, dry_run=dry_run)
    print("[clamp] 已夹住第 %d 档 (x=%s)" % (i, x))


def goto_level(i, dry_run=False):
    """只移动到第 i 档(保持张开, 不夹)。"""
    if not 0 <= i < len(LEVELS):
        raise SystemExit("level 必须是 0..%d" % (len(LEVELS) - 1))
    open_jaw(dry_run=dry_run)              # 先确保松开再移动
    send(LEVELS[i], JAW_OPEN, dry_run=dry_run, settle=MOVE_SETTLE)


def home_to_zero(dry_run=False):
    """巡检/拆卸前先把 x 一路降到 0, 并确保夹爪张开。
    发两遍 x=0(张开): 即使一开始夹爪是闭的、第一条没压到底, 第二条(夹爪已开)也能让 x 真正到 0。"""
    st = load_state()
    if st["x"] is not None and st["y"] != JAW_OPEN:
        send(st["x"], JAW_OPEN, dry_run=dry_run)           # 先在当前高度张开
    print("[clamp] 归零: x -> 0.0 (张开, 发两遍确保到位)")
    send(LEVELS[0], JAW_OPEN, dry_run=dry_run, settle=MOVE_SETTLE)
    send(LEVELS[0], JAW_OPEN, dry_run=dry_run, settle=MOVE_SETTLE)


def sweep(dwell, loop=False, dry_run=False):
    """先归零(x到0), 再从第0档开始逐档: 升x前先张开 -> 到位后闭合 -> 停留 dwell 秒。
    走完全部档位; loop=True 则到底后回到第0档继续循环。"""
    home_to_zero(dry_run=dry_run)        # 一开始确保 x 真正到 0
    rounds = 0
    while True:
        rounds += 1
        for i in range(len(LEVELS)):
            grip_level(i, dry_run=dry_run)            # 自动: 当前高度张开->移到第i档->闭合
            print("[clamp] 第 %d 档 (x=%s) 已闭合, 停留 %.1fs" % (i, LEVELS[i], dwell))
            if not dry_run:
                time.sleep(dwell)
        if not loop:
            break
        print("[clamp] —— 第 %d 轮结束, 回到第0档继续 ——" % rounds)


def main():
    ap = argparse.ArgumentParser(description="fawkes-robotino 夹持器控制")
    sub = ap.add_subparsers(dest="action", required=True)

    p = sub.add_parser("grip", help="换到某一档并夹住(自动先松开再移动)")
    p.add_argument("--level", type=int, required=True, help="0..%d" % (len(LEVELS) - 1))

    p = sub.add_parser("goto", help="只移动到某一档, 保持张开不夹")
    p.add_argument("--level", type=int, required=True)

    sub.add_parser("open", help="在当前高度张开/松开")
    sub.add_parser("close", help="在当前高度闭合/夹住(不移动)")

    p = sub.add_parser("sweep", help="从0.0开始逐档: 张开->升到下一档->闭合->停留, 走完全部")
    p.add_argument("--dwell", type=float, default=2.0, help="每档闭合后停留秒数(默认2)")
    p.add_argument("--loop", action="store_true", help="走到最低档后回到0.0继续循环")

    p = sub.add_parser("raw", help="直接发任意 x/y/z")
    p.add_argument("--x", type=float, required=True)
    p.add_argument("--y", type=float, required=True)
    p.add_argument("--z", type=float, default=Z_FIXED)

    for sp in sub.choices.values():
        sp.add_argument("--dry-run", action="store_true", help="只打印不真正发送")
        sp.add_argument("--settle", type=float, default=None,
                        help="覆盖每条命令后等待秒数(现场调换档速度用)")

    a = ap.parse_args()
    apply_config()                       # 先读 clamp_config.json 覆盖默认值
    global SETTLE
    if getattr(a, "settle", None) is not None:
        SETTLE = a.settle                # 命令行 --settle 优先级最高
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
    elif a.action == "sweep":
        sweep(a.dwell, loop=a.loop, dry_run=dry)
    elif a.action == "raw":
        send(a.x, a.y, a.z, dry_run=dry)


if __name__ == "__main__":
    main()
