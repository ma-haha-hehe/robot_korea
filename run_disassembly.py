#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""总控【拆卸】脚本: 从上往下拆塔。

每拆一块的循环:
    1) 夹持器(fawkes-robotino) 夹住"目标块下面那块"   -> clamp_control.grip_level
    2) 机械臂把目标块竖直拔起、放到一边               -> disassembly_arm_pull.sh
    3) 塔变矮, 夹持器档位下移一格, 重复
最后夹持器松开。

*** 只负责拆卸。完全独立于装配流程(run_auto_pick_pipeline.py / run_pick.sh)。***
*** 不要在这里 import 或调用任何装配代码, 也不要改装配配置。***

配置见同目录 disassembly_config.yaml (与装配 yaml 互不相干)。
"""
import argparse
import os
import subprocess
import sys
import time

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import clamp_control as clamp   # 同目录的夹持器控制(也只管拆卸)

CONFIG = os.path.join(HERE, "disassembly_config.yaml")
ARM_SCRIPT = os.path.join(HERE, "disassembly_arm_pull.sh")


def load_config(path):
    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    if not cfg or "steps" not in cfg:
        raise SystemExit("配置缺少 steps: %s" % path)
    return cfg


def arm_pull(pose, dry_run=False):
    """调用独立的机械臂拔取脚本(点到点)。绝不走装配 pipeline。"""
    print("[arm] 拔取动作 pose=%s" % pose)
    if dry_run:
        print("[arm DRY] %s %s" % (ARM_SCRIPT, pose))
        return
    if not os.path.exists(ARM_SCRIPT):
        raise SystemExit("找不到机械臂拔取脚本: %s" % ARM_SCRIPT)
    subprocess.run(["bash", ARM_SCRIPT, pose], check=True)


def run(cfg, args):
    steps = cfg["steps"]
    settle = float(cfg.get("settle_after_clamp", 1.0))
    lo = args.from_step if args.from_step is not None else 0
    hi = args.to_step + 1 if args.to_step is not None else len(steps)

    for idx in range(lo, hi):
        s = steps[idx]
        name = s.get("name", "step%d" % idx)
        print("\n========== 拆卸步骤 %d/%d: %s ==========" % (idx, len(steps) - 1, name))

        # 1) 夹持器夹住目标块下面那块
        clamp.grip_level(int(s["clamp_level"]), dry_run=args.dry_run)

        if args.clamp_only:
            print("[i] --clamp-only: 跳过机械臂, 仅演示夹持器编排")
            continue

        # 2) 夹紧稳定后再让机械臂动(关键时序: 拔之前夹持器必须已夹紧)
        if not args.dry_run:
            time.sleep(settle)
        arm_pull(s["arm_pose"], dry_run=args.dry_run)

    # 全部拆完, 夹持器松开
    print("\n========== 拆卸结束, 夹持器松开 ==========")
    clamp.open_jaw(dry_run=args.dry_run)


def main():
    ap = argparse.ArgumentParser(description="总控拆卸(从上往下), 夹持器+机械臂协调")
    ap.add_argument("--config", default=CONFIG)
    ap.add_argument("--dry-run", action="store_true", help="只打印, 夹持器和机械臂都不真发")
    ap.add_argument("--clamp-only", action="store_true", help="只跑夹持器编排, 不动机械臂")
    ap.add_argument("--from-step", type=int, default=None, help="从第几步开始(含)")
    ap.add_argument("--to-step", type=int, default=None, help="到第几步结束(含)")
    args = ap.parse_args()
    run(load_config(args.config), args)


if __name__ == "__main__":
    main()
