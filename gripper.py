#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""OnRobot RG2/RG6 独立命令行控制 (UR5, 单线缆接工具端口, 无 Compute Box)。

原理: 复用项目里已经验证过的 URScript 协议 (cpp_pick_node_ur5.cpp 的
      appendOnRobotRgHelpers / onrobot_rg_powerup / onrobot_rg_bit)。
      通过 tool 端口的数字 I/O(DO8/DO9, DI8/DI9) + tool 24V 电源做位操作,
      给 RG2 发"张开度(mm)+力(N)"命令。脚本把 URScript 发到 UR 驱动的
      /urscript_interface/script_command 话题, 驱动再转给机器人执行。

前提:
  - UR 机器人处于 Remote Control(远程) 且已上电、无保护停止;
  - ur_robot_driver 正在跑 (话题 /urscript_interface/script_command 有订阅);
  - RG2 单线缆插在 UR **工具端口**(不是 Compute Box)。

用法:
  source /opt/ros/humble/setup.bash && source install/setup.bash
  python3 gripper.py activate            # 仅上电激活(第一次用/重新上电后跑一次)
  python3 gripper.py open                # 张到 open-width (默认 100mm)
  python3 gripper.py close               # 合到 close-width (默认 0mm) 夹紧
  python3 gripper.py width 55            # 张到指定 mm
  python3 gripper.py width 30 --force 30 # 指定 mm + 力(N)
  常用可选: --force N  --open-width mm  --close-width mm  --wait s  --model rg2|rg6
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

TOPIC = "/urscript_interface/script_command"

MODELS = {
    "rg2": dict(max_width=110, max_force=40, force_divide=2, force_factor=111),
    "rg6": dict(max_width=160, max_force=120, force_divide=5, force_factor=161),
}


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def command_value(width_mm, force_n, m):
    """与 cpp_pick_node_ur5.cpp onRobotRgCommandValue 完全一致的编码。"""
    width_mm = clamp(int(round(width_mm)), 0, m["max_width"])
    force_n = clamp(int(round(force_n)), 0, m["max_force"])
    value = width_mm * 4
    value += (force_n // m["force_divide"]) * 4 * m["force_factor"]
    return value


HELPERS = """  def onrobot_rg_wait(seconds):
    local sync_cnt = 0
    local sync_target = floor(seconds / 0.008)
    while sync_cnt < sync_target:
      sync()
      sync_cnt = sync_cnt + 1
    end
  end
  def onrobot_rg_bit(input):
    local i = 0
    local output = 0
    while i < 17:
      set_digital_out(8, True)
      if input >= 65536:
        input = input - 65536
        set_digital_out(9, False)
      else:
        set_digital_out(9, True)
      end
      if get_digital_in(8):
        output = 1
      end
      sync()
      set_digital_out(8, False)
      sync()
      input = input * 2
      i = i + 1
    end
    return output
  end
  def onrobot_rg_powerup():
    textmsg("OnRobot RG powerup")
    set_tool_voltage(0)
    onrobot_rg_wait(2.0)
    set_digital_out(8, False)
    set_digital_out(9, False)
    local pulse = 0
    while pulse < 4:
      set_tool_voltage(24)
      onrobot_rg_wait(0.032)
      set_tool_voltage(0)
      onrobot_rg_wait(0.064)
      pulse = pulse + 1
    end
    set_tool_voltage(24)
    local timeout = 0
    while get_digital_in(8) == False:
      timeout = timeout + 1
      sync()
      if timeout > 1875:
        textmsg("OnRobot RG not responding on DI8")
        return False
      end
    end
    timeout = 0
    while get_digital_in(9):
      timeout = timeout + 1
      sync()
      if timeout > 1875:
        textmsg("OnRobot RG not ready on DI9")
        return False
      end
    end
    return True
  end
  def onrobot_rg_wait_motion(max_seconds):
    local timeout = 0
    while get_digital_in(9) == True:
      timeout = timeout + 1
      sync()
      if timeout > 20:
        break
      end
    end
    timeout = 0
    local timeout_limit = floor(max_seconds / 0.008)
    while get_digital_in(9) == False:
      timeout = timeout + 1
      sync()
      if timeout > timeout_limit:
        break
      end
    end
  end
"""


def build_script(width_mm, force_n, wait_s, m, move=True):
    lines = ["def onrobot_rg_cmd():", HELPERS.rstrip("\n"),
             "  onrobot_rg_ready = onrobot_rg_powerup()"]
    if move:
        cmd = command_value(width_mm, force_n, m)
        lines += [
            "  if (onrobot_rg_ready):",
            '    textmsg("OnRobot RG move width=%d force=%d")' % (
                clamp(int(round(width_mm)), 0, m["max_width"]),
                clamp(int(round(force_n)), 0, m["max_force"])),
            "    onrobot_rg_bit(%d)" % cmd,
            "    onrobot_rg_wait_motion(%s)" % ("%.3f" % max(0.0, wait_s)),
            "  end",
        ]
    lines.append("end")
    return "\n".join(lines) + "\n"


def publish(script, timeout=8.0):
    rclpy.init()
    node = Node("onrobot_gripper_cmd")
    pub = node.create_publisher(String, TOPIC, 1)
    # 等驱动的 urscript_interface 订阅上来
    t0 = time.time()
    while pub.get_subscription_count() == 0 and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        node.get_logger().error(
            "%s 无订阅者: UR 驱动没跑? 先启动 ur_robot_driver。" % TOPIC)
        node.destroy_node()
        rclpy.shutdown()
        return False
    msg = String()
    msg.data = script
    pub.publish(msg)
    # 稍等确保发出去
    t0 = time.time()
    while time.time() - t0 < 1.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.get_logger().info("已发送 URScript 到机器人。")
    node.destroy_node()
    rclpy.shutdown()
    return True


def main():
    ap = argparse.ArgumentParser(description="OnRobot RG2/RG6 独立控制 (UR tool 端口)")
    sub = ap.add_subparsers(dest="action", required=True)
    sub.add_parser("activate", help="仅上电激活(不动夹爪)")
    sub.add_parser("open", help="张开到 --open-width")
    sub.add_parser("close", help="合到 --close-width 夹紧")
    pw = sub.add_parser("width", help="张到指定宽度(mm)")
    pw.add_argument("mm", type=float)
    for sp in sub.choices.values():
        sp.add_argument("--force", type=float, default=20, help="夹持力 N (rg2:0-40)")
        sp.add_argument("--open-width", type=float, default=100, help="open 命令宽度 mm")
        sp.add_argument("--close-width", type=float, default=0, help="close 命令宽度 mm")
        sp.add_argument("--wait", type=float, default=2.0, help="发送后等运动完成秒数")
        sp.add_argument("--model", default="rg2", choices=list(MODELS))
        sp.add_argument("--dry-run", action="store_true", help="只打印 URScript 不发送")
    a = ap.parse_args()
    m = MODELS[a.model]

    if a.action == "activate":
        script = build_script(0, 0, a.wait, m, move=False)
    elif a.action == "open":
        script = build_script(a.open_width, a.force, a.wait, m)
    elif a.action == "close":
        script = build_script(a.close_width, a.force, a.wait, m)
    elif a.action == "width":
        script = build_script(a.mm, a.force, a.wait, m)
    else:
        ap.error("未知命令")

    print("---- URScript ----")
    print(script)
    print("------------------")
    if a.dry_run:
        return
    ok = publish(script)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
