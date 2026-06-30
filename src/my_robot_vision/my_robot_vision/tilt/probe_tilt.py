#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""斜抓诊断 (host-only, 不动机器人/不编译).

算 T_base_obj = observe_T_base_tool · T_tool_camera · T_cam_obj, 再报告:
  - 物体 z 轴在 base_link 的方向
  - 与竖直(base -Z 向下接近)的夹角 = "倾角"
  - 物体中心 base 坐标
用来判断是否真需要斜抓、以及斜抓朝向是否合理。属于 tilt 实验分支, 与现版本完全分离。

用法:
  python3 probe_tilt.py [vision_cache.yaml]
默认读 FoundationPose/root/FoundationPose/vision_cache/00_white_2x4_brick.yaml
"""
import sys
import math
import numpy as np
import yaml

REPO = "/home/i6user/Desktop/robot_lego"
BRIDGE_CFG = f"{REPO}/src/my_robot_vision/config/vision_execution_bridge.yaml"
HANDEYE = f"{REPO}/src/my_robot_vision/config/camera_to_tool.yaml"
DEFAULT_VIS = f"{REPO}/FoundationPose/root/FoundationPose/vision_cache/00_white_2x4_brick.yaml"


def load(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def main():
    vis_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_VIS
    cfg = load(BRIDGE_CFG)
    he = load(HANDEYE)
    vis = load(vis_path)

    T_base_tool = np.asarray(cfg["handeye"]["observe_T_base_tool"], dtype=float)
    T_tool_cam = np.asarray(he["T_tool_camera"], dtype=float)
    T_cam_obj = np.asarray(vis["T_cam_obj"], dtype=float)

    T_base_obj = T_base_tool @ T_tool_cam @ T_cam_obj
    R = T_base_obj[:3, :3]
    center = T_base_obj[:3, 3]

    x_axis = R[:, 0]
    y_axis = R[:, 1]
    z_axis = R[:, 2]

    # 倾角: 物体 z 轴与 base +Z 的夹角(0=完全水平放置/z朝上, 90=立着)
    cos_up = float(np.clip(np.dot(z_axis, [0, 0, 1]), -1, 1))
    tilt_from_up = math.degrees(math.acos(abs(cos_up)))  # 不分上下, 取与竖直的偏离

    print(f"vision cache : {vis_path}")
    print(f"target       : {vis.get('target')}")
    print(f"中心 base_link: [{center[0]:.4f}, {center[1]:.4f}, {center[2]:.4f}]")
    print(f"X轴(红/base)  : [{x_axis[0]:.3f}, {x_axis[1]:.3f}, {x_axis[2]:.3f}]")
    print(f"Y轴(绿/base)  : [{y_axis[0]:.3f}, {y_axis[1]:.3f}, {y_axis[2]:.3f}]")
    print(f"Z轴(蓝/base)  : [{z_axis[0]:.3f}, {z_axis[1]:.3f}, {z_axis[2]:.3f}]")
    print(f"Z轴朝向(cos与+Z)= {cos_up:+.3f}  ({'朝上' if cos_up>0 else '朝下'})")
    print(f"==> 倾角(偏离竖直): {tilt_from_up:.1f}°  "
          f"{'≈水平放置, 斜抓≈竖直抓' if tilt_from_up < 8 else '明显倾斜, 斜抓有意义'}")


if __name__ == "__main__":
    main()
