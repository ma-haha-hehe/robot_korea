#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""Host: 解析 print_pose 的 T_base_tool(4x4), 配对高清图追加进 samples.yaml。

O3P 高清彩色(鱼眼)标定版: 内参/畸变/深度->彩色外参全部从 camera_params.json 读,
保证标定用的鱼眼模型与运行时 o3p_align 完全一致。
"""
import argparse
import json
import os
import re

import yaml

PAT = re.compile(
    r'-\s*\[\s*([-\d.eE+]+),\s*([-\d.eE+]+),\s*([-\d.eE+]+),\s*([-\d.eE+]+)\s*\]')


def parse_T(pp_log):
    rows = []
    for line in open(pp_log, errors="ignore"):
        m = PAT.search(line)
        if m:
            rows.append([float(x) for x in m.groups()])
    if len(rows) < 4:
        raise SystemExit(
            f"[X] 没从 print_pose 解析到 4x4 矩阵(得到{len(rows)}行)。看 {pp_log}")
    return rows[-4:]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--id", required=True)
    ap.add_argument("--pp", required=True)
    ap.add_argument("--handeye", required=True)
    ap.add_argument("--image", required=True, help="干净高清图的绝对路径")
    ap.add_argument("--o3p-dir",
                    default="/home/i6user/Desktop/robot_lego/src/panda_pick/src/o3p")
    ap.add_argument("--board-size", type=int, nargs=2, default=[4, 6])
    ap.add_argument("--square-size", type=float, default=0.030)
    a = ap.parse_args()

    T = parse_T(a.pp)
    params = json.load(open(os.path.join(a.o3p_dir, "camera_params.json")))

    sf = os.path.join(a.handeye, "samples.yaml")
    if os.path.exists(sf):
        data = yaml.safe_load(open(sf))
    else:
        data = {
            "pattern": "chessboard_fisheye",
            "frame": "color_hi",
            "board_size": list(a.board_size),
            "square_size": a.square_size,
            "camera_matrix": params["color_K"],
            "dist_coeffs_fisheye": params["color_D"],
            "T_depth_to_color": params["T_depth_to_color"],
            "samples": [],
        }
    data["samples"].append({"image": a.image, "T_base_tool": T})
    yaml.safe_dump(data, open(sf, "w"), sort_keys=False)
    print(f"  T_base_tool 平移 = [{T[0][3]:.4f}, {T[1][3]:.4f}, {T[2][3]:.4f}]"
          f"  (累计样本数 = {len(data['samples'])})")


if __name__ == "__main__":
    main()
