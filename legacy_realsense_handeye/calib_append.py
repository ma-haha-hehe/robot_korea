#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""宿主机: 从 print_pose 日志解析 T_base_tool(4x4), 配对图像追加进 samples.yaml。
samples.yaml 头部焊入【正确的实时内参(605)】, 保证标定与 FoundationPose 运行时一致。"""
import argparse
import os
import re
import yaml

# 正确的实时 RealSense 彩色内参(与运行时 FoundationPose 一致)
CAM_MATRIX = [[605.2904663, 0.0, 318.4192505],
              [0.0, 604.7702637, 249.9142151],
              [0.0, 0.0, 1.0]]
DIST = [0.0, 0.0, 0.0, 0.0, 0.0]

PAT = re.compile(r'-\s*\[\s*([-\d.eE+]+),\s*([-\d.eE+]+),\s*([-\d.eE+]+),\s*([-\d.eE+]+)\s*\]')


def parse_T(pp_log):
    rows = []
    for line in open(pp_log, errors="ignore"):
        m = PAT.search(line)
        if m:
            rows.append([float(x) for x in m.groups()])
    if len(rows) < 4:
        raise SystemExit(f"[X] 没从 print_pose 解析到 4x4 矩阵(得到{len(rows)}行)。看 {pp_log}")
    return rows[-4:]   # 取最近一次打印的矩阵


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--id", required=True)
    ap.add_argument("--pp", required=True)
    ap.add_argument("--handeye", required=True)
    ap.add_argument("--board-size", type=int, nargs=2, default=[4, 6])
    ap.add_argument("--square-size", type=float, default=0.030)
    a = ap.parse_args()

    T = parse_T(a.pp)
    sf = os.path.join(a.handeye, "samples.yaml")
    if os.path.exists(sf):
        data = yaml.safe_load(open(sf))
    else:
        data = {"pattern": "chessboard",
                "board_size": list(a.board_size),
                "square_size": a.square_size,
                "camera_matrix": CAM_MATRIX,
                "dist_coeffs": DIST,
                "samples": []}
    data["samples"].append({
        "image": f"/vision_code/handeye/img_{a.id}.jpg",
        "T_base_tool": T,
    })
    yaml.safe_dump(data, open(sf, "w"), sort_keys=False)
    print(f"  T_base_tool 平移 = [{T[0][3]:.4f}, {T[1][3]:.4f}, {T[2][3]:.4f}]"
          f"  (累计样本数 = {len(data['samples'])})")


if __name__ == "__main__":
    main()
