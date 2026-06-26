#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""Host 端: 从 o3p grabber 写的【高清彩色 color_hi.png】抓一帧, 检测棋盘并存图。
替代 RealSense 版 calib_grab.py(O3P 无 pyrealsense2, 且高清图本就是文件)。

存两张:
  <out>        干净高清图  -> 给手眼标定 solvePnP 用(不画角点!)
  <out>_viz    画了角点的  -> 给人眼检查检测对不对

用法: calib_grab_o3p.py <输出路径> <cols> <rows> [--o3p-dir DIR]
退出码: 0=检测到棋盘并存图, 1=没检测到(已存图供查看), 2=没拿到帧。"""
import argparse
import os
import sys
import time

import cv2


def wait_fresh(o3p_dir, timeout=3.0):
    """等 frame.txt 变一次, 避免相机卡死时抓到旧图(参考之前的'旧照片'坑)。"""
    fpath = os.path.join(o3p_dir, "frame.txt")

    def read():
        try:
            with open(fpath) as f:
                return f.read().strip()
        except Exception:
            return None

    start = time.time()
    first = read()
    while time.time() - start < timeout:
        cur = read()
        if cur and cur != first:
            return True
        time.sleep(0.05)
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("cols", type=int)
    ap.add_argument("rows", type=int)
    ap.add_argument("--o3p-dir",
                    default="/home/i6user/Desktop/robot_lego/src/panda_pick/src/o3p")
    a = ap.parse_args()

    hi_path = os.path.join(a.o3p_dir, "color_hi.png")
    fresh = wait_fresh(a.o3p_dir)
    img = cv2.imread(hi_path, cv2.IMREAD_COLOR)
    if img is None:
        print("NO_FRAME: 读不到 %s (grabber 在跑吗?)" % hi_path)
        sys.exit(2)
    if not fresh:
        print("[警告] 3s 内 frame.txt 没变, 可能相机卡死/grabber 没刷新; 仍用当前帧。")

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, cor = cv2.findChessboardCorners(gray, (a.cols, a.rows), flags)

    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    cv2.imwrite(a.out, img)  # 干净图给标定
    if ok:
        viz = img.copy()
        cv2.drawChessboardCorners(viz, (a.cols, a.rows), cor, ok)
        root, ext = os.path.splitext(a.out)
        cv2.imwrite(root + "_viz" + ext, viz)

    print("OK" if ok else "NO_BOARD")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
