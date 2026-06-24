#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""容器内: 抓一帧 RealSense 彩色(关红外), 检测棋盘并存图。
用法: calib_grab.py <输出路径> <cols> <rows>
退出码: 0=检测到棋盘并存图, 1=没检测到(已存图供查看), 2=没拿到帧。"""
import sys
import numpy as np
import cv2
import pyrealsense2 as rs

out = sys.argv[1]
cols, rows = int(sys.argv[2]), int(sys.argv[3])

p = rs.pipeline()
c = rs.config()
c.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pr = p.start(c)
for s in pr.get_device().query_sensors():
    if s.supports(rs.option.emitter_enabled):
        s.set_option(rs.option.emitter_enabled, 0)
f = None
for _ in range(30):
    fr = p.wait_for_frames().get_color_frame()
    if fr:
        f = np.asanyarray(fr.get_data())
p.stop()
if f is None:
    print("NO_FRAME")
    sys.exit(2)

g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
ok, cor = cv2.findChessboardCorners(
    g, (cols, rows), cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
if ok:
    cv2.drawChessboardCorners(f, (cols, rows), cor, ok)
cv2.imwrite(out, f)
print("OK" if ok else "NO_BOARD")
sys.exit(0 if ok else 1)
