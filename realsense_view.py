#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RealSense 实时预览(读 grabber 写的文件, 不占相机、不冲突)。
左=彩色 右=深度彩色化, 放大显示。按 q 退出。"""
import time
import cv2
import numpy as np

O3P = "/home/i6user/Desktop/robot_lego/src/panda_pick/src/o3p"


def main():
    win = "RealSense Live (color | depth)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 1600, 640)
    while True:
        color = cv2.imread(O3P + "/color.png")
        try:
            depth = np.load(O3P + "/depth.npy")
        except Exception:
            depth = None
        if color is None:
            time.sleep(0.1)
            continue
        h, w = color.shape[:2]
        if depth is not None:
            dn = np.clip(depth.astype(np.float32), 0, 1500)
            dn = (dn / 1500.0 * 255).astype(np.uint8)
            dcol = cv2.applyColorMap(dn, cv2.COLORMAP_JET)
            dcol[depth == 0] = 0
            dcol = cv2.resize(dcol, (w, h))
            vis = np.hstack([color, dcol])
        else:
            vis = color
        try:
            f = open(O3P + "/frame.txt").read().strip()
        except Exception:
            f = "?"
        cv2.putText(vis, f"frame {f}  {w}x{h}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow(win, cv2.resize(vis, (1600, 640)))
        if (cv2.waitKey(50) & 0xFF) == ord('q'):
            break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
