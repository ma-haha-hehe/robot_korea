#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""容器内一次性跑视觉(像 pipeline 触发视觉那一步): 检测(高清)->投影->FoundationPose,
输出 T_cam_obj + 把所有 imshow 画面存成文件供查看。不动机械臂。

用法(容器内):
  /opt/conda/envs/my/bin/python /vision_code/run_vision_once.py "green 2x4 brick."
"""
import os
import sys
import time

import cv2
import numpy as np

OUT = "/shared_data/o3p"


def _save(name, img):
    try:
        cv2.imwrite(os.path.join(OUT, "show_%s.png" % name.replace(" ", "_")), img)
    except Exception:
        pass


# 无显示环境: 把 imshow 重定向到存文件
cv2.imshow = lambda n, i: _save(n, i)
cv2.waitKey = lambda *a, **k: -1
cv2.namedWindow = lambda *a, **k: None
cv2.resizeWindow = lambda *a, **k: None

sys.path.insert(0, "/vision_code")
import vision_node_test1_5_bridge as B

target = sys.argv[1] if len(sys.argv) > 1 else "green 2x4 brick."
print(f"[run_vision_once] target={target!r}")

bridge = B.RobotVisionBridge()
t0 = time.time()
pose = bridge.estimate_once(target)
dt = time.time() - t0
print(f"[run_vision_once] estimate_once 用时 {dt:.1f}s")

if pose is None:
    print("[run_vision_once] RESULT: 没拿到位姿(检测或位姿失败)")
    sys.exit(0)

bridge.write_result(target, pose)
P = np.array(pose)
np.set_printoptions(suppress=True, precision=4)
print("[run_vision_once] T_cam_obj =\n", P)
print("[run_vision_once] 物体在相机系 xyz[m] =", P[:3, 3].round(4).tolist(),
      " 距离 z=%.3f m" % P[2, 3])
print("[run_vision_once] 已写 /shared_data/vision_output.yaml; 可视化在", OUT, "show_*.png / vision_pose_debug.jpg")
