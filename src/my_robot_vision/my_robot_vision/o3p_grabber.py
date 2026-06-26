#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""Host 端相机 grabber: 订阅 ifm O3P 的 ROS 话题, 写帧+标定参数到共享目录,
供容器内视觉桥读取(替代 RealSense pyrealsense2)。

高清检测方案需要的全部素材:
  color_hi.png        高清彩色 1344x1008 (bgr8, 鱼眼)        -> 给 GroundingDINO/SAM 检测
  color.png           SDK 已对齐彩色 240x180 (bgr8)          -> 给 FoundationPose 当 rgb
  depth.npy           深度 240x180 (uint16, 毫米)            -> FoundationPose depth + 投影
  camera_params.json  depth_K, color_K, color_D(鱼眼4), T_depth_to_color(4x4)
  frame.txt           帧计数

o3p ROS 节点内部已 alignColorToDepth, 故 color.png 与 depth.npy 逐像素对齐。
高清 mask -> 深度帧 的投影由桥用 camera_params.json 完成(并可用 color.png 校验)。

用法:
  /usr/bin/python3 o3p_grabber.py --out /shared.../o3p --once     # 抓一组检查
  /usr/bin/python3 o3p_grabber.py --out /shared.../o3p --rate 15  # 持续喂帧
"""
import argparse
import json
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

try:
    import cv2
except Exception:
    cv2 = None

import tf2_ros

ALIGNED_COLOR_TOPIC = "/camera/aligned_depth_to_color/image_raw"  # 彩色对齐到深度帧 240x180
HI_COLOR_TOPIC = "/camera/frame_rgb"                              # 高清彩色 1344x1008
DEPTH_TOPIC = "/camera/depth/image_raw"                           # 深度 240x180 毫米
DEPTH_INFO_TOPIC = "/camera/depth/camera_info"                    # 深度内参
COLOR_INFO_TOPIC = "/camera/camera_info"                          # 彩色内参(鱼眼系数在 d[:4])
DEPTH_FRAME = "camera_depth_optical_frame"
COLOR_FRAME = "camera_color_optical_frame"

ENC = {"bgr8": ("uint8", 3), "rgb8": ("uint8", 3), "mono8": ("uint8", 1),
       "mono16": ("uint16", 1), "16uc1": ("uint16", 1), "32fc1": ("float32", 1)}


def img_to_np(msg):
    dt, ch = ENC.get(msg.encoding.lower(), ("uint8", 1))
    a = np.frombuffer(msg.data, dtype=np.dtype(dt))
    return a.reshape(msg.height, msg.width, ch) if ch > 1 else a.reshape(msg.height, msg.width)


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=float)


def atomic_save(path, save_fn):
    tmp = path + ".tmp"
    save_fn(tmp)
    os.replace(tmp, path)


class Grabber(Node):
    def __init__(self, out, rate, once):
        super().__init__("o3p_grabber")
        self.out = out
        self.once = once
        os.makedirs(out, exist_ok=True)
        self.aligned = None
        self.hi = None
        self.depth = None
        self.depth_K = None
        self.color_K = None
        self.color_D = None
        self.params_written = False
        self.frame_id = 0
        self.done = False
        self.create_subscription(Image, ALIGNED_COLOR_TOPIC, self.on_aligned, 5)
        self.create_subscription(Image, HI_COLOR_TOPIC, self.on_hi, 5)
        self.create_subscription(Image, DEPTH_TOPIC, self.on_depth, 5)
        self.create_subscription(CameraInfo, DEPTH_INFO_TOPIC, self.on_depth_info, 5)
        self.create_subscription(CameraInfo, COLOR_INFO_TOPIC, self.on_color_info, 5)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def on_aligned(self, m):
        self.aligned = img_to_np(m)

    def on_hi(self, m):
        self.hi = img_to_np(m)

    def on_depth(self, m):
        self.depth = img_to_np(m)

    def on_depth_info(self, m):
        self.depth_K = np.array(m.k, dtype=float).reshape(3, 3)

    def on_color_info(self, m):
        self.color_K = np.array(m.k, dtype=float).reshape(3, 3)
        self.color_D = list(m.d)[:4]   # F-Theta 鱼眼前4个系数

    def try_T(self):
        try:
            t = self.tf_buffer.lookup_transform(COLOR_FRAME, DEPTH_FRAME, rclpy.time.Time())
        except Exception:
            return None
        tr, q = t.transform.translation, t.transform.rotation
        T = np.eye(4)
        T[:3, :3] = quat_to_R(q.x, q.y, q.z, q.w)
        T[:3, 3] = [tr.x, tr.y, tr.z]
        return T

    def write_params(self):
        if self.params_written or self.depth_K is None or self.color_K is None:
            return False
        T = self.try_T()
        if T is None:
            return False
        atomic_save(os.path.join(self.out, "camera_params.json"),
                    lambda p: json.dump({"depth_K": self.depth_K.tolist(),
                                         "color_K": self.color_K.tolist(),
                                         "color_D": self.color_D,
                                         "T_depth_to_color": T.tolist()},
                                        open(p, "w"), indent=2))
        self.params_written = True
        print("[grabber] camera_params.json: depth fx=%.1f | color fx=%.1f cx=%.1f D=%s | t_d2c=%s" %
              (self.depth_K[0, 0], self.color_K[0, 0], self.color_K[0, 2],
               np.round(self.color_D, 4).tolist(), np.round(T[:3, 3], 4).tolist()))
        return True

    def tick(self):
        self.write_params()
        if self.aligned is None or self.depth is None or self.hi is None:
            return
        atomic_save(os.path.join(self.out, "color.png"),
                    lambda p: open(p, "wb").write(cv2.imencode(".png", self.aligned)[1].tobytes()))
        atomic_save(os.path.join(self.out, "color_hi.png"),
                    lambda p: open(p, "wb").write(cv2.imencode(".png", self.hi)[1].tobytes()))
        atomic_save(os.path.join(self.out, "depth.npy"),
                    lambda p: np.save(open(p, "wb"), self.depth))
        self.frame_id += 1
        atomic_save(os.path.join(self.out, "frame.txt"),
                    lambda p: open(p, "w").write(str(self.frame_id)))
        if self.once and self.params_written:
            print("[grabber] aligned %s  hi %s  depth %s mm[%d-%d] -> %s" %
                  (self.aligned.shape, self.hi.shape, self.depth.shape,
                   int(self.depth.min()), int(self.depth.max()), self.out))
            self.done = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/tmp/o3p_cap")
    ap.add_argument("--rate", type=float, default=15.0)
    ap.add_argument("--once", action="store_true")
    ap.add_argument("--timeout", type=float, default=20.0)
    a = ap.parse_args()
    rclpy.init()
    node = Grabber(a.out, a.rate, a.once)
    period = 1.0 / max(1.0, a.rate)
    deadline = time.time() + a.timeout
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.tick()
            if node.done:
                break
            if a.once and time.time() > deadline:
                miss = [n for n, v in [("aligned", node.aligned), ("hi", node.hi),
                                       ("depth", node.depth), ("params", node.params_written)]
                        if not (v is True or v is not None)]
                print("[grabber] 超时, 还缺:", miss)
                break
            time.sleep(period)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
