#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RealSense(D435i) 帧 grabber —— 写成和 o3p_grabber 完全相同的文件格式,
供容器内视觉桥 vision_node_test1_5_bridge.py 直接读取(桥无需改动)。

RealSense 的彩色/深度经 rs.align 已【同分辨率(640x480)对齐】, 所以:
  color_hi.png / color.png   都写这张对齐彩色(检测 + FoundationPose 共用)
  depth.npy                  640x480 uint16 毫米(z16, depth_scale=0.001)
  camera_params.json         depth_K = color_K = 640x480 内参; color_D=[0,0,0,0];
                             T_depth_to_color = 单位阵
  -> 桥里的鱼眼投影因 D=0、T=I、depth_K=color_K 退化成恒等映射, 桥无需改。
直接用 pyrealsense2, 不走 ROS/DDS。

健壮性: 流 15fps(降 USB 带宽); 断线(Frame didn't arrive / 设备节点消失)自动重连, 永不退出。
用法: python3 realsense_grabber.py --out src/panda_pick/src/o3p --rate 15
"""
import argparse
import json
import os
import time

import cv2
import numpy as np
import pyrealsense2 as rs


def atomic_save(path, writer):
    tmp = path + ".tmp"
    writer(tmp)
    os.replace(tmp, path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="src/panda_pick/src/o3p")
    ap.add_argument("--rate", type=float, default=15.0)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=15)   # 15 比 30 省一半 USB 带宽, 更不易掉线
    ap.add_argument("--once", action="store_true")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)
    cfg.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)

    period = 1.0 / max(0.1, args.rate)
    frame_id = 0

    while True:  # 外层: 断线重连, 永不退出(除非 --once 或被 kill)
        pipe = rs.pipeline()
        try:
            profile = pipe.start(cfg)
        except Exception as e:
            print("[rs-grabber] 打开相机失败(%s), 3s 后重试...(检查 USB3.0 直连)" % e, flush=True)
            time.sleep(3.0)
            continue
        align = rs.align(rs.stream.color)
        try:
            intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            K = [[intr.fx, 0.0, intr.ppx], [0.0, intr.fy, intr.ppy], [0.0, 0.0, 1.0]]
            atomic_save(os.path.join(args.out, "camera_params.json"),
                        lambda p: json.dump({"depth_K": K, "color_K": K,
                                             "color_D": [0.0, 0.0, 0.0, 0.0],
                                             "T_depth_to_color": np.eye(4).tolist()},
                                            open(p, "w"), indent=2))
            print("[rs-grabber] 已连接: fx=%.1f cx=%.1f (%dx%d @%dfps)" %
                  (intr.fx, intr.ppx, args.width, args.height, args.fps), flush=True)
            while True:
                try:
                    frames = align.process(pipe.wait_for_frames(5000))
                except Exception as e:
                    print("[rs-grabber] 出帧中断(%s) -> 重连相机" % e, flush=True)
                    break  # 跳出内层 -> 外层重连
                d = frames.get_depth_frame()
                c = frames.get_color_frame()
                if not d or not c:
                    continue
                depth = np.asanyarray(d.get_data())    # uint16 毫米
                color = np.asanyarray(c.get_data())    # 640x480x3 bgr8
                atomic_save(os.path.join(args.out, "color.png"),
                            lambda p: open(p, "wb").write(cv2.imencode(".png", color)[1].tobytes()))
                atomic_save(os.path.join(args.out, "color_hi.png"),
                            lambda p: open(p, "wb").write(cv2.imencode(".png", color)[1].tobytes()))
                atomic_save(os.path.join(args.out, "depth.npy"),
                            lambda p: np.save(open(p, "wb"), depth))
                frame_id += 1
                atomic_save(os.path.join(args.out, "frame.txt"),
                            lambda p: open(p, "w").write(str(frame_id)))
                if args.once:
                    nz = int(np.count_nonzero(depth))
                    print("[rs-grabber] once: color %s depth %s 非零 %d -> %s" %
                          (color.shape, depth.shape, nz, args.out), flush=True)
                    try:
                        pipe.stop()
                    except Exception:
                        pass
                    return
                time.sleep(period)
        finally:
            try:
                pipe.stop()
            except Exception:
                pass
        time.sleep(2.0)  # 重连前等相机重新枚举


if __name__ == "__main__":
    main()
