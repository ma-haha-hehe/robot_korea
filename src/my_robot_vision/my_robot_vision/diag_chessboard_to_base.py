#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""方法A诊断（容器内运行）：棋盘 solvePnP(≈精确) 定位一个角点，经【运行时同一套】
observe_T_base_tool @ T_tool_camera 变换到机械臂基座系。不经过 FoundationPose，
所以基座系坐标的偏差≈手眼/内参误差，与感知无关。

配置由 run_diag.sh 拷进 /vision_code:
  /vision_code/camera_to_tool.yaml           -> T_tool_camera
  /vision_code/vision_execution_bridge.yaml  -> handeye.observe_T_base_tool
前提: 棋盘在视野内且夹爪够得着; 机械臂停在观察/home 姿态(UR5_OBSERVE_JOINTS);
      相机没被别的程序(视觉bridge)占用。
"""
import argparse
import numpy as np
import yaml
import cv2
import pyrealsense2 as rs

EXTR = "/vision_code/camera_to_tool.yaml"
BRIDGE = "/vision_code/vision_execution_bridge.yaml"
OUT = "/vision_code/diag_corner.jpg"


def load_mat(v):
    m = np.asarray(v, dtype=float)
    assert m.shape == (4, 4), "需要 4x4 矩阵"
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--board-size", type=int, nargs=2, default=[4, 6],
                    help="内角点数(列 行)")
    ap.add_argument("--square-size", type=float, default=0.030, help="方格边长(米)")
    ap.add_argument("--corner", type=int, default=0, help="测哪个内角点(默认0)")
    args = ap.parse_args()

    T_tool_camera = load_mat(yaml.safe_load(open(EXTR))["T_tool_camera"])
    T_base_tool = load_mat(yaml.safe_load(open(BRIDGE))["handeye"]["observe_T_base_tool"])

    # RealSense 一帧彩色(关红外), 用实时内参(与 FoundationPose 运行时一致)
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    profile = pipe.start(cfg)
    for s in profile.get_device().query_sensors():
        if s.supports(rs.option.emitter_enabled):
            s.set_option(rs.option.emitter_enabled, 0)
    try:
        frame = None
        for _ in range(30):
            f = pipe.wait_for_frames().get_color_frame()
            if f:
                frame = np.asanyarray(f.get_data())
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    finally:
        pipe.stop()
    if frame is None:
        raise RuntimeError("没拿到彩色帧")
    K = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]], dtype=float)
    dist = np.array(intr.coeffs, dtype=float)
    print(f"[i] 实时内参 fx={intr.fx:.2f} fy={intr.fy:.2f} cx={intr.ppx:.2f} cy={intr.ppy:.2f}")

    cols, rows = args.board_size
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ok, corners = cv2.findChessboardCorners(
        gray, (cols, rows),
        cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not ok:
        cv2.imwrite(OUT, frame)
        raise RuntimeError(f"没检测到 {cols}x{rows} 内角点棋盘。看 {OUT}，"
                           "确认整块板清晰可见、board-size 填的是【内角点数】。")
    corners = cv2.cornerSubPix(
        gray, corners, (11, 11), (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-3))
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * args.square_size
    ok, rvec, tvec = cv2.solvePnP(objp, corners, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise RuntimeError("solvePnP 失败")
    R, _ = cv2.Rodrigues(rvec)

    c = args.corner
    P_cam = (R @ objp[c].reshape(3, 1) + tvec).reshape(3)
    P_base = (T_base_tool @ T_tool_camera @ np.r_[P_cam, 1.0])[:3]

    cv2.drawChessboardCorners(frame, (cols, rows), corners, ok)
    px = tuple(np.round(corners[c].reshape(2)).astype(int))
    # 精确十字 + 箭头 + 远处文字, 让"碰哪个点"一目了然
    cv2.drawMarker(frame, px, (0, 0, 255), cv2.MARKER_CROSS, 26, 3)
    cv2.circle(frame, px, 16, (0, 0, 255), 2)
    lbl = (px[0] - 150 if px[0] > frame.shape[1] // 2 else px[0] + 30,
           px[1] - 30 if px[1] > frame.shape[0] // 2 else px[1] + 40)
    cv2.arrowedLine(frame, lbl, px, (0, 0, 255), 2, tipLength=0.2)
    cv2.putText(frame, "TOUCH HERE (#%d)" % c, (lbl[0] - 10, lbl[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    cv2.imwrite(OUT, frame)
    # 角点在画面的方位, 便于口头描述
    hpos = "右" if px[0] > frame.shape[1] * 0.55 else ("左" if px[0] < frame.shape[1] * 0.45 else "中")
    vpos = "下" if px[1] > frame.shape[0] * 0.55 else ("上" if px[1] < frame.shape[0] * 0.45 else "中")
    print(f"[i] 角点#{c} 在画面的 [{vpos}{hpos}] 位置, 像素{px}")

    print("\n================= 结果 (角点 #%d) =================" % c)
    print(f"[相机系] X={P_cam[0]:+.4f}  Y={P_cam[1]:+.4f}  Z={P_cam[2]:+.4f}  (米)")
    print("         ↳ Z = 相机到该角点距离, 拿尺子量核对 -> 查深度/内参")
    print(f"[基座系] X={P_base[0]:+.4f}  Y={P_base[1]:+.4f}  Z={P_base[2]:+.4f}  (米)")
    print("         ↳ 夹爪竖直碰该角点, print_pose 读 TCP 的 X,Y 来比 -> 查手眼")
    print(f"\n标注图(看测的是哪个角): host=src/my_robot_vision/my_robot_vision/diag_corner.jpg")


if __name__ == "__main__":
    main()
