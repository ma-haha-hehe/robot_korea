#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""O3P 高清彩色(鱼眼) eye-in-hand 手眼标定, 并组合回深度系。

为什么要组合: FoundationPose 运行时输出的 T_cam_obj 是在【深度光学系】里的,
而我们在高清彩色系标定, 所以最后要把 T_tool_color 组合成 T_tool_depth:

    p_color = T_depth_to_color @ p_depth          (camera_params.json 给的外参)
    p_tool  = T_tool_color   @ p_color
    => p_tool = (T_tool_color @ T_depth_to_color) @ p_depth
    => T_tool_camera(运行时用) = T_tool_color @ T_depth_to_color

鱼眼: 角点先用 cv2.fisheye.undistortPoints(color_K, color_D) 去畸变, 与运行时
o3p_align 的 cv2.fisheye.projectPoints 同一套模型, 再 solvePnP。
"""
import argparse
import os

import cv2
import numpy as np
import yaml


def load_yaml(p):
    with open(p) as f:
        return yaml.safe_load(f)


def chessboard_objp(cols, rows, sq):
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    return objp * float(sq)


def find_corners(img_path, cols, rows, debug_dir):
    img = cv2.imread(img_path, cv2.IMREAD_COLOR)
    if img is None:
        raise SystemExit(f"读不到图: {img_path}")
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, cor = cv2.findChessboardCorners(gray, (cols, rows), flags)
    if not ok:
        raise SystemExit(f"棋盘检测失败(检查 board_size/图像清晰度): {img_path}")
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.001)
    cor = cv2.cornerSubPix(gray, cor, (11, 11), (-1, -1), crit)
    if debug_dir:
        os.makedirs(debug_dir, exist_ok=True)
        ann = img.copy()
        cv2.drawChessboardCorners(ann, (cols, rows), cor, ok)
        cv2.imwrite(os.path.join(debug_dir, os.path.basename(img_path)), ann)
    return cor.reshape(-1, 1, 2).astype(np.float64)


def _rot_angle(R):
    return np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2.0, -1.0, 1.0)))


def _deflip_targets(base_tool, cam_tgt, cols, rows, sq):
    """以 #0 为锚, 把棋盘 180° 翻转的样本目标位姿乘 F 翻回一致。

    F = 绕板心 z 轴 180°(自逆): 对每个样本独立判定, 取 [臂相对转角] 与
    [相机相对转角] 更接近的那个(翻 or 不翻)。返回纠正后的 cam_tgt 列表。
    """
    F = np.eye(4)
    F[0, 0] = -1.0
    F[1, 1] = -1.0
    F[0, 3] = (cols - 1) * sq
    F[1, 3] = (rows - 1) * sq
    R0g = base_tool[0][:3, :3]
    R0c = cam_tgt[0][:3, :3]
    out = [cam_tgt[0]]
    flipped = []
    for i in range(1, len(cam_tgt)):
        aA = _rot_angle(R0g.T @ base_tool[i][:3, :3])
        aB = _rot_angle(R0c @ cam_tgt[i][:3, :3].T)
        aBf = _rot_angle(R0c @ (cam_tgt[i] @ F)[:3, :3].T)
        if abs(aA - aBf) < abs(aA - aB):
            out.append(cam_tgt[i] @ F)
            flipped.append(i + 1)
        else:
            out.append(cam_tgt[i])
    if flipped:
        print(f"[deflip] 自动纠正了 {len(flipped)} 个翻转样本: {flipped}")
    else:
        print("[deflip] 没检测到棋盘翻转样本。")
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--samples", required=True)
    ap.add_argument("--output", required=True)
    ap.add_argument("--debug-dir", default="")
    ap.add_argument("--method", default="tsai",
                    choices=["tsai", "park", "horaud", "andreff", "daniilidis"])
    a = ap.parse_args()

    cfg = load_yaml(a.samples)
    cols, rows = cfg["board_size"]
    sq = float(cfg["square_size"])
    K = np.array(cfg["camera_matrix"], dtype=np.float64)
    D = np.array(cfg["dist_coeffs_fisheye"], dtype=np.float64).reshape(4, 1)
    T_d2c = np.array(cfg["T_depth_to_color"], dtype=np.float64)
    samples = cfg["samples"]
    if not isinstance(samples, list) or len(samples) < 5:
        raise SystemExit("至少 5 个样本, 建议 15-20 个旋转差异大的姿态。")

    objp = chessboard_objp(cols, rows, sq)
    base_tool_list, cam_tgt_list = [], []
    for s in samples:
        cor = find_corners(s["image"], cols, rows, a.debug_dir)
        # 去鱼眼畸变 -> 理想针孔像素(用 color_K 当投影矩阵 P), 再 solvePnP(零畸变)
        und = cv2.fisheye.undistortPoints(cor, K, D, P=K)
        ok, rvec, tvec = cv2.solvePnP(
            objp, und.reshape(-1, 2), K, np.zeros(5),
            flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            raise SystemExit(f"solvePnP 失败: {s['image']}")
        Tb = np.array(s["T_base_tool"], dtype=np.float64)
        Tc = np.eye(4)
        Tc[:3, :3] = cv2.Rodrigues(rvec)[0]
        Tc[:3, 3] = tvec.reshape(3)
        base_tool_list.append(Tb)
        cam_tgt_list.append(Tc)

    # 自动纠正棋盘 180° 朝向歧义(4x6 等对称板在大旋转下 OpenCV 角点顺序会翻转)。
    # 判据(与外参无关): 相邻/对锚 姿态的 [机械臂相对转角] 必须等于 [相机看板相对转角],
    # 不等就把该样本目标位姿乘 180° 修正 F(F 自逆), 把棋盘坐标系翻回一致。
    cam_tgt_list = _deflip_targets(base_tool_list, cam_tgt_list, cols, rows, sq)

    R_g2b = [T[:3, :3] for T in base_tool_list]
    t_g2b = [T[:3, 3].reshape(3, 1) for T in base_tool_list]
    R_t2c = [T[:3, :3] for T in cam_tgt_list]
    t_t2c = [T[:3, 3].reshape(3, 1) for T in cam_tgt_list]

    mm = {"tsai": cv2.CALIB_HAND_EYE_TSAI, "park": cv2.CALIB_HAND_EYE_PARK,
          "horaud": cv2.CALIB_HAND_EYE_HORAUD, "andreff": cv2.CALIB_HAND_EYE_ANDREFF,
          "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS}
    Rc2g, tc2g = cv2.calibrateHandEye(R_g2b, t_g2b, R_t2c, t_t2c, method=mm[a.method])
    T_tool_color = np.eye(4)
    T_tool_color[:3, :3] = Rc2g
    T_tool_color[:3, 3] = tc2g.reshape(3)

    # 组合回深度系(运行时 T_cam_obj 在深度系)
    T_tool_depth = T_tool_color @ T_d2c

    # 诊断: 棋盘在基座系应固定不动, 看三轴标准差(在 color 系下一致评估)
    base_tgt = [(Tb @ T_tool_color @ Tc)[:3, 3]
                for Tb, Tc in zip(base_tool_list, cam_tgt_list)]
    std = np.std(np.array(base_tgt), axis=0)

    out = {
        "T_tool_camera": T_tool_depth.tolist(),
        "T_tool_color_raw": T_tool_color.tolist(),
        "camera_matrix": K.tolist(),
        "dist_coeffs_fisheye": D.reshape(-1).tolist(),
        "frame": "depth (composed from color via T_depth_to_color)",
        "diagnostics": {
            "used_samples": [s["image"] for s in samples],
            "base_target_translation_std_m": std.tolist(),
        },
    }
    os.makedirs(os.path.dirname(os.path.abspath(a.output)), exist_ok=True)
    yaml.safe_dump(out, open(a.output, "w"), sort_keys=False)

    print("T_tool_camera (已组合回深度系) 写入:", a.output)
    for r in T_tool_depth:
        print("  ", ["%.9f" % v for v in r])
    print("base target translation std [m]:", ["%.6f" % v for v in std])
    print("经验: 三个值都 < 0.005~0.010 m 算合格。")


if __name__ == "__main__":
    main()
