#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""O3P 深度帧 <-> 高清彩色 的鱼眼投影工具。

build_depth_to_hi_map(depth_m, params) -> (uc, vc, valid):
  对每个深度像素, 反投影成3D(深度系) -> 外参变换到彩色系 -> 鱼眼投影到高清彩色像素。
  用于把【在高清彩色上得到的 mask】采样回深度帧(240x180), 喂给 FoundationPose。

sample_hi_to_depth(hi_img, uc, vc, valid) -> 240x180 图: 把高清图按映射采样到深度帧。
"""
import numpy as np
import cv2


def load_params(p):
    Kd = np.array(p["depth_K"], dtype=np.float64)
    Kc = np.array(p["color_K"], dtype=np.float64)
    Dc = np.array(p["color_D"], dtype=np.float64).reshape(4, 1)
    T = np.array(p["T_depth_to_color"], dtype=np.float64)
    return Kd, Kc, Dc, T


def build_depth_to_hi_map(depth_m, params, hi_w=1344, hi_h=1008, zmin=0.05):
    """返回与深度同形(H,W)的 uc, vc(高清彩色像素坐标) 和 valid 掩码。"""
    Kd, Kc, Dc, T = load_params(params)
    R, t = T[:3, :3], T[:3, 3]
    H, W = depth_m.shape
    us, vs = np.meshgrid(np.arange(W), np.arange(H))
    Z = depth_m.reshape(-1).astype(np.float64)
    x = (us.reshape(-1) - Kd[0, 2]) / Kd[0, 0] * Z
    y = (vs.reshape(-1) - Kd[1, 2]) / Kd[1, 1] * Z
    Pd = np.stack([x, y, Z], axis=1)              # 深度系 3D
    Pc = (R @ Pd.T).T + t                          # 彩色系 3D

    uc = np.full(H * W, -1, dtype=np.int32)
    vc = np.full(H * W, -1, dtype=np.int32)
    good = (Z > zmin) & (Pc[:, 2] > 1e-6)
    idx = np.where(good)[0]
    if idx.size:
        obj = Pc[idx].reshape(-1, 1, 3)
        img_pts, _ = cv2.fisheye.projectPoints(obj, np.zeros(3), np.zeros(3), Kc, Dc)
        uv = img_pts.reshape(-1, 2)
        u = np.round(uv[:, 0]).astype(np.int32)
        v = np.round(uv[:, 1]).astype(np.int32)
        inb = (u >= 0) & (u < hi_w) & (v >= 0) & (v < hi_h)
        sel = idx[inb]
        uc[sel] = u[inb]
        vc[sel] = v[inb]
    valid = (uc >= 0)
    return uc.reshape(H, W), vc.reshape(H, W), valid.reshape(H, W)


def sample_hi_to_depth(hi_img, uc, vc, valid):
    """把高清图(任意通道)按映射采样到深度帧(uc/vc 同深度形)。无效像素填0。"""
    H, W = uc.shape
    ch = () if hi_img.ndim == 2 else (hi_img.shape[2],)
    out = np.zeros((H, W) + ch, dtype=hi_img.dtype)
    yy, xx = np.where(valid)
    out[yy, xx] = hi_img[vc[yy, xx], uc[yy, xx]]
    return out


if __name__ == "__main__":
    # 自校验: 用 SDK 的对齐彩色(color.png) 验证我们的投影
    import json
    import sys
    D = sys.argv[1] if len(sys.argv) > 1 else "/home/i6user/Desktop/robot_lego/src/panda_pick/src/o3p"
    params = json.load(open(D + "/camera_params.json"))
    depth_m = np.load(D + "/depth.npy").astype(np.float64) / 1000.0
    hi = cv2.imread(D + "/color_hi.png", cv2.IMREAD_COLOR)
    sdk = cv2.imread(D + "/color.png", cv2.IMREAD_COLOR)   # SDK 对齐彩色 240x180

    uc, vc, valid = build_depth_to_hi_map(depth_m, params, hi.shape[1], hi.shape[0])
    mine = sample_hi_to_depth(hi, uc, vc, valid)

    # 比较(只在双方都有效的像素上)
    m = valid & (sdk.sum(axis=2) > 0)
    diff = np.abs(mine[m].astype(int) - sdk[m].astype(int)).mean()
    print("有效像素数: 我的=%d  SDK=%d  重叠=%d" % (valid.sum(), (sdk.sum(2) > 0).sum(), m.sum()))
    print("重叠像素上 平均逐通道差(0-255): %.1f  <- 越小越说明投影和SDK一致" % diff)
    side = np.hstack([sdk, mine, cv2.absdiff(sdk, mine)])
    cv2.imwrite(D + "/align_check.png", cv2.resize(side, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST))
    print("对比图(左:SDK对齐 中:我的投影 右:差异) ->", D + "/align_check.png")
