#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""斜抓换算器 (tilt 实验, 与主线分离, 不动现版本).

思路: 让夹爪接近轴(tool +Z, 指向积木) 对齐 物体 z 轴的反向(-object_z),
即垂直压上积木顶面; 然后沿这条斜轴下探。

坐标系: T_base_obj 在 base_link; 执行端在 URScript base(差绕Z 180°)。
  - 位置 x,y 取负 (= 现有 flip_x/flip_y)
  - 朝向 rotvec 的 x,y 取负、z 不变 (绕Z 180° 共轭的结果)

输出 pick_place_task_tilt.yaml:
  tilt_pick:
    grasp_xyz_base:  [x,y,z]      # URScript base 系 (已 flip) 积木中心
    grasp_rotvec_base: [rx,ry,rz] # URScript base 系 绝对朝向(轴角向量)
    standoff_m: 0.05              # 沿接近轴回退多少做 pre-grasp
    descend_m:  ...               # 沿接近轴下探到中心的距离(=standoff)
    tilt_deg:   22.x              # 仅供参考
用法: python3 vision_to_execution_tilt.py <vision_yaml> [--standoff 0.06]
"""
import sys
import math
import argparse
import numpy as np
import yaml

REPO = "/home/i6user/Desktop/robot_lego"
BRIDGE_CFG = f"{REPO}/src/my_robot_vision/config/vision_execution_bridge.yaml"
HANDEYE = f"{REPO}/src/my_robot_vision/config/camera_to_tool.yaml"
OUT = f"{REPO}/src/panda_pick/config/pick_place_task_tilt.yaml"


def load(p):
    with open(p, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def rotmat_to_rotvec(R):
    """旋转矩阵 -> 轴角向量 (URScript p[...] 用的 rx,ry,rz)。"""
    angle = math.acos(max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0)))
    if angle < 1e-9:
        return np.zeros(3)
    if abs(angle - math.pi) < 1e-6:
        # 180°特殊: 从对角线取轴
        axis = np.sqrt(np.clip((np.diag(R) + 1.0) / 2.0, 0, None))
        # 符号修正
        if R[0, 1] < 0: axis[1] = -axis[1]
        if R[0, 2] < 0: axis[2] = -axis[2]
        return axis * angle
    rx = R[2, 1] - R[1, 2]
    ry = R[0, 2] - R[2, 0]
    rz = R[1, 0] - R[0, 1]
    v = np.array([rx, ry, rz]) / (2.0 * math.sin(angle))
    return v * angle


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("vision")
    ap.add_argument("--standoff", type=float, default=None,
                    help="固定回退距离(m); 不给则 pre 落在观察高度(先高处对齐角度再接近)")
    args = ap.parse_args()

    cfg = load(BRIDGE_CFG)
    he = load(HANDEYE)
    vis = load(args.vision)

    T_base_tool = np.asarray(cfg["handeye"]["observe_T_base_tool"], float)
    T_tool_cam = np.asarray(he["T_tool_camera"], float)
    T_cam_obj = np.asarray(vis["T_cam_obj"], float)
    T_base_obj = T_base_tool @ T_tool_cam @ T_cam_obj  # base_link
    R = T_base_obj[:3, :3]
    center_bl = T_base_obj[:3, 3]

    obj_x = R[:, 0]
    obj_y = R[:, 1]
    obj_z = R[:, 2]
    # 物体 z 若朝下则翻正(顶面法线朝上)
    if obj_z[2] < 0:
        obj_z = -obj_z
        obj_x = -obj_x  # 保持右手系
    tilt_deg = math.degrees(math.acos(max(-1, min(1, abs(obj_z[2])))))

    # 夹爪坐标系(base_link): 接近轴 tool_z = -obj_z(压向顶面);
    # tool_x 沿物体长轴(绿轴Y, 抓长边); tool_y = tool_z × tool_x
    tool_z = -obj_z
    tool_x = obj_y - np.dot(obj_y, tool_z) * tool_z   # 去掉z分量, 投影到垂直tool_z
    nx = np.linalg.norm(tool_x)
    tool_x = tool_x / nx if nx > 1e-9 else np.array([1.0, 0, 0])
    tool_y = np.cross(tool_z, tool_x)
    R_grasp_bl = np.column_stack([tool_x, tool_y, tool_z])  # base_link 系夹爪朝向

    # ---- base_link -> URScript base: 绕Z 180° ----
    # 位置: x,y 取负
    center_base = np.array([-center_bl[0], -center_bl[1], center_bl[2]])
    # 朝向: Rz(180) 共轭 => rotvec 的 x,y 取负, z 不变
    rotvec_bl = rotmat_to_rotvec(R_grasp_bl)
    rotvec_base = np.array([-rotvec_bl[0], -rotvec_bl[1], rotvec_bl[2]])

    # pick_offset(基座系微调, 与主线一致地加在中心上)
    pick_off = np.asarray(cfg.get("pick", {}).get("pick_offset_xyz", [0, 0, 0]), float)
    # 主线里 pick_offset 加在 base_link 再 flip; 这里 center 已是 base, 故 xy 取负后加
    center_base = center_base + np.array([-pick_off[0], -pick_off[1], pick_off[2]])

    # 接近轴(base, URScript): = 抓取朝向的 tool +Z = Rz(180)·tool_z_bl
    approach_base = np.array([-tool_z[0], -tool_z[1], tool_z[2]])
    n = np.linalg.norm(approach_base)
    approach_base = approach_base / n if n > 1e-9 else np.array([0.0, 0.0, -1.0])

    # pre-grasp 位置: 默认落在【观察高度】(沿斜轴回退到 z=observe_z), 这样先在高处对齐角度再接近;
    # 传 --standoff 则改用固定回退距离。
    observe_z = float(T_base_tool[2, 3])
    if args.standoff is not None:
        standoff = float(args.standoff)
    elif abs(approach_base[2]) > 1e-6:
        standoff = (center_base[2] - observe_z) / approach_base[2]  # 使 pre_z = observe_z
    else:
        standoff = 0.10
    pre_base = center_base - standoff * approach_base

    out = {
        "tilt_pick": {
            "grasp_xyz_base": [round(float(v), 6) for v in center_base],
            "pre_xyz_base": [round(float(v), 6) for v in pre_base],
            "grasp_rotvec_base": [round(float(v), 6) for v in rotvec_base],
            "approach_base": [round(float(v), 4) for v in approach_base],
            "standoff_m": round(float(standoff), 4),
            "pre_at_observe_height": args.standoff is None,
            "tilt_deg": round(float(tilt_deg), 2),
            "obj_z_base_link": [round(float(v), 4) for v in obj_z],
        }
    }
    with open(OUT, "w", encoding="utf-8") as f:
        yaml.safe_dump(out, f, sort_keys=False)
    print(f"wrote {OUT}")
    print(yaml.safe_dump(out, sort_keys=False, allow_unicode=True))


if __name__ == "__main__":
    main()
