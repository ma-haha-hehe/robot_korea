#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""Bridge planner/vision output to the current UR5 execution YAML.

The execution node expects small offsets relative to taught joint poses.
This script reads:
  - planner output: panda_pick/src/plan.yaml
  - vision output:  T_base_obj / center_base_xyz from FoundationPose
  - bridge config:  reference taught TCP positions and safety limits

and writes:
  - panda_pick/config/pick_place_task.yaml
"""

import argparse
import math
import os
from typing import Any, Dict, Iterable, List, Optional, Tuple

import numpy as np
import yaml


def load_yaml(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return data


def require_xyz(config: Dict[str, Any], section: str, key: str) -> np.ndarray:
    value = config.get(section, {}).get(key)
    if value is None:
        raise ValueError(
            f"请先在 bridge config 里填写 {section}.{key}。"
            "这些值用 print_pose 从真实机器人示教点读取。"
        )
    arr = np.asarray(value, dtype=float)
    if arr.shape != (3,):
        raise ValueError(f"{section}.{key} must be a list of 3 numbers")
    return arr


def optional_xyz(config: Dict[str, Any], section: str, key: str, default: Iterable[float]) -> np.ndarray:
    value = config.get(section, {}).get(key, default)
    arr = np.asarray(value, dtype=float)
    if arr.shape != (3,):
        raise ValueError(f"{section}.{key} must be a list of 3 numbers")
    return arr


def normalize_angle_deg(angle: float) -> float:
    return (float(angle) + 180.0) % 360.0 - 180.0


def normalize_lego_yaw_deg(angle: float) -> float:
    """Normalize yaw for rectangular Lego-like parts with 180 deg symmetry."""
    return (float(angle) + 90.0) % 180.0 - 90.0


def blueprint_yaw_to_deg(value: float) -> float:
    """Accept planner yaw in radians or degrees.

    Older planner files in this workspace use both conventions. Values whose
    magnitude is larger than 2*pi are treated as degrees; smaller values are
    treated as radians.
    """
    yaw = float(value)
    if abs(yaw) > (2.0 * math.pi + 1e-6):
        return yaw
    return math.degrees(yaw)


def matrix_from_value(value: Any, label: str) -> np.ndarray:
    mat = np.asarray(value, dtype=float)
    if mat.shape != (4, 4):
        raise ValueError(f"{label} must be a 4x4 transform")
    return mat


def yaw_from_matrix_deg(transform: np.ndarray) -> float:
    return math.degrees(math.atan2(transform[1, 0], transform[0, 0]))


def yaw_from_matrix_y_deg(transform: np.ndarray) -> float:
    """物体 Y 轴(FoundationPose 显示的绿轴)在基座 XY 平面的方位角。
    用户确认: 夹爪平行于绿轴 = 抓长边。"""
    return math.degrees(math.atan2(transform[1, 1], transform[0, 1]))


def rpy_from_matrix_deg(transform: np.ndarray) -> List[float]:
    rot = transform[:3, :3]
    sy = math.sqrt(rot[0, 0] * rot[0, 0] + rot[1, 0] * rot[1, 0])
    singular = sy < 1e-6
    if not singular:
        roll = math.atan2(rot[2, 1], rot[2, 2])
        pitch = math.atan2(-rot[2, 0], sy)
        yaw = math.atan2(rot[1, 0], rot[0, 0])
    else:
        roll = math.atan2(-rot[1, 2], rot[1, 1])
        pitch = math.atan2(-rot[2, 0], sy)
        yaw = 0.0
    return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]


def quat_xyzw_from_matrix(transform: np.ndarray) -> List[float]:
    rot = transform[:3, :3]
    trace = float(np.trace(rot))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (rot[2, 1] - rot[1, 2]) / s
        qy = (rot[0, 2] - rot[2, 0]) / s
        qz = (rot[1, 0] - rot[0, 1]) / s
    elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
        qw = (rot[2, 1] - rot[1, 2]) / s
        qx = 0.25 * s
        qy = (rot[0, 1] + rot[1, 0]) / s
        qz = (rot[0, 2] + rot[2, 0]) / s
    elif rot[1, 1] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
        qw = (rot[0, 2] - rot[2, 0]) / s
        qx = (rot[0, 1] + rot[1, 0]) / s
        qy = 0.25 * s
        qz = (rot[1, 2] + rot[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
        qw = (rot[1, 0] - rot[0, 1]) / s
        qx = (rot[0, 2] + rot[2, 0]) / s
        qy = (rot[1, 2] + rot[2, 1]) / s
        qz = 0.25 * s
    return [qx, qy, qz, qw]


def rounded_list(values: Iterable[float], digits: int = 6) -> List[float]:
    return [round(float(value), digits) for value in values]


def rounded_matrix(transform: np.ndarray, digits: int = 6) -> List[List[float]]:
    return [rounded_list(row, digits) for row in transform.tolist()]


def pose_6d_from_transform(transform: np.ndarray) -> Dict[str, Any]:
    return {
        "xyz": rounded_list(transform[:3, 3]),
        "rpy_deg": rounded_list(rpy_from_matrix_deg(transform)),
        "quat_xyzw": rounded_list(quat_xyzw_from_matrix(transform)),
        "matrix": rounded_matrix(transform),
    }


def yaw_pose_6d(position_xyz: np.ndarray, yaw_deg: float) -> Dict[str, Any]:
    yaw_rad = math.radians(float(yaw_deg))
    return {
        "xyz": rounded_list(position_xyz),
        "rpy_deg": [0.0, 0.0, round(float(yaw_deg), 6)],
        "yaw_rad": round(yaw_rad, 6),
    }


def load_optional_transform(path: Optional[str], key: str) -> Optional[np.ndarray]:
    if not path or not os.path.exists(path):
        return None
    data = load_yaml(path)
    if key not in data:
        return None
    return matrix_from_value(data[key], key)


def handeye_base_object_transform(vision_item: Dict[str, Any],
                                  config: Dict[str, Any]) -> Optional[np.ndarray]:
    if "T_cam_obj" not in vision_item:
        return None

    handeye = config.get("handeye", {})
    tool_camera = load_optional_transform(handeye.get("camera_to_tool_yaml"), "T_tool_camera")
    if tool_camera is None:
        return None

    observe_base_tool_value = handeye.get("observe_T_base_tool")
    if observe_base_tool_value is None:
        return None
    observe_base_tool = matrix_from_value(observe_base_tool_value, "handeye.observe_T_base_tool")
    cam_obj = matrix_from_value(vision_item["T_cam_obj"], "T_cam_obj")
    return observe_base_tool @ tool_camera @ cam_obj


def collect_vision_objects(vision: Dict[str, Any], config: Dict[str, Any]) -> List[Dict[str, Any]]:
    objects: List[Dict[str, Any]] = []
    prefer_handeye = bool(config.get("handeye", {}).get("prefer_handeye", True))

    if isinstance(vision.get("objects"), list):
        for item in vision["objects"]:
            if isinstance(item, dict):
                objects.extend(collect_vision_objects(item, config))
        return objects

    handeye_transform = handeye_base_object_transform(vision, config)
    if "T_base_obj" in vision or handeye_transform is not None:
        if prefer_handeye and handeye_transform is not None:
            transform = handeye_transform
        elif "T_base_obj" in vision:
            transform = matrix_from_value(vision["T_base_obj"], "T_base_obj")
        else:
            transform = handeye_transform
        objects.append({
            "name": str(vision.get("target", vision.get("name", "object"))),
            "T_base_obj": transform,
            "T_cam_obj": matrix_from_value(vision["T_cam_obj"], "T_cam_obj") if "T_cam_obj" in vision else None,
            "center_base_xyz": np.asarray(
                vision.get("center_base_xyz", transform[:3, 3]),
                dtype=float,
            ),
        })
        return objects

    # Support older format: {"white 2x4 brick.": [[4x4 matrix]]}
    for name, value in vision.items():
        try:
            transform = matrix_from_value(value, name)
        except Exception:
            continue
        objects.append({
            "name": str(name),
            "T_base_obj": transform,
            "T_cam_obj": None,
            "center_base_xyz": transform[:3, 3],
        })
    return objects


def name_score(task_name: str, object_name: str) -> int:
    task_tokens = set(task_name.lower().replace("_", " ").replace("-", " ").split())
    obj_tokens = set(object_name.lower().replace("_", " ").replace("-", " ").replace(".", " ").split())
    return len(task_tokens & obj_tokens)


def choose_vision_object(task: Dict[str, Any],
                         objects: List[Dict[str, Any]],
                         used_indices: set) -> Tuple[int, Dict[str, Any]]:
    if not objects:
        raise ValueError("vision output 没有任何 T_base_obj / center_base_xyz")

    task_name = str(task.get("name", ""))
    best_index: Optional[int] = None
    best_score = -1
    for i, obj in enumerate(objects):
        if i in used_indices and len(objects) > 1:
            continue
        score = name_score(task_name, str(obj.get("name", "")))
        if score > best_score:
            best_score = score
            best_index = i

    if best_index is None:
        best_index = 0
    return best_index, objects[best_index]


def check_offset(label: str,
                 offset: np.ndarray,
                 descend: float,
                 max_xy: float,
                 max_z: float,
                 max_descend: float) -> None:
    if abs(offset[0]) > max_xy or abs(offset[1]) > max_xy:
        raise ValueError(
            f"{label} dx/dy={offset[:2].tolist()} 超过安全限制 {max_xy:.3f} m。"
            "先检查 reference 示教坐标、相机标定和单位。"
        )
    if abs(offset[2]) > max_z:
        raise ValueError(
            f"{label} dz={offset[2]:.3f} 超过安全限制 {max_z:.3f} m。"
            "先不要让视觉改太多高度。"
        )
    if descend <= 0.0 or descend > max_descend:
        raise ValueError(f"{label} descend={descend:.3f} 不在 (0, {max_descend:.3f}] m")


def build_tasks(config: Dict[str, Any],
                plan: Dict[str, Any],
                vision: Dict[str, Any],
                task_limit: Optional[int]) -> Dict[str, Any]:
    planner_tasks = plan.get("tasksh", plan.get("tasks", []))
    if not isinstance(planner_tasks, list) or not planner_tasks:
        raise ValueError("planner file must contain tasksh: or tasks: list")
    if task_limit is not None:
        planner_tasks = planner_tasks[:task_limit]

    vision_objects = collect_vision_objects(vision, config)
    pick_ref = require_xyz(config, "reference", "pick_pre_base_xyz")
    place_ref = require_xyz(config, "reference", "place_pre_base_xyz")
    assembly_origin = require_xyz(config, "placement", "assembly_origin_base_xyz")

    pick_offset = optional_xyz(config, "pick", "pick_offset_xyz", [0.0, 0.0, 0.0])
    place_offset = optional_xyz(config, "placement", "place_offset_xyz", [0.0, 0.0, 0.0])

    pick_descend = float(config.get("pick", {}).get("descend", 0.08))
    pick_slow_final_descend = float(config.get("pick", {}).get("slow_final_descend", 0.0))
    place_descend = float(config.get("place", {}).get("descend", 0.16))
    place_slow_final_descend = float(config.get("place", {}).get("slow_final_descend", 0.0))
    use_vision_z = bool(config.get("pick", {}).get("use_vision_z", False))

    yaw_cfg = config.get("yaw", {})
    use_vision_yaw = bool(yaw_cfg.get("use_vision_yaw", True))
    use_planner_grasp_spin = bool(yaw_cfg.get("use_planner_grasp_spin", True))
    use_planner_blueprint_yaw = bool(yaw_cfg.get("use_planner_blueprint_yaw", True))
    pick_yaw_offset = float(yaw_cfg.get("pick_yaw_offset_deg", 0.0))
    place_yaw_offset = float(yaw_cfg.get("place_yaw_offset_deg", 0.0))

    safety = config.get("safety", {})
    max_xy = float(safety.get("max_xy_offset", 0.15))
    max_z = float(safety.get("max_z_offset", 0.08))
    max_descend = float(safety.get("max_descend", 0.30))
    axis_cfg = config.get("execution_axis", {})
    flip_x = bool(axis_cfg.get("flip_x", False))
    flip_y = bool(axis_cfg.get("flip_y", False))
    final_pick_offset = optional_xyz(config, "execution_axis", "final_pick_offset_xyz", [0.0, 0.0, 0.0])

    output_tasks = []
    used_vision_indices = set()
    for order, task in enumerate(planner_tasks):
        obj_index, obj = choose_vision_object(task, vision_objects, used_vision_indices)
        used_vision_indices.add(obj_index)

        obj_xyz = np.asarray(obj["center_base_xyz"], dtype=float)
        if obj_xyz.shape != (3,):
            raise ValueError(f"vision object {obj.get('name')} center_base_xyz is invalid")
        if not use_vision_z:
            obj_xyz = obj_xyz.copy()
            obj_xyz[2] = pick_ref[2]

        # 2026-06-27: 沿物体长轴(Y/绿轴)平移抓取点(FoundationPose origin 不在 brick 几何中心时).
        # 对象系偏移, 跟着积木朝向走, 任意角度都成立(不像 base 系 pick_offset)。
        long_off = float(config.get("pick", {}).get("long_axis_offset_m", 0.0))
        if long_off and obj.get("T_base_obj") is not None:
            yaxis = np.asarray(obj["T_base_obj"], dtype=float)[:2, 1]
            n = np.linalg.norm(yaxis)
            if n > 1e-9:
                obj_xyz = obj_xyz.copy()
                obj_xyz[:2] = obj_xyz[:2] + long_off * (yaxis / n)

        pick_delta = obj_xyz - pick_ref + pick_offset

        place_plan = np.asarray(task.get("place", {}).get("pos", [0.0, 0.0, 0.0]), dtype=float)
        if place_plan.shape != (3,):
            raise ValueError(f"task {task.get('name', order)} place.pos must be 3 values")
        place_xyz = assembly_origin + place_plan + place_offset
        place_delta = place_xyz - place_ref
        raw_pick_delta = pick_delta.copy()
        raw_place_delta = place_delta.copy()
        if flip_x:
            pick_delta = pick_delta.copy()
            place_delta = place_delta.copy()
            pick_delta[0] *= -1.0
            place_delta[0] *= -1.0
        if flip_y:
            pick_delta = pick_delta.copy()
            place_delta = place_delta.copy()
            pick_delta[1] *= -1.0
            place_delta[1] *= -1.0
        pick_delta = pick_delta + final_pick_offset

        # 2026-06-27: 用绿轴(Y)对齐长边; pick_yaw 归一到 [-90,90](2x4 180°对称)走最短路径, 杜绝315°绕远
        # 2026-06-27: 绿轴取负 —— 两点标定实测 wrist 相对 brick 反向(斜率a≈-1), 故 vision_yaw = -绿轴方位角
        vision_yaw = normalize_lego_yaw_deg(-yaw_from_matrix_y_deg(obj["T_base_obj"]))
        grasp_spin = float(task.get("grasp_spin", 0.0)) if use_planner_grasp_spin else 0.0
        pick_yaw = normalize_lego_yaw_deg((vision_yaw if use_vision_yaw else 0.0) + grasp_spin + pick_yaw_offset)

        blueprint_yaw = float(task.get("blueprint_yaw", 0.0))
        place_yaw_from_plan = blueprint_yaw_to_deg(blueprint_yaw) if use_planner_blueprint_yaw else 0.0
        place_yaw = normalize_angle_deg(place_yaw_from_plan + place_yaw_offset)

        check_offset(f"{task.get('name', order)} pick", pick_delta, pick_descend, max_xy, max_z, max_descend)
        check_offset(f"{task.get('name', order)} place", place_delta, place_descend, max_xy, max_z, max_descend)

        pick_pre_xyz = pick_ref + pick_delta
        pick_grasp_xyz = pick_pre_xyz.copy()
        pick_grasp_xyz[2] -= pick_descend
        pick_slow_start_xyz = pick_grasp_xyz.copy()
        if pick_slow_final_descend > 0.0:
            pick_slow_start_xyz[2] += min(pick_slow_final_descend, pick_descend)
        place_pre_xyz = place_ref + place_delta
        place_down_xyz = place_pre_xyz.copy()
        place_down_xyz[2] -= place_descend
        slow_start_xyz = place_down_xyz.copy()
        if place_slow_final_descend > 0.0:
            slow_start_xyz[2] += min(place_slow_final_descend, place_descend)

        rotation_info = {
            "vision_yaw_deg": round(float(vision_yaw), 2),
            "grasp_spin_deg": round(float(grasp_spin), 2),
            "pick_yaw_deg": round(float(pick_yaw), 2),
            "blueprint_yaw_rad": round(float(blueprint_yaw), 6),
            "blueprint_yaw_deg": round(float(place_yaw_from_plan), 2),
            "place_yaw_deg": round(float(place_yaw), 2),
        }
        axis_info = {
            "flip_x": flip_x,
            "flip_y": flip_y,
            "final_pick_offset_xyz": rounded_list(final_pick_offset, 4),
            "raw_pick_delta_xyz": rounded_list(raw_pick_delta, 4),
            "raw_place_delta_xyz": rounded_list(raw_place_delta, 4),
            "execution_pick_delta_xyz": rounded_list(pick_delta, 4),
            "execution_place_delta_xyz": rounded_list(place_delta, 4),
        }
        vision_pose_6d = {"T_base_obj": pose_6d_from_transform(obj["T_base_obj"])}
        if obj.get("T_cam_obj") is not None:
            vision_pose_6d["T_cam_obj"] = pose_6d_from_transform(obj["T_cam_obj"])

        pick_info = {
            "dx": round(float(pick_delta[0]), 4),
            "dy": round(float(pick_delta[1]), 4),
            "dz": round(float(pick_delta[2]), 4),
            "yaw_deg": round(float(pick_yaw), 2),
            "descend": round(float(pick_descend), 4),
            "slow_final_descend": round(float(pick_slow_final_descend), 4),
            "pre_pose_6d": yaw_pose_6d(pick_pre_xyz, pick_yaw),
            "grasp_pose_6d": yaw_pose_6d(pick_grasp_xyz, pick_yaw),
        }
        if pick_slow_final_descend > 0.0:
            pick_info["slow_start_pose_6d"] = yaw_pose_6d(pick_slow_start_xyz, pick_yaw)

        place_info = {
            "dx": round(float(place_delta[0]), 4),
            "dy": round(float(place_delta[1]), 4),
            "dz": round(float(place_delta[2]), 4),
            "yaw_deg": round(float(place_yaw), 2),
            "descend": round(float(place_descend), 4),
            "slow_final_descend": round(float(place_slow_final_descend), 4),
            "pre_pose_6d": yaw_pose_6d(place_pre_xyz, place_yaw),
            "place_pose_6d": yaw_pose_6d(place_down_xyz, place_yaw),
        }
        if place_slow_final_descend > 0.0:
            place_info["slow_start_pose_6d"] = yaw_pose_6d(slow_start_xyz, place_yaw)

        output_tasks.append({
            "id": f"{int(task.get('id', order)):02d}_{task.get('name', 'task')}",
            "vision_target": obj.get("name", "object"),
            "rotation": rotation_info,
            "execution_axis": axis_info,
            "vision_pose_6d": vision_pose_6d,
            "pick": pick_info,
            "place": place_info,
        })

    return {"tasks": output_tasks}


def write_yaml(path: str, data: Dict[str, Any]) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    header = (
        "# Generated by my_robot_vision/vision_to_execution_yaml.py\n"
        "# Units: dx/dy/dz/descend are meters, yaw_deg is degrees.\n"
        "# These are small offsets relative to taught pregrasp/preplace joints.\n"
    )
    with open(path, "w", encoding="utf-8") as f:
        f.write(header)
        yaml.safe_dump(data, f, sort_keys=False)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        default="/home/i6user/Desktop/robot_lego/src/my_robot_vision/config/vision_execution_bridge.yaml",
    )
    parser.add_argument("--plan", default=None)
    parser.add_argument("--vision", default=None)
    parser.add_argument("--output", default=None)
    parser.add_argument("--task-limit", type=int, default=None)
    args = parser.parse_args()

    config = load_yaml(args.config)
    paths = config.get("paths", {})
    plan_path = args.plan or paths.get("plan")
    vision_path = args.vision or paths.get("vision_output")
    output_path = args.output or paths.get("output_task")
    if not plan_path or not vision_path or not output_path:
        raise ValueError("plan, vision, and output paths must be provided")

    result = build_tasks(
        config,
        load_yaml(plan_path),
        load_yaml(vision_path),
        args.task_limit,
    )
    write_yaml(output_path, result)
    print(f"wrote {len(result['tasks'])} execution task(s): {output_path}")


if __name__ == "__main__":
    main()
