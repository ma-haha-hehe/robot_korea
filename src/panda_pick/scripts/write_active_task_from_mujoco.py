#!/usr/bin/env python3
import yaml
import numpy as np
import mujoco

ACTIVE_TASK_PATH = "/home/i6user/Desktop/robot_lego/src/panda_pick/src/active_task.yaml"

# 你当前环境里已经验证可用的俯抓朝向
DEFAULT_PICK_QUAT = [0.9238795, 0.3826834, 0.0, 0.0]
DEFAULT_PLACE_QUAT = [0.9238795, 0.3826834, 0.0, 0.0]


def get_body_pose(model, data, body_name: str):
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if body_id == -1:
        raise ValueError(f"Body '{body_name}' not found in MuJoCo model.")

    pos = data.xpos[body_id].copy()
    quat = data.xquat[body_id].copy()  # [w, x, y, z]
    return pos, quat


def write_active_task_yaml(
    task_name: str,
    pick_pos,
    pick_quat,
    place_pos,
    place_quat,
    out_path: str = ACTIVE_TASK_PATH,
):
    task = {
        "name": task_name,
        "pick": {
            "pos": [float(x) for x in pick_pos],
            "orientation": [float(x) for x in pick_quat],
        },
        "place": {
            "pos": [float(x) for x in place_pos],
            "orientation": [float(x) for x in place_quat],
        },
    }

    with open(out_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(task, f, sort_keys=False)

    print(f"[OK] active_task.yaml written to: {out_path}")
    print(task)


def main():
    # ===== 这里改成你自己的 MuJoCo XML / MJCF 路径 =====
    model_path = "/home/i6user/Desktop/robot_lego/src/mj_bridge/mj_bridge/scene.xml"

    # ===== 这里改成你要抓的物体 body 名字 =====
    body_name = "leg_left"

    # ===== 放置目标先手工指定 =====
    place_pos = [0.398, 0.2, 0.025]

    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    pick_pos, body_quat = get_body_pose(model, data, body_name)

    # 先只用中心点 + 固定俯抓朝向
    write_active_task_yaml(
        task_name=body_name,
        pick_pos=pick_pos,
        pick_quat=DEFAULT_PICK_QUAT,
        place_pos=place_pos,
        place_quat=DEFAULT_PLACE_QUAT,
    )


if __name__ == "__main__":
    main()