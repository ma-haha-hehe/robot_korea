# scene_builder.py
from __future__ import annotations

import math
import os
import shutil
import xml.etree.ElementTree as ET
from xml.dom import minidom

import yaml
import numpy as np


# =========================
# 路径配置
# =========================
BASE_DIR = os.path.dirname(__file__)

INITIAL_YAML = os.path.join(BASE_DIR, "initial.yaml")
SCENE_TEMPLATE_XML = os.path.join(BASE_DIR, "scene_template.xml")
SCENE_OUTPUT_XML = os.path.join(BASE_DIR, "scene.xml")


# =========================
# 场景放置参数
# =========================
# 你的桌子在 scene_template.xml 里是:
# <body name="table" pos="0.40 0 0.02">
# <geom type="box" size="0.30 0.50 0.02" />
#
# 所以桌面上表面 z 大约是:
TABLE_TOP_Z = 0.04


BRICK_HALF_HEIGHT = 0.0191 / 2.0
DESIRED_PARTS_CENTER = np.array(
    [0.52, -0.12, TABLE_TOP_Z + BRICK_HALF_HEIGHT + 0.002],
    dtype=float
)
world_z = TABLE_TOP_Z + BRICK_HALF_HEIGHT


# 如果你想让积木再整体往右一点 / 往前一点，改这里
EXTRA_OFFSET = np.array([0.00, 0.00, 0.00], dtype=float)


# =========================
# 物理/几何参数
# =========================
DENSITY = 400
FRICTION = "3.0 0.1 0.01"
SOLREF = "0.001 1"
SOLIMP = "0.999 0.9999 0.0001"
# 视觉 mesh 是否参与碰撞
VISUAL_CONTYPE = "0"
VISUAL_CONAFFINITY = "0"


# =========================
# 颜色映射
# =========================
COLOR_MAP = {
    "red": "1 0 0 1",
    "green": "0 1 0 1",
    "yellow": "1 1 0 1",
    "blue": "0 0 1 1",
    "gray": "0.5 0.5 0.5 1",
    "grey": "0.5 0.5 0.5 1",
    "white": "1 1 1 1",
    "black": "0.1 0.1 0.1 1",
}


# =========================
# 类型到 mesh / collision box 映射
# 注意：MuJoCo box 的 size 是半尺寸
# =========================
BRICK_SPECS = {
    "brick_2x2": {
        "mesh": "lego_2x2",
        "box_half_size": np.array([0.016, 0.016, BRICK_HALF_HEIGHT], dtype=float),
    },
    "brick_4x2": {
        "mesh": "lego_2x4",
        "box_half_size": np.array([0.032, 0.016, BRICK_HALF_HEIGHT], dtype=float),
    },
}

# =========================
# 工具函数
# =========================
def prettify_xml(elem: ET.Element) -> str:
    rough = ET.tostring(elem, encoding="utf-8")
    parsed = minidom.parseString(rough)
    return parsed.toprettyxml(indent="  ")


def load_yaml(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def parse_xml(path: str) -> ET.ElementTree:
    return ET.parse(path)


def get_worldbody(root: ET.Element) -> ET.Element:
    wb = root.find("worldbody")
    if wb is None:
        raise RuntimeError("scene_template.xml 中没有找到 <worldbody>")
    return wb


def find_and_remove_old_dynamic_bricks(worldbody: ET.Element) -> None:
    """
    删除以前生成的动态积木 body。
    只删名字里带 brick 的 body，保留 table / panda / floor 等。
    """
    to_remove = []
    for child in list(worldbody):
        if child.tag != "body":
            continue
        name = child.get("name", "")
        if "brick" in name:
            to_remove.append(child)

    for child in to_remove:
        worldbody.remove(child)


def quat_from_yaw(yaw_rad: float) -> str:
    """
    MuJoCo quat 顺序: w x y z
    这里只处理绕 z 轴旋转
    """
    w = math.cos(yaw_rad / 2.0)
    z = math.sin(yaw_rad / 2.0)
    return f"{w:.10f} 0 0 {z:.10f}"


def get_rgba(color_name: str) -> str:
    return COLOR_MAP.get(color_name.lower(), "0.7 0.7 0.7 1")


def local_to_world(block_pos, yaml_parts_center) -> np.ndarray:
    """
    把 initial.yaml 里的局部坐标，映射到 MuJoCo 桌子右侧区域
    """
    local = np.array(block_pos, dtype=float)
    world = local - yaml_parts_center + DESIRED_PARTS_CENTER + EXTRA_OFFSET
    return world


def create_brick_body(block: dict, yaml_parts_center: np.ndarray) -> ET.Element:
    """
    为一个 block 创建:
    - freejoint body
    - visual mesh geom
    - transparent collision box geom
    """
    name = block["name"]
    brick_type = block["type"]
    color = block.get("color", "gray")
    pos = block["pos"]
    rotation = block.get("rotation", [0.0, 0.0, 0.0])

    if brick_type not in BRICK_SPECS:
        raise ValueError(f"未知积木类型: {brick_type}")

    spec = BRICK_SPECS[brick_type]
    mesh_name = spec["mesh"]
    box_half = spec["box_half_size"]
    rgba = get_rgba(color)

    yaw = float(rotation[2])
    world_pos = local_to_world(pos, yaml_parts_center)

    # MuJoCo 中 mesh 原点和 STL 原点有时不在理想位置
    # 这里先不额外补 z
    # 如果你后面发现砖块轻微陷桌，可改 world_pos[2] += 0.002
    quat = quat_from_yaw(yaw)

    body = ET.Element("body")
    body.set("name", name)
    body.set("pos", f"{world_pos[0]:.4f} {world_pos[1]:.4f} {world_pos[2]:.4f}")
    body.set("quat", quat)

    freejoint = ET.SubElement(body, "freejoint")

    # visual geom: STL mesh
    geom_visual = ET.SubElement(body, "geom")
    geom_visual.set("type", "mesh")
    geom_visual.set("mesh", mesh_name)
    geom_visual.set("rgba", rgba)
    geom_visual.set("contype", VISUAL_CONTYPE)
    geom_visual.set("conaffinity", VISUAL_CONAFFINITY)

    # collision geom: transparent box
    geom_collision = ET.SubElement(body, "geom")
    geom_collision.set("type", "box")
    geom_collision.set("size", f"{box_half[0]:.4f} {box_half[1]:.4f} {box_half[2]:.4f}")
    geom_collision.set("rgba", "1 0 0 0.35")
    geom_collision.set("density", str(DENSITY))
    geom_collision.set("friction", FRICTION)
    geom_collision.set("solref", SOLREF)
    geom_collision.set("solimp", SOLIMP)
    geom_collision.set("pos", "0 0 -0.009")

    return body


def add_duplo_collision_geoms(body, brick_type):
    if brick_type == "brick_2x2":
        nx, ny = 2, 2
        half_x, half_y = 0.016, 0.016
    elif brick_type == "brick_4x2":
        nx, ny = 4, 2
        half_x, half_y = 0.032, 0.016
    else:
        raise ValueError(f"未知积木类型: {brick_type}")

    body_half_z = 0.0095
    stud_radius = 0.0048
    stud_half_height = 0.0022
    stud_pitch = 0.016

    # 1. 主体 collision
    geom_body = ET.SubElement(body, "geom")
    geom_body.set("type", "box")
    geom_body.set("size", f"{half_x:.4f} {half_y:.4f} {body_half_z:.4f}")
    geom_body.set("pos", "0 0 0")
    geom_body.set("rgba", "0 0 0 0")
    geom_body.set("density", str(DENSITY))
    geom_body.set("friction", FRICTION)
    geom_body.set("solref", SOLREF)
    geom_body.set("solimp", SOLIMP)

    # 2. 顶部 studs collision
    start_x = -(nx - 1) * stud_pitch / 2.0
    start_y = -(ny - 1) * stud_pitch / 2.0
    stud_z = body_half_z + stud_half_height

    for ix in range(nx):
        for iy in range(ny):
            x = start_x + ix * stud_pitch
            y = start_y + iy * stud_pitch

            geom_stud = ET.SubElement(body, "geom")
            geom_stud.set("type", "cylinder")
            geom_stud.set("size", f"{stud_radius:.4f} {stud_half_height:.4f}")
            geom_stud.set("pos", f"{x:.4f} {y:.4f} {stud_z:.4f}")
            geom_stud.set("rgba", "0 0 0 0")
            geom_stud.set("density", "50")
            geom_stud.set("friction", FRICTION)
            geom_stud.set("solref", SOLREF)
            geom_stud.set("solimp", SOLIMP)

    # 3. 底部凹槽近似：先用一个轻微抬高的底部边框近似
    # 真实 hollow cavity 很难用负几何表示，MuJoCo 没有“减去一个洞”的 primitive。
    # 所以这里用边框 box 近似底部接触。

def build(initial_yaml: str = INITIAL_YAML,
          template_xml: str = SCENE_TEMPLATE_XML,
          output_xml: str = SCENE_OUTPUT_XML) -> str:
    """
    读取 initial.yaml + scene_template.xml
    生成新的 scene.xml
    """
    if not os.path.exists(initial_yaml):
        raise FileNotFoundError(f"找不到 initial.yaml: {initial_yaml}")
    if not os.path.exists(template_xml):
        raise FileNotFoundError(f"找不到 scene_template.xml: {template_xml}")

    data = load_yaml(initial_yaml)
    tree = parse_xml(template_xml)
    root = tree.getroot()
    worldbody = get_worldbody(root)

    find_and_remove_old_dynamic_bricks(worldbody)

    parts_plate = data.get("parts_plate", {})
    yaml_parts_center = np.array(parts_plate.get("center", [0.0, 0.0, 0.0]), dtype=float)

    blocks = data.get("blocks", [])
    for block in blocks:
        brick_body = create_brick_body(block, yaml_parts_center)
        worldbody.append(brick_body)

    xml_text = prettify_xml(root)

    with open(output_xml, "w", encoding="utf-8") as f:
        f.write(xml_text)

    print("scene generated")
    print(f"template : {template_xml}")
    print(f"initial  : {initial_yaml}")
    print(f"output   : {output_xml}")
    print(f"blocks   : {len(blocks)}")
    print(f"parts center mapped to world: {DESIRED_PARTS_CENTER.tolist()}")

    return output_xml


if __name__ == "__main__":
    build()