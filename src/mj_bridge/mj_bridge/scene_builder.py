# scene_builder.py
from __future__ import annotations

import math
import os
import xml.etree.ElementTree as ET
from xml.dom import minidom

import yaml
import numpy as np


# ============================================================
# 1. 文件路径
# ============================================================

BASE_DIR = os.path.dirname(__file__)

INITIAL_YAML = os.path.join(BASE_DIR, "initial.yaml")
SCENE_TEMPLATE_XML = os.path.join(BASE_DIR, "scene_template.xml")
SCENE_OUTPUT_XML = os.path.join(BASE_DIR, "scene.xml")


# ============================================================
# 2. 桌子与积木高度
# ============================================================

# 你的 scene_template.xml 里桌子一般是：
# <body name="table" pos="0.40 0 0.02">
#   <geom type="box" size="0.30 0.50 0.02"/>
# </body>
#
# MuJoCo box size 是半尺寸，所以桌面上表面：
# table center z 0.02 + table half height 0.02 = 0.04
TABLE_TOP_Z = 0.04

# 积木主体高度近似 0.019m，所以半高约 0.0095m
BRICK_BODY_HALF_HEIGHT = 0.0095

# 顶部凸点半高
STUD_HALF_HEIGHT = 0.0020

# 积木 body 的中心高度。
# 这样主体底面刚好在桌面上：
# 0.04 + 0.0095 = 0.0495
BRICK_CENTER_Z = TABLE_TOP_Z + BRICK_BODY_HALF_HEIGHT

# 把 initial.yaml 里的 parts_plate.center 映射到 MuJoCo 桌子上的位置
DESIRED_PARTS_CENTER = np.array([0.52, -0.12, BRICK_CENTER_Z], dtype=float)

# 整体平移微调。如果积木整体位置不合适，改这里。
EXTRA_OFFSET = np.array([0.00, 0.00, 0.00], dtype=float)


# ============================================================
# 3. collision 高低微调
# ============================================================

# 这个参数控制 collision 相对 STL visual 的上下偏移。
#
# 负数：collision 往下
# 正数：collision 往上
#
# 如果你觉得 collision 太高，让积木看起来“悬空”，用 -0.001 / -0.002。
# 如果积木陷进桌子，应该用正数，比如 0.001 / 0.002。
COLLISION_Z_OFFSET = -0.009


# ============================================================
# 4. 物理参数
# ============================================================

# 积木密度。太大容易把接触压得更深，太小容易被夹爪碰飞。
DENSITY = 150

# MuJoCo friction 三个数：
# 1. sliding friction 滑动摩擦
# 2. torsional friction 扭转摩擦
# 3. rolling friction 滚动摩擦
FRICTION = "5.0 0.2 0.02"

# 接触参数。
# SOLREF 越小，接触越硬；太小容易抖。
# 这里选择稍微容易插入的参数。
SOLREF = "0.004 1"
SOLIMP = "0.90 0.98 0.002"

# STL 视觉 mesh 不参与碰撞。
# 碰撞由我们自己添加的 box + cylinder 完成。
VISUAL_CONTYPE = "0"
VISUAL_CONAFFINITY = "0"


# ============================================================
# 5. Duplo 几何参数
# ============================================================

# 你的 generator 里 STUD = 0.016，所以这里保持一致
STUD_PITCH = 0.016

# ============================================================
# 装配区 12x12 Duplo 底板
# ============================================================

ADD_ASSEMBLY_BASE_PLATE = True

ASSEMBLY_BASE_NAME = "assembly_base_plate"

# 你的 C++ 里装配区中心是:
# ASSEMBLY_ORIGIN_X = 0.25
# ASSEMBLY_ORIGIN_Y = 0.0
ASSEMBLY_BASE_CENTER_X = 0.35
ASSEMBLY_BASE_CENTER_Y = 0.35

BASE_PLATE_STUDS_X = 12
BASE_PLATE_STUDS_Y = 12

# 12 * 0.016 = 0.192 m
BASE_PLATE_HALF_X = BASE_PLATE_STUDS_X * STUD_PITCH / 2.0
BASE_PLATE_HALF_Y = BASE_PLATE_STUDS_Y * STUD_PITCH / 2.0

# 底板厚度
BASE_PLATE_THICKNESS = 0.006
BASE_PLATE_HALF_HEIGHT = BASE_PLATE_THICKNESS / 2.0

# 桌面高度是 0.04
BASE_PLATE_CENTER_Z = TABLE_TOP_Z + BASE_PLATE_HALF_HEIGHT

# stud 参数
BASE_STUD_RADIUS = 0.0042
BASE_STUD_HALF_HEIGHT = 0.0020

# stud 的局部 z 坐标：
# 底板中心为 0，底板上表面是 +0.003
# stud 半高 0.002，所以 stud 中心是 0.003 + 0.002 = 0.005
BASE_STUD_CENTER_Z_LOCAL = BASE_PLATE_HALF_HEIGHT + BASE_STUD_HALF_HEIGHT

# 顶部凸点半径。
# 稍微小一点，更容易插入。
STUD_RADIUS = 0.0042

# 每种积木对应：
# mesh: scene_template.xml asset 里定义的 mesh name
# studs_x / studs_y: 顶部凸点数量
# body_half_size: 主体 collision box 半尺寸
BRICK_SPECS = {
    "brick_2x2": {
        "mesh": "lego_2x2",
        "studs_x": 2,
        "studs_y": 2,
        "body_half_size": np.array([0.016, 0.016, BRICK_BODY_HALF_HEIGHT], dtype=float),
    },
    "brick_4x2": {
        "mesh": "lego_2x4",
        "studs_x": 4,
        "studs_y": 2,
        "body_half_size": np.array([0.032, 0.016, BRICK_BODY_HALF_HEIGHT], dtype=float),
    },
}


# ============================================================
# 6. 颜色
# ============================================================

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


# ============================================================
# 7. 基础工具函数
# ============================================================

def prettify_xml(elem: ET.Element) -> str:
    """把 XML 元素格式化成可读文本。"""
    rough = ET.tostring(elem, encoding="utf-8")
    parsed = minidom.parseString(rough)
    return parsed.toprettyxml(indent="  ")


def load_yaml(path: str) -> dict:
    """读取 yaml 文件。"""
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def parse_xml(path: str) -> ET.ElementTree:
    """读取 xml 文件。"""
    return ET.parse(path)


def get_worldbody(root: ET.Element) -> ET.Element:
    """找到 scene_template.xml 里的 <worldbody>。"""
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("scene_template.xml 中没有找到 <worldbody>")
    return worldbody


def find_and_remove_old_dynamic_bricks(worldbody: ET.Element) -> None:
    """
    删除旧的动态积木 body 和旧装配底板。
    避免每次 build() 都重复添加。
    """
    to_remove = []

    for child in list(worldbody):
        if child.tag != "body":
            continue

        name = child.get("name", "")

        if "brick" in name:
            to_remove.append(child)

        if name == ASSEMBLY_BASE_NAME:
            to_remove.append(child)

    for child in to_remove:
        worldbody.remove(child)

def quat_from_yaw(yaw_rad: float) -> str:
    """
    MuJoCo quat 顺序是 w x y z。
    这里只处理绕 z 轴旋转。
    """
    w = math.cos(yaw_rad / 2.0)
    z = math.sin(yaw_rad / 2.0)
    return f"{w:.10f} 0 0 {z:.10f}"


def get_rgba(color_name: str) -> str:
    """把颜色名转成 MuJoCo rgba 字符串。"""
    return COLOR_MAP.get(color_name.lower(), "0.7 0.7 0.7 1")


def local_to_world(block_pos, yaml_parts_center) -> np.ndarray:
    """
    把 initial.yaml 里的局部坐标映射到 MuJoCo 世界坐标。

    initial.yaml 中的 block.pos 是相对 benchmark plate 的坐标。
    这里把它平移到桌面上的 DESIRED_PARTS_CENTER 附近。
    """
    local = np.array(block_pos, dtype=float)

    world = local - yaml_parts_center + DESIRED_PARTS_CENTER + EXTRA_OFFSET

    return world


# ============================================================
# 8. collision 创建
# ============================================================

def set_common_collision_params(geom: ET.Element, density: float | None = None) -> None:
    """给 collision geom 设置通用物理参数。"""
    geom.set("rgba", "0 0 0 0")
    geom.set("friction", FRICTION)
    geom.set("solref", SOLREF)
    geom.set("solimp", SOLIMP)

    if density is not None:
        geom.set("density", str(density))


def add_duplo_collision_geoms(body: ET.Element, brick_type: str) -> None:
    """
    给一个积木添加 collision。

    这里使用：
    1. 主体 box collision
    2. 顶部 stud cylinder collision

    这样比普通单个 box 更像 Duplo。
    底部凹槽暂时不做真实空洞，因为 MuJoCo primitive 不支持布尔减法。
    """

    if brick_type not in BRICK_SPECS:
        raise ValueError(f"未知积木类型: {brick_type}")

    spec = BRICK_SPECS[brick_type]

    body_half = spec["body_half_size"]
    studs_x = spec["studs_x"]
    studs_y = spec["studs_y"]

    # ---------- 1. 主体 box collision ----------
    geom_body = ET.SubElement(body, "geom")
    geom_body.set("type", "box")

    # MuJoCo box size 是半尺寸
    geom_body.set(
        "size",
        f"{body_half[0]:.4f} {body_half[1]:.4f} {body_half[2]:.4f}",
    )

    # 调 collision 高低就在这里
    geom_body.set(
        "pos",
        f"0 0 {COLLISION_Z_OFFSET:.4f}",
    )

    set_common_collision_params(geom_body, density=DENSITY)

    # ---------- 2. 顶部 stud cylinder collision ----------
    start_x = -(studs_x - 1) * STUD_PITCH / 2.0
    start_y = -(studs_y - 1) * STUD_PITCH / 2.0

    stud_center_z = BRICK_BODY_HALF_HEIGHT + STUD_HALF_HEIGHT

    for ix in range(studs_x):
        for iy in range(studs_y):
            x = start_x + ix * STUD_PITCH
            y = start_y + iy * STUD_PITCH

            geom_stud = ET.SubElement(body, "geom")
            geom_stud.set("type", "cylinder")

            # cylinder size: radius half-height
            geom_stud.set(
                "size",
                f"{STUD_RADIUS:.4f} {STUD_HALF_HEIGHT:.4f}",
            )

            # stud 也跟着 COLLISION_Z_OFFSET 一起移动
            geom_stud.set(
                "pos",
                f"{x:.4f} {y:.4f} {stud_center_z + COLLISION_Z_OFFSET:.4f}",
            )

            # stud 密度给小一点，避免多个 stud 把质量加太多
            set_common_collision_params(geom_stud, density=30)


def create_assembly_base_plate() -> ET.Element:
    """
    创建装配区中心的 12x12 Duplo 底板。
    它是固定 body，没有 freejoint。
    包含：
    1. 一个 box 主体
    2. 12x12 个 cylinder stud
    """

    body = ET.Element("body")
    body.set("name", ASSEMBLY_BASE_NAME)
    body.set(
        "pos",
        f"{ASSEMBLY_BASE_CENTER_X:.4f} {ASSEMBLY_BASE_CENTER_Y:.4f} {BASE_PLATE_CENTER_Z:.4f}",
    )

    # ---------- 底板主体 ----------
    geom_base = ET.SubElement(body, "geom")
    geom_base.set("name", "assembly_base_plate_body")
    geom_base.set("type", "box")
    geom_base.set(
        "size",
        f"{BASE_PLATE_HALF_X:.4f} {BASE_PLATE_HALF_Y:.4f} {BASE_PLATE_HALF_HEIGHT:.4f}",
    )
    geom_base.set("rgba", "0.12 0.12 0.12 1")
    geom_base.set("friction", FRICTION)
    geom_base.set("solref", SOLREF)
    geom_base.set("solimp", SOLIMP)

    # ---------- 12x12 studs ----------
    start_x = -(BASE_PLATE_STUDS_X - 1) * STUD_PITCH / 2.0
    start_y = -(BASE_PLATE_STUDS_Y - 1) * STUD_PITCH / 2.0

    for ix in range(BASE_PLATE_STUDS_X):
        for iy in range(BASE_PLATE_STUDS_Y):
            x = start_x + ix * STUD_PITCH
            y = start_y + iy * STUD_PITCH

            geom_stud = ET.SubElement(body, "geom")
            geom_stud.set("name", f"assembly_base_stud_{ix}_{iy}")
            geom_stud.set("type", "cylinder")
            geom_stud.set(
                "size",
                f"{BASE_STUD_RADIUS:.4f} {BASE_STUD_HALF_HEIGHT:.4f}",
            )
            geom_stud.set(
                "pos",
                f"{x:.4f} {y:.4f} {BASE_STUD_CENTER_Z_LOCAL:.4f}",
            )
            geom_stud.set("rgba", "0.12 0.12 0.12 1")
            geom_stud.set("friction", FRICTION)
            geom_stud.set("solref", SOLREF)
            geom_stud.set("solimp", SOLIMP)

    return body

# ============================================================
# 9. 创建单个积木 body
# ============================================================

def create_brick_body(block: dict, yaml_parts_center: np.ndarray) -> ET.Element:
    """
    根据 initial.yaml 里的一个 block 创建 MuJoCo body。

    body 里包含：
    1. freejoint：让积木能被物理仿真移动
    2. visual mesh：负责显示 STL
    3. collision geoms：负责物理碰撞
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
    rgba = get_rgba(color)

    yaw = float(rotation[2])
    world_pos = local_to_world(pos, yaml_parts_center)
    quat = quat_from_yaw(yaw)

    body = ET.Element("body")
    body.set("name", name)
    body.set(
        "pos",
        f"{world_pos[0]:.4f} {world_pos[1]:.4f} {world_pos[2]:.4f}",
    )
    body.set("quat", quat)

    # freejoint 让这个积木成为自由物体
    ET.SubElement(body, "freejoint")

    # ---------- visual mesh ----------
    geom_visual = ET.SubElement(body, "geom")
    geom_visual.set("type", "mesh")
    geom_visual.set("mesh", mesh_name)
    geom_visual.set("rgba", rgba)

    # 视觉 STL 不参与碰撞
    geom_visual.set("contype", VISUAL_CONTYPE)
    geom_visual.set("conaffinity", VISUAL_CONAFFINITY)

    # ---------- collision ----------
    add_duplo_collision_geoms(body, brick_type)

    return body


# ============================================================
# 10. 生成 scene.xml
# ============================================================

def build(
    initial_yaml: str = INITIAL_YAML,
    template_xml: str = SCENE_TEMPLATE_XML,
    output_xml: str = SCENE_OUTPUT_XML,
) -> str:
    """
    读取：
    1. initial.yaml
    2. scene_template.xml

    输出：
    scene.xml
    """

    if not os.path.exists(initial_yaml):
        raise FileNotFoundError(f"找不到 initial.yaml: {initial_yaml}")

    if not os.path.exists(template_xml):
        raise FileNotFoundError(f"找不到 scene_template.xml: {template_xml}")

    data = load_yaml(initial_yaml)

    tree = parse_xml(template_xml)
    root = tree.getroot()
    worldbody = get_worldbody(root)

    # 删除旧积木，避免重复生成
    find_and_remove_old_dynamic_bricks(worldbody)

    if ADD_ASSEMBLY_BASE_PLATE:
        base_plate = create_assembly_base_plate()
        worldbody.append(base_plate)

    # 读取 parts_plate.center
    parts_plate = data.get("parts_plate", {})
    yaml_parts_center = np.array(
        parts_plate.get("center", [0.0, 0.0, 0.0]),
        dtype=float,
    )

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
    print(f"collision z offset: {COLLISION_Z_OFFSET}")
    print(f"brick body half height: {BRICK_BODY_HALF_HEIGHT}")
    print(f"stud radius: {STUD_RADIUS}")
    print(f"friction: {FRICTION}")
    print(f"solref: {SOLREF}")
    print(f"solimp: {SOLIMP}")

    return output_xml


if __name__ == "__main__":
    build()