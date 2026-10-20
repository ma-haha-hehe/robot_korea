#!/usr/bin/python3
# -*- coding: utf-8 -*-

import numpy as np
import yaml
import os
import copy
import argparse
import math
from scipy.spatial.transform import Rotation as R

# 禁用 YAML 锚点
yaml.Dumper.ignore_aliases = lambda *args: True

# ================= 1. 配置区域 =================
THRESHOLD = 0.045       # 避障探测距离 (4.5cm)
TABLE_HEIGHT = 0.42     # 桌面高度
ASSEMBLY_CENTER = np.array([0.0, 0.0, 0.0])

# 原材料区位置
SOURCE_LOCATIONS = {
    "base_4x2_lvl1":    [0.55,  0.25],
    "support_2x2_left": [0.55,  0.10],
    "support_2x2_right":[0.55, -0.10],
    "top_4x2_lvl4":     [0.62,  0.00]
}

def to_native(obj):
    if isinstance(obj, (np.integer, int)): return int(obj)
    elif isinstance(obj, (np.floating, float)): return float(obj)
    elif isinstance(obj, (np.ndarray, list)): return [to_native(x) for x in obj]
    else: return obj

def get_strategy_quaternion(spin_deg):
    base_rot = R.from_euler('x', 180, degrees=True)
    strategy_rot = R.from_euler('z', spin_deg, degrees=True)
    return to_native((strategy_rot * base_rot).as_quat().tolist())


def infer_vision_label(block):
    for key in ("vision_label", "vision_name", "target"):
        if block.get(key):
            return str(block[key]).strip()

    name = str(block.get("name", "")).strip()
    block_type = str(block.get("type", "")).lower()
    text = f"{name.lower()} {block_type}"
    if "4x2" in text or "2x4" in text:
        size = "2x4"
    elif "2x2" in text:
        size = "2x2"
    else:
        size = "lego"

    color = str(block.get("color", "")).strip()
    if color:
        return f"{color} {size} brick."
    if " " in name:
        return name if name.endswith(".") else name + "."
    return f"{size} brick."

# ================= 2. 核心判定逻辑 (带详细推理打印) =================

def check_accessibility(test_pos, scene_blocks, self_name, check_mode=90):
    tx, ty, tz = test_pos
    mode_str = "90度(探测X轴轨道)" if check_mode == 90 else "0度(探测Y轴轨道)"
    
    for blk in scene_blocks:
        if blk['name'] == self_name: continue
        ox, oy, oz = blk['abs_pos']
        
        # 只检查同一层 (dz < 1cm)
        if abs(tz - oz) < 0.01:
            dx = abs(tx - ox)
            dy = abs(ty - oy)

            if check_mode == 0:
                # 0度抓：检查 Y 轴轨道 (要求 X 坐标相同)
                if dx < 0.005:#在同一条纵向线上
                    if dy < THRESHOLD:
                        print(f"[ 碰撞] {mode_str} 轨道受阻！邻居: {blk['name']} | 距离 dy={dy:.3f} < {THRESHOLD}")
                        return False
            elif check_mode == 90:
                # 90度抓：检查 X 轴轨道 (要求 Y 坐标相同)
                if dy < 0.005:#同一条横向线上
                    if dx < THRESHOLD:
                        print(f" [碰撞] {mode_str} 轨道受阻！邻居: {blk['name']} | 距离 dx={dx:.3f} < {THRESHOLD}")
                        return False
    
    print(f"      [ 安全] {mode_str} 路径清空")
    return True

def is_covered(target, remaining_blocks):
    tx, ty, tz = target['abs_pos']
    for blk in remaining_blocks:
        if blk == target: continue
        ox, oy, oz = blk['abs_pos']
        if oz > (tz + 0.01) and abs(tx - ox) < 0.016 and abs(ty - oy) < 0.016:
            return True
    return False

def check_balance(target, remaining_blocks):
    if target['pos'][2] <= 0.005: return True
    tx, ty, tz = target['abs_pos']
    for blk in remaining_blocks:
        if blk == target: continue
        ox, oy, oz = blk['abs_pos']
        if abs(tz - (oz + 0.01)) < 0.005:
            if abs(tx - ox) < 0.0165 and abs(ty - oy) < 0.0165:
                return True 
    return False

# ================= 3. 主规划程序 =================

def process_blueprint(input_yaml, output_yaml):
    """Export the shared assembly-by-disassembly plan in the legacy task format."""
    from mj_bridge.benchmark_core import normalize_product
    from mj_bridge.assembly_planner import plan_assembly
    with open(input_yaml, 'r') as stream:
        product = normalize_product(yaml.safe_load(stream))
    blocks = {block['id']: block for block in product['blocks']}
    targets = [dict(id=block['id'], type=block['type'],
                    position=block['target']['position'],
                    yaw_rad=math.radians(block['target']['yaw_deg']))
               for block in product['blocks']]
    plan = plan_assembly(targets)
    tasks = []
    for index, step in enumerate(plan):
        block = blocks[step['block_id']]
        spin = step['grasp_spin_deg']
        src_xy = SOURCE_LOCATIONS.get(block['id'], [0.5, 0.0])
        tasks.append({
            'id': index, 'name': block['id'], 'type': block['type'],
            'color': block['color'],
            'vision_label': infer_vision_label(dict(block, name=block['id'])),
            'grasp_spin': spin,
            'blueprint_yaw': math.radians(block['target']['yaw_deg']),
            'grasp_offset_rad': step['grasp_offset_rad'],
            'pick': {'pos': [src_xy[0], src_xy[1], .015],
                     'orientation': get_strategy_quaternion(spin)},
            'place': {'pos': block['target']['position'],
                      'orientation': get_strategy_quaternion(spin)},
        })
    with open(output_yaml, 'w') as stream:
        yaml.safe_dump({'tasksh': tasks}, stream, sort_keys=False)
    print(f'Assembly-by-disassembly plan written to {output_yaml}')

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "input_yaml",
        nargs="?",
        default="simple_shape.yaml",
        help="final product YAML with a blocks: list",
    )
    parser.add_argument(
        "output_yaml",
        nargs="?",
        default="plan.yaml",
        help="planner output YAML consumed by vision/execution",
    )
    args = parser.parse_args()
    process_blueprint(args.input_yaml, args.output_yaml)
