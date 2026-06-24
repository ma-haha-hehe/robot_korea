#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import torch
import numpy as np
import pyrealsense2 as rs
import os
import yaml
import trimesh
import time
import sys
from PIL import Image
from dataclasses import dataclass
from typing import Optional
from transformers import pipeline
from scipy.spatial.transform import Rotation as R

# ================= 1. 路径与环境配置 =================
FP_REPO = "/FoundationPose"
RESULT_FILE = "/shared_data/active_task.yaml"
TASKS_YAML = "/vision_code/task_test_flower.yaml"
CAMERA_PARAMS_YAML = "/vision_code/camera_params.yaml"
MESH_DIR = "/FoundationPose/meshes"
ASSEMBLY_CENTER_BASE = np.array([0.35, 0.2, 0.025])

ROBOT_READY_YAW_OFFSET = -45

if FP_REPO not in sys.path:
    sys.path.append(FP_REPO)
    sys.path.append(os.path.join(FP_REPO, "root"))

from estimater import FoundationPose, ScorePredictor, PoseRefinePredictor

try:
    from segment_anything import sam_model_registry, SamPredictor
    SAM_AVAILABLE = True
except ImportError:
    SAM_AVAILABLE = False

# 定义颜色的理想中心坐标 (Hue, Saturation, Value)
COLOR_CENTERS = {
    "red":    (0, 200, 150),
    "red_alt": (175, 200, 150),
    "blue":   (115, 200, 150),
    "green":  (60, 200, 120),
    "yellow": (30, 200, 200),
    "white":  (0, 30, 230),  # 调高一点 S，给环境光留余地
    "black":  (0, 0, 40)
}
def clear_buffer(self):
        for _ in range(30): self.pipeline.wait_for_frames()

def get_color_distance(c1, c2):
    """计算两个 HSV 颜色之间的欧几里得距离，并考虑 Hue 的循环性"""
    h1, s1, v1 = c1
    h2, s2, v2 = c2
    # Hue 是圆形的 (0 和 180 是相连的)
    dh = min(abs(h1 - h2), 180 - abs(h1 - h2))
    ds = s1 - s2
    dv = v1 - v2
    return np.sqrt(dh**2 + ds**2 + dv**2)

def classify_color(avg_hsv):
    """判定当前 HSV 颜色距离哪个预设中心最近"""
    min_dist = float('inf')
    best_color = "unknown"
    
    for color_name, center in COLOR_CENTERS.items():
        dist = get_color_distance(avg_hsv, center)
        if dist < min_dist:
            min_dist = dist
            best_color = color_name
            
    # 如果最接近红色的两个中心之一，都归类为红色
    if best_color == "red_alt": best_color = "red"
    return best_color

@dataclass
class BoundingBox:
    xmin: int; ymin: int; xmax: int; ymax: int
    @property
    def xyxy(self): return [self.xmin, self.ymin, self.xmax, self.ymax]

@dataclass
class DetectionResult:
    score: float; label: str; box: BoundingBox

# ================= 2. 核心估计类 =================
class LegoPoseEstimator:
    def __init__(self, scorer, refiner):
        self.scorer = scorer
        self.refiner = refiner

    def update_mesh(self, mesh_path):
        self.mesh = trimesh.load(mesh_path)
        if np.linalg.norm(self.mesh.extents) > 0.1:
            self.mesh.apply_scale(0.001)
        self.mesh.vertices -= self.mesh.bounds.mean(axis=0)
        model_pts, _ = trimesh.sample.sample_surface(self.mesh, 2048)
        self.model_pts = torch.from_numpy(model_pts.astype(np.float32)).cuda()
        self.estimator = FoundationPose(
            model_pts=self.model_pts, model_normals=None, mesh=self.mesh,
            scorer=self.scorer, refiner=self.refiner
        )

# ================= 3. 自动化视觉节点类 =================
class RobotVisionNode:
    def __init__(self):
        with open(CAMERA_PARAMS_YAML, 'r') as f:
            params = yaml.safe_load(f)
        self.T_base_camera = np.array(params['extrinsic_matrix']).reshape(4, 4)
        with open(TASKS_YAML, 'r') as f:
            data = yaml.safe_load(f)
            self.task_list = data.get('tasksh', data.get('tasks', []))

        self.init_realsense()
        self.detector = pipeline(model="IDEA-Research/grounding-dino-tiny", task="zero-shot-object-detection", device="cuda")
        if SAM_AVAILABLE:
            sam = sam_model_registry["vit_h"](checkpoint="/FoundationPose/weights/sam_vit_h_4b8939.pth").to("cuda")
            self.sam_predictor = SamPredictor(sam)

        self.pose_est = LegoPoseEstimator(ScorePredictor(), PoseRefinePredictor())

    def init_realsense(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        profile = self.pipeline.start(config)
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.K_MATRIX = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]])
        self.align = rs.align(rs.stream.color)

    def clear_buffer(self):
        for _ in range(30):
            self.pipeline.wait_for_frames()
    def run(self):
        # ==================== 新增：纯净相机预览与截图功能 ====================
        print("📸 进入纯净预览模式...")
        print("操作说明: [空格键] 拍照保存 | [q] 退出预览并开始执行 AI 任务")
        
        while True:
            # 获取画面
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            if not color_frame: continue
            
            # 转换为 OpenCV 格式
            preview_img = np.asanyarray(color_frame.get_data())
            
            # 显示纯净画面（没有任何标签和框）
            cv2.imshow("PURE CAMERA VIEW - PRESS SPACE TO SCAN", preview_img)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord(' '): # 按空格键截图
                photo_path = f"/vision_code/screenshot_{int(time.time())}.png"
                cv2.imwrite(photo_path, preview_img)
                print(f"✅ 已截图并保存至: {photo_path}")
                # 在画面上闪烁提示一下“已保存”
                cv2.putText(preview_img, "SAVED!", (250, 240), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
                cv2.imshow("PURE CAMERA VIEW - PRESS SPACE TO SCAN", preview_img)
                cv2.waitKey(500) 
                
            elif key == ord('q'): # 按 q 键退出预览
                cv2.destroyWindow("PURE CAMERA VIEW - PRESS SPACE TO SCAN")
                print("🚀 退出预览，开始执行 AI 算法...")
                break
        # ====================================================================
        """
        全流程视觉控制逻辑：将调试标签从通用的 "Lego" 改为 YAML 中的具体任务名称。
        """
        # 比例容差
        RATIO_TOLERANCE = 0.4 
        
        for task in self.task_list:
            # 获取当前任务的真实名称，例如 "yellow_4x2_brick"
            name = task['name']
            
            # --- 解析任务要求的颜色 ---
            target_color_name = None
            for color_key in ["red", "blue", "green", "yellow", "white", "black"]:
                if color_key in name.lower():
                    target_color_name = color_key
                    break

            task_success = False
            blacklist = [] 

            while not task_success:
                # 0. 任务同步
                while os.path.exists(RESULT_FILE):
                    print(f"⏳ [{name}] 等待机器人拿走上一块积木...")
                    time.sleep(0.5)

                print(f"\n🎯 [新一轮扫描] 寻找目标: {name}")
                
                # 更新 3D 模型
                mesh_path = self.get_mesh_path(name)
                self.pose_est.update_mesh(mesh_path)
                target_ratio = 2.0 if ("4x2" in name or "2x4" in name) else 1.0

                # --- 1. 图像采集 ---
                self.clear_buffer()
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)
                img_bgr = np.asanyarray(aligned_frames.get_color_frame().get_data())
                img_pil = Image.fromarray(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB))
                
                # --- 2. GroundingDINO 检测阶段 ---
                print(f"🔍 [步骤 1] 正在全图搜索: {name}")
                results = self.detector(img_pil, candidate_labels=["lego block"], threshold=0.15)
                
                debug_dino = img_bgr.copy()
                for r in results:
                    b = r['box']
                    # 【核心修改点】将标签改为当前任务的 name
                    target_label = f"GOAL: {name} ({r['score']:.2f})"
                    
                    cv2.rectangle(debug_dino, (int(b['xmin']), int(b['ymin'])), (int(b['xmax']), int(b['ymax'])), (255, 255, 0), 2)
                    cv2.putText(debug_dino, target_label, (int(b['xmin']), int(b['ymin'])-10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                
                cv2.imshow("Step 1: GroundingDINO Detection", debug_dino)
                print(f"👉 DINO 找到了候选框。按任意键验证它们是否匹配 {name}...")
                cv2.waitKey(0)

                valid_candidates = []

                # --- 3. 几何与颜色校验阶段 ---
                for r in results:
                    box = [r['box']['xmin'], r['box']['ymin'], r['box']['xmax'], r['box']['ymax']]
                    cx, cy = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2

                    # A. 检查黑名单
                    is_blacklisted = False
                    for (bx, by) in blacklist:
                        if np.sqrt((cx - bx)**2 + (cy - by)**2) < 20:
                            is_blacklisted = True
                            break
                    if is_blacklisted: continue

                    # B. SAM 分割
                    self.sam_predictor.set_image(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB))
                    masks, _, _ = self.sam_predictor.predict(box=np.array(box), multimask_output=False)
                    mask = masks[0]
                    
                    # C. 几何与 D. 颜色分类
                    contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    if not contours: continue
                    rect = cv2.minAreaRect(max(contours, key=cv2.contourArea))
                    w, h = rect[1]; actual_ratio = max(w, h) / (min(w, h) + 1e-6)
                    
                    hsv_img = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
                    eroded_mask = cv2.erode(mask.astype(np.uint8), np.ones((5, 5), np.uint8), iterations=1)
                    mask_pixels = hsv_img[eroded_mask > 0]
                    detected_color = "unknown"
                    if len(mask_pixels) > 0:
                        detected_color = classify_color(np.mean(mask_pixels, axis=0))

                    # --- 校验结果展示 ---
                    ratio_ok = abs(actual_ratio - target_ratio) <= RATIO_TOLERANCE
                    color_ok = (target_color_name is None) or (detected_color == target_color_name)
                    
                    debug_logic = img_bgr.copy()
                    debug_logic[mask > 0] = debug_logic[mask > 0] * 0.5 + np.array([0, 255, 0], dtype=np.uint8) * 0.5
                    
                    label_color = (0, 255, 0) if (ratio_ok and color_ok) else (0, 0, 255)
                    # 【核心修改点】标签前缀显示任务名
                    status_txt = f"{name} | R:{actual_ratio:.1f} C:{detected_color}"
                    
                    cv2.rectangle(debug_logic, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), label_color, 2)
                    cv2.putText(debug_logic, status_txt, (int(box[0]), int(box[1])-10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, label_color, 2)
                    
                    cv2.imshow("Step 2: SAM & Logic Check", debug_logic)
                    print(f"📊 校验中: {status_txt}. 按任意键继续...")
                    cv2.waitKey(0)

                    if not ratio_ok or not color_ok:
                        blacklist.append((cx, cy))
                        continue 

                    match_score = r['score'] * (1.0 / (abs(actual_ratio - target_ratio) + 0.1))
                    valid_candidates.append({'mask': mask, 'score': match_score, 'box': box, 'color': detected_color})

                # --- 4. 重试判定 ---
                if not valid_candidates:
                    print(f" ❌ 未找到匹配 {name} 的积木，刷新中...")
                    time.sleep(1); continue

                winner = max(valid_candidates, key=lambda x: x['score'])
                
                # --- 5. 6D 姿态追踪 ---
                print(f"📐 [步骤 3] 6D 锁定目标: {name}")
                pose_samples = []
                start_track = time.time()
                while (time.time() - start_track) < 4.0:
                    f = self.pipeline.wait_for_frames(); a = self.align.process(f)
                    rgb = np.asanyarray(a.get_color_frame().get_data())
                    dep = np.asanyarray(a.get_depth_frame().get_data()).astype(np.float32) / 1000.0
                    
                    T_curr = self.pose_est.estimator.register(K=self.K_MATRIX, rgb=rgb, depth=dep, ob_mask=winner['mask'], iteration=20)
                    if T_curr is not None:
                        pose_samples.append(T_curr)
                        self.visualize_result(rgb, T_curr)
                    
                    # 追踪时也显示目标名称
                    cv2.putText(rgb, f"Tracking Goal: {name}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                    cv2.imshow("Step 3: 6D Pose Tracking", rgb)
                    if cv2.waitKey(1) & 0xFF == ord('q'): break

                print(f"✅ {name} 追踪完成。按任意键正式执行动作...")
                cv2.waitKey(0)

                # --- 6. 发送结果 ---
                if pose_samples:
                    self.send_to_robot(name, pose_samples[-1], task)
                    task_success = True

    
    def visualize_result(self, image, T_cam_obj):
        l = 0.05
        pts_3d = np.float32([[0,0,0], [l,0,0], [0,l,0], [0,0,l]])
        pts_cam = (T_cam_obj[:3,:3] @ pts_3d.T).T + T_cam_obj[:3,3]
        p2d = []
        for p in pts_cam:
            u = int(self.K_MATRIX[0,0]*p[0]/p[2] + self.K_MATRIX[0,2])
            v = int(self.K_MATRIX[1,1]*p[1]/p[2] + self.K_MATRIX[1,2])
            p2d.append((u,v))
        cv2.line(image, p2d[0], p2d[1], (0,0,255), 3)
        cv2.line(image, p2d[0], p2d[2], (0,255,0), 2)
        cv2.line(image, p2d[0], p2d[3], (255,0,0), 2)

    def send_to_robot(self, name, T_cam_obj, task_cfg):
        T_base_obj = self.T_base_camera @ T_cam_obj
        raw_yaw = np.degrees(np.arctan2(T_base_obj[1,0], T_base_obj[0,0]))
        refined_yaw = ((raw_yaw + 90) % 180) - 90
        spin = float(task_cfg.get('grasp_spin', 0))
        blueprint_yaw = float(task_cfg.get('blueprint_yaw', 0))
        
        brick_q = R.from_euler('xyz', [0,0,refined_yaw], degrees=True).as_quat().tolist()
        pick_q = R.from_euler('xyz', [180,0,refined_yaw + spin - 135], degrees=True).as_quat().tolist()
        place_q = R.from_euler('xyz', [180,0,blueprint_yaw + spin - 45], degrees=True).as_quat().tolist()

        data = {
            'name': name,
            'brick_info': {'pos': T_base_obj[:3,3].tolist(), 'orientation': brick_q},
            'robot_pick': {'orientation': pick_q},
            'place': {'pos': (ASSEMBLY_CENTER_BASE + np.array(task_cfg['place']['pos'])).tolist(), 'orientation': place_q}
        }
        with open(RESULT_FILE, 'w') as f:
            yaml.dump(data, f)

    def get_mesh_path(self, task_name):
        kw = "4x2" if ("2x4" in task_name or "4x2" in task_name) else "2x2"
        for f in os.listdir(MESH_DIR):
            if kw in f and f.endswith(".stl"): return os.path.join(MESH_DIR, f)
        return ""

if __name__ == "__main__":
    node = RobotVisionNode()
    node.run()