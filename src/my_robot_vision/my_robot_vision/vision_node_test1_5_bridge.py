#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""File-based vision bridge for the UR5 perception-to-execution pipeline.

Run this inside the existing vision Docker with:
  /opt/conda/envs/my/bin/python /vision_code/vision_node_test1_5_bridge.py

It waits for current_target.txt, estimates T_cam_obj with
GroundingDINO + SAM + FoundationPose, and writes vision_output.yaml in the
format consumed by my_robot_vision/vision_to_execution_yaml.py.
"""

import os
import sys
import time
from dataclasses import dataclass
from typing import Optional

import cv2
import json
import numpy as np
try:
    import pyrealsense2 as rs   # 相机已换成 ifm O3P; RealSense 不再使用, 缺失也不报错
except Exception:
    rs = None
import torch
import trimesh
import yaml
from PIL import Image
from transformers import pipeline

# 同目录(容器内 /vision_code): O3P 高清 mask -> 深度帧 的鱼眼投影
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from o3p_align import build_depth_to_hi_map, sample_hi_to_depth


FP_REPO = os.environ.get("FP_REPO", "/FoundationPose")
MESH_DIR = os.environ.get("MESH_DIR", "/FoundationPose/meshes")

PLAN_YAML = os.environ.get(
    "PLAN_YAML",
    "/vision_code/plan_perception_test.yaml",
)
TARGET_FILE = os.environ.get(
    "TARGET_FILE",
    "/shared_data/current_target.txt",
)
OUTPUT_FILE = os.environ.get(
    "OUTPUT_FILE",
    "/shared_data/vision_output.yaml",
)
DEBUG_IMAGE_FILE = os.environ.get(
    "VISION_DEBUG_IMAGE",
    "/shared_data/vision_pose_debug.jpg",
)
FROZEN_DEBUG_IMAGE_FILE = os.environ.get(
    "VISION_FROZEN_DEBUG_IMAGE",
    "/shared_data/vision_frozen_observation.jpg",
)
USED_REGIONS_DEBUG_IMAGE_FILE = os.environ.get(
    "VISION_USED_REGIONS_DEBUG_IMAGE",
    "/shared_data/vision_used_regions.jpg",
)

ACC_SECONDS = float(os.environ.get("VISION_ACC_SECONDS", "4.0"))
DETECTION_THRESHOLD = float(os.environ.get("VISION_DETECTION_THRESHOLD", "0.15"))
RATIO_TOLERANCE = float(os.environ.get("VISION_RATIO_TOLERANCE", "0.35"))
COLOR_MIN_FRACTION = float(os.environ.get("VISION_COLOR_MIN_FRACTION", "0.08"))
USED_REGION_OVERLAP_THRESHOLD = float(os.environ.get("VISION_USED_REGION_OVERLAP_THRESHOLD", "0.35"))
DISABLE_EMITTER = os.environ.get("VISION_DISABLE_EMITTER", "1") != "0"
FROZEN_OBSERVATION = os.environ.get("VISION_FROZEN_OBSERVATION", "0") == "1"
# ifm O3P 相机: host 的 o3p_grabber.py 把【已对齐】的彩色+深度+内参写到这个目录,
# 桥从这里读帧(替代 RealSense)。color.png(bgr8 240x180) / depth.npy(uint16 mm) / camera_K.json
O3P_FRAME_DIR = os.environ.get("O3P_FRAME_DIR", "/shared_data/o3p")
O3P_WAIT_RETRIES = int(os.environ.get("O3P_WAIT_RETRIES", "60"))
BRIDGE_VERSION = "2026-06-25-o3p-tof-camera"


if FP_REPO not in sys.path:
    sys.path.append(FP_REPO)
    sys.path.append(os.path.join(FP_REPO, "root"))

from estimater import FoundationPose, PoseRefinePredictor, ScorePredictor
import learning.training.predict_pose_refine as predict_pose_refine
import learning.training.predict_score as predict_score

try:
    from segment_anything import SamPredictor, sam_model_registry
    SAM_AVAILABLE = True
except ImportError:
    SAM_AVAILABLE = False


def patch_foundationpose_crop_dtype():
    """Replace FoundationPose's crop helper with an explicit float32 version.

    The stock helper builds some tensors without dtype/device. On this Docker
    that can mix CUDA float32 with double tensors and crash at K @ pts.
    FoundationPose calls this helper only with method='box_3d' in our path.
    """

    def compute_crop_window_tf_batch_float32(
        pts=None,
        H=None,
        W=None,
        poses=None,
        K=None,
        crop_ratio=1.2,
        out_size=None,
        rgb=None,
        uvs=None,
        method="min_box",
        mesh_diameter=None,
    ):
        if method != "box_3d":
            raise RuntimeError(
                f"patched compute_crop_window_tf_batch only supports box_3d, got {method}"
            )

        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        dtype = torch.float32
        poses_t = torch.as_tensor(poses, dtype=dtype, device=device)
        K_t = torch.as_tensor(K, dtype=dtype, device=device).reshape(3, 3)
        B = int(poses_t.shape[0])

        radius = float(mesh_diameter) * float(crop_ratio) / 2.0
        offsets = torch.tensor(
            [
                [0.0, 0.0, 0.0],
                [radius, 0.0, 0.0],
                [-radius, 0.0, 0.0],
                [0.0, radius, 0.0],
                [0.0, -radius, 0.0],
            ],
            dtype=dtype,
            device=device,
        )

        pts_t = poses_t[:, :3, 3].reshape(-1, 1, 3) + offsets.reshape(1, -1, 3)
        projected = (K_t @ pts_t.reshape(-1, 3).T).T
        uvs_t = projected[:, :2] / projected[:, 2:3].clamp(min=1e-6)
        uvs_t = uvs_t.reshape(B, -1, 2)

        center = uvs_t[:, 0]
        radius_px = torch.abs(uvs_t - center.reshape(-1, 1, 2)).reshape(B, -1).max(dim=-1)[0]
        left = (center[:, 0] - radius_px).round()
        right = (center[:, 0] + radius_px).round()
        top = (center[:, 1] - radius_px).round()
        bottom = (center[:, 1] + radius_px).round()

        width = (right - left).clamp(min=1.0)
        height = (bottom - top).clamp(min=1.0)
        tf = torch.eye(3, dtype=dtype, device=device).reshape(1, 3, 3).repeat(B, 1, 1)
        tf[:, 0, 2] = -left
        tf[:, 1, 2] = -top

        new_tf = torch.eye(3, dtype=dtype, device=device).reshape(1, 3, 3).repeat(B, 1, 1)
        new_tf[:, 0, 0] = float(out_size[0]) / width
        new_tf[:, 1, 1] = float(out_size[1]) / height
        return new_tf @ tf

    predict_pose_refine.compute_crop_window_tf_batch = compute_crop_window_tf_batch_float32
    predict_score.compute_crop_window_tf_batch = compute_crop_window_tf_batch_float32


patch_foundationpose_crop_dtype()


@dataclass
class BoundingBox:
    xmin: int
    ymin: int
    xmax: int
    ymax: int

    @property
    def xyxy(self):
        return [self.xmin, self.ymin, self.xmax, self.ymax]


@dataclass
class DetectionResult:
    score: float
    label: str
    box: BoundingBox


class LegoPoseEstimator:
    def __init__(self, scorer, refiner):
        self.scorer = scorer
        self.refiner = refiner
        self.mesh_path = None
        self.estimator = None
        self.mesh_bbox = None

    def update_mesh(self, mesh_path):
        if self.mesh_path == mesh_path and self.estimator is not None:
            return
        self.mesh_path = mesh_path
        mesh = trimesh.load(mesh_path)
        if np.linalg.norm(mesh.extents) > 0.1:
            mesh.apply_scale(0.001)
        mesh.vertices -= mesh.bounds.mean(axis=0)
        mesh.vertices = mesh.vertices.astype(np.float32)
        self.mesh_bbox = np.array([mesh.vertices.min(axis=0), mesh.vertices.max(axis=0)], dtype=np.float32)
        model_pts, _ = trimesh.sample.sample_surface(mesh, 2048)
        model_pts = torch.from_numpy(model_pts.astype(np.float32)).cuda()
        self.estimator = FoundationPose(
            model_pts=model_pts,
            model_normals=None,
            mesh=mesh,
            scorer=self.scorer,
            refiner=self.refiner,
        )


class RobotVisionBridge:
    def __init__(self):
        print(f"[VISION] bridge version: {BRIDGE_VERSION}")
        self.task_list = self.load_plan_tasks(PLAN_YAML)
        self.frozen_observation = FROZEN_OBSERVATION
        self.frozen_frame = None
        self.used_region_masks = []
        self.pending_selected_mask = None
        self.init_realsense()

        self.detector = pipeline(
            model="IDEA-Research/grounding-dino-tiny",
            task="zero-shot-object-detection",
            device="cuda",
        )

        if not SAM_AVAILABLE:
            raise RuntimeError("segment_anything is not available in this Docker environment")
        sam = sam_model_registry["vit_h"](
            checkpoint="/FoundationPose/weights/sam_vit_h_4b8939.pth"
        ).to("cuda")
        self.sam_predictor = SamPredictor(sam)
        self.pose_est = LegoPoseEstimator(ScorePredictor(), PoseRefinePredictor())
        if self.frozen_observation:
            print(
                "[VISION] frozen observation mode enabled; "
                "selected regions will be excluded for later targets"
            )

    def load_plan_tasks(self, path):
        if not os.path.exists(path):
            raise FileNotFoundError(
                f"plan file not found: {path}. "
                "Copy the planner YAML to /vision_code/plan_perception_test.yaml "
                "or run the host auto pipeline."
            )
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        tasks = data.get("tasksh", data.get("tasks", []))
        if not tasks:
            raise RuntimeError(f"no tasks found in {path}")
        return tasks

    def init_realsense(self):
        # 相机已从 RealSense 换成 ifm O3P(ToF)。帧由 host 的 o3p_grabber.py 写到
        # O3P_FRAME_DIR。这里读 camera_params.json: depth_K(FoundationPose 用) +
        # color_K/color_D/外参(高清 mask 投影回深度帧用), 不接相机硬件。
        self._last_frame_id = None
        ppath = os.path.join(O3P_FRAME_DIR, "camera_params.json")
        for _ in range(O3P_WAIT_RETRIES):
            if os.path.exists(ppath):
                with open(ppath) as f:
                    self.cam_params = json.load(f)
                K = np.array(self.cam_params["depth_K"], dtype=np.float32)
                self.K_MATRIX = np.ascontiguousarray(K, dtype=np.float32)
                print(f"[VISION] O3P depth intrinsics from {ppath}: "
                      f"fx={K[0,0]:.2f} cx={K[0,2]:.2f} cy={K[1,2]:.2f}")
                return
            print(f"[VISION] waiting for {ppath} (host 上跑 o3p_grabber.py 了吗?)")
            time.sleep(1.0)
        raise RuntimeError(f"找不到 {ppath}; 先在 host 跑 o3p_grabber.py --out {O3P_FRAME_DIR}")

    def capture_hi(self):
        # 高清彩色 1344x1008(给检测用, 比 240x180 清晰得多)。
        path = os.path.join(O3P_FRAME_DIR, "color_hi.png")
        if not os.path.exists(path):
            return None
        hi = cv2.imread(path, cv2.IMREAD_COLOR)
        return np.ascontiguousarray(hi, dtype=np.uint8) if hi is not None else None

    def clear_buffer(self, count=30):
        # 文件帧没有"陈旧缓冲"概念: grabber 持续写最新帧, capture_aligned 总读最新。
        # 这里等一帧确实更新过, 避免拿到机械臂移动前的旧帧。
        self._wait_fresh_frame(timeout=2.0)

    def _wait_fresh_frame(self, timeout=2.0):
        fpath = os.path.join(O3P_FRAME_DIR, "frame.txt")
        start = time.time()
        while time.time() - start < timeout:
            try:
                with open(fpath) as f:
                    fid = f.read().strip()
            except Exception:
                fid = None
            if fid and fid != self._last_frame_id:
                self._last_frame_id = fid
                return
            time.sleep(0.05)

    @staticmethod
    def task_label(task):
        for key in ("vision_label", "vision_name", "target", "name"):
            value = task.get(key)
            if value:
                return str(value).strip()
        return "lego block"

    def get_task(self, target):
        for task in self.task_list:
            labels = {
                str(task.get("name", "")).strip(),
                str(task.get("vision_label", "")).strip(),
                str(task.get("vision_name", "")).strip(),
                str(task.get("target", "")).strip(),
            }
            if target in labels:
                return task
        print(f"[WARN] target {target!r} not found in plan, using first task")
        task = dict(self.task_list[0])
        task["vision_label"] = target
        return task

    def get_mesh_path(self, task):
        task_text = " ".join([
            str(task.get("name", "")),
            str(task.get("type", "")),
            str(task.get("vision_label", "")),
        ]).lower()
        keyword = "4x2" if ("2x4" in task_text or "4x2" in task_text) else "2x2"
        for filename in os.listdir(MESH_DIR):
            if keyword in filename and filename.endswith(".stl"):
                return os.path.join(MESH_DIR, filename)
        raise RuntimeError(f"cannot find mesh for {task_text} in {MESH_DIR}")

    def read_target(self) -> Optional[str]:
        if not os.path.exists(TARGET_FILE):
            return None
        with open(TARGET_FILE, "r", encoding="utf-8") as f:
            target = f.read().strip()
        return target or None

    def capture_aligned(self):
        # 从 host grabber 写的共享目录读【已对齐】的彩色+深度(O3P, 240x180)。
        # color.png: bgr8 已对齐到深度帧; depth.npy: uint16 毫米; 两者逐像素对齐。
        cpath = os.path.join(O3P_FRAME_DIR, "color.png")
        dpath = os.path.join(O3P_FRAME_DIR, "depth.npy")
        if not (os.path.exists(cpath) and os.path.exists(dpath)):
            return None, None

        img_bgr = cv2.imread(cpath, cv2.IMREAD_COLOR)
        try:
            depth_mm = np.load(dpath)
        except Exception:
            return None, None
        if img_bgr is None or depth_mm is None:
            return None, None
        if img_bgr.shape[:2] != depth_mm.shape[:2]:
            return None, None

        depth_m = depth_mm.astype(np.float32) / 1000.0   # 毫米 -> 米
        img_bgr = np.ascontiguousarray(img_bgr, dtype=np.uint8)
        depth_m = np.ascontiguousarray(depth_m, dtype=np.float32)

        return img_bgr, depth_m

    def get_perception_frame(self):
        # 返回 (hi 高清彩色1344x1008, aligned 对齐彩色240x180, depth 深度240x180 米)。
        # 检测用 hi(清晰); FoundationPose 用 aligned+depth(深度帧)。
        if not self.frozen_observation:
            aligned, depth_m = self.capture_aligned()
            hi = self.capture_hi()
            return hi, aligned, depth_m

        if self.frozen_frame is None:
            aligned, depth_m = self.capture_aligned()
            hi = self.capture_hi()
            if aligned is None or depth_m is None or hi is None:
                return None, None, None
            self.frozen_frame = (hi.copy(), aligned.copy(), depth_m.copy())
            os.makedirs(os.path.dirname(FROZEN_DEBUG_IMAGE_FILE), exist_ok=True)
            cv2.imwrite(FROZEN_DEBUG_IMAGE_FILE, hi)
            print(f"[VISION] froze observe RGB-D frame; wrote {FROZEN_DEBUG_IMAGE_FILE}")

        hi, aligned, depth_m = self.frozen_frame
        return hi.copy(), aligned.copy(), depth_m.copy()

    @staticmethod
    def target_color(target):
        text = target.lower()
        for color in ("white", "yellow", "orange", "blue", "red", "green", "black"):
            if color in text:
                return color
        return None

    @staticmethod
    def color_fraction(img_bgr, mask, color_name):
        if not color_name:
            return 1.0
        pixels = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)[mask.astype(bool)]
        if pixels.size == 0:
            return 0.0

        h = pixels[:, 0]
        s = pixels[:, 1]
        v = pixels[:, 2]
        if color_name == "white":
            color_mask = (s < 75) & (v > 115)
        elif color_name == "yellow":
            color_mask = (h >= 15) & (h <= 42) & (s > 45) & (v > 70)
        elif color_name == "orange":
            color_mask = (h >= 5) & (h <= 22) & (s > 55) & (v > 70)
        elif color_name == "blue":
            color_mask = (h >= 85) & (h <= 135) & (s > 45) & (v > 45)
        elif color_name == "red":
            color_mask = (((h <= 10) | (h >= 170)) & (s > 45) & (v > 45))
        elif color_name == "green":
            color_mask = (h >= 40) & (h <= 85) & (s > 45) & (v > 45)
        elif color_name == "black":
            color_mask = v < 70
        else:
            return 1.0
        return float(np.count_nonzero(color_mask)) / float(len(pixels))

    def used_region_overlap(self, mask):
        if not self.used_region_masks:
            return 0.0
        mask_bool = mask.astype(bool)
        mask_area = float(np.count_nonzero(mask_bool))
        if mask_area <= 0.0:
            return 0.0
        return max(
            float(np.count_nonzero(mask_bool & used_mask)) / mask_area
            for used_mask in self.used_region_masks
        )

    def remember_used_region(self, target, mask):
        if not self.frozen_observation:
            return
        mask_u8 = mask.astype(np.uint8)
        kernel = np.ones((15, 15), dtype=np.uint8)
        dilated = cv2.dilate(mask_u8, kernel, iterations=1).astype(bool)
        self.used_region_masks.append(dilated)
        print(
            f"[VISION] remembered used region #{len(self.used_region_masks)} "
            f"for target={target!r}"
        )

        if self.frozen_frame is None:
            return
        debug = self.frozen_frame[0].copy()
        overlay = debug.copy()
        for used_mask in self.used_region_masks:
            overlay[used_mask] = (0, 0, 255)
        debug = cv2.addWeighted(overlay, 0.35, debug, 0.65, 0.0)
        os.makedirs(os.path.dirname(USED_REGIONS_DEBUG_IMAGE_FILE), exist_ok=True)
        cv2.imwrite(USED_REGIONS_DEBUG_IMAGE_FILE, debug)
        print(f"[VISION] wrote used-region debug image {USED_REGIONS_DEBUG_IMAGE_FILE}")

    def select_mask(self, target, img_bgr):
        img_pil = Image.fromarray(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB))
        results = self.detector(
            img_pil,
            candidate_labels=[target, "lego block"],
            threshold=DETECTION_THRESHOLD,
        )
        if not results:
            return None, None

        target_ratio = 2.0 if ("4x2" in target or "2x4" in target) else 1.0
        target_color = self.target_color(target)
        self.sam_predictor.set_image(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB))
        candidates = []
        viz = img_bgr.copy()
        for cand_index, result in enumerate(results):
            box = result["box"]
            xyxy = np.array([box["xmin"], box["ymin"], box["xmax"], box["ymax"]])
            masks, _, _ = self.sam_predictor.predict(box=xyxy, multimask_output=False)
            mask = masks[0]

            contours, _ = cv2.findContours(
                mask.astype(np.uint8),
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )
            if not contours:
                continue
            rect = cv2.minAreaRect(max(contours, key=cv2.contourArea))
            w, h = rect[1]
            actual_ratio = max(w, h) / (min(w, h) + 1e-6)
            ratio_diff = abs(actual_ratio - target_ratio)
            color_fraction = self.color_fraction(img_bgr, mask, target_color)
            ratio_score = 1.0 / (ratio_diff + 0.1)
            color_score = 1.0 if target_color is None else max(0.05, color_fraction)
            label = str(result.get("label", ""))
            label_score = 1.25 if target.lower().strip(".") in label.lower() else 1.0
            used_overlap = self.used_region_overlap(mask) if self.frozen_observation else 0.0
            score = float(result["score"]) * ratio_score * color_score * label_score
            print(
                "[VISION] candidate "
                f"#{cand_index}: label={label!r}, det={float(result['score']):.3f}, "
                f"ratio={actual_ratio:.2f}, ratio_diff={ratio_diff:.2f}, "
                f"target_color={target_color}, color_fraction={color_fraction:.2f}, "
                f"used_overlap={used_overlap:.2f}, score={score:.3f}, box={xyxy.tolist()}"
            )
            if ratio_diff > RATIO_TOLERANCE:
                color = (0, 0, 255)
            elif target_color is not None and color_fraction < COLOR_MIN_FRACTION:
                color = (0, 165, 255)
            elif used_overlap > USED_REGION_OVERLAP_THRESHOLD:
                color = (255, 0, 255)
                print(
                    "[VISION] skipping candidate because it overlaps a previously "
                    f"used region: overlap={used_overlap:.2f}"
                )
            else:
                color = (255, 0, 0)
                candidates.append((score, mask, xyxy.tolist(), actual_ratio, color_fraction, label, used_overlap))

            cv2.rectangle(viz, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), color, 2)
            cv2.putText(
                viz,
                f"{cand_index}:{label[:10]} r={actual_ratio:.2f} c={color_fraction:.2f} u={used_overlap:.2f}",
                (int(xyxy[0]), min(img_bgr.shape[0] - 8, int(xyxy[3]) + 18)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                1,
            )

        if not candidates:
            cv2.imshow("Detection Logic", viz)
            cv2.waitKey(500)
            return None, viz

        score, mask, box, ratio, color_fraction, label, used_overlap = max(candidates, key=lambda item: item[0])
        self.pending_selected_mask = mask.copy()
        print(
            "[VISION] selected candidate: "
            f"label={label!r}, score={score:.3f}, ratio={ratio:.2f}, "
            f"color_fraction={color_fraction:.2f}, used_overlap={used_overlap:.2f}"
        )
        cv2.rectangle(viz, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 255, 0), 4)
        cv2.putText(
            viz,
            f"WINNER ratio={ratio:.2f} color={color_fraction:.2f}",
            (int(box[0]), max(25, int(box[1]) - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )
        cv2.imshow("Detection Logic", viz)
        cv2.waitKey(500)
        return mask, viz

    def visualize_result(self, image, transform):
        length = 0.05
        axis_pts_3d = np.float32([[0, 0, 0], [length, 0, 0], [0, length, 0], [0, 0, length]])
        pts_cam = (transform[:3, :3] @ axis_pts_3d.T).T + transform[:3, 3]
        pts_2d = []
        for point in pts_cam:
            if point[2] <= 1e-6:
                return image
            u = int(self.K_MATRIX[0, 0] * point[0] / point[2] + self.K_MATRIX[0, 2])
            v = int(self.K_MATRIX[1, 1] * point[1] / point[2] + self.K_MATRIX[1, 2])
            pts_2d.append((u, v))
        cv2.circle(image, pts_2d[0], 5, (255, 255, 255), -1)
        cv2.line(image, pts_2d[0], pts_2d[1], (0, 0, 255), 3)
        cv2.line(image, pts_2d[0], pts_2d[2], (0, 255, 0), 2)
        cv2.line(image, pts_2d[0], pts_2d[3], (255, 0, 0), 2)
        cv2.putText(
            image,
            f"t=[{transform[0,3]:.3f},{transform[1,3]:.3f},{transform[2,3]:.3f}]",
            (10, image.shape[0] - 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
        )
        return image

    def estimate_once(self, target):
        task = self.get_task(target)
        mesh_path = self.get_mesh_path(task)
        print(f"[VISION] target={target}, task={self.task_label(task)}, mesh={mesh_path}")
        self.pose_est.update_mesh(mesh_path)

        if not self.frozen_observation or self.frozen_frame is None:
            self.clear_buffer()
        hi, aligned, depth = self.get_perception_frame()
        if hi is None or aligned is None or depth is None:
            return None
        # 检测/分割在【高清彩色 1344x1008】上做(清晰, 不输 RealSense)
        mask_hi, _ = self.select_mask(target, hi)
        if mask_hi is None:
            print("[VISION] no valid candidate mask")
            return None
        # 把高清 mask 鱼眼投影回【深度帧 240x180】, 给 FoundationPose(深度密集帧)
        uc, vc, valid = build_depth_to_hi_map(depth, self.cam_params, hi.shape[1], hi.shape[0])
        mask = sample_hi_to_depth(mask_hi.astype(np.uint8), uc, vc, valid) > 0
        if int(mask.sum()) < 10:
            print("[VISION] mask 投影到深度帧后像素太少, 放弃")
            return None

        pose_samples = []
        last_debug = None
        start = time.time()
        while time.time() - start < ACC_SECONDS:
            _, aligned, depth = self.get_perception_frame()
            if aligned is None or depth is None:
                continue
            pose_input = np.ascontiguousarray(aligned.copy())
            debug_img = aligned.copy()
            pose = self.pose_est.estimator.register(
                K=np.ascontiguousarray(self.K_MATRIX, dtype=np.float32),
                rgb=pose_input,
                depth=depth,
                ob_mask=mask,
                iteration=20,
            )
            if pose is not None:
                pose_samples.append(pose)
                debug_img = self.visualize_result(debug_img, pose)
                last_debug = debug_img
            else:
                cv2.putText(debug_img, "FoundationPose returned no pose", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.imshow("6D Pose Tracking", debug_img)
            cv2.waitKey(1)

        if last_debug is not None:
            os.makedirs(os.path.dirname(DEBUG_IMAGE_FILE), exist_ok=True)
            cv2.imwrite(DEBUG_IMAGE_FILE, last_debug)
            print(f"[VISION] wrote debug image {DEBUG_IMAGE_FILE}")

        if not pose_samples:
            return None
        # "已用区域"全程用高清 mask(检测/重叠比较/叠加都在高清帧), 不能用深度帧 mask
        self.remember_used_region(target, mask_hi)
        return pose_samples[-1]

    def write_result(self, target, transform, timing=None):
        os.makedirs(os.path.dirname(OUTPUT_FILE), exist_ok=True)
        output = {
            "target": target,
            "timestamp": time.time(),
            "bridge_version": BRIDGE_VERSION,
            "frozen_observation": self.frozen_observation,
            "used_region_count": len(self.used_region_masks),
            "t_cam_xyz": transform[:3, 3].tolist(),
            "T_cam_obj": transform.tolist(),
        }
        if timing is not None:
            output["timing"] = timing
        with open(OUTPUT_FILE, "w", encoding="utf-8") as f:
            yaml.safe_dump(output, f, sort_keys=False)
        print(f"[VISION] wrote {OUTPUT_FILE}")

    def run(self):
        print("[VISION] ready")
        print(f"[VISION] target file: {TARGET_FILE}")
        print(f"[VISION] output file: {OUTPUT_FILE}")
        last_target = None
        while True:
            target = self.read_target()
            if not target:
                if self.frozen_observation and self.frozen_frame is not None:
                    rgb = self.frozen_frame[0].copy()
                    status_text = "IDLE: frozen observe frame"
                else:
                    rgb, _ = self.capture_aligned()
                    status_text = "IDLE: waiting current_target.txt"
                if rgb is not None:
                    cv2.putText(rgb, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    cv2.imshow("Vision Bridge", rgb)
                    cv2.waitKey(1)
                time.sleep(0.1)
                continue
            if target == last_target and os.path.exists(OUTPUT_FILE):
                time.sleep(0.2)
                continue
            last_target = target
            if os.path.exists(OUTPUT_FILE):
                os.remove(OUTPUT_FILE)
            estimate_start = time.perf_counter()
            pose = self.estimate_once(target)
            vision_compute_time_s = time.perf_counter() - estimate_start
            if pose is None:
                print(f"[VISION] failed for {target} after {vision_compute_time_s:.3f}s")
                continue
            print(
                "[VISION][TIMING] "
                f"vision_compute_time_s={vision_compute_time_s:.3f} target={target!r}"
            )
            self.write_result(
                target,
                pose,
                {
                    "vision_compute_time_s": round(float(vision_compute_time_s), 3),
                    "frozen_observation": bool(self.frozen_observation),
                    "used_region_count": int(len(self.used_region_masks)),
                },
            )


def run_patch_self_test():
    print(f"[VISION] bridge version: {BRIDGE_VERSION}")
    K = np.array(
        [
            [600.0, 0.0, 320.0],
            [0.0, 600.0, 240.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    poses = np.eye(4, dtype=np.float64).reshape(1, 4, 4)
    poses[0, 2, 3] = 0.45
    tf = predict_pose_refine.compute_crop_window_tf_batch(
        pts=np.zeros((8, 3), dtype=np.float64),
        H=480,
        W=640,
        poses=poses,
        K=K,
        crop_ratio=1.2,
        out_size=(160, 160),
        method="box_3d",
        mesh_diameter=0.07,
    )
    print(
        "[VISION] patch self-test ok: "
        f"shape={tuple(tf.shape)}, dtype={tf.dtype}, device={tf.device}"
    )


if __name__ == "__main__":
    if os.environ.get("VISION_SELF_TEST_PATCH", "0") == "1":
        run_patch_self_test()
        sys.exit(0)

    node = RobotVisionBridge()
    try:
        node.run()
    finally:
        # O3P 无 RealSense pipeline 句柄; 仅在存在时停止
        if getattr(node, "pipeline", None) is not None:
            node.pipeline.stop()
        cv2.destroyAllWindows()
