#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""容器内运行: 用和视觉桥一致的 GroundingDINO+SAM, 在 O3P 的 240x180 对齐彩色上
测检测/分割质量。存可视化到 /shared_data/o3p/detect_viz.png。

用法(容器内):
  /opt/conda/envs/my/bin/python /vision_code/detect_test.py "white 2x2 brick."
"""
import sys
import cv2
import numpy as np
from PIL import Image
from transformers import pipeline
from segment_anything import SamPredictor, sam_model_registry

IMG = "/shared_data/o3p/color.png"
OUT = "/shared_data/o3p/detect_viz.png"
target = sys.argv[1] if len(sys.argv) > 1 else "lego block"

img = cv2.imread(IMG, cv2.IMREAD_COLOR)
print(f"[test] image {img.shape} target={target!r}")

det = pipeline(model="IDEA-Research/grounding-dino-tiny",
               task="zero-shot-object-detection", device="cuda")
results = det(Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB)),
              candidate_labels=[target, "lego block"], threshold=0.1)
print(f"[test] GroundingDINO 检到 {len(results)} 个候选:")
for r in results:
    print("   ", round(float(r["score"]), 3), r.get("label"), r["box"])

sam = sam_model_registry["vit_h"](
    checkpoint="/FoundationPose/weights/sam_vit_h_4b8939.pth").to("cuda")
pred = SamPredictor(sam)
pred.set_image(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))

viz = img.copy()
for r in results:
    b = r["box"]
    xyxy = np.array([b["xmin"], b["ymin"], b["xmax"], b["ymax"]])
    masks, _, _ = pred.predict(box=xyxy, multimask_output=False)
    m = masks[0]
    cnts, _ = cv2.findContours(m.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    ratio = 0.0
    if cnts:
        (w, h) = cv2.minAreaRect(max(cnts, key=cv2.contourArea))[1]
        ratio = max(w, h) / (min(w, h) + 1e-6)
    overlay = viz.copy()
    overlay[m] = (0, 255, 0)
    viz = cv2.addWeighted(overlay, 0.4, viz, 0.6, 0)
    cv2.rectangle(viz, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), (0, 0, 255), 1)
    cv2.putText(viz, f"{r['score']:.2f} r={ratio:.2f}", (int(xyxy[0]), max(8, int(xyxy[1]) - 2)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)
    print(f"   -> SAM mask 像素数={int(m.sum())}, 长宽比={ratio:.2f}")

cv2.imwrite(OUT, viz)
print(f"[test] 存可视化 -> {OUT}")
