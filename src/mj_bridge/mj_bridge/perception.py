"""Optional RGB-D inference. No simulator state is accepted by the vision backend."""
from __future__ import annotations
import importlib.util
import math
import os
from pathlib import Path
import sys
import numpy as np
from .benchmark_core import load_registry

VISION_BACKEND = 'groundingdino-sam-foundationpose'


def vision_preflight():
    errors = []
    for name in ('torch', 'transformers', 'segment_anything', 'trimesh', 'nvdiffrast'):
        if importlib.util.find_spec(name) is None:
            errors.append(f'missing Python module: {name}')
    root = Path(os.environ.get('FP_REPO', 'FoundationPose')).resolve()
    if not (root / 'estimater.py').is_file():
        errors.append(f'FP_REPO must contain estimater.py: {root}')
    for run in ('2023-10-28-18-33-37', '2024-01-11-20-02-45'):
        for filename in ('config.yml', 'model_best.pth'):
            if not (root / 'weights' / run / filename).is_file():
                errors.append(f'missing FoundationPose weight/config: weights/{run}/{filename}')
    checkpoint = os.environ.get('SAM_CHECKPOINT', '')
    if not checkpoint or not Path(checkpoint).is_file():
        errors.append('SAM_CHECKPOINT must name an existing SAM checkpoint')
    if importlib.util.find_spec('torch') is not None:
        import torch
        if not torch.cuda.is_available():
            errors.append('FoundationPose requires an available CUDA device')
    return {'backend': VISION_BACKEND, 'ready': not errors, 'errors': errors, 'fp_repo': str(root)}


def validate_frame(rgb, depth, K, camera_to_world):
    if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError('rgb must be HxWx3 uint8 RGB')
    if depth.shape != rgb.shape[:2] or not np.isfinite(depth).all() or np.any(depth < 0):
        raise ValueError('depth must be aligned finite HxW metres, with 0 for missing pixels')
    if K.shape != (3, 3) or not np.isfinite(K).all() or K[0, 0] <= 0 or K[1, 1] <= 0:
        raise ValueError('K must be a finite camera intrinsic matrix')
    if camera_to_world.shape != (4, 4) or not np.isfinite(camera_to_world).all():
        raise ValueError('camera_to_world must be a finite 4x4 transform')
    rotation = camera_to_world[:3, :3]
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(rotation), 1, atol=1e-5)
            or not np.allclose(camera_to_world[3], [0, 0, 0, 1])):
        raise ValueError('camera_to_world must be a rigid transform')


class GroundedPoseBackend:
    def __init__(self):
        status = vision_preflight()
        if not status['ready']:
            raise RuntimeError('; '.join(status['errors']))
        sys.path.insert(0, status['fp_repo'])
        from transformers import pipeline
        from segment_anything import SamPredictor, sam_model_registry
        from estimater import FoundationPose, ScorePredictor, PoseRefinePredictor
        self.FoundationPose = FoundationPose
        self.detector = pipeline(task='zero-shot-object-detection',
            model=os.environ.get('GROUNDING_DINO_MODEL', 'IDEA-Research/grounding-dino-tiny'), device=0)
        self.sam = SamPredictor(sam_model_registry[os.environ.get('SAM_MODEL_TYPE', 'vit_h')](
            checkpoint=os.environ['SAM_CHECKPOINT']).to('cuda'))
        self.scorer, self.refiner = ScorePredictor(), PoseRefinePredictor()
        self.estimators = {}

    def estimator(self, part_type):
        # Same primitive dimensions and body frame as the public MuJoCo assets.
        if part_type not in self.estimators:
            import trimesh
            spec = load_registry()[part_type]
            mesh = trimesh.creation.box(extents=spec['size_m'])
            mesh.apply_translation([0, 0, -.009])
            components = [mesh]
            nx, ny = spec['studs']
            for ix in range(nx):
                for iy in range(ny):
                    stud = trimesh.creation.cylinder(radius=.0042, height=.004, sections=24)
                    stud.apply_translation([(ix-(nx-1)/2)*.016, (iy-(ny-1)/2)*.016,
                                            -.009 + spec['size_m'][2]/2 + .002])
                    components.append(stud)
            mesh = trimesh.util.concatenate(components)
            self.estimators[part_type] = self.FoundationPose(
                model_pts=mesh.vertices, model_normals=mesh.vertex_normals, mesh=mesh,
                scorer=self.scorer, refiner=self.refiner, debug=0,
                debug_dir=os.environ.get('FP_DEBUG_DIR', '/tmp/lego-bench-foundationpose'))
        return self.estimators[part_type]

    def observe(self, rgb, depth, K, camera_to_world, product):
        from PIL import Image
        validate_frame(rgb, depth, K, camera_to_world)
        self.sam.set_image(rgb)
        observations, used = [], np.zeros(depth.shape, dtype=bool)
        # Instance IDs intentionally differ from target IDs: same-colour parts are interchangeable.
        groups = sorted({(b['type'], b['color']) for b in product['blocks']})
        for part_type, color in groups:
            shape = '2 by 2' if part_type == 'brick_2x2' else '2 by 4'
            detections = self.detector(Image.fromarray(rgb), candidate_labels=[f'{color} {shape} brick.'])
            for detection in sorted(detections, key=lambda d: d['score'], reverse=True):
                if detection['score'] < float(os.environ.get('DINO_THRESHOLD', '.25')):
                    continue
                box = detection['box']
                masks, _, _ = self.sam.predict(box=np.array([box['xmin'], box['ymin'], box['xmax'], box['ymax']]),
                                               multimask_output=False)
                mask = masks[0].astype(bool) & (depth > 0)
                if mask.sum() < 16 or (mask & used).sum() > .5 * mask.sum():
                    continue
                pose = self.estimator(part_type).register(K=K.astype(np.float32), rgb=rgb,
                    depth=depth.astype(np.float32), ob_mask=mask, iteration=5)
                if pose is None or not np.isfinite(pose).all():
                    continue
                world = camera_to_world @ pose
                observations.append({'id': f'detection_{len(observations)}', 'type': part_type,
                    'color': color, 'position': world[:3, 3].tolist(),
                    'yaw_rad': math.atan2(world[1, 0], world[0, 0]),
                    'pose_world': world.tolist(), 'confidence': float(detection['score'])})
                used |= mask
        return {'backend': VISION_BACKEND, 'detections': observations}


def capture_frame(model, data, renderer):
    """Read only camera pixels and calibration, never dynamic object poses."""
    if renderer is None:
        raise RuntimeError('RGB-D renderer is not initialized')
    renderer.update_scene(data, camera='realsense')
    rgb = renderer.render().copy()
    renderer.enable_depth_rendering()
    try:
        renderer.update_scene(data, camera='realsense')
        depth = renderer.render().astype(np.float32).copy()
    finally:
        renderer.disable_depth_rendering()
    camera_id = model.camera('realsense').id
    height, width = depth.shape
    focal = height / (2 * math.tan(math.radians(model.cam_fovy[camera_id]) / 2))
    K = np.array([[focal, 0, width/2], [0, focal, height/2], [0, 0, 1]])
    transform = np.eye(4)
    transform[:3, :3] = data.cam_xmat[camera_id].reshape(3, 3) @ np.diag([1, -1, -1])
    transform[:3, 3] = data.cam_xpos[camera_id]
    validate_frame(rgb, depth, K, transform)
    return rgb, depth, K, transform
