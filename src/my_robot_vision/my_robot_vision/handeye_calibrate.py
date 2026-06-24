#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""Compute eye-in-hand calibration for a camera mounted on UR5 tool0.

Input is a YAML file containing image paths and the matching T_base_tool
matrix printed by cpp_pick_node_ur5 motion_control_mode:=print_pose.
The output is T_tool_camera, which can be used at runtime as:

    T_base_camera = T_base_tool_current @ T_tool_camera
    T_base_object = T_base_camera @ T_camera_object
"""

import argparse
import math
import os
from typing import Any, Dict, List, Tuple

import cv2
import numpy as np
import yaml


def load_yaml(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return data


def as_matrix4(value: Any, label: str) -> np.ndarray:
    mat = np.asarray(value, dtype=float)
    if mat.shape != (4, 4):
        raise ValueError(f"{label} must be a 4x4 matrix")
    return mat


def chessboard_object_points(board_size: Tuple[int, int], square_size: float) -> np.ndarray:
    cols, rows = board_size
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= float(square_size)
    return objp


def find_chessboard(image_path: str,
                    board_size: Tuple[int, int],
                    annotated_dir: str) -> Tuple[np.ndarray, Tuple[int, int]]:
    image = cv2.imread(image_path, cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"cannot read image: {image_path}")

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, corners = cv2.findChessboardCorners(gray, board_size, flags)
    if not ok:
        raise ValueError(f"chessboard not found: {image_path}")

    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        50,
        0.001,
    )
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

    if annotated_dir:
        os.makedirs(annotated_dir, exist_ok=True)
        annotated = image.copy()
        cv2.drawChessboardCorners(annotated, board_size, corners, ok)
        out_path = os.path.join(annotated_dir, os.path.basename(image_path))
        cv2.imwrite(out_path, annotated)

    h, w = gray.shape[:2]
    return corners.reshape(-1, 2).astype(np.float32), (w, h)


def aruco_dictionary(name: str):
    if not hasattr(cv2, "aruco"):
        raise ValueError("OpenCV was built without cv2.aruco")
    if not hasattr(cv2.aruco, name):
        raise ValueError(f"unknown ArUco dictionary: {name}")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def aruco_dictionary_names(name: str) -> List[str]:
    if name.upper() != "AUTO":
        return [name]
    preferred = [
        "DICT_4X4_50",
        "DICT_4X4_100",
        "DICT_5X5_50",
        "DICT_5X5_100",
        "DICT_6X6_50",
        "DICT_6X6_100",
        "DICT_APRILTAG_16h5",
        "DICT_APRILTAG_25h9",
        "DICT_APRILTAG_36h11",
        "DICT_ARUCO_ORIGINAL",
    ]
    return [item for item in preferred if hasattr(cv2.aruco, item)]


def board_match_points(board, marker_corners, marker_ids) -> Tuple[np.ndarray, np.ndarray]:
    if marker_ids is None or len(marker_ids) == 0:
        raise ValueError("no ArUco markers detected")

    board_ids = np.asarray(board.getIds(), dtype=np.int32).reshape(-1)
    board_obj_points = list(board.getObjPoints())
    board_by_id = {
        int(marker_id): np.asarray(points, dtype=np.float32).reshape(4, 3)
        for marker_id, points in zip(board_ids, board_obj_points)
    }

    object_rows = []
    image_rows = []
    for corners, marker_id in zip(marker_corners, marker_ids.reshape(-1)):
        marker_id = int(marker_id)
        if marker_id not in board_by_id:
            continue
        image_corners = np.asarray(corners, dtype=np.float32).reshape(4, 2)
        object_rows.append(board_by_id[marker_id])
        image_rows.append(image_corners)

    if not object_rows:
        raise ValueError("detected marker ids do not belong to this board")

    obj_points = np.concatenate(object_rows, axis=0)
    img_points = np.concatenate(image_rows, axis=0)
    if len(obj_points) < 4:
        raise ValueError(f"not enough board points for pose: {len(obj_points)} < 4")
    return obj_points, img_points


def find_charuco_or_grid(image_path: str,
                         board_size: Tuple[int, int],
                         square_size: float,
                         marker_size: float,
                         dictionary_name: str,
                         marker_ids_config: List[int],
                         annotated_dir: str) -> Tuple[np.ndarray, np.ndarray, Tuple[int, int]]:
    image = cv2.imread(image_path, cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"cannot read image: {image_path}")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    squares_x, squares_y = board_size
    last_error = None
    best = None
    for candidate_name in aruco_dictionary_names(dictionary_name):
        dictionary = aruco_dictionary(candidate_name)
        board_args = [
            (int(squares_x), int(squares_y)),
            float(square_size),
            float(marker_size),
            dictionary,
        ]
        if marker_ids_config:
            board_args.append(np.asarray(marker_ids_config, dtype=np.int32))
        charuco_board = cv2.aruco.CharucoBoard(*board_args)
        aruco_detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
        marker_corners, marker_ids, _ = aruco_detector.detectMarkers(gray)
        if marker_ids is not None and len(marker_ids) > 0:
            best = (candidate_name, dictionary, charuco_board, marker_corners, marker_ids)
            break
        last_error = f"{candidate_name}: no markers"

    if best is None:
        raise ValueError(
            f"no ArUco/ChArUco markers detected in {image_path}. "
            "Use the color image, keep the board sharp, and turn off the RealSense emitter."
        )
    used_dictionary_name, dictionary, charuco_board, marker_corners, marker_ids = best

    # Prefer interpolated ChArUco corners when enough exist. A true 2x2 board
    # has only one ChArUco corner, so it falls back to marker-corner GridBoard
    # matching below.
    charuco_detector = cv2.aruco.CharucoDetector(charuco_board)
    charuco_corners, charuco_ids, _, _ = charuco_detector.detectBoard(gray)
    annotated = image.copy()

    if charuco_ids is not None and len(charuco_ids) >= 4:
        all_corners = charuco_board.getChessboardCorners()
        obj_points = np.asarray(
            [all_corners[int(idx[0])] for idx in charuco_ids],
            dtype=np.float32,
        ).reshape(-1, 3)
        img_points = np.asarray(charuco_corners, dtype=np.float32).reshape(-1, 2)
        cv2.aruco.drawDetectedCornersCharuco(annotated, charuco_corners, charuco_ids)
    else:
        obj_points, img_points = board_match_points(charuco_board, marker_corners, marker_ids)
        cv2.aruco.drawDetectedMarkers(annotated, marker_corners, marker_ids)

    if annotated_dir:
        os.makedirs(annotated_dir, exist_ok=True)
        out_path = os.path.join(annotated_dir, os.path.basename(image_path))
        cv2.putText(
            annotated,
            used_dictionary_name,
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.imwrite(out_path, annotated)

    h, w = gray.shape[:2]
    return obj_points, img_points, (w, h)


def solve_target_to_camera(objp: np.ndarray,
                           imgp: np.ndarray,
                           camera_matrix: np.ndarray,
                           dist_coeffs: np.ndarray) -> np.ndarray:
    ok, rvec, tvec = cv2.solvePnP(
        objp,
        imgp,
        camera_matrix,
        dist_coeffs,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        raise ValueError("solvePnP failed")
    rot, _ = cv2.Rodrigues(rvec)
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rot
    transform[:3, 3] = tvec.reshape(3)
    return transform


def write_output(path: str,
                 transform: np.ndarray,
                 camera_matrix: np.ndarray,
                 dist_coeffs: np.ndarray,
                 translation_std: np.ndarray,
                 used_samples: List[str]) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    data = {
        "T_tool_camera": transform.tolist(),
        "camera_matrix": camera_matrix.tolist(),
        "dist_coeffs": dist_coeffs.reshape(-1).tolist(),
        "diagnostics": {
            "used_samples": used_samples,
            "base_target_translation_std_m": translation_std.tolist(),
        },
    }
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False)


def run(args: argparse.Namespace) -> None:
    config = load_yaml(args.samples)
    pattern = str(config.get("pattern", "chessboard")).lower()
    board_size = tuple(int(v) for v in config.get("board_size", [9, 6]))
    if len(board_size) != 2:
        raise ValueError("board_size must be [cols, rows] inner corners")
    square_size = float(config.get("square_size", 0.025))
    marker_size = float(config.get("marker_size", square_size * 0.7))
    dictionary_name = str(config.get("dictionary", "DICT_4X4_50"))
    marker_ids_config = [int(item) for item in config.get("marker_ids", [])]
    samples = config.get("samples", [])
    if not isinstance(samples, list) or len(samples) < 5:
        raise ValueError("need at least 5 samples; 10-20 is much better")

    chessboard_objp = chessboard_object_points(board_size, square_size)
    object_points = []
    image_points = []
    image_size = None
    base_tool_transforms = []
    used_samples = []

    for sample in samples:
        image_path = str(sample["image"])
        if pattern == "chessboard":
            corners, current_size = find_chessboard(image_path, board_size, args.annotated_dir)
            current_obj_points = chessboard_objp
            current_img_points = corners
        elif pattern in ("charuco", "aruco_grid", "gridboard"):
            current_obj_points, current_img_points, current_size = find_charuco_or_grid(
                image_path,
                board_size,
                square_size,
                marker_size,
                dictionary_name,
                marker_ids_config,
                args.annotated_dir,
            )
        else:
            raise ValueError("pattern must be chessboard, charuco, aruco_grid, or gridboard")
        if image_size is None:
            image_size = current_size
        elif image_size != current_size:
            raise ValueError("all calibration images must have the same resolution")
        object_points.append(current_obj_points)
        image_points.append(current_img_points)
        base_tool_transforms.append(as_matrix4(sample["T_base_tool"], f"{image_path} T_base_tool"))
        used_samples.append(image_path)

    if config.get("camera_matrix") is None:
        rms, camera_matrix, dist_coeffs, _, _ = cv2.calibrateCamera(
            object_points,
            image_points,
            image_size,
            None,
            None,
        )
        print(f"camera intrinsics estimated from photos, reprojection rms={rms:.4f}")
    else:
        camera_matrix = np.asarray(config["camera_matrix"], dtype=float)
        if camera_matrix.shape != (3, 3):
            raise ValueError("camera_matrix must be 3x3")
        dist_coeffs = np.asarray(config.get("dist_coeffs") or [0, 0, 0, 0, 0], dtype=float).reshape(-1, 1)

    cam_target_transforms = [
        solve_target_to_camera(obj, img, camera_matrix, dist_coeffs)
        for obj, img in zip(object_points, image_points)
    ]

    rotations_gripper_to_base = [tf[:3, :3] for tf in base_tool_transforms]
    translations_gripper_to_base = [tf[:3, 3].reshape(3, 1) for tf in base_tool_transforms]
    rotations_target_to_cam = [tf[:3, :3] for tf in cam_target_transforms]
    translations_target_to_cam = [tf[:3, 3].reshape(3, 1) for tf in cam_target_transforms]

    method_map = {
        "tsai": cv2.CALIB_HAND_EYE_TSAI,
        "park": cv2.CALIB_HAND_EYE_PARK,
        "horaud": cv2.CALIB_HAND_EYE_HORAUD,
        "andreff": cv2.CALIB_HAND_EYE_ANDREFF,
        "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }
    method = method_map.get(args.method.lower())
    if method is None:
        raise ValueError(f"unknown hand-eye method: {args.method}")

    r_cam_to_gripper, t_cam_to_gripper = cv2.calibrateHandEye(
        rotations_gripper_to_base,
        translations_gripper_to_base,
        rotations_target_to_cam,
        translations_target_to_cam,
        method=method,
    )

    tool_camera = np.eye(4, dtype=float)
    tool_camera[:3, :3] = r_cam_to_gripper
    tool_camera[:3, 3] = t_cam_to_gripper.reshape(3)

    base_target_translations = []
    for base_tool, cam_target in zip(base_tool_transforms, cam_target_transforms):
        base_target = base_tool @ tool_camera @ cam_target
        base_target_translations.append(base_target[:3, 3])
    translation_std = np.std(np.asarray(base_target_translations), axis=0)

    write_output(
        args.output,
        tool_camera,
        camera_matrix,
        dist_coeffs,
        translation_std,
        used_samples,
    )

    print("T_tool_camera written to:", args.output)
    print("T_tool_camera:")
    for row in tool_camera:
        print("  ", ["%.9f" % v for v in row])
    print(
        "base target translation std [m]:",
        ["%.6f" % v for v in translation_std],
    )
    print("Rule of thumb: each std value should ideally be below 0.005-0.010 m.")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", required=True, help="YAML file with calibration samples")
    parser.add_argument(
        "--output",
        default="/home/i6user/Desktop/robot_lego/src/my_robot_vision/config/camera_to_tool.yaml",
        help="output YAML path",
    )
    parser.add_argument(
        "--annotated-dir",
        default="/home/i6user/Desktop/robot_lego/src/my_robot_vision/calibration_debug",
        help="where detected-corner images are written; empty disables it",
    )
    parser.add_argument(
        "--method",
        default="tsai",
        choices=["tsai", "park", "horaud", "andreff", "daniilidis"],
    )
    run(parser.parse_args())


if __name__ == "__main__":
    main()
