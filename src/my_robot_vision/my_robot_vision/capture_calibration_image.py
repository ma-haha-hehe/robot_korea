#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""Save one calibration image from the gripper-mounted camera.

Default backend uses OpenCV /dev/videoN because it works for many USB and
RealSense color streams without starting a ROS camera node.
"""

import argparse
import os
import time
from datetime import datetime

import cv2


def reset_realsense():
    import pyrealsense2 as rs

    ctx = rs.context()
    devices = list(ctx.query_devices())
    if not devices:
        raise RuntimeError("no RealSense device found")
    for device in devices:
        print(
            "resetting",
            device.get_info(rs.camera_info.name),
            device.get_info(rs.camera_info.serial_number),
        )
        device.hardware_reset()
    time.sleep(5)


def capture_with_opencv(device: int, width: int, height: int, warmup: int):
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        raise RuntimeError(f"cannot open camera device index {device}")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, 30)

    frame = None
    for _ in range(max(1, warmup)):
        ok, frame = cap.read()
        if not ok:
            frame = None
    cap.release()

    if frame is None:
        raise RuntimeError("camera opened but no frame was received")
    return frame


def capture_with_realsense(width: int, height: int, warmup: int, disable_emitter: bool):
    import pyrealsense2 as rs
    import numpy as np

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 30)
    try:
        profile = pipeline.start(config)
    except RuntimeError as error:
        raise RuntimeError(
            "cannot start RealSense stream. Close realsense-viewer or any other "
            "camera program first. A safe calibration setting is "
            "--backend realsense --width 640 --height 480."
        ) from error
    if disable_emitter:
        for sensor in profile.get_device().query_sensors():
            if sensor.supports(rs.option.emitter_enabled):
                sensor.set_option(rs.option.emitter_enabled, 0)
            if sensor.supports(rs.option.laser_power):
                sensor.set_option(rs.option.laser_power, 0)
    try:
        frame = None
        for _ in range(max(1, warmup)):
            frames = pipeline.wait_for_frames()
            color = frames.get_color_frame()
            if color:
                frame = np.asanyarray(color.get_data())
        if frame is None:
            raise RuntimeError("RealSense opened but no color frame was received")
        return frame
    finally:
        pipeline.stop()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--backend",
        choices=["opencv", "realsense"],
        default="opencv",
        help="opencv is /dev/videoN, realsense uses pyrealsense2",
    )
    parser.add_argument("--device", type=int, default=0, help="OpenCV camera index")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--warmup", type=int, default=30)
    parser.add_argument(
        "--reset",
        action="store_true",
        help="hardware-reset RealSense before capturing",
    )
    parser.add_argument(
        "--disable-emitter",
        action="store_true",
        help="turn off RealSense IR projector before capturing",
    )
    parser.add_argument(
        "--output-dir",
        default="/home/i6user/Desktop/robot_lego/calibration_images",
    )
    parser.add_argument("--name", default="", help="optional filename without extension")
    args = parser.parse_args()

    if args.reset:
        reset_realsense()

    if args.backend == "realsense":
        frame = capture_with_realsense(args.width, args.height, args.warmup, args.disable_emitter)
    else:
        frame = capture_with_opencv(args.device, args.width, args.height, args.warmup)

    os.makedirs(args.output_dir, exist_ok=True)
    stem = args.name or datetime.now().strftime("calib_%Y%m%d_%H%M%S")
    output_path = os.path.join(args.output_dir, stem + ".jpg")
    if not cv2.imwrite(output_path, frame):
        raise RuntimeError(f"failed to write {output_path}")
    print(output_path)


if __name__ == "__main__":
    main()
