#!/usr/bin/env python3
"""Read-only ChArUco RGB/depth collector for the running RealSense ROS node."""

import argparse
import collections
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from realsense2_camera_msgs.msg import Extrinsics
from sensor_msgs.msg import CameraInfo, Image


BOARD_COLS = 9
BOARD_ROWS = 7
SQUARE_SIZE_M = 0.020
MARKER_SIZE_M = 0.015
TARGETS = (
    "近距离约 0.35 m，画面中央，板面基本正对相机",
    "近距离约 0.35 m，画面左侧，板面轻微向右倾斜",
    "近距离约 0.35 m，画面右侧，板面轻微向下倾斜",
    "中距离约 0.50 m，画面中央，板面轻微向左倾斜",
    "中距离约 0.50 m，画面左侧，板面轻微向上倾斜",
    "中距离约 0.50 m，画面右侧，板面基本正对相机",
    "远距离约 0.65 m，画面中央，板面轻微向下倾斜",
    "远距离约 0.65 m，画面左侧，板面基本正对相机",
    "远距离约 0.65 m，画面右侧，板面轻微向右倾斜",
)


def stamp_seconds(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1.0e-9


def image_array(message):
    encoding = message.encoding.lower()
    if encoding in ("bgr8", "rgb8"):
        row = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
        image = row[:, :message.width * 3].reshape(message.height, message.width, 3).copy()
        return image if encoding == "bgr8" else cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if encoding in ("16uc1", "mono16"):
        row = np.frombuffer(message.data, dtype=np.uint16).reshape(
            message.height, message.step // 2)
        return row[:, :message.width].copy()
    if encoding == "32fc1":
        row = np.frombuffer(message.data, dtype=np.float32).reshape(
            message.height, message.step // 4)
        return row[:, :message.width].copy()
    raise RuntimeError("unsupported image encoding: {}".format(message.encoding))


def create_board(dictionary):
    if hasattr(cv2.aruco, "CharucoBoard_create"):
        return cv2.aruco.CharucoBoard_create(
            BOARD_COLS, BOARD_ROWS, SQUARE_SIZE_M, MARKER_SIZE_M, dictionary)
    return cv2.aruco.CharucoBoard(
        (BOARD_COLS, BOARD_ROWS), SQUARE_SIZE_M, MARKER_SIZE_M, dictionary)


def board_points(board):
    if hasattr(board, "chessboardCorners"):
        return np.asarray(board.chessboardCorners, dtype=np.float32)
    return np.asarray(board.getChessboardCorners(), dtype=np.float32)


class ImageBuffer(Node):
    def __init__(self, color_topic, depth_topic, color_info_topic,
                 depth_info_topic, extrinsics_topic, buffer_size):
        super().__init__("plasma_depth_charuco_collector")
        self.lock = threading.Lock()
        self.colors = collections.deque(maxlen=buffer_size)
        self.depths = collections.deque(maxlen=buffer_size)
        self.color_info = None
        self.depth_info = None
        self.extrinsics = None
        self.create_subscription(Image, color_topic, self.color_callback,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, depth_topic, self.depth_callback,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, color_info_topic, self.color_info_callback,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, depth_info_topic, self.depth_info_callback,
                                 qos_profile_sensor_data)
        extrinsics_qos = QoSProfile(depth=1)
        extrinsics_qos.reliability = ReliabilityPolicy.RELIABLE
        extrinsics_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(Extrinsics, extrinsics_topic, self.extrinsics_callback,
                                 extrinsics_qos)

    def color_callback(self, message):
        try:
            value = image_array(message)
        except RuntimeError as error:
            self.get_logger().error(str(error))
            return
        with self.lock:
            self.colors.append((stamp_seconds(message), value, message.encoding))

    def depth_callback(self, message):
        try:
            value = image_array(message)
        except RuntimeError as error:
            self.get_logger().error(str(error))
            return
        with self.lock:
            self.depths.append((stamp_seconds(message), value, message.encoding))

    def color_info_callback(self, message):
        with self.lock:
            self.color_info = message

    def depth_info_callback(self, message):
        with self.lock:
            self.depth_info = message

    def extrinsics_callback(self, message):
        with self.lock:
            self.extrinsics = message

    def snapshot(self, median_frames):
        with self.lock:
            if (not self.colors or not self.depths or self.color_info is None or
                    self.depth_info is None or self.extrinsics is None):
                return None
            color_stamp, color, color_encoding = self.colors[-1]
            candidates = [entry for entry in self.depths
                          if abs(entry[0] - color_stamp) <= 0.75]
            candidates = candidates[-median_frames:]
            if not candidates:
                return None
            shape = candidates[-1][1].shape
            candidates = [entry for entry in candidates if entry[1].shape == shape]
            depth_stack = np.stack([entry[1].astype(np.float64) for entry in candidates])
            valid = depth_stack > 0
            depth_stack[~valid] = np.nan
            with np.errstate(all="ignore"):
                depth = np.nanmedian(depth_stack, axis=0)
            depth = np.nan_to_num(depth, nan=0.0)
            return {
                "color_stamp": color_stamp,
                "depth_stamp": candidates[-1][0],
                "color": color.copy(),
                "depth": depth,
                "depth_encoding": candidates[-1][2],
                "color_encoding": color_encoding,
                "frames": len(candidates),
                "color_info": self.color_info,
                "depth_info": self.depth_info,
                "extrinsics": self.extrinsics,
            }


def camera_parameters(info):
    return {
        "width": int(info.width),
        "height": int(info.height),
        "distortion_model": info.distortion_model,
        "K": [float(value) for value in info.k],
        "D": [float(value) for value in info.d],
    }


def detect_board(color, info, min_corners):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    board = create_board(dictionary)
    gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
    marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
    if marker_ids is None:
        return None
    camera = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
    distortion = np.asarray(info.d, dtype=np.float64)
    _, corners, ids = cv2.aruco.interpolateCornersCharuco(
        marker_corners, marker_ids, gray, board,
        cameraMatrix=camera, distCoeffs=distortion)
    if ids is None or len(ids) < min_corners:
        return None
    indices = ids.flatten().astype(int)
    objects = board_points(board)[indices]
    ok, rvec, tvec = cv2.solvePnP(
        objects, corners, camera, distortion, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    projected, _ = cv2.projectPoints(objects, rvec, tvec, camera, distortion)
    reprojection = float(np.mean(np.linalg.norm(
        corners.reshape(-1, 2) - projected.reshape(-1, 2), axis=1)))
    rotation, _ = cv2.Rodrigues(rvec)
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = tvec.reshape(3)
    return {
        "corners": corners,
        "ids": ids,
        "marker_corners": marker_corners,
        "marker_ids": marker_ids,
        "T_camera_to_board": transform,
        "reprojection_px": reprojection,
        "board_distance_m": float(np.linalg.norm(tvec)),
    }


def factory_transform(extrinsics):
    transform = np.eye(4, dtype=np.float64)
    # librealsense publishes rs2_extrinsics.rotation in column-major order.
    transform[:3, :3] = np.asarray(
        extrinsics.rotation, dtype=np.float64).reshape(3, 3, order="F")
    transform[:3, 3] = np.asarray(extrinsics.translation, dtype=np.float64)
    return transform


def sample_depth_points(depth, detection, color_info, depth_info, extrinsics, max_points):
    corners = detection["corners"].reshape(-1, 2)
    hull = cv2.convexHull(corners.astype(np.float32))
    color_mask = np.zeros((color_info.height, color_info.width), dtype=np.uint8)
    cv2.fillConvexPoly(color_mask, np.round(hull).astype(np.int32), 255)
    color_mask = cv2.erode(color_mask, np.ones((11, 11), dtype=np.uint8), iterations=1)

    valid = np.isfinite(depth) & (depth > 0)
    vv, uu = np.nonzero(valid)
    # ROS RealSense 16UC1 depth uses millimetres. Use this only for projection;
    # the raw values are retained for the offline scale/bias fit.
    raw = depth[vv, uu]
    stride = max(1, int(np.sqrt(len(raw) / 50000.0)))
    uu, vv, raw = uu[::stride], vv[::stride], raw[::stride]
    fx_d, fy_d = float(depth_info.k[0]), float(depth_info.k[4])
    cx_d, cy_d = float(depth_info.k[2]), float(depth_info.k[5])
    z_nominal = raw * 0.001
    depth_points = np.column_stack((
        (uu - cx_d) * z_nominal / fx_d,
        (vv - cy_d) * z_nominal / fy_d,
        z_nominal,
    ))
    transform = factory_transform(extrinsics)
    rotation_vector, _ = cv2.Rodrigues(transform[:3, :3])
    camera = np.asarray(color_info.k, dtype=np.float64).reshape(3, 3)
    distortion = np.asarray(color_info.d, dtype=np.float64)
    projected, _ = cv2.projectPoints(
        depth_points, rotation_vector, transform[:3, 3], camera, distortion)
    projected = np.rint(projected.reshape(-1, 2)).astype(int)
    inside = ((projected[:, 0] >= 0) & (projected[:, 0] < color_info.width) &
              (projected[:, 1] >= 0) & (projected[:, 1] < color_info.height))
    projected_valid = projected[inside]
    selected = np.zeros(len(raw), dtype=bool)
    inside_indices = np.nonzero(inside)[0]
    selected[inside_indices] = color_mask[
        projected_valid[:, 1], projected_valid[:, 0]] > 0
    uu, vv, raw = uu[selected], vv[selected], raw[selected]
    if len(raw) < 250:
        raise RuntimeError("投影到标定板内部的有效深度点不足: {}".format(len(raw)))
    if len(raw) > max_points:
        positions = np.linspace(0, len(raw) - 1, max_points, dtype=int)
        uu, vv, raw = uu[positions], vv[positions], raw[positions]
    rays = np.column_stack(((uu - cx_d) / fx_d, (vv - cy_d) / fy_d, raw))
    return rays, color_mask, transform


def matrix_list(matrix):
    return [[float(value) for value in row] for row in matrix]


def save_dataset(output, args, samples, snapshot, detection, points, mask, transform):
    index = len(samples) + 1
    stem = "sample_{:02d}".format(index)
    cv2.imwrite(str(output / (stem + "_color.png")), snapshot["color"])
    depth_to_save = np.rint(snapshot["depth"]).astype(np.uint16)
    cv2.imwrite(str(output / (stem + "_depth_raw.png")), depth_to_save)
    cv2.imwrite(str(output / (stem + "_mask.png")), mask)
    sample = {
        "name": stem,
        "target": TARGETS[index - 1] if index <= len(TARGETS) else "additional",
        "captured_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "color_stamp": float(snapshot["color_stamp"]),
        "depth_stamp": float(snapshot["depth_stamp"]),
        "color_depth_stamp_delta_ms": float(abs(
            snapshot["color_stamp"] - snapshot["depth_stamp"]) * 1000.0),
        "median_depth_frames": int(snapshot["frames"]),
        "depth_encoding": snapshot["depth_encoding"],
        "corners": int(len(detection["ids"])),
        "reprojection_px": detection["reprojection_px"],
        "board_distance_m": detection["board_distance_m"],
        "T_camera_to_board": matrix_list(detection["T_camera_to_board"]),
        "depth_ray_raw": [[float(value) for value in row] for row in points],
        "files": {
            "color": stem + "_color.png",
            "depth_raw": stem + "_depth_raw.png",
            "mask": stem + "_mask.png",
        },
    }
    samples.append(sample)
    payload = {
        "schema_version": 1,
        "status": "raw_offline_depth_calibration_dataset",
        "camera_parameters_modified": False,
        "robot_connected": False,
        "motion_commands_published": False,
        "topics": {
            "color": args.color_topic,
            "depth": args.depth_topic,
            "color_camera_info": args.color_info_topic,
            "depth_camera_info": args.depth_info_topic,
            "depth_to_color_extrinsics": args.extrinsics_topic,
        },
        "board": {
            "type": "ChArUco",
            "squares": [BOARD_COLS, BOARD_ROWS],
            "square_size_mm": SQUARE_SIZE_M * 1000.0,
            "marker_size_mm": MARKER_SIZE_M * 1000.0,
            "dictionary": "DICT_4X4_50",
        },
        "color_camera_info": camera_parameters(snapshot["color_info"]),
        "depth_camera_info": camera_parameters(snapshot["depth_info"]),
        "T_color_from_depth_factory": matrix_list(transform),
        "raw_depth_unit_hint_m": 0.001,
        "quality_gates": {
            "minimum_charuco_corners": int(args.min_corners),
            "maximum_color_depth_stamp_delta_ms": float(args.max_sync_offset_ms),
            "minimum_projected_depth_points": 250,
        },
        "samples": samples,
    }
    with open(output / "dataset.yaml", "w", encoding="utf-8") as stream:
        yaml.safe_dump(payload, stream, allow_unicode=True, sort_keys=False)


def preview_image(snapshot, detection, sample_count, target_count, sync_ready):
    preview = snapshot["color"].copy()
    if detection:
        cv2.aruco.drawDetectedMarkers(
            preview, detection["marker_corners"], detection["marker_ids"])
        cv2.aruco.drawDetectedCornersCharuco(
            preview, detection["corners"], detection["ids"])
    status = "READY" if detection and sync_ready else (
        "WAIT FOR RGB/DEPTH SYNC" if detection else "BOARD NOT DETECTED")
    cv2.rectangle(preview, (0, 0), (preview.shape[1], 62), (0, 0, 0), -1)
    cv2.putText(preview, "{}  samples {}/{}".format(status, sample_count, target_count),
                (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (80, 220, 80) if detection and sync_ready else (40, 80, 230), 2)
    cv2.putText(preview, "SPACE: capture   Q: finish", (12, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (230, 230, 230), 1)
    return preview


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--samples", type=int, default=9)
    parser.add_argument("--median-frames", type=int, default=15)
    parser.add_argument("--max-depth-points", type=int, default=800)
    parser.add_argument("--min-corners", type=int, default=35)
    parser.add_argument("--max-sync-offset-ms", type=float, default=50.0)
    parser.add_argument("--color-topic", default="/camera/color/image_raw")
    parser.add_argument("--depth-topic", default="/camera/depth/image_rect_raw")
    parser.add_argument("--color-info-topic", default="/camera/color/camera_info")
    parser.add_argument("--depth-info-topic", default="/camera/depth/camera_info")
    parser.add_argument("--extrinsics-topic", default="/camera/extrinsics/depth_to_color")
    args = parser.parse_args()
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = ImageBuffer(args.color_topic, args.depth_topic, args.color_info_topic,
                       args.depth_info_topic, args.extrinsics_topic,
                       max(30, args.median_frames * 2))
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    samples = []
    window = "Depth ChArUco Collector"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, 960, 720)
    print("只读订阅现有 ROS 相机，不修改参数，不连接机械臂。")
    print("输出目录: {}".format(output))

    try:
        while rclpy.ok() and len(samples) < args.samples:
            snapshot = node.snapshot(args.median_frames)
            if snapshot is None:
                print("等待彩色、对齐深度和 CameraInfo...", end="\r", flush=True)
                time.sleep(0.1)
                cv2.waitKey(1)
                continue
            detection = detect_board(snapshot["color"], snapshot["color_info"],
                                     args.min_corners)
            sync_offset_ms = abs(
                snapshot["color_stamp"] - snapshot["depth_stamp"]) * 1000.0
            sync_ready = sync_offset_ms <= args.max_sync_offset_ms
            target = TARGETS[len(samples)] if len(samples) < len(TARGETS) else "additional"
            preview = preview_image(
                snapshot, detection, len(samples), args.samples, sync_ready)
            cv2.imshow(window, preview)
            print("下一张: {}                         ".format(target), end="\r", flush=True)
            key = cv2.waitKey(30) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
            if key not in (ord(" "), ord("c"), ord("C")):
                continue
            if detection is None:
                print("\n未检测到足够角点，本次未保存。")
                continue
            if not sync_ready:
                print("\n彩深时间差 {:.1f} ms 超过 {:.1f} ms，本次未保存。".format(
                    sync_offset_ms, args.max_sync_offset_ms))
                continue
            try:
                points, mask, transform = sample_depth_points(
                    snapshot["depth"], detection, snapshot["color_info"],
                    snapshot["depth_info"], snapshot["extrinsics"],
                    args.max_depth_points)
                save_dataset(output, args, samples, snapshot, detection, points, mask,
                             transform)
            except RuntimeError as error:
                print("\n{}，本次未保存。".format(error))
                continue
            print("\n已保存 {}/{}: 距离 {:.3f} m，角点 {}，PnP {:.3f} px，深度点 {}".format(
                len(samples), args.samples, detection["board_distance_m"],
                len(detection["ids"]), detection["reprojection_px"], len(points)))
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)
    print("\n采集结束，已保存 {}/{} 组。".format(len(samples), args.samples))
    if len(samples) < args.samples:
        print("数据不足，不允许生成校正候选。")


if __name__ == "__main__":
    main()
