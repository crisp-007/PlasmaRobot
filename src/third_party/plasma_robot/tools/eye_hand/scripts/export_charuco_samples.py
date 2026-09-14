#!/usr/bin/env python3
"""Export synchronized ChArUco/robot poses for the C++ offline analyzer."""

import argparse
import csv
import json
from pathlib import Path

import cv2
import numpy as np
import yaml


def matrix(rotation, translation):
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation
    result[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return result.tolist()


def quaternion_rotation(qw, qx, qy, qz):
    quaternion = np.array([qx, qy, qz, qw], dtype=np.float64)
    quaternion /= np.linalg.norm(quaternion)
    x, y, z, w = quaternion
    return np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y],
    ], dtype=np.float64)


def create_board(dictionary):
    if hasattr(cv2.aruco, "CharucoBoard_create"):
        return cv2.aruco.CharucoBoard_create(9, 7, 0.020, 0.015, dictionary)
    return cv2.aruco.CharucoBoard((9, 7), 0.020, 0.015, dictionary)


def chessboard_corners(board):
    if hasattr(board, "chessboardCorners"):
        return board.chessboardCorners
    return board.getChessboardCorners()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    dataset = Path(args.dataset).resolve()
    output = Path(args.output).resolve()

    with open(dataset / "camera_intrinsics.json", encoding="utf-8") as stream:
        intrinsics = json.load(stream)
    camera = np.array([
        [intrinsics["fx"], 0.0, intrinsics["cx"]],
        [0.0, intrinsics["fy"], intrinsics["cy"]],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)
    distortion = np.asarray(intrinsics["dist_coeffs"], dtype=np.float64).reshape(-1, 1)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    board = create_board(dictionary)
    board_points = chessboard_corners(board)

    samples = []
    with open(dataset / "robot_poses.csv", encoding="utf-8") as stream:
        for pose in csv.DictReader(stream):
            name = pose["image"].strip()
            image = cv2.imread(str(dataset / "images" / name))
            if image is None:
                raise RuntimeError("cannot read {}".format(name))
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
            if marker_ids is None:
                raise RuntimeError("no markers in {}".format(name))
            _, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                marker_corners, marker_ids, gray, board,
                cameraMatrix=camera, distCoeffs=distortion)
            if charuco_ids is None or len(charuco_ids) < 25:
                raise RuntimeError("fewer than 25 corners in {}".format(name))
            ids = charuco_ids.flatten().astype(int)
            object_points = np.asarray(board_points, dtype=np.float32)[ids]
            image_points = charuco_corners
            ok, rvec, tvec = cv2.solvePnP(
                object_points, image_points, camera, distortion,
                flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok:
                raise RuntimeError("solvePnP failed for {}".format(name))
            projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera, distortion)
            reprojection = float(np.mean(np.linalg.norm(
                image_points.reshape(-1, 2) - projected.reshape(-1, 2), axis=1)))
            rotation, _ = cv2.Rodrigues(rvec)
            gripper_rotation = quaternion_rotation(
                float(pose["qw"]), float(pose["qx"]),
                float(pose["qy"]), float(pose["qz"]))
            samples.append({
                "name": name,
                "T_base_to_gripper": matrix(gripper_rotation, [
                    float(pose["x"]), float(pose["y"]), float(pose["z"])]),
                "T_camera_to_board": matrix(rotation, tvec),
                "reprojection_px": reprojection,
                "corners": int(len(charuco_ids)),
            })

    payload = {
        "schema_version": 1,
        "status": "offline_pnp_cache",
        "source_dataset": str(dataset),
        "camera_serial": intrinsics.get("serial"),
        "samples": samples,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    with open(output, "w", encoding="utf-8") as stream:
        yaml.safe_dump(payload, stream, allow_unicode=True, sort_keys=False)
    print("exported {} samples to {}".format(len(samples), output))


if __name__ == "__main__":
    main()
