#!/usr/bin/env python3
"""Robust eye-to-hand calibration from synchronised 4x4 pose samples.

For the fixed-camera setup used by Thor7 the optimized model is

    T_base_camera * T_camera_target_i = T_base_eef_i * T_eef_target

Both the camera pose and the rigid board-to-eef mount are optimized together.
This is a nonlinear SE(3) bundle adjustment with a robust loss, rather than a
single closed-form OpenCV hand-eye result.  The input also accepts the list
format written by the MoveIt RViz plugin for backwards-compatible diagnostics.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import tempfile
from pathlib import Path
from typing import Any, Iterable, Optional

import cv2
import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def matrix_from_value(value: Any, name: str) -> np.ndarray:
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.size != 16:
        raise ValueError(f"{name} must contain 16 numbers")
    matrix = matrix.reshape(4, 4)
    if not np.all(np.isfinite(matrix)):
        raise ValueError(f"{name} contains NaN or infinity")
    if np.linalg.norm(matrix[3, :3]) > 1e-8 or abs(matrix[3, 3] - 1.0) > 1e-8:
        raise ValueError(f"{name} is not homogeneous")
    if abs(np.linalg.det(matrix[:3, :3])) < 1e-8:
        raise ValueError(f"{name} has a singular rotation block")
    return matrix


def load_rows(path: Path) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    try:
        root = yaml.safe_load(path.read_text())
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f"cannot read {path}: {exc}") from exc
    metadata: dict[str, Any] = root if isinstance(root, dict) else {}
    if isinstance(root, list):
        rows = root
    elif isinstance(root, dict) and isinstance(root.get("samples"), list):
        rows = root["samples"]
    elif isinstance(root, dict) and isinstance(root.get("data"), list):
        rows = root["data"]
    else:
        raise ValueError("input must be a YAML list or a map containing samples:")
    valid = [row for row in rows if isinstance(row, dict)]
    if len(valid) != len(rows):
        raise ValueError("input contains a non-map sample")
    return valid, metadata


def exp_se3(delta: np.ndarray) -> np.ndarray:
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = Rotation.from_rotvec(delta[3:]).as_matrix()
    result[:3, 3] = delta[:3]
    return result


def log_se3(transform: np.ndarray) -> np.ndarray:
    return np.concatenate((transform[:3, 3], Rotation.from_matrix(transform[:3, :3]).as_rotvec()))


def rotation_angle(transform: np.ndarray) -> float:
    return float(np.linalg.norm(Rotation.from_matrix(transform[:3, :3]).as_rotvec()))


def average_transforms(transforms: Iterable[np.ndarray]) -> np.ndarray:
    values = list(transforms)
    if not values:
        return np.eye(4, dtype=np.float64)
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = Rotation.from_matrix(np.stack([value[:3, :3] for value in values])).mean().as_matrix()
    result[:3, 3] = np.mean(np.stack([value[:3, 3] for value in values]), axis=0)
    return result


def opencv_initial(base_eef: list[np.ndarray], camera_target: list[np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    """Get a physically correct eye-to-hand initial guess from OpenCV.

    OpenCV expects base->eef (the inverse of the ROS parent<-child matrix
    convention used by the plugin) for the eye-to-hand case.  The returned
    camera pose is T_base_camera.
    """
    rotations = []
    translations = []
    target_rotations = []
    target_translations = []
    for base, target in zip(base_eef, camera_target):
        base_to_eef = np.linalg.inv(base)
        rotations.append(base_to_eef[:3, :3])
        translations.append(base_to_eef[:3, 3].reshape(3, 1))
        target_rotations.append(target[:3, :3])
        target_translations.append(target[:3, 3].reshape(3, 1))
    rotation, translation = cv2.calibrateHandEye(
        rotations,
        translations,
        target_rotations,
        target_translations,
        method=cv2.CALIB_HAND_EYE_PARK,
    )
    camera = np.eye(4, dtype=np.float64)
    camera[:3, :3] = np.asarray(rotation, dtype=np.float64)
    camera[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    if not np.all(np.isfinite(camera)):
        raise ValueError("OpenCV produced a non-finite initial camera pose")
    mounts = [np.linalg.inv(base) @ camera @ target for base, target in zip(base_eef, camera_target)]
    return camera, average_transforms(mounts)


def deduplicate(
    base_eef: list[np.ndarray],
    camera_target: list[np.ndarray],
    threshold_m: float,
    threshold_deg: float,
) -> tuple[list[int], list[int]]:
    kept: list[int] = []
    duplicates: list[int] = []
    for index, current in enumerate(base_eef):
        duplicate = False
        for previous in kept:
            delta = np.linalg.inv(base_eef[previous]) @ current
            if np.linalg.norm(delta[:3, 3]) < threshold_m and rotation_angle(delta) < math.radians(threshold_deg):
                duplicate = True
                break
        if duplicate:
            duplicates.append(index)
        else:
            kept.append(index)
    return kept, duplicates


def residuals(
    parameters: np.ndarray,
    base_eef: list[np.ndarray],
    camera_target: list[np.ndarray],
    camera0: np.ndarray,
    mount0: np.ndarray,
    translation_scale: float,
    rotation_scale: float,
) -> np.ndarray:
    camera = exp_se3(parameters[:6]) @ camera0
    mount = exp_se3(parameters[6:]) @ mount0
    values = []
    for base, target in zip(base_eef, camera_target):
        # Both products map target coordinates into the robot base frame.
        error = np.linalg.inv(base @ mount) @ (camera @ target)
        values.extend((error[:3, 3] / translation_scale).tolist())
        values.extend((Rotation.from_matrix(error[:3, :3]).as_rotvec() / rotation_scale).tolist())
    return np.asarray(values, dtype=np.float64)


def solve_subset(
    base_eef: list[np.ndarray],
    camera_target: list[np.ndarray],
    args: argparse.Namespace,
    robust: bool,
) -> tuple[np.ndarray, np.ndarray, Any]:
    camera0, mount0 = opencv_initial(base_eef, camera_target)
    initial = np.zeros(12, dtype=np.float64)
    result = least_squares(
        residuals,
        initial,
        args=(base_eef, camera_target, camera0, mount0, args.translation_scale, args.rotation_scale),
        loss="soft_l1" if robust else "linear",
        f_scale=1.0,
        x_scale="jac",
        max_nfev=args.max_iterations,
    )
    camera = exp_se3(result.x[:6]) @ camera0
    mount = exp_se3(result.x[6:]) @ mount0
    return camera, mount, result


def physical_errors(
    camera: np.ndarray,
    mount: np.ndarray,
    base_eef: list[np.ndarray],
    camera_target: list[np.ndarray],
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    translations = []
    rotations = []
    scores = []
    for base, target in zip(base_eef, camera_target):
        error = np.linalg.inv(base @ mount) @ (camera @ target)
        translation = float(np.linalg.norm(error[:3, 3]))
        rotation = rotation_angle(error)
        translations.append(translation)
        rotations.append(rotation)
        scores.append((translation, rotation))
    return np.asarray(translations), np.asarray(rotations), np.asarray(scores)


def robust_inlier_mask(translations: np.ndarray, rotations: np.ndarray, args: argparse.Namespace) -> np.ndarray:
    normalized = np.sqrt(
        (translations / args.translation_scale) ** 2 + (rotations / args.rotation_scale) ** 2
    )
    median = float(np.median(normalized))
    mad = float(np.median(np.abs(normalized - median)))
    cutoff = max(args.outlier_min_score, median + args.outlier_sigma * 1.4826 * max(mad, 1e-9))
    return normalized <= cutoff


def quaternion_xyzw(matrix: np.ndarray) -> list[float]:
    return [float(value) for value in Rotation.from_matrix(matrix[:3, :3]).as_quat()]


def flatten(matrix: np.ndarray) -> list[float]:
    return [float(value) for value in matrix.reshape(4, 4).ravel()]


def atomic_dump(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=path.name + ".", dir=str(path.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            yaml.safe_dump(data, stream, sort_keys=False)
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def launch_text(camera: np.ndarray, parent: str, child: str) -> str:
    q = quaternion_xyzw(camera)
    t = camera[:3, 3]
    return f'''"""Robust Thor7 eye-to-hand calibration result.
Generated by solve_handeye_robust.py.  Start only after stopping every other
static transform publisher that uses the same parent/child frames.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            arguments=[
                "--frame-id", {parent!r},
                "--child-frame-id", {child!r},
                "--x", "{t[0]:.12f}",
                "--y", "{t[1]:.12f}",
                "--z", "{t[2]:.12f}",
                "--qx", "{q[0]:.12f}",
                "--qy", "{q[1]:.12f}",
                "--qz", "{q[2]:.12f}",
                "--qw", "{q[3]:.12f}",
            ],
        ),
    ])
'''


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output", type=Path, default=Path("handeye_result.yaml"))
    parser.add_argument("--emit-launch", type=Path, default=None)
    parser.add_argument("--base-frame", default=None)
    parser.add_argument("--sensor-frame", default=None)
    parser.add_argument("--eef-frame", default=None)
    parser.add_argument("--deduplicate", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--min-motion-m", type=float, default=0.005)
    parser.add_argument("--min-motion-deg", type=float, default=2.0)
    parser.add_argument("--translation-scale", type=float, default=0.005)
    parser.add_argument("--rotation-scale-deg", type=float, default=0.5)
    parser.add_argument("--outlier-sigma", type=float, default=3.5)
    parser.add_argument("--outlier-min-score", type=float, default=3.0)
    parser.add_argument("--max-iterations", type=int, default=3000)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.rotation_scale = math.radians(args.rotation_scale_deg)
    try:
        rows, metadata = load_rows(args.input)
        base_eef = [matrix_from_value(row["effector_wrt_world"], f"sample {i} effector_wrt_world") for i, row in enumerate(rows)]
        camera_target = [matrix_from_value(row["object_wrt_sensor"], f"sample {i} object_wrt_sensor") for i, row in enumerate(rows)]
    except (KeyError, TypeError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    if len(base_eef) < 6:
        print(f"error: {len(base_eef)} samples found; at least 6 are required", file=sys.stderr)
        return 2

    original_indices = list(range(len(base_eef)))
    duplicate_indices: list[int] = []
    if args.deduplicate:
        keep, duplicate_indices = deduplicate(base_eef, camera_target, args.min_motion_m, args.min_motion_deg)
        base_eef = [base_eef[i] for i in keep]
        camera_target = [camera_target[i] for i in keep]
        original_indices = [original_indices[i] for i in keep]
        if len(base_eef) < 6:
            print(
                f"error: deduplication left {len(base_eef)} distinct poses; collect at least 6 (prefer 15-25)",
                file=sys.stderr,
            )
            return 2

    try:
        camera, mount, first_result = solve_subset(base_eef, camera_target, args, robust=True)
    except (ValueError, cv2.error, np.linalg.LinAlgError) as exc:
        print(f"error: initial hand-eye solve failed: {exc}", file=sys.stderr)
        return 2

    translations, rotations, _ = physical_errors(camera, mount, base_eef, camera_target)
    inliers = robust_inlier_mask(translations, rotations, args)
    if int(np.count_nonzero(inliers)) >= 6 and int(np.count_nonzero(inliers)) < len(base_eef):
        try:
            camera, mount, _ = solve_subset(
                [base_eef[i] for i in np.flatnonzero(inliers)],
                [camera_target[i] for i in np.flatnonzero(inliers)],
                args,
                robust=False,
            )
            translations, rotations, _ = physical_errors(camera, mount, base_eef, camera_target)
        except (ValueError, cv2.error, np.linalg.LinAlgError) as exc:
            print(f"warning: inlier refit failed, retaining robust fit: {exc}", file=sys.stderr)
            inliers = np.ones(len(base_eef), dtype=bool)

    used_original = [original_indices[i] for i in np.flatnonzero(inliers)]
    outlier_original = [original_indices[i] for i in np.flatnonzero(~inliers)] + duplicate_indices
    rms_translation = float(np.sqrt(np.mean(translations[inliers] ** 2)))
    rms_rotation = float(np.degrees(np.sqrt(np.mean(rotations[inliers] ** 2))))
    max_translation = float(np.max(translations[inliers]))
    max_rotation = float(np.degrees(np.max(rotations[inliers])))
    base_frame = args.base_frame or metadata.get("base_frame", "base_link")
    sensor_frame = args.sensor_frame or metadata.get("sensor_frame", "camera_color_optical_frame")
    eef_frame = args.eef_frame or metadata.get("eef_frame", "link6")

    result_data = {
        "format": "thor7_handeye_robust_se3_v1",
        "model": "T_base_camera * T_camera_target = T_base_eef * T_eef_target",
        "mount": "eye_to_hand",
        "base_frame": base_frame,
        "sensor_frame": sensor_frame,
        "eef_frame": eef_frame,
        "target_frame": metadata.get("target_frame", "handeye_target"),
        "input": str(args.input),
        "sample_count": len(rows),
        "distinct_pose_count": len(base_eef),
        "inlier_count": int(np.count_nonzero(inliers)),
        "inlier_indices_zero_based": used_original,
        "rejected_indices_zero_based": outlier_original,
        "quality": {
            "rms_translation_m": rms_translation,
            "rms_rotation_deg": rms_rotation,
            "max_translation_m": max_translation,
            "max_rotation_deg": max_rotation,
            "translation_scale_m": args.translation_scale,
            "rotation_scale_deg": args.rotation_scale_deg,
        },
        "camera_pose": {
            "parent": base_frame,
            "child": sensor_frame,
            "matrix": flatten(camera),
            "translation_m": [float(value) for value in camera[:3, 3]],
            "quaternion_xyzw": quaternion_xyzw(camera),
        },
        "board_mount_pose": {
            "parent": eef_frame,
            "child": metadata.get("target_frame", "handeye_target"),
            "matrix": flatten(mount),
            "translation_m": [float(value) for value in mount[:3, 3]],
            "quaternion_xyzw": quaternion_xyzw(mount),
        },
        "per_sample": [
            {
                "index_zero_based": int(original_indices[i]),
                "used": bool(inliers[i]),
                "translation_error_m": float(translations[i]),
                "rotation_error_deg": float(np.degrees(rotations[i])),
            }
            for i in range(len(base_eef))
        ],
    }
    atomic_dump(args.output, result_data)
    if args.emit_launch is not None:
        args.emit_launch.parent.mkdir(parents=True, exist_ok=True)
        args.emit_launch.write_text(launch_text(camera, base_frame, sensor_frame), encoding="utf-8")

    print(f"samples={len(rows)}, distinct={len(base_eef)}, inliers={int(np.count_nonzero(inliers))}")
    print(f"RMS translation: {rms_translation * 1000.0:.2f} mm")
    print(f"RMS rotation:    {rms_rotation:.3f} deg")
    print(f"max translation: {max_translation * 1000.0:.2f} mm")
    print(f"max rotation:    {max_rotation:.3f} deg")
    print(f"T_{base_frame}_{sensor_frame} translation: {camera[:3, 3].tolist()}")
    print(f"quaternion xyzw: {quaternion_xyzw(camera)}")
    if outlier_original:
        print(f"rejected/duplicate sample indices: {outlier_original}")
    if rms_translation > 0.005 or rms_rotation > 0.5:
        print(
            "warning: residual is above the usual first-pass target (5 mm / 0.5 deg); "
            "check robot model, TCP, timestamps and board dimensions before publishing",
            file=sys.stderr,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
