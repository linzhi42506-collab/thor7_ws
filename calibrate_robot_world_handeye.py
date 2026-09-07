#!/usr/bin/env python3
"""Offline robot-world/hand-eye calibration for MoveIt Calibration samples.

The MoveIt RViz plugin stores row-major matrices with the following ROS TF
semantics (the first frame is the TF parent):

    effector_wrt_world: T_base_eef       (_base T_eef)
    object_wrt_sensor:  T_camera_board   (_camera T_board)

For an eye-to-hand camera these samples satisfy::

    T_camera_board = inv(T_base_camera) * T_base_eef * T_eef_board

OpenCV's ``calibrateRobotWorldHandEye`` uses the opposite virtual frame
assignment for this setup.  Therefore this tool passes
``inv(T_camera_board)`` and ``inv(T_base_eef)`` to OpenCV, then inverts its
``T_camera_base`` result to obtain the ROS TF transform ``T_base_camera``.

The OpenCV estimate is refined jointly with the constant end-effector-to-board
transform on SE(3).  A robust loss and an optional MAD-based rejection pass
make bad image detections visible without silently hiding a globally wrong
frame or board-size configuration.

Typical use::

    python3 calibrate_robot_world_handeye.py \
        --samples "movoelt vi.yaml" \
        --output handeye_calibration_result.yaml \
        --launch-output static_camera_tf.launch.py \
        --compare-methods

The generated launch file publishes ``base_link -> camera_color_optical_frame``
by default.  Check the parent/child names against the Context tab before use.
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys
from dataclasses import dataclass
from typing import Any, Iterable, Sequence

try:
    import cv2  # type: ignore
except ImportError as exc:  # pragma: no cover - exercised only on bad installs
    cv2 = None  # type: ignore
    _CV2_IMPORT_ERROR = exc

try:
    import numpy as np
except ImportError as exc:  # pragma: no cover - exercised only on bad installs
    np = None  # type: ignore
    _NUMPY_IMPORT_ERROR = exc

try:
    import yaml  # type: ignore
except ImportError as exc:  # pragma: no cover - exercised only on bad installs
    yaml = None  # type: ignore
    _YAML_IMPORT_ERROR = exc

try:
    from scipy.optimize import least_squares  # type: ignore
except ImportError:  # pragma: no cover - SciPy is optional at runtime
    least_squares = None


EPS = 1.0e-12


def _require_dependencies() -> None:
    missing = []
    if np is None:
        missing.append(f"numpy ({_NUMPY_IMPORT_ERROR})")
    if cv2 is None:
        missing.append(f"opencv-python ({_CV2_IMPORT_ERROR})")
    if yaml is None:
        missing.append(f"PyYAML ({_YAML_IMPORT_ERROR})")
    if missing:
        raise RuntimeError("Missing Python dependencies: " + ", ".join(missing))


def _as_matrix(value: Any, label: str) -> np.ndarray:
    """Parse and validate a 4x4 row-major homogeneous transform."""

    _require_dependencies()
    try:
        arr = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} is not numeric: {exc}") from exc

    if arr.shape == (16,):
        arr = arr.reshape((4, 4))
    elif arr.shape != (4, 4):
        raise ValueError(f"{label} must contain 16 values or a 4x4 matrix, got {arr.shape}")
    if not np.all(np.isfinite(arr)):
        raise ValueError(f"{label} contains NaN or infinity")
    if not np.allclose(arr[3, :], [0.0, 0.0, 0.0, 1.0], atol=2.0e-4):
        raise ValueError(f"{label} has invalid homogeneous last row {arr[3, :].tolist()}")

    # TF quaternions should already make a proper rotation.  Project tiny
    # floating-point drift onto SO(3), but reject a genuinely malformed input.
    rotation = arr[:3, :3]
    u, _, vh = np.linalg.svd(rotation)
    projected = u @ vh
    if np.linalg.det(projected) < 0.0:
        u[:, -1] *= -1.0
        projected = u @ vh
    if np.linalg.norm(rotation - projected, ord="fro") > 5.0e-2:
        raise ValueError(f"{label} rotation is not orthonormal")
    arr = arr.copy()
    arr[:3, :3] = projected
    arr[3, :] = [0.0, 0.0, 0.0, 1.0]
    return arr


def _load_samples(path: pathlib.Path) -> tuple[list[np.ndarray], list[np.ndarray]]:
    """Load the exact YAML format written by MoveIt Calibration.

    A mapping containing ``samples`` and a mapping containing two parallel
    arrays are accepted as conveniences; the plugin's native format is a
    sequence of mappings.
    """

    _require_dependencies()
    try:
        content = yaml.safe_load(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise ValueError(f"Unable to read samples file {path}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise ValueError(f"Invalid YAML in {path}: {exc}") from exc

    if isinstance(content, dict) and "samples" in content:
        content = content["samples"]
    elif isinstance(content, dict) and {
        "effector_wrt_world",
        "object_wrt_sensor",
    }.issubset(content):
        effector_values = content["effector_wrt_world"]
        object_values = content["object_wrt_sensor"]
        if not isinstance(effector_values, Sequence) or not isinstance(object_values, Sequence):
            raise ValueError("Parallel sample arrays must be sequences")
        content = [
            {"effector_wrt_world": a, "object_wrt_sensor": b}
            for a, b in zip(effector_values, object_values)
        ]

    if not isinstance(content, Sequence) or isinstance(content, (str, bytes)):
        raise ValueError("Samples YAML must be a sequence of sample mappings")

    base_eef: list[np.ndarray] = []
    camera_board: list[np.ndarray] = []
    for index, sample in enumerate(content, start=1):
        if not isinstance(sample, dict):
            raise ValueError(f"Sample {index} is not a mapping")
        if "effector_wrt_world" not in sample or "object_wrt_sensor" not in sample:
            raise ValueError(
                f"Sample {index} must contain effector_wrt_world and object_wrt_sensor"
            )
        base_eef.append(_as_matrix(sample["effector_wrt_world"], f"sample {index} effector_wrt_world"))
        camera_board.append(_as_matrix(sample["object_wrt_sensor"], f"sample {index} object_wrt_sensor"))

    if len(base_eef) < 3:
        raise ValueError(f"At least 3 pose samples are required, got {len(base_eef)}")
    if len(base_eef) != len(camera_board):  # defensive; construction keeps them parallel
        raise ValueError("The two pose sample arrays have different lengths")
    return base_eef, camera_board


def _inverse(transform: np.ndarray) -> np.ndarray:
    result = np.eye(4, dtype=float)
    result[:3, :3] = transform[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ transform[:3, 3]
    return result


def _exp_se3(twist: np.ndarray) -> np.ndarray:
    """Exponential map for a twist [translation(m), rotation(rad)]."""

    _require_dependencies()
    twist = np.asarray(twist, dtype=float).reshape(6)
    result = np.eye(4, dtype=float)
    rotation, _ = cv2.Rodrigues(twist[3:].reshape(3, 1))
    result[:3, :3] = rotation
    result[:3, 3] = twist[:3]
    return result


def _log_so3(rotation: np.ndarray) -> np.ndarray:
    vector, _ = cv2.Rodrigues(np.asarray(rotation, dtype=float))
    return vector.reshape(3)


def _log_se3(transform: np.ndarray) -> np.ndarray:
    return np.concatenate((np.asarray(transform[:3, 3], dtype=float), _log_so3(transform[:3, :3])))


def _mean_transform(transforms: Iterable[np.ndarray], iterations: int = 50) -> np.ndarray:
    values = [np.asarray(value, dtype=float) for value in transforms]
    if not values:
        raise ValueError("Cannot average an empty transform set")
    estimate = values[0].copy()
    for _ in range(iterations):
        deltas = np.asarray([_log_se3(_inverse(estimate) @ value) for value in values])
        delta = np.mean(deltas, axis=0)
        if np.linalg.norm(delta) < 1.0e-12:
            break
        estimate = estimate @ _exp_se3(delta)
    return estimate


def _opencv_method(name: str) -> int:
    _require_dependencies()
    normalized = name.lower()
    if normalized == "shah":
        return int(getattr(cv2, "CALIB_ROBOT_WORLD_HAND_EYE_SHAH", 0))
    if normalized == "li":
        if not hasattr(cv2, "CALIB_ROBOT_WORLD_HAND_EYE_LI"):
            raise RuntimeError("This OpenCV build does not provide the Li robot-world solver")
        return int(cv2.CALIB_ROBOT_WORLD_HAND_EYE_LI)
    raise ValueError(f"Unknown robot-world solver {name!r}; choose shah or li")


def _opencv_initial(
    base_eef: Sequence[np.ndarray], camera_board: Sequence[np.ndarray], method: str
) -> np.ndarray:
    """Return T_base_camera from the OpenCV robot-world/hand-eye solver."""

    _require_dependencies()
    # Virtual world = physical camera and virtual camera = calibration board.
    # This is why both plugin transforms are inverted before the API call.
    virtual_world_to_camera = [_inverse(value) for value in camera_board]
    base_to_gripper = [_inverse(value) for value in base_eef]
    rotations_a = [value[:3, :3].astype(np.float64) for value in virtual_world_to_camera]
    translations_a = [value[:3, 3].reshape(3, 1).astype(np.float64) for value in virtual_world_to_camera]
    rotations_b = [value[:3, :3].astype(np.float64) for value in base_to_gripper]
    translations_b = [value[:3, 3].reshape(3, 1).astype(np.float64) for value in base_to_gripper]

    try:
        result = cv2.calibrateRobotWorldHandEye(
            rotations_a,
            translations_a,
            rotations_b,
            translations_b,
            method=_opencv_method(method),
        )
    except cv2.error as exc:
        raise RuntimeError(f"OpenCV robot-world/hand-eye solver failed: {exc}") from exc
    if len(result) != 4:
        raise RuntimeError(f"Unexpected OpenCV solver result with {len(result)} values")

    # API output is _camera T_base for our virtual camera/world assignment;
    # static TF needs _base T_camera, hence this final inverse.
    rotation_camera_base, translation_camera_base = result[0], result[1]
    estimate = np.eye(4, dtype=float)
    estimate[:3, :3] = np.asarray(rotation_camera_base, dtype=float).reshape(3, 3)
    estimate[:3, 3] = np.asarray(translation_camera_base, dtype=float).reshape(3)
    return _inverse(estimate)


def _residuals(
    base_camera: np.ndarray,
    eef_board: np.ndarray,
    base_eef: Sequence[np.ndarray],
    camera_board: Sequence[np.ndarray],
) -> tuple[np.ndarray, np.ndarray]:
    translation = []
    rotation = []
    camera_base = _inverse(base_camera)
    for robot_pose, observed in zip(base_eef, camera_board):
        predicted = camera_base @ robot_pose @ eef_board
        error = _inverse(observed) @ predicted
        translation.append(float(np.linalg.norm(error[:3, 3])))
        rotation.append(float(np.linalg.norm(_log_so3(error[:3, :3]))))
    return np.asarray(translation), np.asarray(rotation)


def _refine(
    base_camera: np.ndarray,
    eef_board: np.ndarray,
    base_eef: Sequence[np.ndarray],
    camera_board: Sequence[np.ndarray],
    indices: Sequence[int],
    rotation_weight: float,
    f_scale: float,
) -> tuple[np.ndarray, np.ndarray, str]:
    """Robustly refine T_base_camera and T_eef_board on the selected samples."""

    if least_squares is None:
        return base_camera, eef_board, "scipy unavailable; OpenCV estimate used"

    initial_base_camera = np.asarray(base_camera, dtype=float)
    initial_eef_board = np.asarray(eef_board, dtype=float)
    selected = list(indices)

    def unpack(parameters: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        return (
            initial_base_camera @ _exp_se3(parameters[:6]),
            initial_eef_board @ _exp_se3(parameters[6:]),
        )

    def residual(parameters: np.ndarray) -> np.ndarray:
        current_base_camera, current_eef_board = unpack(parameters)
        camera_base = _inverse(current_base_camera)
        values: list[float] = []
        for index in selected:
            error = _inverse(camera_board[index]) @ camera_base @ base_eef[index] @ current_eef_board
            twist = _log_se3(error)
            values.extend(twist[:3].tolist())
            values.extend((rotation_weight * twist[3:]).tolist())
        return np.asarray(values, dtype=float)

    result = least_squares(
        residual,
        np.zeros(12, dtype=float),
        loss="soft_l1",
        f_scale=max(float(f_scale), 1.0e-8),
        max_nfev=2500,
        xtol=1.0e-12,
        ftol=1.0e-12,
        gtol=1.0e-12,
    )
    refined_base_camera, refined_eef_board = unpack(result.x)
    status = "scipy least_squares: " + ("converged" if result.success else result.message)
    return refined_base_camera, refined_eef_board, status


@dataclass
class CalibrationResult:
    method: str
    base_camera: np.ndarray
    eef_board: np.ndarray
    inliers: list[int]
    outliers: list[int]
    translation_residuals: np.ndarray
    rotation_residuals: np.ndarray
    refinement_status: str
    initial_base_camera: np.ndarray


def _robust_fit(
    base_eef: Sequence[np.ndarray],
    camera_board: Sequence[np.ndarray],
    method: str,
    refine: bool = True,
    reject_outliers: bool = True,
    max_rounds: int = 3,
    rotation_weight: float = 0.1,
    f_scale: float = 0.01,
    translation_threshold: float | None = None,
    rotation_threshold: float | None = None,
    min_samples: int = 6,
) -> CalibrationResult:
    if len(base_eef) != len(camera_board):
        raise ValueError("Pose arrays must have equal length")
    if len(base_eef) < 3:
        raise ValueError("At least 3 samples are required")

    initial = _opencv_initial(base_eef, camera_board, method)
    board_candidates = [
        _inverse(robot_pose) @ initial @ observed
        for robot_pose, observed in zip(base_eef, camera_board)
    ]
    board = _mean_transform(board_candidates)
    inliers = list(range(len(base_eef)))
    refinement_status = "disabled"

    rounds = max(1, max_rounds if reject_outliers else 1)
    for round_index in range(rounds):
        if refine:
            initial, board, refinement_status = _refine(
                initial,
                board,
                base_eef,
                camera_board,
                inliers,
                rotation_weight,
                f_scale,
            )
        translation, rotation = _residuals(initial, board, base_eef, camera_board)
        if not reject_outliers or round_index + 1 >= rounds:
            break

        current_t = translation[inliers]
        current_r = rotation[inliers]
        median_t = float(np.median(current_t))
        median_r = float(np.median(current_r))
        mad_t = float(np.median(np.abs(current_t - median_t)))
        mad_r = float(np.median(np.abs(current_r - median_r)))
        # MAD thresholds are deliberately conservative.  The floors stop a
        # nearly perfect data set from rejecting harmless sub-millimetre noise.
        threshold_t = (
            float(translation_threshold)
            if translation_threshold is not None
            else max(0.003, median_t + 4.0 * max(1.4826 * mad_t, 1.0e-5))
        )
        threshold_r = (
            float(rotation_threshold)
            if rotation_threshold is not None
            else max(math.radians(0.5), median_r + 4.0 * max(1.4826 * mad_r, 1.0e-5))
        )
        bad = [
            index
            for index in inliers
            if translation[index] > threshold_t or rotation[index] > threshold_r
        ]
        if not bad:
            break
        allowed = max(0, len(inliers) - max(3, min_samples))
        if len(bad) > allowed:
            # Keep at least the requested number of poses and remove the
            # highest normalized residuals first.
            score = lambda index: max(
                translation[index] / max(threshold_t, EPS),
                rotation[index] / max(threshold_r, EPS),
            )
            bad = sorted(bad, key=score, reverse=True)[:allowed]
        if not bad:
            break
        inliers = [index for index in inliers if index not in set(bad)]
        print(
            f"outlier pass {round_index + 1}: rejecting "
            + ", ".join(str(index + 1) for index in bad)
            + f" (thresholds {threshold_t * 1000.0:.2f} mm, "
            + f"{math.degrees(threshold_r):.3f} deg)",
            file=sys.stderr,
        )

    # One final fit after the last rejection pass.
    if refine:
        initial, board, refinement_status = _refine(
            initial,
            board,
            base_eef,
            camera_board,
            inliers,
            rotation_weight,
            f_scale,
        )
    translation, rotation = _residuals(initial, board, base_eef, camera_board)
    outliers = [index for index in range(len(base_eef)) if index not in set(inliers)]
    return CalibrationResult(
        method=method.lower(),
        base_camera=initial,
        eef_board=board,
        inliers=inliers,
        outliers=outliers,
        translation_residuals=translation,
        rotation_residuals=rotation,
        refinement_status=refinement_status,
        initial_base_camera=_opencv_initial(base_eef, camera_board, method),
    )


def _quaternion_xyzw(rotation: np.ndarray) -> list[float]:
    matrix = np.asarray(rotation, dtype=float)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (matrix[2, 1] - matrix[1, 2]) / s
        y = (matrix[0, 2] - matrix[2, 0]) / s
        z = (matrix[1, 0] - matrix[0, 1]) / s
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        s = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        w = (matrix[2, 1] - matrix[1, 2]) / s
        x = 0.25 * s
        y = (matrix[0, 1] + matrix[1, 0]) / s
        z = (matrix[0, 2] + matrix[2, 0]) / s
    elif matrix[1, 1] > matrix[2, 2]:
        s = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        w = (matrix[0, 2] - matrix[2, 0]) / s
        x = (matrix[0, 1] + matrix[1, 0]) / s
        y = 0.25 * s
        z = (matrix[1, 2] + matrix[2, 1]) / s
    else:
        s = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        w = (matrix[1, 0] - matrix[0, 1]) / s
        x = (matrix[0, 2] + matrix[2, 0]) / s
        y = (matrix[1, 2] + matrix[2, 1]) / s
        z = 0.25 * s
    quaternion = np.asarray([x, y, z, w], dtype=float)
    quaternion /= max(float(np.linalg.norm(quaternion)), EPS)
    return [float(value) for value in quaternion]


def _rpy(rotation: np.ndarray) -> list[float]:
    matrix = np.asarray(rotation, dtype=float)
    pitch = math.asin(float(np.clip(-matrix[2, 0], -1.0, 1.0)))
    if abs(math.cos(pitch)) > 1.0e-8:
        roll = math.atan2(float(matrix[2, 1]), float(matrix[2, 2]))
        yaw = math.atan2(float(matrix[1, 0]), float(matrix[0, 0]))
    else:
        roll = math.atan2(float(-matrix[1, 2]), float(matrix[1, 1]))
        yaw = 0.0
    return [roll, pitch, yaw]


def _float_text(value: float) -> str:
    return f"{float(value):.12g}"


def _result_mapping(
    result: CalibrationResult,
    parent_frame: str,
    child_frame: str,
    base_eef: Sequence[np.ndarray],
    camera_board: Sequence[np.ndarray],
) -> dict[str, Any]:
    transform = result.base_camera
    quaternion = _quaternion_xyzw(transform[:3, :3])
    rpy = _rpy(transform[:3, :3])
    per_sample = []
    for index, (translation, rotation) in enumerate(
        zip(result.translation_residuals, result.rotation_residuals), start=1
    ):
        per_sample.append(
            {
                "index": index,
                "inlier": (index - 1) in result.inliers,
                "translation_m": float(translation),
                "rotation_rad": float(rotation),
                "rotation_deg": math.degrees(float(rotation)),
            }
        )
    inlier_mask = np.asarray([index in result.inliers for index in range(len(base_eef))])
    inlier_t = result.translation_residuals[inlier_mask]
    inlier_r = result.rotation_residuals[inlier_mask]
    return {
        "format": "thor7_robot_world_handeye_v1",
        "solver": result.method,
        "setup": "eye_to_hand",
        "input_convention": {
            "effector_wrt_world": "T_base_eef",
            "object_wrt_sensor": "T_camera_board",
            "opencv_virtual_inputs": "A=inv(T_camera_board), B=inv(T_base_eef)",
            "output": "T_base_camera (invert OpenCV R_base2world/T_camera_base)",
        },
        "parent_frame": parent_frame,
        "child_frame": child_frame,
        "transform": {
            "translation": {
                "x": float(transform[0, 3]),
                "y": float(transform[1, 3]),
                "z": float(transform[2, 3]),
            },
            "rotation_quaternion_xyzw": quaternion,
            "rotation_rpy_rad": rpy,
            "matrix": transform.tolist(),
        },
        "diagnostics": {
            "sample_count": len(base_eef),
            "inlier_count": len(result.inliers),
            "outlier_samples_1_based": [index + 1 for index in result.outliers],
            "translation_rms_m": float(np.sqrt(np.mean(np.square(inlier_t)))) if len(inlier_t) else None,
            "translation_mean_m": float(np.mean(inlier_t)) if len(inlier_t) else None,
            "rotation_rms_rad": float(np.sqrt(np.mean(np.square(inlier_r)))) if len(inlier_r) else None,
            "rotation_mean_rad": float(np.mean(inlier_r)) if len(inlier_r) else None,
            "refinement": result.refinement_status,
            "per_sample": per_sample,
        },
        "static_transform_publisher": {
            "package": "tf2_ros",
            "executable": "static_transform_publisher",
            "arguments": [
                "--x",
                _float_text(transform[0, 3]),
                "--y",
                _float_text(transform[1, 3]),
                "--z",
                _float_text(transform[2, 3]),
                "--qx",
                _float_text(quaternion[0]),
                "--qy",
                _float_text(quaternion[1]),
                "--qz",
                _float_text(quaternion[2]),
                "--qw",
                _float_text(quaternion[3]),
                "--frame-id",
                parent_frame,
                "--child-frame-id",
                child_frame,
            ],
        },
    }


def _write_launch(path: pathlib.Path, transform: np.ndarray, parent_frame: str, child_frame: str) -> None:
    quaternion = _quaternion_xyzw(transform[:3, :3])
    values = [
        _float_text(transform[0, 3]),
        _float_text(transform[1, 3]),
        _float_text(transform[2, 3]),
        _float_text(quaternion[0]),
        _float_text(quaternion[1]),
        _float_text(quaternion[2]),
        _float_text(quaternion[3]),
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    content = f'''#!/usr/bin/env python3
"""Static TF generated by calibrate_robot_world_handeye.py."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="calibrated_camera_tf",
            arguments=[
                "--x", "{values[0]}",
                "--y", "{values[1]}",
                "--z", "{values[2]}",
                "--qx", "{values[3]}",
                "--qy", "{values[4]}",
                "--qz", "{values[5]}",
                "--qw", "{values[6]}",
                "--frame-id", {parent_frame!r},
                "--child-frame-id", {child_frame!r},
            ],
            output="screen",
        ),
    ])
'''
    path.write_text(content, encoding="utf-8")
    path.chmod(path.stat().st_mode | 0o111)


def _print_report(result: CalibrationResult) -> None:
    inlier_mask = np.asarray([index in result.inliers for index in range(len(result.translation_residuals))])
    inlier_t = result.translation_residuals[inlier_mask]
    inlier_r = result.rotation_residuals[inlier_mask]
    rms_t = math.sqrt(float(np.mean(np.square(inlier_t)))) if len(inlier_t) else float("nan")
    rms_r = math.sqrt(float(np.mean(np.square(inlier_r)))) if len(inlier_r) else float("nan")
    print(f"solver: {result.method}")
    print(f"samples: {len(result.translation_residuals)} (inliers {len(result.inliers)}, outliers {len(result.outliers)})")
    print(f"inlier RMS residual: {rms_t * 1000.0:.3f} mm, {math.degrees(rms_r):.4f} deg")
    print(f"refinement: {result.refinement_status}")
    print("per-sample residuals:")
    for index, (translation, rotation) in enumerate(
        zip(result.translation_residuals, result.rotation_residuals), start=1
    ):
        flag = "IN" if index - 1 in result.inliers else "OUT"
        print(f"  {index:3d} [{flag}] {translation * 1000.0:9.3f} mm  {math.degrees(rotation):8.4f} deg")
    if rms_t > 0.01 or rms_r > math.radians(1.0):
        print(
            "WARNING: residual is high. Verify camera/board frame direction, "
            "board metric size, TF timestamps, and pose excitation.",
            file=sys.stderr,
        )


def _random_transform(rng: np.random.Generator, translation_range: float = 0.5) -> np.ndarray:
    translation = rng.uniform(-translation_range, translation_range, 3)
    rotation = rng.uniform(-math.pi, math.pi, 3)
    return _exp_se3(np.concatenate((translation, rotation)))


def _self_test() -> bool:
    """Exercise virtual-frame inversions, refinement, and rejection."""

    _require_dependencies()
    rng = np.random.default_rng(20260904)
    true_base_camera = _exp_se3(np.array([0.42, -0.18, 0.63, 0.2, -0.3, 0.15]))
    true_eef_board = _exp_se3(np.array([0.03, 0.01, 0.16, -0.15, 0.2, 0.25]))
    base_eef: list[np.ndarray] = []
    camera_board: list[np.ndarray] = []
    for _ in range(20):
        robot = _random_transform(rng, translation_range=0.45)
        board = _inverse(true_base_camera) @ robot @ true_eef_board
        # Realistic TF/PnP noise: sub-mm and a few hundredths of a degree.
        robot_noisy = robot @ _exp_se3(
            np.concatenate((rng.normal(0.0, 0.00025, 3), rng.normal(0.0, 0.0005, 3)))
        )
        board_noisy = board @ _exp_se3(
            np.concatenate((rng.normal(0.0, 0.00045, 3), rng.normal(0.0, 0.0008, 3)))
        )
        base_eef.append(robot_noisy)
        camera_board.append(board_noisy)
    # One deliberate bad detection should be diagnosed and removed.
    camera_board[7] = camera_board[7] @ _exp_se3(np.array([0.07, -0.04, 0.03, 0.0, 0.0, 0.35]))

    result = _robust_fit(
        base_eef,
        camera_board,
        method="shah",
        refine=True,
        reject_outliers=True,
        max_rounds=4,
        min_samples=6,
    )
    translation_error = float(np.linalg.norm(result.base_camera[:3, 3] - true_base_camera[:3, 3]))
    rotation_error = float(np.linalg.norm(_log_so3(result.base_camera[:3, :3].T @ true_base_camera[:3, :3])))
    passed = translation_error < 0.005 and rotation_error < math.radians(0.5) and 7 in result.outliers
    print(
        "self-test: "
        + ("PASS" if passed else "FAIL")
        + f" (camera error {translation_error * 1000.0:.3f} mm, "
        + f"{math.degrees(rotation_error):.4f} deg; rejected {[index + 1 for index in result.outliers]})"
    )
    return passed


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--samples", type=pathlib.Path, help="MoveIt Calibration samples YAML")
    parser.add_argument("--output", type=pathlib.Path, default=pathlib.Path("handeye_calibration_result.yaml"), help="result YAML path")
    parser.add_argument("--launch-output", type=pathlib.Path, default=pathlib.Path("static_camera_tf.launch.py"), help="ROS 2 static TF launch path")
    parser.add_argument("--method", choices=("shah", "li"), default="shah", help="OpenCV robot-world solver")
    parser.add_argument("--compare-methods", action="store_true", help="also report the other OpenCV solver")
    parser.add_argument("--parent-frame", default="base_link", help="static TF parent frame")
    parser.add_argument("--child-frame", default="camera_color_optical_frame", help="static TF child frame")
    parser.add_argument("--no-refine", dest="refine", action="store_false", help="skip SciPy SE(3) refinement")
    parser.add_argument("--no-reject-outliers", dest="reject_outliers", action="store_false", help="keep all samples")
    parser.add_argument("--max-rejection-rounds", type=int, default=4, help="MAD outlier passes (default: 4)")
    parser.add_argument("--min-samples", type=int, default=6, help="minimum samples retained by rejection")
    parser.add_argument("--rotation-weight", type=float, default=0.1, help="metres per radian in refinement")
    parser.add_argument("--f-scale", type=float, default=0.01, help="soft-L1 residual scale")
    parser.add_argument("--translation-threshold-mm", type=float, help="explicit outlier translation threshold")
    parser.add_argument("--rotation-threshold-deg", type=float, help="explicit outlier rotation threshold")
    parser.add_argument("--self-test", action="store_true", help="run synthetic solver/rejection test")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    try:
        _require_dependencies()
        if args.self_test:
            return 0 if _self_test() else 1
        if args.samples is None:
            parser.error("--samples is required unless --self-test is used")
        if args.min_samples < 3:
            parser.error("--min-samples must be at least 3")
        base_eef, camera_board = _load_samples(args.samples)

        if args.compare_methods:
            for method in ("shah", "li"):
                try:
                    comparison = _robust_fit(
                        base_eef,
                        camera_board,
                        method,
                        refine=False,
                        reject_outliers=False,
                        min_samples=args.min_samples,
                    )
                    t = comparison.translation_residuals
                    r = comparison.rotation_residuals
                    print(
                        f"{method} OpenCV-only RMS: "
                        f"{math.sqrt(float(np.mean(np.square(t)))) * 1000.0:.3f} mm, "
                        f"{math.degrees(math.sqrt(float(np.mean(np.square(r))))):.4f} deg"
                    )
                except (RuntimeError, ValueError) as exc:
                    print(f"{method} comparison unavailable: {exc}", file=sys.stderr)

        result = _robust_fit(
            base_eef,
            camera_board,
            args.method,
            refine=args.refine,
            reject_outliers=args.reject_outliers,
            max_rounds=args.max_rejection_rounds,
            rotation_weight=args.rotation_weight,
            f_scale=args.f_scale,
            translation_threshold=(
                None if args.translation_threshold_mm is None else args.translation_threshold_mm / 1000.0
            ),
            rotation_threshold=(
                None if args.rotation_threshold_deg is None else math.radians(args.rotation_threshold_deg)
            ),
            min_samples=args.min_samples,
        )
        _print_report(result)
        mapping = _result_mapping(result, args.parent_frame, args.child_frame, base_eef, camera_board)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(yaml.safe_dump(mapping, sort_keys=False), encoding="utf-8")
        if args.launch_output is not None:
            _write_launch(args.launch_output, result.base_camera, args.parent_frame, args.child_frame)
        print(f"wrote result: {args.output}")
        if args.launch_output is not None:
            print(f"wrote static TF launch: {args.launch_output}")
        return 0
    except (RuntimeError, ValueError, OSError) as exc:
        print(f"calibration failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
