#!/usr/bin/env python3
"""Publish the current eye-to-hand result from the workspace calibration YAML."""

import math
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    path = Path(LaunchConfiguration("calibration_file").perform(context)).expanduser()
    data = yaml.safe_load(path.read_text())
    if data.get("parent_frame") != "base_link" or data.get("child_frame") != "camera_color_optical_frame":
        raise RuntimeError("Calibration must describe base_link -> camera_color_optical_frame")
    transform = data["transform"]
    xyz = [float(transform["translation"][key]) for key in ("x", "y", "z")]
    q = [float(value) for value in transform["rotation_quaternion_xyzw"]]
    if len(q) != 4 or not all(math.isfinite(value) for value in xyz + q):
        raise RuntimeError("Calibration contains invalid coordinates")
    norm = math.sqrt(sum(value * value for value in q))
    if abs(norm - 1.0) > 0.01:
        raise RuntimeError("Calibration quaternion is not normalized")
    args = []
    for name, value in zip(("x", "y", "z", "qx", "qy", "qz", "qw"),
                           xyz + [value / norm for value in q]):
        args.extend(["--" + name, str(value)])
    args += ["--frame-id", "base_link", "--child-frame-id", "camera_color_optical_frame"]
    return [
        LogInfo(msg=f"Camera calibration: {path}"),
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="thor7_calibrated_camera_tf", arguments=args, output="screen"),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "calibration_file",
            default_value="/home/landau/Documents/thor7_ws/handeye_calibration_result.yaml",
            description="Current T_base_camera calibration YAML",
        ),
        OpaqueFunction(function=setup),
    ])
