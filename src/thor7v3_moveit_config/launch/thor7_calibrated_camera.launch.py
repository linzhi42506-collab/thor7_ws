#!/usr/bin/env python3
"""Publish the latest Thor7 eye-to-hand calibration.

This is the calibrated transform from 234.launch.py.  Start exactly one
publisher for this parent/child pair; do not run the older pose1/123 launch
files at the same time.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="thor7_calibrated_camera_tf",
            output="screen",
            arguments=[
                "--frame-id", "base_link",
                "--child-frame-id", "camera_color_optical_frame",
                "--x", "-0.426692",
                "--y", "0.961884",
                "--z", "0.711320",
                "--qx", "-0.304199",
                "--qy", "0.888102",
                "--qz", "-0.326487",
                "--qw", "0.110199",
            ],
        ),
    ])
