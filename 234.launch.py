"""Compatibility entry point using the current workspace calibration."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from pathlib import Path


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory("thor7v3_moveit_config"))
            / "launch" / "thor7_calibrated_camera.launch.py"
        ))),
    ])
