""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """
""" EYE-TO-HAND: base_link -> camera_color_optical_frame """
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="log",
            arguments=[
                "--frame-id",
                "base_link",
                "--child-frame-id",
                "camera_color_optical_frame",
                "--x",
                "-0.426692",
                "--y",
                "0.961884",
                "--z",
                "0.71132",
                "--qx",
                "-0.304199",
                "--qy",
                "0.888102",
                "--qz",
                "-0.326487",
                "--qw",
                "0.110199",
                # "--roll",
                # "2.5495",
                # "--pitch",
                # "0.405382",
                # "--yaw",
                # "2.60679",
            ],
        ),
    ]
    return LaunchDescription(nodes)
