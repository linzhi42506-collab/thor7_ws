import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import (
    get_package_share_directory,
)


def generate_launch_description():

    operator_confirmation = LaunchConfiguration(
        "operator_confirmation"
    )

    ag145_force_percent = LaunchConfiguration(
        "ag145_force_percent"
    )

    moveit_share = get_package_share_directory(
        "thor7v3_moveit_config"
    )

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                moveit_share,
                "launch",
                "thor7_real_servoj_moveit_execute.launch.py",
            )
        ),
        launch_arguments={
            "operator_confirmation":
                operator_confirmation,
            "start_arm_controller": "true",
        }.items(),
    )

    ag145 = Node(
        package="thor7_hardware",
        executable="ag145_gripper_action_server",
        name="ag145_gripper_action_server",
        output="screen",
        parameters=[
            {
                "serial_port": "/dev/ttyUSB0",
                "baud_rate": 115200,
                "slave_id": 1,
                "timeout_ms": 200,
                "retries": 3,
                "publish_rate": 10.0,

                # Same state stream consumed by
                # robot_state_publisher / MoveIt.
                "joint_state_topic": "/joint_states",

                # Keep initial grasp commissioning force low.
                "force_percent":
                    ag145_force_percent,

                "position_tolerance_counts": 15,
                "motion_timeout_sec": 10.0,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "operator_confirmation",
                default_value="",
            ),

            DeclareLaunchArgument(
                "ag145_force_percent",
                default_value="20",
            ),

            base_launch,
            ag145,
        ]
    )
