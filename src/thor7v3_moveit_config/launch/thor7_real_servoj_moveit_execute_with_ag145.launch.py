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

    # Automatically load + activate the arm JTC.
    #
    # spawner waits for /controller_manager, then configures and activates
    # thor7_arm_controller. Hardware activation itself remains responsible
    # for initializing commands from the current measured robot state.
    thor7_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "thor7_arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
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
            thor7_arm_controller_spawner,
        ]
    )
