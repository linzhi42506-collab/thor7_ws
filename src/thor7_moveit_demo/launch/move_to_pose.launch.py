from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "thor7v3",
        package_name="thor7v3_moveit_config",
    ).to_moveit_configs()

    move_to_pose_node = Node(
        package="thor7_moveit_demo",
        executable="move_to_pose",
        name="thor7_move_to_pose",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([move_to_pose_node])
