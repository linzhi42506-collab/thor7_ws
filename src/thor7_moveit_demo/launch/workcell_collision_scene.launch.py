from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("table_center_x", default_value="-0.473"),
        DeclareLaunchArgument("table_center_y", default_value="0.502"),
        DeclareLaunchArgument("table_top_z", default_value="-0.028433"),
        DeclareLaunchArgument("table_size_x", default_value="0.600"),
        DeclareLaunchArgument("table_size_y", default_value="0.600"),
        DeclareLaunchArgument("table_thickness", default_value="0.050"),
    ]

    table_node = Node(
        package="thor7_moveit_demo",
        executable="workcell_collision_scene",
        name="thor7_workcell_collision_scene",
        output="screen",
        parameters=[{
            "frame_id": "base_link",
            "table_center_x": LaunchConfiguration("table_center_x"),
            "table_center_y": LaunchConfiguration("table_center_y"),
            "table_top_z": LaunchConfiguration("table_top_z"),
            "table_size_x": LaunchConfiguration("table_size_x"),
            "table_size_y": LaunchConfiguration("table_size_y"),
            "table_thickness": LaunchConfiguration("table_thickness"),
        }],
    )

    return LaunchDescription(arguments + [table_node])
