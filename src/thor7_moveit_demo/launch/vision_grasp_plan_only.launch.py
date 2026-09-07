from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pregrasp_clearance = LaunchConfiguration("pregrasp_clearance")
    near_clearance = LaunchConfiguration("near_clearance")
    grasp_descent = LaunchConfiguration("grasp_descent")
    plan_descent = LaunchConfiguration("plan_descent")
    auto_replan = LaunchConfiguration("auto_replan")
    ik_strict_joint_delta = LaunchConfiguration("ik_strict_joint_delta")
    ik_relaxed_joint_delta = LaunchConfiguration("ik_relaxed_joint_delta")
    cartesian_step = LaunchConfiguration("cartesian_step")
    max_cartesian_joint_step = LaunchConfiguration("max_cartesian_joint_step")
    eef_link = LaunchConfiguration("eef_link")
    moveit_config = (
        MoveItConfigsBuilder(
            "thor7v3",
            package_name="thor7v3_moveit_config",
        )
        .robot_description(
            file_path="config/thor7v3.real_readonly.urdf.xacro"
        )
        .trajectory_execution(
            file_path="config/moveit_controllers.plan_only.yaml",
            moveit_manage_controllers=False,
        )
        .to_moveit_configs()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "pregrasp_clearance", default_value="0.050",
            description="Height above the detected target for the pregrasp pose"),
        DeclareLaunchArgument(
            "near_clearance", default_value="0.005",
            description="Height above the detected target before the final descent"),
        DeclareLaunchArgument(
            "grasp_descent", default_value="0.013",
            description="Final descent below the detected target point"),
        DeclareLaunchArgument(
            "plan_descent", default_value="false",
            description="Include near/final descent; false stops at pregrasp"),
        DeclareLaunchArgument(
            "auto_replan", default_value="false",
            description="Automatically replan when the detected target moves"),
        DeclareLaunchArgument(
            "ik_strict_joint_delta", default_value="1.50",
            description="Preferred maximum IK change per joint in radians"),
        DeclareLaunchArgument(
            "ik_relaxed_joint_delta", default_value="2.50",
            description="Fallback maximum IK change per joint in radians"),
        DeclareLaunchArgument(
            "cartesian_step", default_value="0.002",
            description="Cartesian interpolation step in meters"),
        DeclareLaunchArgument(
            "max_cartesian_joint_step", default_value="0.20",
            description="Maximum allowed joint change between Cartesian samples"),
        DeclareLaunchArgument(
            "eef_link", default_value="tool0",
            description="MoveIt end-effector link"),
        Node(
            package="thor7_moveit_demo",
            executable="vision_grasp_plan_only",
            name="vision_grasp_plan_only",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
                {
                    "pregrasp_clearance": pregrasp_clearance,
                    "near_clearance": near_clearance,
                    "grasp_descent": grasp_descent,
                    "plan_descent": plan_descent,
                    "auto_replan": auto_replan,
                    "ik_strict_joint_delta": ik_strict_joint_delta,
                    "ik_relaxed_joint_delta": ik_relaxed_joint_delta,
                    "cartesian_step": cartesian_step,
                    "max_cartesian_joint_step": max_cartesian_joint_step,
                    "eef_link": eef_link,
                },
            ],
        ),
    ])
