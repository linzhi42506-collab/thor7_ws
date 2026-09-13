from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "thor7v3",
            package_name="thor7v3_moveit_config",
        )
        .robot_description(
            file_path="config/thor7v3.real_readonly.urdf.xacro"
        )
        .trajectory_execution(
            file_path="config/moveit_controllers.real_servoj.yaml",
            moveit_manage_controllers=False,
        )
        .to_moveit_configs()
    )

    arguments = [
        DeclareLaunchArgument("enable_execution", default_value="false"),
        DeclareLaunchArgument("operator_confirmation", default_value="DENIED"),
        DeclareLaunchArgument("grasp_yaw", default_value="0.0",
                              description="Opening axis yaw in base_link, radians; tool0 +Z points down"),
        DeclareLaunchArgument("use_vision_yaw", default_value="false",
                              description="Use stable /yolo_grasp_pose orientation instead of grasp_yaw"),
        DeclareLaunchArgument("target_yaw_tolerance", default_value="0.08726646260"),
        DeclareLaunchArgument("target_motion_confirmations", default_value="3"),
        DeclareLaunchArgument("orientation_tolerance", default_value="0.05235987756"),
        DeclareLaunchArgument("endpoint_position_tolerance", default_value="0.003"),
        DeclareLaunchArgument("pregrasp_clearance", default_value="0.070"),
        DeclareLaunchArgument("near_clearance", default_value="0.020"),
        DeclareLaunchArgument("pregrasp_candidate_planning_time", default_value="8.0"),
        DeclareLaunchArgument("pregrasp_total_planning_time", default_value="40.0"),
        DeclareLaunchArgument("plan_max_age_sec", default_value="60.0"),
        DeclareLaunchArgument("target_lock_sec", default_value="60.0"),
        DeclareLaunchArgument("velocity_scaling", default_value="0.15"),
        DeclareLaunchArgument("acceleration_scaling", default_value="0.10"),
        DeclareLaunchArgument("descent_velocity_scaling", default_value="0.03"),
        DeclareLaunchArgument("descent_acceleration_scaling", default_value="0.03"),
        DeclareLaunchArgument("target_x_min", default_value="-0.70"),
        DeclareLaunchArgument("target_x_max", default_value="-0.25"),
        DeclareLaunchArgument("target_y_min", default_value="0.30"),
        DeclareLaunchArgument("target_y_max", default_value="0.75"),
        DeclareLaunchArgument("target_z_min", default_value="-0.05"),
        DeclareLaunchArgument("target_z_max", default_value="0.12"),
    ]

    node = Node(
        package="thor7_moveit_demo",
        executable="vision_grasp_pregrasp_guarded",
        name="vision_grasp_pregrasp_guarded",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {
                "enable_execution": LaunchConfiguration("enable_execution"),
                "operator_confirmation": LaunchConfiguration("operator_confirmation"),
                "grasp_yaw": LaunchConfiguration("grasp_yaw"),
                "use_vision_yaw": LaunchConfiguration("use_vision_yaw"),
                "target_yaw_tolerance": LaunchConfiguration("target_yaw_tolerance"),
                "target_motion_confirmations": LaunchConfiguration("target_motion_confirmations"),
                "orientation_tolerance": LaunchConfiguration("orientation_tolerance"),
                "endpoint_position_tolerance": LaunchConfiguration("endpoint_position_tolerance"),
                "pregrasp_clearance": LaunchConfiguration("pregrasp_clearance"),
                "near_clearance": LaunchConfiguration("near_clearance"),
                "pregrasp_candidate_planning_time": LaunchConfiguration(
                    "pregrasp_candidate_planning_time"
                ),
                "pregrasp_total_planning_time": LaunchConfiguration(
                    "pregrasp_total_planning_time"
                ),
                "plan_max_age_sec": LaunchConfiguration("plan_max_age_sec"),
                "target_lock_sec": LaunchConfiguration("target_lock_sec"),
                "velocity_scaling": LaunchConfiguration("velocity_scaling"),
                "acceleration_scaling": LaunchConfiguration("acceleration_scaling"),
                "descent_velocity_scaling": LaunchConfiguration(
                    "descent_velocity_scaling"
                ),
                "descent_acceleration_scaling": LaunchConfiguration(
                    "descent_acceleration_scaling"
                ),
                "target_x_min": LaunchConfiguration("target_x_min"),
                "target_x_max": LaunchConfiguration("target_x_max"),
                "target_y_min": LaunchConfiguration("target_y_min"),
                "target_y_max": LaunchConfiguration("target_y_max"),
                "target_z_min": LaunchConfiguration("target_z_min"),
                "target_z_max": LaunchConfiguration("target_z_max"),
            },
        ],
    )

    return LaunchDescription(arguments + [node])
