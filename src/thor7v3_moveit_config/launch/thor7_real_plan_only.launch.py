from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # Use the real READ-ONLY URDF instead of the default FakeSystem URDF.
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

    # Do not pass any trajectory-controller configuration to move_group.
    # ROS 2 launch cannot infer the type of an empty controller_names list.
    # Execution remains disabled independently by allow_trajectory_execution.
    move_group_parameters = moveit_config.to_dict()

    for parameter_name in (
        "moveit_controller_manager",
        "moveit_manage_controllers",
        "moveit_simple_controller_manager",
    ):
        move_group_parameters.pop(parameter_name, None)

    readonly_controllers_yaml = PathJoinSubstitution([
        FindPackageShare("thor7_hardware"),
        "config",
        "real_readonly_controllers.yaml",
    ])

    rviz_config = PathJoinSubstitution([
        FindPackageShare("thor7v3_moveit_config"),
        "config",
        "moveit.rviz",
    ])

    # Publishes TF using the same READ-ONLY robot description.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
        ],
    )

    # Real hardware connection, but the selected hardware mode exports
    # state interfaces only and has no command interface.
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            readonly_controllers_yaml,
        ],
    )

    # Publish only real joint states.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # Software execution barrier:
    # MoveIt can plan but must reject trajectory execution.
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            move_group_parameters,
            {
                "allow_trajectory_execution": False,
                "publish_robot_description_semantic": True,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
                "monitor_dynamics": False,
            },
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            rviz_config,
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        move_group,
        rviz,
    ])
