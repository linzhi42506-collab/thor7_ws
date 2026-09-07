from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder


CONFIRMATION_TOKEN = "ARM_THOR7"


def _require_operator_confirmation(context):
    actual = LaunchConfiguration(
        "operator_confirmation"
    ).perform(context)

    if actual != CONFIRMATION_TOKEN:
        raise RuntimeError(
            "REAL SERVOJ launch refused. "
            "Run the READ-ONLY preflight first, then launch with "
            "operator_confirmation:=ARM_THOR7"
        )

    return []


def generate_launch_description():

    operator_confirmation_argument = DeclareLaunchArgument(
        "operator_confirmation",
        default_value="DENIED",
        description=(
            "Explicit real-hardware acknowledgement. "
            "Required value: ARM_THOR7"
        ),
    )

    start_arm_controller_argument = DeclareLaunchArgument(
        "start_arm_controller",
        default_value="false",
        description=(
            "Load and activate the arm controller together with the joint "
            "state broadcaster. Keep false unless a guarded execution "
            "launcher explicitly enables it."
        ),
    )

    operator_confirmation_gate = OpaqueFunction(
        function=_require_operator_confirmation
    )

    moveit_config = (
        MoveItConfigsBuilder(
            "thor7v3",
            package_name="thor7v3_moveit_config",
        )
        .robot_description(
            file_path="config/thor7v3.real_servoj.urdf.xacro"
        )
        .trajectory_execution(
            file_path="config/moveit_controllers.real_servoj.yaml",
            moveit_manage_controllers=False,
        )
        .to_moveit_configs()
    )

    real_servoj_controllers_yaml = PathJoinSubstitution([
        FindPackageShare("thor7_hardware"),
        "config",
        "real_servoj_controllers.yaml",
    ])

    rviz_config = PathJoinSubstitution([
        FindPackageShare("thor7v3_moveit_config"),
        "config",
        "moveit.rviz",
    ])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
        ],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            real_servoj_controllers_yaml,
        ],
    )

    # Fail closed: if ros2_control/controller_manager exits,
    # MoveGroup and RViz must not remain alive with stale joint state.
    shutdown_on_controller_manager_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=controller_manager,
            on_exit=[
                LogInfo(
                    msg=(
                        "[THOR7 SAFETY] controller_manager exited; "
                        "shutting down the complete REAL SERVOJ stack."
                    )
                ),
                EmitEvent(
                    event=Shutdown(
                        reason=(
                            "Thor7 controller_manager exited; "
                            "real-hardware execution is no longer valid."
                        )
                    )
                ),
            ],
        )
    )

    # Safety gate:
    # only publish real joint states automatically.
    # thor7_arm_controller is activated manually after inspection.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
        condition=UnlessCondition(
            LaunchConfiguration("start_arm_controller")
        ),
    )

    # Use one spawner for both controllers during guarded execution. Separate
    # spawners can race while controller_manager is starting and leave the arm
    # controller loaded but unconfigured.
    execution_controllers_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "thor7_arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--activate-as-group",
        ],
        output="screen",
        condition=IfCondition(
            LaunchConfiguration("start_arm_controller")
        ),
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                # HARD SOFTWARE EXECUTION BARRIER.
                "allow_trajectory_execution": True,

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
        operator_confirmation_argument,
        start_arm_controller_argument,
        operator_confirmation_gate,
        shutdown_on_controller_manager_exit,
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        execution_controllers_spawner,
        move_group,
        rviz,
    ])
