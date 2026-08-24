from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    robot_xacro = PathJoinSubstitution([
        FindPackageShare("thor7v3_moveit_config"),
        "config",
        "thor7v3.real_readonly.urdf.xacro",
    ])

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare("thor7_hardware"),
        "config",
        "real_readonly_controllers.yaml",
    ])

    # 必须显式声明为字符串，否则 launch_ros 会尝试把 URDF 当 YAML 解析
    robot_description_content = Command([
        FindExecutable(name="xacro"),
        " ",
        robot_xacro,
    ])

    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content,
            value_type=str,
        )
    }

    # RobotStatePublisher:
    # 根据 /joint_states 发布 TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # ros2_control:
    # 加载 Thor7 READ-ONLY SystemInterface
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            robot_description,
            controllers_yaml,
        ],
    )

    # 只启动 joint_state_broadcaster。
    # 不启动 thor7_arm_controller。
    # 不启动 gripper_controller。
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

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
    ])

