# thor7_sim 仿真启动：RViz + MoveIt + FakeSystem 机械臂/夹爪 + 抓取演示
#
# 用法：
#   ros2 launch thor7_sim sim.launch.py                 # 完整流程（含 RViz + 抓取演示）
#   ros2 launch thor7_sim sim.launch.py use_rviz:=false run_grasp_demo:=false   # 无界面，只起仿真
#
# 启动内容：
#   1) thor7v3_moveit_config/demo.launch.py
#      （robot_state_publisher + move_group + ros2_control FakeSystem + 控制器 + RViz）
#   2) gripper_state_publisher  （AG-145 mimic 关节计算发布）
#   3) spawn_test_object        （规划场景中添加测试立方体）
#   4) grasp_demo（可选）        （自动演示 预抓取→夹紧→抬升→松开）

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_share = get_package_share_directory('thor7v3_moveit_config')
    sim_share = get_package_share_directory('thor7_sim')
    sim_config = os.path.join(sim_share, 'config', 'sim.yaml')

    moveit_config = MoveItConfigsBuilder(
        'thor7v3', package_name='thor7v3_moveit_config'
    ).to_moveit_configs()

    demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_share, 'launch', 'demo.launch.py')
        ),
        launch_arguments={'use_rviz': LaunchConfiguration('use_rviz')}.items(),
    )

    gripper_state = Node(
        package='thor7_sim',
        executable='gripper_state_publisher',
        name='gripper_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description],
    )

    spawn_object = Node(
        package='thor7_sim',
        executable='spawn_test_object',
        name='spawn_test_object',
        output='screen',
        parameters=[sim_config],
    )

    grasp_demo = Node(
        package='thor7_sim',
        executable='grasp_demo',
        name='grasp_demo',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            sim_config,
        ],
        condition=IfCondition(LaunchConfiguration('run_grasp_demo')),
    )

    cartesian_diagnostic = Node(
        package='thor7_sim',
        executable='cartesian_diagnostic',
        name='cartesian_diagnostic',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            sim_config,
        ],
        condition=IfCondition(LaunchConfiguration('run_diagnostic')),
    )

    collision_diagnostic = Node(
        package='thor7_sim',
        executable='collision_diagnostic',
        name='collision_diagnostic',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            sim_config,
        ],
        condition=IfCondition(LaunchConfiguration('run_collision_diag')),
    )

    gripper_test = Node(
        package='thor7_sim',
        executable='gripper_test',
        name='gripper_test',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            sim_config,
        ],
        condition=IfCondition(LaunchConfiguration('run_gripper_test')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('run_grasp_demo', default_value='false'),
        DeclareLaunchArgument('run_diagnostic', default_value='false'),
        DeclareLaunchArgument('run_collision_diag', default_value='false'),
        DeclareLaunchArgument('run_gripper_test', default_value='false'),
        demo,
        gripper_state,
        spawn_object,
        # 延迟启动，等 move_group 完全就绪（避免 action 抢跑）
        TimerAction(period=10.0, actions=[grasp_demo]),
        TimerAction(period=10.0, actions=[cartesian_diagnostic]),
        TimerAction(period=10.0, actions=[collision_diagnostic]),
        TimerAction(period=10.0, actions=[gripper_test]),
    ])
