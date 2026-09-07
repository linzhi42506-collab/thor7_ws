# thor7_hardware: 只读硬件验证 launch
#
# 只启动两个只读 probe（Diana 机械臂 + AG145 夹爪）。
# 绝不启动 MoveIt move_group / RViz / ros2_control / 任何 controller / grasp_demo。
#
# 用法：
#   ros2 launch thor7_hardware hardware_readonly.launch.py            # 两个 probe 都启动
#   ros2 launch thor7_hardware hardware_readonly.launch.py start_gripper:=false   # 只读机械臂
#   ros2 launch thor7_hardware hardware_readonly.launch.py start_arm:=false       # 只读夹爪
#
# 参数文件可覆盖：--params-file <你的.yaml>

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('thor7_hardware')
    params_file = os.path.join(pkg, 'config', 'real_readonly.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('start_arm', default_value='true'),
        DeclareLaunchArgument('start_gripper', default_value='true'),
        DeclareLaunchArgument(
            'gripper_joint_state_topic',
            default_value='/ag145/readback_joint_state',
            description=(
                'AG145 readback topic. Use /joint_states when MoveIt/preflight '
                'must consume the gripper feedback.'
            ),
        ),

        LogInfo(msg='========== READ-ONLY HARDWARE MODE =========='),
        LogInfo(msg='NO MOTION COMMANDS ARE IMPLEMENTED. This launch only READS sensor/joint feedback.'),
        LogInfo(msg='No move_group, no RViz, no ros2_control, no controllers, no grasp_demo will start.'),

        Node(
            package='thor7_hardware',
            executable='thor7_readonly_probe',
            name='thor7_readonly_probe',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(LaunchConfiguration('start_arm')),
        ),
        Node(
            package='thor7_hardware',
            executable='ag145_readonly_probe',
            name='ag145_readonly_probe',
            output='screen',
            parameters=[
                params_file,
                {
                    'joint_state_topic': LaunchConfiguration(
                        'gripper_joint_state_topic'
                    ),
                },
            ],
            condition=IfCondition(LaunchConfiguration('start_gripper')),
        ),
    ])
