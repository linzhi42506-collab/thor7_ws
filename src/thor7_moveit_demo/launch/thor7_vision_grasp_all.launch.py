"""Complete THOR7 workcell; planning-only by default."""
import math
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def include(package, filename, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(get_package_share_directory(package)) / 'launch' / filename)),
        launch_arguments=(arguments or {}).items())


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    mode = value('mode')
    if mode not in ('plan', 'execute'):
        raise RuntimeError('mode must be plan or execute')
    execute = mode == 'execute'
    auto_grasp = value('auto_grasp').lower() in ('true', '1', 'yes')
    pick_place = value('pick_place').lower() in ('true', '1', 'yes')
    loop = value('loop').lower() in ('true', '1', 'yes')
    if execute and value('operator_confirmation') != 'ARM_THOR7':
        raise RuntimeError('Execute mode requires operator_confirmation:=ARM_THOR7')
    if auto_grasp and not execute:
        raise RuntimeError('auto_grasp requires execute mode')
    if auto_grasp and value('auto_grasp_confirmation') != 'ARM_THOR7_AUTO_GRASP':
        raise RuntimeError('auto_grasp requires auto_grasp_confirmation:=ARM_THOR7_AUTO_GRASP')
    if loop and not auto_grasp:
        raise RuntimeError('loop requires auto_grasp=true')
    root = Path(value('workspace_root')).expanduser().resolve()
    calibration = Path(value('calibration_file')).expanduser()
    model = Path(value('model_path')).expanduser()
    for path in (calibration, model, root / 'ros2_vision_grasp.py', root / 'grasp_orientation.py'):
        if not path.is_file():
            raise RuntimeError(f'Required file missing: {path}')
    data = yaml.safe_load(calibration.read_text())
    if data.get('parent_frame') != 'base_link' or data.get('child_frame') != 'camera_color_optical_frame':
        raise RuntimeError('Calibration must describe base_link -> camera_color_optical_frame')
    t = data['transform']['translation']
    xyz = [float(t[k]) for k in ('x', 'y', 'z')]
    q = [float(x) for x in data['transform']['rotation_quaternion_xyzw']]
    if len(q) != 4 or not all(math.isfinite(x) for x in xyz + q):
        raise RuntimeError('Calibration contains invalid coordinates')
    norm = math.sqrt(sum(x*x for x in q))
    if abs(norm - 1.0) > 0.01:
        raise RuntimeError('Calibration quaternion is not normalized')
    q = [x / norm for x in q]
    tf_args = []
    for name, number in zip(('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'), xyz + q):
        tf_args.extend(['--' + name, str(number)])
    tf_args += ['--frame-id', 'base_link', '--child-frame-id', 'camera_color_optical_frame']

    automatic_status = (
        'run_grasp service enabled; no motion until explicitly called'
        if auto_grasp else 'no automatic planning or execution')
    actions = [LogInfo(msg=f'THOR7 mode={mode}; calibration={calibration}; {automatic_status}.')]
    if execute:
        actions += [LogInfo(msg='REAL controllers enabled. AG145 may move during initialization.'),
                    include('thor7v3_moveit_config', 'thor7_real_servoj_moveit_execute_with_ag145.launch.py', {
                        'operator_confirmation': value('operator_confirmation'),
                        'ag145_force_percent': value('ag145_force_percent')})]
    else:
        actions += [include('thor7v3_moveit_config', 'thor7_real_plan_only.launch.py'),
                    include('thor7_hardware', 'hardware_readonly.launch.py', {
                        'start_arm': 'false', 'start_gripper': 'true',
                        'gripper_joint_state_topic': '/joint_states'})]

    actions += [
        include('realsense2_camera', 'rs_launch.py', {
            'serial_no': '_' + value('camera_serial'),
            'camera_namespace': 'camera', 'camera_name': 'camera',
            'enable_color': 'true', 'enable_depth': 'true',
            'rgb_camera.color_profile': '1280,720,30',
            'depth_module.depth_profile': '1280,720,30',
            'align_depth.enable': 'true', 'enable_sync': 'true', 'publish_tf': 'false'}),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='calibrated_camera_tf', arguments=tf_args, output='screen'),
        Node(executable='/usr/bin/python3', arguments=[str(root / 'ros2_vision_grasp.py')],
             name='vision_grasp_node', output='screen', parameters=[{
                 'model_path': str(model), 'target_class': value('target_class'),
                 'show_window': False,
                 'enable_orientation_estimation':
                     value('enable_orientation_estimation').lower() in ('true', '1', 'yes'),
                 'orientation_height_tolerance': float(value('orientation_height_tolerance')),
                 'orientation_min_aspect_ratio': float(value('orientation_min_aspect_ratio')),
                 'orientation_max_grasp_width': float(value('orientation_max_grasp_width')),
                 'orientation_expected_length': float(value('orientation_expected_length')),
                 'orientation_expected_width': float(value('orientation_expected_width')),
                 'orientation_length_tolerance': float(value('orientation_length_tolerance')),
                 'orientation_width_tolerance': float(value('orientation_width_tolerance')),
                 'orientation_max_std_deg': float(value('orientation_max_std_deg'))}]),
        include('thor7_moveit_demo', 'workcell_collision_scene.launch.py', {
            name: value(name) for name in (
                'table_center_x', 'table_center_y', 'table_top_z',
                'table_size_x', 'table_size_y', 'table_thickness')}),
        include('thor7_moveit_demo', 'vision_grasp_pregrasp_guarded.launch.py', {
            'enable_execution': 'true' if execute else 'false',
            'operator_confirmation': 'ARM_THOR7_PREGRASP' if execute else 'DENIED',
            **{name: value(name) for name in (
                'pregrasp_clearance', 'near_clearance', 'grasp_yaw', 'use_vision_yaw',
                'pregrasp_candidate_planning_time', 'pregrasp_total_planning_time',
                'target_yaw_tolerance', 'target_motion_confirmations',
                'plan_max_age_sec', 'target_lock_sec',
                'velocity_scaling', 'acceleration_scaling',
                'descent_velocity_scaling', 'descent_acceleration_scaling',
                'target_x_min', 'target_x_max', 'target_y_min', 'target_y_max',
                'target_z_min', 'target_z_max')}}),
    ]
    if auto_grasp or pick_place:
        moveit_config = (MoveItConfigsBuilder('thor7v3', package_name='thor7v3_moveit_config')
                        .robot_description(file_path='config/thor7v3.real_readonly.urdf.xacro')
                        .trajectory_execution(file_path='config/moveit_controllers.real_servoj.yaml',
                                              moveit_manage_controllers=False)
                        .to_moveit_configs())
        actions += [Node(
            package='thor7_moveit_demo', executable='vision_grasp_auto_sequence',
            name='vision_grasp_auto_sequence', output='screen', parameters=[
                moveit_config.to_dict(),
                str(Path(get_package_share_directory('thor7_moveit_demo')) / 'config' / 'pick_place.yaml'), {
                'enable_auto_grasp': auto_grasp,
                'enable_pick_place': pick_place and auto_grasp,
                'enable_loop': loop,
                'use_vision_yaw': value('use_vision_yaw').lower() in ('true', '1', 'yes'),
                'max_cycles': int(value('max_cycles')),
                'new_target_min_distance': float(value('new_target_min_distance')),
                'new_target_wait_sec': float(value('new_target_wait_sec')),
                'close_during_return': value('close_during_return').lower() in ('true', '1', 'yes'),
                # A fixed drop pose cannot retain its old box and also accept
                # the next cycle. Continuous mode owns this policy.
                'retain_placed_objects': False if loop else
                    value('retain_placed_objects').lower() in ('true', '1', 'yes'),
                **{name: float(value(name)) for name in (
                    'target_x_min', 'target_x_max', 'target_y_min', 'target_y_max',
                    'target_z_min', 'target_z_max')},
                'operator_confirmation': value('auto_grasp_confirmation'),
                'near_clearance': float(value('near_clearance')),
                'pregrasp_settle_sec': float(value('pregrasp_settle_sec')),
                'pregrasp_replan_attempts': int(value('pregrasp_replan_attempts')),
            }])]
    return actions


def generate_launch_description():
    defaults = {
        'workspace_root': '/home/landau/Documents/thor7_ws',
        'mode': 'plan', 'operator_confirmation': 'DENIED',
        'auto_grasp': 'false', 'auto_grasp_confirmation': 'DENIED',
        'pick_place': 'false',
        'loop': 'false',
        'max_cycles': '0',
        'new_target_min_distance': '0.050',
        'new_target_wait_sec': '0.0',
        'close_during_return': 'true',
        # Continuous mode defaults to removing released boxes from the
        # planning scene so the fixed drop pose can be reused. The physical
        # stack must still be kept clear by the operator.
        'retain_placed_objects': 'true',
        'camera_serial': '802212060718', 'target_class': 'tree',
        'ag145_force_percent': '20',
        'pregrasp_clearance': '0.070', 'near_clearance': '0.020', 'grasp_yaw': '0.0',
        'pregrasp_candidate_planning_time': '8.0',
        'pregrasp_total_planning_time': '40.0',
        'pregrasp_settle_sec': '0.5',
        'pregrasp_replan_attempts': '2',
        'enable_orientation_estimation': 'true', 'use_vision_yaw': 'false',
        'orientation_height_tolerance': '0.008',
        'orientation_min_aspect_ratio': '1.30',
        'orientation_max_grasp_width': '0.145',
        'orientation_expected_length': '0.100',
        'orientation_expected_width': '0.050',
        'orientation_length_tolerance': '0.015',
        'orientation_width_tolerance': '0.010',
        'orientation_max_std_deg': '5.0', 'target_yaw_tolerance': '0.08726646260',
        'target_motion_confirmations': '3',
        'plan_max_age_sec': '60.0', 'target_lock_sec': '120.0', 'velocity_scaling': '0.15', 'acceleration_scaling': '0.10',
        'descent_velocity_scaling': '0.03', 'descent_acceleration_scaling': '0.03',
        'table_center_x': '-0.473', 'table_center_y': '0.502', 'table_top_z': '-0.028433',
        'table_size_x': '0.600', 'table_size_y': '0.600', 'table_thickness': '0.050',
        'target_x_min': '-0.70', 'target_x_max': '-0.25',
        'target_y_min': '0.30', 'target_y_max': '0.75',
        # The current pick fixture raises the wood top surface to about 92 mm
        # in base_link. Keep a modest guard margin above it.
        'target_z_min': '-0.05', 'target_z_max': '0.12',
    }
    args = [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
    args += [
        DeclareLaunchArgument('calibration_file', default_value=[
            LaunchConfiguration('workspace_root'), '/handeye_calibration_result.yaml']),
        DeclareLaunchArgument('model_path', default_value=[
            LaunchConfiguration('workspace_root'), '/best.pt']),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=setup)])
