"""Guided intrinsic calibration of the Go2 body IMU, separate from FAST-LIO."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from go2_dds_ros2_bridge.dds_runtime import DEFAULT_HARDWARE_DDS_RUNTIME


def nodes(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    result = []
    if value('start_bridge').lower() in ('true', '1'):
        result.append(Node(package='go2_dds_ros2_bridge', executable='imu_bridge',
                           name='go2_body_static_calibration_input', output='screen', arguments=[
                               '--run-mode', 'hardware', '--dds-domain-id', value('dds_domain_id'),
                               '--dds-interface', value('dds_interface'),
                               '--ros-topic', value('imu_topic'), '--frame-id', 'body']))
    parameters = {key: value(key) for key in
                  ('mode', 'accelerometer_calibration', 'imu_topic', 'output_dir')}
    parameters['min_pose_angle_deg'] = float(value('min_pose_angle_deg'))
    parameters['vector_topic'] = '/go2_body_imu_calibration/acceleration_vector'
    parameters['body_axis_guidance'] = True
    parameters['ui_port'] = int(value('ui_port'))
    parameters['open_browser'] = value('open_browser').lower() in ('true', '1')
    result.append(Node(package='lidar_imu_init', executable='imu_static_calibration',
                       name='go2_body_imu_static_calibration', output='screen',
                       parameters=[parameters]))
    return result


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='full', choices=['accelerometer', 'gyroscope', 'full']),
        DeclareLaunchArgument('accelerometer_calibration', default_value=''),
        DeclareLaunchArgument('start_bridge', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('imu_topic', default_value='/go2_body_imu_calibration/imu'),
        DeclareLaunchArgument('output_dir', default_value=str(Path.home() / '.ros/go2_body_imu_static_calibration')),
        DeclareLaunchArgument('min_pose_angle_deg', default_value='15.0'),
        DeclareLaunchArgument('ui_port', default_value='8766'),
        DeclareLaunchArgument('open_browser', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('dds_domain_id', default_value=str(DEFAULT_HARDWARE_DDS_RUNTIME.domain_id)),
        DeclareLaunchArgument('dds_interface', default_value=DEFAULT_HARDWARE_DDS_RUNTIME.interface),
        OpaqueFunction(function=nodes),
    ])
