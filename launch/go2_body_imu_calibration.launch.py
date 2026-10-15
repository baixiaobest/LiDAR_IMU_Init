"""Calibrate the Unitree L2 cloud against the Go2 body IMU."""
from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from go2_dds_ros2_bridge.dds_runtime import DEFAULT_HARDWARE_DDS_RUNTIME


def _nodes(context):
    output_dir = LaunchConfiguration('output_dir').perform(context)
    dds_args = [
        '--run-mode', 'hardware',
        '--dds-domain-id', LaunchConfiguration('dds_domain_id').perform(context),
        '--dds-interface', LaunchConfiguration('dds_interface').perform(context),
    ]
    local_lib = Path.home() / '.local/li_init_deps/usr/lib'
    libraries = [str(path) for path in (local_lib / 'x86_64-linux-gnu', local_lib) if path.exists()]
    environment = {'LI_INIT_OUTPUT_DIR': output_dir}
    if libraries:
        environment['LD_LIBRARY_PATH'] = ':'.join(libraries + [os.environ.get('LD_LIBRARY_PATH', '')])
    return [
        Node(package='go2_dds_ros2_bridge', executable='clock_offset_bridge',
             name='li_init_go2_cloud_input', output='screen',
             arguments=dds_args + ['--no-publish-tf',
                                   '--output-topic', '/li_init_go2/input/cloud',
                                   '--unfiltered-output-topic', '/li_init_go2/input/cloud_unfiltered']),
        Node(package='go2_dds_ros2_bridge', executable='imu_bridge',
             name='li_init_go2_imu_input', output='screen',
             arguments=dds_args + ['--ros-topic', '/li_init_go2/input/imu', '--frame-id', 'body']),
        Node(package='lidar_imu_init', executable='li_init', name='li_init_go2',
             output='screen', parameters=[LaunchConfiguration('config')],
             additional_env=environment,
             remappings=[
                 ('/livox/imu/async', '/li_init_go2/imu_synced'),
                 ('/cloud_registered', '/li_init_go2/cloud_registered'),
                 ('/cloud_registered_body', '/li_init_go2/cloud_registered_lidar'),
                 ('/cloud_effected', '/li_init_go2/cloud_effected'),
                 ('/Laser_map', '/li_init_go2/laser_map'),
                 ('/aft_mapped_to_init', '/li_init_go2/odom'),
                 ('/path', '/li_init_go2/path'),
             ]),
    ]


def generate_launch_description():
    config = str(Path(get_package_share_directory('lidar_imu_init')) / 'config/go2_body_imu_ros2.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('dds_domain_id', default_value=str(DEFAULT_HARDWARE_DDS_RUNTIME.domain_id)),
        DeclareLaunchArgument('dds_interface', default_value=DEFAULT_HARDWARE_DDS_RUNTIME.interface),
        DeclareLaunchArgument('config', default_value=config),
        DeclareLaunchArgument('output_dir', default_value=str(Path.home() / '.ros/li_init_go2_body')),
        OpaqueFunction(function=_nodes),
    ])
