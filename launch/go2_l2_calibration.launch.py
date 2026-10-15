"""Calibrate the Unitree L2 cloud against its internal IMU with LI-Init."""
from pathlib import Path
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from go2_dds_ros2_bridge.dds_runtime import DEFAULT_HARDWARE_DDS_RUNTIME


def _nodes(context):
    output_dir = LaunchConfiguration('output_dir').perform(context)
    local_lib = Path.home() / '.local/li_init_deps/usr/lib'
    local_arch_lib = local_lib / 'x86_64-linux-gnu'
    libraries = [str(path) for path in (local_arch_lib, local_lib) if path.exists()]
    environment = {'LI_INIT_OUTPUT_DIR': output_dir}
    if libraries:
        environment['LD_LIBRARY_PATH'] = ':'.join(libraries + [os.environ.get('LD_LIBRARY_PATH', '')])
    dds_args = [
        '--run-mode', 'hardware',
        '--dds-domain-id', LaunchConfiguration('dds_domain_id').perform(context),
        '--dds-interface', LaunchConfiguration('dds_interface').perform(context),
        '--cloud-ros', '/li_init/input/cloud',
        '--imu-ros', '/li_init/input/imu',
        '--lidar-frame', 'li_init_lidar',
        '--imu-frame', 'li_init_imu',
        '--preserve-sensor-stamps',
    ]
    return [
        Node(package='go2_dds_ros2_bridge', executable='fastlio_l2_input',
             name='li_init_input', output='screen', arguments=dds_args),
        Node(package='lidar_imu_init', executable='li_init', name='li_init',
             output='screen', parameters=[LaunchConfiguration('config')],
             additional_env=environment,
             remappings=[
                 ('/livox/imu/async', '/li_init/imu_synced'),
                 ('/cloud_registered', '/li_init/cloud_registered'),
                 ('/cloud_registered_body', '/li_init/cloud_registered_body'),
                 ('/cloud_effected', '/li_init/cloud_effected'),
                 ('/Laser_map', '/li_init/laser_map'),
                 ('/aft_mapped_to_init', '/li_init/odom'),
                 ('/path', '/li_init/path'),
             ]),
    ]


def generate_launch_description():
    config = str(Path(get_package_share_directory('lidar_imu_init')) / 'config/go2_l2_ros2.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('dds_domain_id', default_value=str(DEFAULT_HARDWARE_DDS_RUNTIME.domain_id)),
        DeclareLaunchArgument('dds_interface', default_value=DEFAULT_HARDWARE_DDS_RUNTIME.interface),
        DeclareLaunchArgument('config', default_value=config),
        DeclareLaunchArgument('output_dir', default_value=str(Path.home() / '.ros/li_init_l2')),
        OpaqueFunction(function=_nodes),
    ])
