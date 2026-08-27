"""Full system bringup: composes the five component launches.

    lidar.launch.py     VLP-16 driver          (use_lidar)
    camera.launch.py    D435 driver            (use_camera)
    imu.launch.py       VN-100 driver, 100Hz   (use_imu)
    slam.launch.py      FAST-LIVO2 + TF tree   (use_slam)
    foxglove.launch.py  Foxglove bridge :8765  (use_foxglove, off by default)

Each component can also be launched on its own with the same files.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory('meridian_slam_bringup'), 'launch')

    def include(name, flag, arg_names):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, name)),
            condition=IfCondition(LaunchConfiguration(flag)),
            launch_arguments={a: LaunchConfiguration(a) for a in arg_names}.items(),
        )

    return LaunchDescription([
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('use_imu', default_value='true'),
        DeclareLaunchArgument('use_slam', default_value='true'),
        # Off by default. This file is the whole-rig convenience launch, but
        # the pieces it composes are also what other bringups compose around,
        # and a second foxglove_bridge just loses the race for 8765. Ask for
        # it with use_foxglove:=true, or run foxglove.launch.py on its own.
        DeclareLaunchArgument('use_foxglove', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('velodyne_ip', default_value='192.168.1.201'),
        DeclareLaunchArgument('vectornav_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('vectornav_baud', default_value='921600'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),
        DeclareLaunchArgument('enable_depth', default_value='true'),

        include('lidar.launch.py', 'use_lidar', ['velodyne_ip']),
        include('camera.launch.py', 'use_camera', ['enable_depth']),
        include('imu.launch.py', 'use_imu', ['vectornav_port', 'vectornav_baud']),
        include('slam.launch.py', 'use_slam',
                ['use_rviz', 'use_sim_time']),
        include('foxglove.launch.py', 'use_foxglove',
                ['foxglove_port', 'use_sim_time']),
    ])
