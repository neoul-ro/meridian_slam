"""Bag replay mode: SLAM + Foxglove (+ RViz) with sim time.

Run `ros2 bag play <bag> --clock` alongside.
NOTE: stop the live system first — replaying while live drivers/SLAM run
mixes two timebases and diverges.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory('meridian_slam_bringup'), 'launch')
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'slam.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'use_rviz': LaunchConfiguration('use_rviz'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'foxglove.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),
    ])
