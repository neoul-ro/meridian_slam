"""View-only mode: RViz + Foxglove with sim time, no SLAM.

For replaying a bag that already contains SLAM outputs:
    ros2 bag play <bag> --clock
Everything recorded (pose, TF, cloud, path, image) is displayed as-is.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('meridian_slam_bringup')
    rviz_config = os.path.join(share, 'rviz', 'meridian_slam.rviz')
    launch_dir = os.path.join(share, 'launch')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        # Backward compatibility: bags recorded before the camera_init -> map
        # frame rename carry camera_init-stamped topics; keep the identity
        # link so they still render under Fixed Frame map. Also covers missed
        # latched /tf_static when the viewer starts late.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_map_camera_init',
            arguments=['--frame-id', 'map', '--child-frame-id', 'camera_init'],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'foxglove.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),
    ])
