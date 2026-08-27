"""FAST-LIVO2 + TF tree (URDF, statics, odom relay). No sensor drivers.

Publishes the platform TF tree (FAST-LIVO2 is patched to use "map" directly
as its world frame; the legacy camera_init frame no longer exists):

    map -+- aft_mapped                      (dynamic, raw SLAM pose, debug)
         +- base_link                       (dynamic, from odom_tf_relay)
             +- chassis                     (URDF)
                 +- velodyne_base_link      (URDF, mount offset)
                 |   +- velodyne
                 +- camera_link             (URDF, mount offset)
                 |   +- <optical * N>       (published by realsense2_camera)
                 +- imu_link                (URDF, mount offset, RFU axes)
                 +- wheel * 16              (URDF)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    fast_livo_share = get_package_share_directory('fast_livo')
    lio_config = os.path.join(fast_livo_share, 'config', 'velodyne16_vn100.yaml')
    cam_config = os.path.join(fast_livo_share, 'config', 'camera_d435.yaml')
    rviz_config = os.path.join(
        get_package_share_directory('meridian_slam_bringup'), 'rviz', 'meridian_slam.rviz')
    urdf_file = os.path.join(
        get_package_share_directory('bunker_description'), 'urdf', 'bunker_d435.urdf.xacro')

    args = [
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ]
    use_sim_time = {'use_sim_time': ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)}

    description_nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', urdf_file]), value_type=str),
            }, use_sim_time],
            output='screen',
        ),
        # Wheels are revolute joints; publish zero joint states for them.
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[use_sim_time],
            output='screen',
        ),
    ]

    slam_nodes = [
        Node(
            package='fast_livo',
            executable='fastlivo_mapping',
            name='laserMapping',
            parameters=[lio_config, cam_config, use_sim_time],
            output='screen',
        ),
        Node(
            package='meridian_slam_bringup',
            executable='odom_tf_relay',
            name='odom_tf_relay',
            # The mount offset is not configured here any more: the relay
            # looks imu_link -> base_link up on TF, where the URDF already
            # publishes it.
            parameters=[{
                'odom_topic': '/aft_mapped_to_init',
                'map_frame': 'map',
                'base_frame': 'base_link',
                'imu_frame': 'imu_link',
            }, use_sim_time],
            output='screen',
        ),
    ]

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        arguments=['-d', rviz_config],
        parameters=[use_sim_time],
        output='screen',
    )

    return LaunchDescription(
        args + description_nodes + slam_nodes + [rviz_node])
