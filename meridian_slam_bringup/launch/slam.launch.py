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
                 +- imu_link                (static, measured mount, RFU axes)
                 +- wheel * 16              (URDF)
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def static_tf(name, parent, child, xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0),
              condition=None):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        condition=condition,
        arguments=[
            '--x', str(xyz[0]), '--y', str(xyz[1]), '--z', str(xyz[2]),
            '--roll', str(rpy[0]), '--pitch', str(rpy[1]), '--yaw', str(rpy[2]),
            '--frame-id', parent, '--child-frame-id', child,
        ],
        output='screen',
    )


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
        DeclareLaunchArgument('use_robot_description', default_value='true'),
    ]
    use_sim_time = {'use_sim_time': ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)}
    use_urdf = LaunchConfiguration('use_robot_description')

    description_nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            condition=IfCondition(use_urdf),
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
            condition=IfCondition(use_urdf),
            parameters=[use_sim_time],
            output='screen',
        ),
    ]

    # Frames the URDF does not provide (IMU mount).
    # Measured mount from bunker_tf/config/frames.yaml: 15cm back from the front
    # edge, sitting on the top plate, 10cm directly below the camera. The yaw is
    # -90deg because imu_link is RFU (the vectornav driver defaults to
    # use_enu=true), not the ROS-standard FLU — see extrinsic_R in
    # velodyne16_vn100.yaml. Keep odom_tf_relay's imu_in_base_* consistent.
    tf_nodes = [
        static_tf('tf_chassis_imu', 'chassis', 'imu_link',
                  xyz=(0.380, 0.0, 0.0371), rpy=(0.0, 0.0, -math.pi / 2)),
        # Fallback statics when the URDF is disabled (identity offsets only).
        static_tf('tf_base_chassis', 'base_link', 'chassis',
                  condition=UnlessCondition(use_urdf)),
        static_tf('tf_chassis_velo_base', 'chassis', 'velodyne_base_link',
                  condition=UnlessCondition(use_urdf)),
        static_tf('tf_velo_base_velo', 'velodyne_base_link', 'velodyne',
                  condition=UnlessCondition(use_urdf)),
        static_tf('tf_chassis_camera', 'chassis', 'camera_link',
                  condition=UnlessCondition(use_urdf)),
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
            parameters=[{
                'odom_topic': '/aft_mapped_to_init',
                'map_frame': 'map',
                'base_frame': 'base_link',
                # Pose of imu_link in base_link: base_link->chassis is
                # z +0.332 (URDF chassis_joint), chassis->imu_link is the
                # measured mount above -> z 0.332 + 0.0371.
                'imu_in_base_xyz': [0.380, 0.0, 0.3691],
                'imu_in_base_rpy': [0.0, 0.0, -math.pi / 2],
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
        args + description_nodes + tf_nodes + slam_nodes + [rviz_node])
