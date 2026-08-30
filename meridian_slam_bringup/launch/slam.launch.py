"""FAST-LIVO2 + TF tree (URDF, statics, odom relay). No sensor drivers.

The single entry point of this package. Sensor drivers live in meridian_sensor;
whole-system bringup (drivers + SLAM + perception) is meridian_bringup, which
includes this file as its SLAM stage.

    live    ros2 launch meridian_slam_bringup slam.launch.py
    replay  ... use_sim_time:=true use_rviz:=true
    view    ... use_slam:=false use_robot_description:=false \
                use_sim_time:=true use_rviz:=true

"replay" runs SLAM against `ros2 bag play <bag> --clock`; stop the live system
first, because two timebases diverge. "view" replays a bag that already carries
SLAM output -- use_slam:=false drops FAST-LIVO2 and the odom relay, and
use_robot_description:=false drops the URDF publishers, so the recorded TF is
displayed as-is rather than published a second time.

The Foxglove bridge is not launched here. It is visualization, it outlives any
one run of this file, and a second one just loses the race for its port. Start
it by hand when you want it:

    ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765

meridian_bringup still includes a foxglove.launch.py from this package by name;
that include is dead now, and its owner has to drop it.

Publishes the platform TF tree (FAST-LIVO2 is patched to use "map" directly
as its world frame; the legacy camera_init frame no longer exists). Everything
below base_link comes from the URDF, which use_robot_description:=false hands
over to whoever else is publishing it:

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
        # Set false when a larger bringup already runs robot_state_publisher for
        # this robot; two of them publish the same TF edges. Nothing here takes
        # the URDF's place when it is off -- the other publisher owns the whole
        # base_link subtree, imu_link included, and odom_tf_relay waits on that
        # lookup before it publishes map -> base_link or /pose.
        DeclareLaunchArgument('use_robot_description', default_value='true'),
        # Set false to view a bag that already carries SLAM output: the mapper
        # and the relay stay down, and the recorded pose/TF play back untouched.
        DeclareLaunchArgument('use_slam', default_value='true'),
    ]
    use_sim_time = {'use_sim_time': ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)}
    use_urdf = LaunchConfiguration('use_robot_description')
    use_slam = LaunchConfiguration('use_slam')

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

    slam_nodes = [
        Node(
            package='fast_livo',
            executable='fastlivo_mapping',
            name='laserMapping',
            condition=IfCondition(use_slam),
            parameters=[lio_config, cam_config, use_sim_time],
            output='screen',
        ),
        Node(
            package='meridian_slam_bringup',
            executable='odom_tf_relay',
            name='odom_tf_relay',
            condition=IfCondition(use_slam),
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
