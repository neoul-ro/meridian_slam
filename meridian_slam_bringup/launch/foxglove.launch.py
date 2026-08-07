"""Foxglove bridge only. Connect Foxglove Studio to ws://<jetson-ip>:8765."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('foxglove_port', default_value='8765'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{
                'port': ParameterValue(
                    LaunchConfiguration('foxglove_port'), value_type=int),
                'address': '0.0.0.0',
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
            }],
            output='screen',
        ),
    ])
