"""개인 실험용 한 방 런치: 리그 전체를 한 명령으로 띄운다.

    lidar.launch.py     VLP-16 driver          (use_lidar)     ← meridian_sensor
    camera.launch.py    D435 driver            (use_camera)    ← meridian_sensor
    imu.launch.py       VN-100 driver, 100Hz   (use_imu)       ← meridian_sensor
    slam.launch.py      FAST-LIVO2 + TF tree   (use_slam, use_robot_description)
    foxglove_bridge     :8765                  (use_foxglove)

**공용 통합 경로가 아니다.** 통합은 meridian_bringup 이 한다 (드라이버 + SLAM +
인식). 이 파일은 인식 스택 없이 내 리그만 혼자 돌려볼 때 쓰는 편의 런치다.
각 조각은 따로도 띄울 수 있다.

드라이버가 이미 떠 있으면 use_lidar/use_camera/use_imu 를 false 로 주거나
slam.launch.py 를 직접 쓴다 — 같은 장치를 두 프로세스가 잡으면 깨진다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 드라이버 3종은 meridian_sensor 소유다 (예전엔 여기 있었지만 옮겨갔다).
    sensor_dir = os.path.join(
        get_package_share_directory('meridian_sensor'), 'launch')
    slam_dir = os.path.join(
        get_package_share_directory('meridian_slam_bringup'), 'launch')

    def include(directory, name, flag, arg_names):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(directory, name)),
            condition=IfCondition(LaunchConfiguration(flag)),
            launch_arguments={a: LaunchConfiguration(a) for a in arg_names}.items(),
        )

    # foxglove.launch.py 는 없앴다. 노드 하나뿐이라 여기 직접 둔다.
    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        condition=IfCondition(LaunchConfiguration('use_foxglove')),
        parameters=[{
            'port': ParameterValue(
                LaunchConfiguration('foxglove_port'), value_type=int),
            'address': '0.0.0.0',
            'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool),
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('use_imu', default_value='true'),
        DeclareLaunchArgument('use_slam', default_value='true'),
        # 실험용이라 켜 두는 쪽이 기본이다. 다른 bringup 이 이미 브릿지를 띄웠으면
        # false 로 준다 -- 두 번째 foxglove_bridge 는 8765 를 못 잡는다.
        DeclareLaunchArgument('use_foxglove', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_robot_description', default_value='true'),
        DeclareLaunchArgument('velodyne_ip', default_value='192.168.1.201'),
        DeclareLaunchArgument('vectornav_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('vectornav_baud', default_value='921600'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),
        DeclareLaunchArgument('enable_depth', default_value='true'),

        include(sensor_dir, 'lidar.launch.py', 'use_lidar', ['velodyne_ip']),
        include(sensor_dir, 'camera.launch.py', 'use_camera', ['enable_depth']),
        include(sensor_dir, 'imu.launch.py', 'use_imu',
                ['vectornav_port', 'vectornav_baud']),
        include(slam_dir, 'slam.launch.py', 'use_slam',
                ['use_rviz', 'use_sim_time', 'use_robot_description']),
        foxglove,
    ])
