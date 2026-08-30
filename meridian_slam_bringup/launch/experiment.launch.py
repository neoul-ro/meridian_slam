"""개인 실험용 한 방 런치 — 드라이버 + SLAM + RViz. 공용 통합 경로가 아니다.

**이 파일은 시스템 통합 런치가 아니다.** 통합은 meridian_bringup 이 한다
(드라이버 + SLAM + 인식). 이 파일은 인식 스택 없이 내 리그만 혼자 돌려볼 때
명령 한 줄로 끝내려고 두는 편의 런치다.

    ros2 launch meridian_slam_bringup experiment.launch.py

    drivers   meridian_sensor  lidar / camera / imu   (use_drivers, 기본 true)
    slam      slam.launch.py   FAST-LIVO2 + TF + RViz (use_slam)

드라이버가 이미 떠 있으면 이 파일을 쓰지 않는다
------------------------------------------------
같은 장치를 두 프로세스가 잡으면 깨진다. 드라이버를 켜 둔 채 SLAM 만 다시
띄우고 싶으면 slam.launch.py 를 직접 쓰거나 use_drivers:=false 를 준다.

    ros2 launch meridian_slam_bringup slam.launch.py            # SLAM 만
    ros2 launch meridian_slam_bringup experiment.launch.py use_drivers:=false

Foxglove 는 여기서 안 띄운다. 파이프라인 수명과 별개라 손으로 켠다.

    ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765

기본값은 여기 없다
------------------
velodyne_ip, vectornav_port, vectornav_baud, enable_depth, use_sim_time,
use_robot_description, use_slam -- 전부 각 자식 launch 가 자기 기본값을 들고
있다. 이 파일은 그 값을 다시 선언하지도, 넘기지도 않는다. 선언적 include 라
명령줄 값이 launch configuration 상속으로 자식까지 그대로 내려간다.

    ros2 launch meridian_slam_bringup experiment.launch.py velodyne_ip:=192.168.1.99
    ros2 launch meridian_slam_bringup experiment.launch.py --show-args

예외는 use_rviz 하나다. slam.launch.py 가 기본 false 로 갖고 있는데, 실험용인
이 파일에서는 켜져 있는 쪽이 기본이라 여기서 먼저 선언해 true 로 뒤집는다.
형제 include 는 스코프를 공유하고 먼저 선언한 쪽의 기본값이 이긴다 (Humble).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

DRIVER_LAUNCH_FILES = ('lidar.launch.py', 'camera.launch.py', 'imu.launch.py')


def source(package, filename):
    return PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare(package), 'launch', filename]))


def generate_launch_description():
    # 드라이버 3종은 meridian_sensor 가 소유한다 (예전에 여기 있었지만 옮겨갔다).
    drivers = [
        IncludeLaunchDescription(
            source('meridian_sensor', filename),
            condition=IfCondition(LaunchConfiguration('use_drivers')),
        )
        for filename in DRIVER_LAUNCH_FILES
    ]

    slam = IncludeLaunchDescription(source('meridian_slam_bringup', 'slam.launch.py'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_drivers', default_value='true',
            description='라이다/카메라/IMU 드라이버까지 띄운다. 드라이버가 이미 '
                        '떠 있으면 false -- 같은 장치를 두 번 잡으면 깨진다.'),
        # slam.launch.py 의 기본값(false)을 실험용으로 뒤집는 의도적 재선언.
        DeclareLaunchArgument('use_rviz', default_value='true'),
        *drivers,
        slam,
    ])
