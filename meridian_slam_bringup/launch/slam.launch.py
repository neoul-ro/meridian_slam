"""FAST-LIVO2 + TF 트리 (URDF, static, odom 릴레이). 센서 드라이버는 안 띄운다.

이 패키지의 진입점이다. 센서 드라이버는 meridian_sensor 에 있고, 전체 시스템
통합(드라이버 + SLAM + 인식)은 meridian_bringup 이 한다 — 그쪽이 이 파일을
SLAM 단계로 include 한다.

    라이브    ros2 launch meridian_slam_bringup slam.launch.py
    재생      ... use_sim_time:=true use_rviz:=true
    보기만    ... use_slam:=false use_robot_description:=false \
                  use_sim_time:=true use_rviz:=true

"재생" 은 `ros2 bag play <bag> --clock` 을 상대로 SLAM 을 돌린다. 라이브 시스템을
먼저 끄고 해야 한다 — 시간축이 둘이면 어긋난다. "보기만" 은 SLAM 결과가 이미 든
bag 을 그대로 표시하는 모드다. use_slam:=false 가 FAST-LIVO2 와 odom 릴레이를
내려두고, use_robot_description:=false 가 URDF 발행자를 내려둬서, 녹화된 TF 가
두 번 발행되지 않고 그대로 보인다.

Foxglove 브릿지는 여기서 안 띄운다. 시각화는 이 스택의 수명과 별개고, 두 번째
foxglove_bridge 는 포트를 못 잡는다. 필요하면 손으로 켠다.

    ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765

개인 실험용으로 드라이버까지 한 번에 띄우려면 experiment.launch.py 를 쓴다
(브릿지도 같이 뜬다).

플랫폼 TF 트리를 발행한다 (FAST-LIVO2 를 패치해서 월드 프레임으로 "map" 을 직접
쓰게 했다. 예전의 camera_init 프레임은 이제 없다). base_link 아래는 전부 URDF 에서
나오고, use_robot_description:=false 는 그 역할을 다른 발행자에게 넘긴다.

    map -+- aft_mapped                      (동적, 원시 SLAM 자세, 디버그용)
         +- base_link                       (동적, odom_tf_relay 가 발행)
             +- chassis                     (URDF)
                 +- velodyne_base_link      (URDF, 장착 오프셋)
                 |   +- velodyne
                 +- camera_link             (URDF, 장착 오프셋)
                 |   +- <optical * N>       (realsense2_camera 가 발행)
                 +- imu_link                (URDF, 장착 오프셋, RFU 축)
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
        # 상위 bringup 이 이미 이 로봇의 robot_state_publisher 를 돌리고 있으면
        # false 로 준다. 안 그러면 같은 TF 엣지를 둘이 발행한다. 끄더라도 여기서
        # URDF 를 대신 채워주지 않는다 — imu_link 를 포함한 base_link 서브트리
        # 전체를 다른 발행자가 책임져야 하고, odom_tf_relay 는 그 lookup 이 될
        # 때까지 기다렸다가 map -> base_link 와 /pose 를 발행한다.
        DeclareLaunchArgument('use_robot_description', default_value='true'),
        # SLAM 결과가 이미 든 bag 을 볼 때 false 로 준다. 매퍼와 릴레이가 내려간
        # 채로, 녹화된 자세/TF 가 손대지 않은 상태로 재생된다.
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
        # 바퀴는 revolute 조인트다. 0 joint state 를 발행해 준다.
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
            # 장착 오프셋은 더 이상 여기서 설정하지 않는다. 릴레이가
            # imu_link -> base_link 를 TF 에서 조회한다 — URDF 가 이미 발행하는
            # 값이다.
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
