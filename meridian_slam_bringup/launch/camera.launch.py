"""RealSense D435 driver only -> /camera/camera/color/image_raw.

Also publishes the Meridian schema aliases (Architecture doc, Camera Driver):
  /camera/rgb    <- color/image_raw   (relay)
  /camera/info   <- color/camera_info (relay)
  /camera/depth  <- aligned depth, 16UC1 mm -> 32FC1 m (depth_image_proc)
The native RealSense topics stay untouched.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _system_cv_bridge_env():
    """LD_LIBRARY_PATH with bunker_ws stripped, for the RealSense node only.

    This machine has two OpenCV installs: 4.5 from apt (what ROS Humble's own
    binaries link) and 4.10 under /usr/local (what fast_livo links). bunker_ws
    ships a cv_bridge built against 4.10, and slam_ws2's install/setup.sh
    chain-sources bunker_ws, so it lands ahead of the system one on the library
    path.

    realsense2_camera_node itself is a 4.5 binary and loads cv_bridge lazily
    through the image_transport plugins. Picking up the 4.10 cv_bridge means
    cv::cvtColor from 4.10 calls cv::_OutputArray::create from 4.5 -- different
    struct layouts -- and the node segfaults the instant anything subscribes to
    a /compressed or /theora topic. Confirmed under gdb 2026-08-10; compressed
    and theora produce identical backtraces.

    Do NOT fix this by dropping bunker_ws globally: fastlivo_mapping is a 4.10
    binary and needs that same 4.10 cv_bridge to stay consistent. They are
    separate processes, so each can have the matching one.
    """
    entries = os.environ.get('LD_LIBRARY_PATH', '').split(':')
    kept = [p for p in entries if p and '/bunker_ws/' not in p]
    return ':'.join(kept)


def generate_launch_description():
    return LaunchDescription([
        # Depth + color-aligned depth on by default (SAM pipeline consumes it).
        DeclareLaunchArgument('enable_depth', default_value='true'),
        # FAST-LIVO2 wants the camera at the LiDAR rate. Every experiment in the
        # paper ran 10 Hz cameras against 10 Hz LiDAR, and on Hilti the authors
        # downsampled a 40 Hz camera to 10 Hz rather than let sync_packages slice
        # at 40 Hz. SR-LIVO (RA-L 2023) makes the rule explicit for spinning
        # LiDAR: at most 2x the sweep rate, because a time slice of a spinning
        # scan is an azimuth wedge, not a subsample. We were running 3:1.
        # 2026-08-26: 848x480 (16:9) -> 640x480 (4:3). meridian_seg 의 letterbox
        # 는 소스 해상도에서 자동으로 계산되는데, 4:3 이 아니면 /segment_image 가
        # 256x192 가 아니라 256x145 로 나오고 원본 좌표 환산 배율도 2.5 가 아니라
        # 3.3125 가 된다. meridian_seg / meridian_geobuilder 문서가 전부 2.5 를
        # 전제하고 있어서 해상도 쪽을 맞췄다.
        # ! camera_d435.yaml 의 intrinsic 은 848x480 에서 Kalibr 로 잰 값이다.
        #   640x480 은 같은 센서의 다른 판독 모드라 그 값을 그대로 쓸 수 없다.
        # Revert to '848x480x30' to undo.
        DeclareLaunchArgument('color_profile', default_value='640x480x30'),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera',
            parameters=[{
                'enable_color': True,
                'rgb_camera.color_profile': LaunchConfiguration('color_profile'),
                # Must stay False on this unit: the HW-timestamp query global time
                # needs (UVC XU control) fails on firmware 5.12.7.150 -- kernel logs
                # "GET_CUR ... -32" and the driver then drops every frame, so the
                # node comes up but publishes nothing. Stamps still land on the host
                # clock (arrival time), ~30-100ms after capture; Kalibr's
                # --time-calibration measures that offset. Retry True after a
                # firmware update.
                'global_time_enabled': False,
                'enable_depth': ParameterValue(
                    LaunchConfiguration('enable_depth'), value_type=bool),
                'align_depth.enable': ParameterValue(
                    LaunchConfiguration('enable_depth'), value_type=bool),
                'enable_infra1': False,
                'enable_infra2': False,
                'enable_sync': True,
            }],
            additional_env={'LD_LIBRARY_PATH': _system_cv_bridge_env()},
            output='screen',
        ),
        # -- SLAM 전용 10 Hz 스트림 --
        # FAST-LIVO2 의 sync_packages 는 이미지 타임스탬프마다 라이다 스윕을 자른다.
        # 회전식 라이다에서 시간 조각은 곧 방위각 쐐기라(preprocess.cpp:390, 3.61 deg/ms),
        # 카메라가 라이다보다 빠르면 LIO 갱신이 360 도가 아니라 쐐기 하나만 보게 된다.
        # 30 Hz 대 10 Hz(3:1)에서는 LIO 갱신의 절반이 빈 조각이었다 -- 3 분에
        # "[ LIO ]: No point!!!" 1,996 회. 카메라 자체를 15 Hz 로 내려보니 8 회로 떨어졌다.
        #
        # 다만 카메라는 30 Hz 로 유지해야 한다(CLIP/인스턴스 파이프라인 등 다른 소비자).
        # 그래서 발행은 30 Hz 그대로 두고, SLAM 이 구독하는 토픽만 여기서 10 Hz 로 솎는다.
        # velodyne16_vn100.yaml 의 common.img_topic 이 이 토픽을 가리킨다.
        #
        # 근거: FAST-LIVO2 논문의 모든 실험이 카메라를 라이다 속도로 맞췄고(Hilti 는
        # 40 Hz 카메라를 10 Hz 로 다운샘플), SR-LIVO(RA-L 2023, arXiv:2312.16800) Sec.V-A 는
        # 회전식 라이다에 대해 "at most twice the frequency of raw LiDAR sweeps" 라고 못박는다.
        Node(
            package='topic_tools',
            executable='throttle',
            name='camera_slam_throttle',
            arguments=['messages', '/camera/camera/color/image_raw', '10.0',
                       '/camera/camera/color/image_slam'],
            output='screen',
        ),
        # -- Meridian 스키마 별칭 (기존 토픽은 그대로, lazy라 구독 전엔 부하 0) --
        Node(
            package='topic_tools',
            executable='relay',
            name='camera_rgb_relay',
            parameters=[{
                'input_topic': '/camera/camera/color/image_raw',
                'output_topic': '/camera/rgb',
                'lazy': True,
            }],
            output='screen',
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='camera_info_relay',
            parameters=[{
                'input_topic': '/camera/camera/color/camera_info',
                'output_topic': '/camera/info',
                'lazy': True,
            }],
            output='screen',
        ),
        # 스키마는 32FC1(미터), RealSense는 16UC1(밀리미터) -> 공식 변환 노드.
        Node(
            package='depth_image_proc',
            executable='convert_metric_node',
            name='camera_depth_metric',
            condition=IfCondition(LaunchConfiguration('enable_depth')),
            remappings=[
                ('image_raw', '/camera/camera/aligned_depth_to_color/image_raw'),
                ('image', '/camera/depth'),
            ],
            output='screen',
        ),
    ])
