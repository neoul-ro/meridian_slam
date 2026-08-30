# meridian_slam

FAST-LIVO2 기반 SLAM (라이다 + IMU + 카메라 tightly coupled). 로봇 자세를 `/pose`
로, 색이 입혀진 월드 프레임 지도를 발행한다.

## 입출력

| 토픽 | 타입 | 방향 |
|---|---|---|
| /velodyne_points | sensor_msgs/PointCloud2 (10Hz) | 구독 |
| /vectornav/imu | sensor_msgs/Imu (100Hz) | 구독 |
| /camera/camera/color/image_slam | sensor_msgs/Image (10Hz, 스로틀됨) | 구독 |
| /pose | geometry_msgs/PoseStamped (프레임 `map`) | 발행 |
| /pose_cov | geometry_msgs/PoseWithCovarianceStamped (같은 자세) | 발행 |
| /aft_mapped_to_init | nav_msgs/Odometry (프레임 `map`) | 발행 |
| /cloud_registered | sensor_msgs/PointCloud2 (월드 프레임 컬러 포인트) | 발행 |
| /path | nav_msgs/Path | 발행 |

TF `map → base_link` 도 브로드캐스트한다 (`base_link` 아래 센서 프레임들은 로봇
URDF 에서 나온다).

VIO 에 들어가는 이미지는 카메라 원본 30Hz 가 아니라 10Hz 다. `meridian_sensor`
가 `/camera/camera/color/image_raw` 를 `/camera/camera/color/image_slam` 으로
줄여서 내보내고, 이쪽은 그걸 받는다 (`fast_livo/config/velodyne16_vn100.yaml` 의
`img_topic`). **이건 VIO 프레임 예산이지 동기화가 아니다** — 스로틀은 라이다
스윕을 모르는 단순 레이트 리미터고 실측 간격이 100~200ms 로 흔들린다. 시간
정렬은 여기서 한다. 스로틀이 원본 스탬프를 손대지 않고 넘겨주므로, 거기에
`img_time_offset`(Kalibr 측정값)을 더해서 맞춘다.

입력 토픽을 만드는 드라이버는 `meridian_sensor` 패키지(별도 저장소)에 있다 —
두 저장소를 같은 워크스페이스에 둔다.

## 파라미터

| 이름 | 타입 | 기본값 | 설명 |
|---|---|---|---|
| use_slam | bool | true | FAST-LIVO2 + odom 릴레이 실행. false 면 SLAM 결과가 이미 든 bag 을 보기만 한다 |
| use_rviz | bool | false | SLAM 뷰로 RViz 를 띄운다 |
| use_sim_time | bool | false | bag 시계를 쓴다 (재생 시 true) |
| use_robot_description | bool | true | URDF TF 트리를 발행한다 (bunker_description) |
| world_frame_id | string | "map" | 고정. 발행 자세에 찍히는 frame_id |

센서 설정: `fast_livo/config/velodyne16_vn100.yaml` (SLAM),
`camera_d435.yaml` (카메라 모델).

## 실행

런치는 두 개다. `slam.launch.py` 가 진입점이고, 아래 세 모드는 **같은 파일에
플래그만 다르게** 준 것이다. 센서 드라이버는 `meridian_sensor` 에서 따로 띄우고,
전체 시스템 통합(드라이버 + SLAM + 인식)은 `meridian_bringup` 이 한다.

```bash
# 라이브 — 드라이버가 이미 떠 있을 때
ros2 launch meridian_slam_bringup slam.launch.py

# 재생 — `ros2 bag play <bag> --clock` 과 같이 돌린다
ros2 launch meridian_slam_bringup slam.launch.py use_sim_time:=true use_rviz:=true

# 보기만 — bag 에 SLAM 결과가 이미 들어 있을 때 그대로 표시
ros2 launch meridian_slam_bringup slam.launch.py \
    use_slam:=false use_robot_description:=false \
    use_sim_time:=true use_rviz:=true
```

`slam.launch.py` 는 Foxglove 브릿지를 띄우지 않는다. 필요하면 손으로 켠다.

```bash
ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765
```

`experiment.launch.py` 는 개인 실험용 편의 런치로, **어떤 통합 경로에도 속하지
않는다.** 리그를 혼자 돌려볼 때 쓰라고 meridian_sensor 드라이버 + 이 SLAM 스택 +
Foxglove 브릿지를 한 명령으로 띄운다.

```bash
ros2 launch meridian_slam_bringup experiment.launch.py
```

드라이버가 이미 떠 있으면 `use_lidar:=false` 처럼 개별로 끄거나
`slam.launch.py` 를 직접 쓴다 — 같은 장치를 두 프로세스가 잡으면 깨진다.
다른 bringup 이 이미 브릿지를 띄웠으면 `use_foxglove:=false` 를 준다.

캘리브레이션 절차는 `CALIBRATION.md` 참고.
