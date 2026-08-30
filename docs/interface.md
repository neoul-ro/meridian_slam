# meridian_slam 인터페이스

구독·발행 토픽, TF 트리, 파라미터, 그리고 왜 그렇게 만들었는지. 실행 방법은
저장소 루트의 `README.md` 를 본다.

## 토픽

| 토픽 | 타입 | 방향 |
|---|---|---|
| `/velodyne_points` | `sensor_msgs/PointCloud2` (10Hz) | 구독 |
| `/vectornav/imu` | `sensor_msgs/Imu` (100Hz) | 구독 |
| `/camera/camera/color/image_slam` | `sensor_msgs/Image` (10Hz) | 구독 |
| `/pose` | `geometry_msgs/PoseStamped` (프레임 `map`) | 발행 |
| `/pose_cov` | `geometry_msgs/PoseWithCovarianceStamped` | 발행 |
| `/aft_mapped_to_init` | `nav_msgs/Odometry` (프레임 `map`) | 발행 |
| `/cloud_registered` | `sensor_msgs/PointCloud2` (월드 프레임 컬러 포인트) | 발행 |
| `/path` | `nav_msgs/Path` | 발행 |

입력 토픽을 만드는 드라이버는 `meridian_sensor`(별도 저장소)에 있다. 두 저장소를
같은 워크스페이스에 둔다.

### /pose 는 계약이다

하류 전체, 특히 geobuilder 가 쓴다. **프레임과 의미를 바꾸지 않는다** — `map`
프레임의 **base_link** 자세다. 타입은 `PoseStamped` 로 유지하기로 정해졌다.

공분산이 필요하면 `/pose_cov` 를 쓴다. 오도메트리 공분산은 IMU 자세를 기술하고
`/pose_cov` 는 base_link 를 기술하므로 그대로 옮기지 않는다. 월드 프레임 회전
오차가 리그 전체를 IMU 기준으로 돌리고, 장착 오프셋이 그걸 base_link 의 위치
오차로 바꾼다. 6x6 야코비안 `J = [[I, -[off]x], [0, I]]` 로 `C_base = J C_imu J'`
를 계산한다. 오프셋이 38cm 라 yaw 불확실성 1도에서 이 항이 6.6mm 다.

## TF 트리

```
map -+- aft_mapped                      (동적, 원시 SLAM 자세, 디버그용)
     +- base_link                       (동적, odom_tf_relay 가 발행)
         +- chassis                     (URDF)
             +- velodyne_base_link      (URDF, 장착 오프셋)
             |   +- velodyne
             +- camera_link             (URDF, 장착 오프셋)
             |   +- <optical * N>       (realsense2_camera 가 발행)
             +- imu_link                (URDF, 장착 오프셋, RFU 축)
             +- wheel * 16              (URDF)
```

FAST-LIVO2 를 패치해서 월드 프레임으로 `map` 을 직접 쓰게 했다. 예전의
`camera_init` 프레임은 이제 없다.

`base_link` 아래는 전부 URDF 에서 나온다. `use_robot_description:=false` 는 그
역할을 다른 발행자에게 넘긴다 — 이때 여기서 대신 채워주는 것은 없다.
`imu_link` 를 포함한 서브트리 전체를 그쪽이 책임져야 하고, `odom_tf_relay` 는 그
lookup 이 될 때까지 기다렸다가 `map → base_link` 와 `/pose` 를 발행한다.

## odom_tf_relay

FAST-LIVO2 는 `/aft_mapped_to_init` (map → aft_mapped, IMU 바디 프레임)을
발행한다. 플랫폼이 원하는 것은 `map → base_link` 다. 이 노드가 고정 장착
오프셋을 적용해서 변환한다.

장착 오프셋은 설정값이 아니라 TF 에서 읽는다. `imu_link → base_link` 는 URDF 의
정적 체인만 타고 `map` 을 거치지 않으므로, `map → base_link` 를 발행하는 게 이
노드 자신이어도 순환이 아니다.

TF lookup 이 성공하기 전까지는 **아무것도 발행하지 않는다.** 가정한 오프셋으로
대체하면 base_link 가 제자리에서 38cm 벗어난 채 아무 말도 안 하게 되는데,
조용히 틀린 자세는 눈에 띄게 없는 자세보다 비싸다.

## VIO 이미지가 10Hz 인 이유

카메라 원본은 30Hz 인데 `meridian_sensor` 가 `/camera/camera/color/image_slam`
으로 10Hz 만 내보내고, 이쪽이 그걸 받는다
(`fast_livo/config/velodyne16_vn100.yaml` 의 `img_topic`).

**프레임 예산이지 동기화가 아니다.** 스로틀은 라이다 스윕을 모르는 단순 레이트
리미터고 실측 간격이 100~200ms 로 흔들린다. 시간 정렬은 여기서 한다 — 스로틀이
원본 스탬프를 손대지 않고 넘겨주므로, 거기에 `img_time_offset`(Kalibr 측정값,
`0.008346`)을 더해서 맞춘다.

## 파라미터

| 이름 | 타입 | 기본값 | 설명 |
|---|---|---|---|
| `use_slam` | bool | true | FAST-LIVO2 + odom 릴레이 실행 |
| `use_rviz` | bool | false | SLAM 뷰로 RViz |
| `use_sim_time` | bool | false | bag 시계 (재생 시 true) |
| `use_robot_description` | bool | true | URDF TF 트리 발행 |
| `world_frame_id` | string | "map" | 고정. 발행 자세의 frame_id |

`experiment.launch.py` 는 위에 더해 `use_lidar` / `use_camera` / `use_imu`
(각 true), `use_foxglove`(true), `foxglove_port`(8765)를 갖는다. 드라이버 쪽
인자(`velodyne_ip` 등)는 재선언하지 않고 `meridian_sensor` 에서 상속받는다.

센서 설정 파일: `fast_livo/config/velodyne16_vn100.yaml`(SLAM),
`camera_d435.yaml`(카메라 모델).

## 런치 구성

런치는 두 개다.

| 파일 | 역할 |
|---|---|
| `slam.launch.py` | 진입점. `meridian_bringup` 이 SLAM 단계로 include 한다 |
| `experiment.launch.py` | 개인 실험용. 드라이버 + SLAM + Foxglove 한 명령 |

모드마다 파일을 새로 만들지 않는다. `slam.launch.py` 하나에 플래그로 라이브 /
재생 / 보기만 세 모드를 가른다. 예전에는 `meridian_slam` / `replay` / `view` /
`foxglove` 로 파일이 다섯 개였는데, 조합만 하는 껍데기가 늘면서 기본값이 여러
군데로 갈라지고 옛 구조가 남았다.

`slam.launch.py` 는 Foxglove 를 안 띄운다. 시각화는 파이프라인 수명과 별개이고
두 번째 `foxglove_bridge` 는 포트를 못 잡는다. `experiment.launch.py` 는 개인용
이라 기본으로 띄운다.

## 캘리브레이션

`docs/calibration.md` 참고.
