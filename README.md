# meridian_slam

FAST-LIVO2 기반 SLAM. 라이다 + IMU + 카메라로 로봇 자세(`/pose`)와 색이 입혀진
지도를 만든다.

## 설치

```bash
sudo apt install \
    ros-humble-sophus \
    ros-humble-pcl-ros ros-humble-pcl-conversions \
    ros-humble-cv-bridge ros-humble-image-transport \
    ros-humble-joint-state-publisher ros-humble-robot-state-publisher \
    ros-humble-xacro ros-humble-urdf-launch \
    ros-humble-foxglove-bridge
```

센서 드라이버는 `meridian_sensor` 저장소에 있다. 같은 워크스페이스에 두고 그쪽
README 대로 설치한다.

## 빌드

```bash
cd ~/meridian_test_juyoung
colcon build --packages-select vikit_common vikit_ros fast_livo \
                               bunker_description meridian_slam_bringup
source install/setup.bash
```

## 실행

드라이버가 이미 떠 있을 때:

```bash
ros2 launch meridian_slam_bringup slam.launch.py
```

혼자 다 띄우고 싶을 때 (드라이버 + SLAM + RViz + Foxglove):

```bash
ros2 launch meridian_slam_bringup experiment.launch.py
```

잘 도는지 확인:

```bash
ros2 topic hz /pose              # 약 9 Hz
ros2 run tf2_ros tf2_echo map base_link
```

### bag 재생

녹화한 bag 으로 SLAM 을 다시 돌린다. 라이브 시스템은 먼저 끈다.

```bash
# 터미널 1
ros2 launch meridian_slam_bringup slam.launch.py use_sim_time:=true use_rviz:=true
# 터미널 2
ros2 bag play <bag> --clock
```

bag 에 SLAM 결과가 이미 들어 있으면 보기만 한다.

```bash
ros2 launch meridian_slam_bringup slam.launch.py \
    use_slam:=false use_robot_description:=false \
    use_sim_time:=true use_rviz:=true
```

### Foxglove

`slam.launch.py` 는 브릿지를 안 띄운다. 필요하면 켠다.

```bash
ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765
```

Foxglove Studio 에서 `ws://<젯슨IP>:8765` 로 접속한다.
`experiment.launch.py` 는 이걸 같이 띄운다.

---

- 토픽·TF·파라미터와 설계 근거: [docs/interface.md](docs/interface.md)
- 캘리브레이션 절차: [docs/calibration.md](docs/calibration.md)
