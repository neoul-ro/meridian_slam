# 캘리브레이션 실행 매뉴얼

사용 라이브러리 (IMU):

- **allan_ros2** (노이즈 파라미터 분석) — https://github.com/CruxDevStuff/allan_ros2
  - 계산 코드 원조: allan_variance_ros — https://github.com/ori-drs/allan_variance_ros
  - 빌드 의존성: px4_msgs — https://github.com/PX4/px4_msgs
- **vnproglib** (VectorNav 공식, 자이로 영점) — vectornav 드라이버에 번들
  (`src/vectornav/vectornav/vnproglib-1.2.0.0/`) — https://github.com/dawonn/vectornav

## 1. 자이로 영점 (몇 초, 로봇 정지 + 런치 전부 끄고)

```bash
~/slam_ws2/tools/vn100_gyro_bias    # y 입력, done 나오면 끝
```

## 2. 녹화 (12~24시간, 로봇 안 건드리는 곳에서)

```bash
# 터미널 1
ros2 launch meridian_slam_bringup imu.launch.py
```

```bash
# 터미널 2 — 다음 날 Ctrl+C
ros2 bag record -o ~/rosbags/imu_allan_$(date +%Y%m%d) /vectornav/imu
```

## 3. 분석 (녹화 끝난 후, 몇 분)

```bash
# bag_path 줄을 녹화된 파일 경로로 수정
nano ~/calib_ws/src/allan_ros2/config/config.yaml
```

```bash
mkdir -p ~/calib_ws/output && cd ~/calib_ws/output
source ~/calib_ws/install/setup.bash
ros2 launch allan_ros2 allan_node.py    # "DONE ... deviation.csv" 뜨면 Ctrl+C
```

```bash
python3 ~/calib_ws/src/allan_ros2/scripts/analysis.py --data deviation.csv
```

```bash
# 마지막 두 줄 수정: rostopic: '/vectornav/imu' / update_rate: 100.0
nano imu.yaml
cp imu.yaml ~/slam_ws2/calib/imu.yaml
```

## 부록: 자이로 영점 도구 다시 만들기 (없어졌을 때만)

```bash
cd ~/slam_ws2
g++ -O2 tools/vn100_gyro_bias.cpp \
  -I src/vectornav/vectornav/vnproglib-1.2.0.0/cpp/include \
  -L install/vectornav/lib -lvncxx \
  -Wl,-rpath,$HOME/slam_ws2/install/vectornav/lib \
  -o tools/vn100_gyro_bias
```
