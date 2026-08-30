# meridian_slam

FAST-LIVO2 based SLAM (LiDAR + IMU + camera tightly coupled): publishes the robot pose on `/pose` and a colorized world-frame map.

## I/O

| Topic | Type | Direction |
|---|---|---|
| /velodyne_points | sensor_msgs/PointCloud2 (10Hz) | subscribe |
| /vectornav/imu | sensor_msgs/Imu (100Hz) | subscribe |
| /camera/camera/color/image_raw | sensor_msgs/Image (30Hz) | subscribe |
| /pose | geometry_msgs/PoseStamped (frame `map`) | publish |
| /aft_mapped_to_init | nav_msgs/Odometry (frame `map`) | publish |
| /cloud_registered | sensor_msgs/PointCloud2 (world-frame color points) | publish |
| /path | nav_msgs/Path | publish |

Also broadcasts TF `map -> base_link` (sensor frames below `base_link` come from the robot URDF).
Drivers for the input topics live in the `meridian_sensor` package (separate repo) —
put both repos in the same workspace.

## Parameters

| Name | Type | Default | Description |
|---|---|---|---|
| use_rviz | bool | false | Open RViz with the SLAM view |
| use_sim_time | bool | false | Use bag clock (replay) |
| use_robot_description | bool | true | Publish URDF TF tree (bunker_description) |
| use_slam | bool | true | Run FAST-LIVO2 + odom relay; false = view a bag that already carries SLAM output |
| world_frame_id | string | "map" | Fixed; frame_id stamped into published poses |

Sensor configs: `fast_livo/config/velodyne16_vn100.yaml` (SLAM), `camera_d435.yaml` (camera model).

## Run

`slam.launch.py` is the entry point; the modes below are the same file with
different flags. Sensor drivers are launched separately from `meridian_sensor`,
and whole-system bringup (drivers + SLAM + perception) is `meridian_bringup`.

```bash
# live -- drivers already running
ros2 launch meridian_slam_bringup slam.launch.py

# replay -- run SLAM against `ros2 bag play <bag> --clock`
ros2 launch meridian_slam_bringup slam.launch.py use_sim_time:=true use_rviz:=true

# view -- bag already carries SLAM output, display it as-is
ros2 launch meridian_slam_bringup slam.launch.py \
    use_slam:=false use_robot_description:=false \
    use_sim_time:=true use_rviz:=true
```

The Foxglove bridge is not launched here; start it by hand when you want it:

```bash
ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765
```

`experiment.launch.py` is a personal convenience launch, not part of any
integration path: it starts the meridian_sensor drivers, this SLAM stack and
RViz in one command, for running the rig on its own.

```bash
ros2 launch meridian_slam_bringup experiment.launch.py
```

Calibration procedure: see `CALIBRATION.md`.
