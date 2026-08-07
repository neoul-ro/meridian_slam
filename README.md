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
| world_frame_id | string | "map" | Fixed; frame_id stamped into published poses |

Sensor configs: `fast_livo/config/velodyne16_vn100.yaml` (SLAM), `camera_d435.yaml` (camera model).

## Run

```bash
# SLAM only (drivers already running)
ros2 launch meridian_slam_bringup slam.launch.py

# everything at once (drivers + SLAM + Foxglove ws://<ip>:8765)
ros2 launch meridian_slam_bringup meridian_slam.launch.py
```

Calibration procedure: see `CALIBRATION.md`.
