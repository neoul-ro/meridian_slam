# meridian_slam

SLAM placeholder node: publishes one identity pose per camera frame (real SLAM is TBD).

## I/O

| Topic | Type | Direction |
|---|---|---|
| /camera/rgb | sensor_msgs/Image | subscribe |
| /pose | geometry_msgs/PoseWithCovarianceStamped | publish |

## Parameters

| Name | Type | Default | Description |
|---|---|---|---|
| world_frame_id | string | "map" | frame_id stamped into published poses |

## Run

```
ros2 run meridian_slam slam_node
```
