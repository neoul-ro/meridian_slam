# meridian_slam

SLAM placeholder node: publishes one identity `PoseEstimate` per `RGBDFrame` (real SLAM is TBD).

## I/O

| Topic | Type | Direction |
|---|---|---|
| /rgbd_frame | meridian_msgs/RGBDFrame | subscribe |
| /pose_estimate | meridian_msgs/PoseEstimate | publish |

## Parameters

None.

## Run

```
ros2 run meridian_slam slam_node
```
