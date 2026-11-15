# iad_calibration

Target-less multi-LiDAR extrinsic calibration while driving. It works in three steps:
1. **Parked.** A ground-plane fit per LiDAR gives roll, pitch and height.
2. **Driving.** Each corner LiDAR's keyframes are registered jointly, point-to-plane, against a voxel-plane
   map accumulated by the reference (roof) LiDAR. This gives yaw, x and y.
3. **Refinement.** A final damped 6-DOF registration jointly adjusts rotation and translation, correcting
   residual roll, pitch and height errors from the ground fit (`max_iterations_6dof`, default 15).

The nominal extrinsics come from `/tf_static`. The estimates are published as TF `calib/<frame>`, as JSON
metrics, and as a YAML file.

| | Topic | Type |
|---|---|---|
| in | `/sensing/lidar/<name>/points` for each corner LiDAR and the reference | `sensor_msgs/PointCloud2` |
| in | `odom_topic` (vehicle pose in the map) | `nav_msgs/Odometry` |
| in | `/tf_static` (nominal extrinsics) | `tf2_msgs/TFMessage` |
| out | `/calibration/metrics` (estimates + residuals), `/calibration/clouds/*`, `/calibration/reference`, TF `calib/*` | |

```bash
ros2 launch iad_calibration calibration.launch.py bag:=data/bags/calibration_port output:=extrinsics.yaml
colcon test --packages-select iad_calibration
```
