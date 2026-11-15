# iad_localization

Localization for GNSS-degraded industrial sites. It is a 5-state EKF over x, y, yaw, gyro bias and
odometry scale:
- Prediction uses wheel speed and gyro.
- GNSS fixes go through a χ² gate.
- Delayed measurements are replayed.

The EKF is fused with multi-resolution NDT, which uses point-to-plane terms on planar voxels and a
degeneracy-aware covariance, against a PCD map.

The node initialises itself in this order of preference:
1. the `initial_pose` parameter;
2. `/initialpose` (RViz "2D Pose Estimate");
3. automatically. A GNSS fix gives the position, a correlative search over all headings against the
   map's structure finds the pose, and NDT polishes it.

For comparison, it also runs three reduced filters on the same inputs: GNSS-gated, GNSS-raw and dead
reckoning.

| | Topic | Type |
|---|---|---|
| in | `/sensing/lidar/top/points` | `sensor_msgs/PointCloud2` |
| in | `/sensing/imu/data` | `sensor_msgs/Imu` |
| in | `/sensing/gnss/fix` | `sensor_msgs/NavSatFix` |
| in | `/vehicle/status/velocity` | `geometry_msgs/TwistWithCovarianceStamped` |
| in | `map_file` (PCD) + map origin (lat/lon), TF `base_link → sensors` | |
| out | `/localization/kinematic_state` + TF `map → base_link` | `nav_msgs/Odometry` |
| out | `/localization/pose_with_covariance` | `geometry_msgs/PoseWithCovarianceStamped` |
| out | `/map/pointcloud_map` (latched) | `sensor_msgs/PointCloud2` |
| out | `~/metrics`, `~/debug/paths/*`, `~/debug/markers`, `~/ndt/aligned_points` | |

```bash
ros2 launch iad_localization localization.launch.py map_dir:=data/maps/port
ros2 launch iad_localization localization.launch.py map_dir:=data/maps/port bag:=data/bags/localization_port
colcon test --packages-select iad_localization
```
