# iad_perception

LiDAR perception for unstructured sites (quarries, ports). Standalone package: it depends only on ROS 2 /
Autoware interface packages, Eigen and tf2.

The pipeline runs in this order:
1. Range image built from any `sensor_msgs/PointCloud2` (organized or not).
2. Dust filter. A point is dust when it fails range consistency with its neighbours along the scan line
   and across rings, and is also weak or isolated, and does not persist over the last sweeps.
3. Concentric-zone ground segmentation (gravity-aligned, with a wheel-plane seed).
4. Overhang split.
5. Clustering, oriented boxes and tracking.
6. Log-odds occupancy grid in the map frame. Only cluster-validated hits are written.

`mode:=naive` is the baseline used for comparison: no dust filter and one RANSAC ground plane.

| | Topic | Type |
|---|---|---|
| in | `/sensing/lidar/top/points` | `sensor_msgs/PointCloud2` |
| in | TF `base_link → <lidar frame>` (static), `map → base_link` | `tf2_msgs/TFMessage` |
| out | `~/points/{ground,obstacle,dust,overhang,classified}` | `sensor_msgs/PointCloud2` (map frame) |
| out | `~/objects` | `autoware_perception_msgs/DetectedObjects` |
| out | `~/objects/markers` | `visualization_msgs/MarkerArray` |
| out | `~/occupancy_grid` | `nav_msgs/OccupancyGrid` |
| out | `~/metrics` | `std_msgs/String` (JSON: per-class counts, timing) |

```bash
ros2 launch iad_perception perception.launch.py                              # live
ros2 launch iad_perception perception.launch.py bag:=data/bags/perception_quarry   # replay, no simulator
colcon test --packages-select iad_perception                                  # gtest + launch test
```

Parameters: [config/perception.param.yaml](config/perception.param.yaml). RViz view: [rviz/perception.rviz](rviz/perception.rviz).
