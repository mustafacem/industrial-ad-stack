# iad_planning

Free-space planning for unstructured sites. The planner is Hybrid A* with Reeds-Shepp analytic expansion,
restricted to the gear directions allowed, plus RS shortcutting. It produces a velocity profile that
respects curvature, terrain and the steering-rate limit. While driving it watches the path ahead: a blocked
path first gets a stop line, and if the obstacle stays, a replan. Route mode follows a surveyed centreline
received as `nav_msgs/Path`.

| | Topic | Type |
|---|---|---|
| in | `costmap_file` (nav2 map YAML/PGM, raw mode) + optional `elevation_file` (PCD) | |
| in | `/perception/occupancy_grid` (live, near field) | `nav_msgs/OccupancyGrid` |
| in | `/localization/kinematic_state` | `nav_msgs/Odometry` |
| in | `/planning/goal` | `geometry_msgs/PoseStamped` |
| in | `/planning/route` | `nav_msgs/Path` |
| out | `/planning/trajectory` (`longitudinal_velocity_mps < 0` = reverse) | `autoware_planning_msgs/Trajectory` |
| out | `/planning/path`, `/planning/markers`, `/planning/search`, `/planning/metrics`, `/map/occupancy_grid` | |

```bash
ros2 launch iad_planning planning.launch.py map_dir:=data/maps/port vehicle:=container_agv   # goals from RViz
colcon test --packages-select iad_planning
```
