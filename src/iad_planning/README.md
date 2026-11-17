# iad_planning

Free-space planning for unstructured sites. The planner is Hybrid A* with Reeds-Shepp analytic expansion,
restricted to the gear directions allowed, plus RS shortcutting. It produces a velocity profile that
respects curvature, terrain and the steering-rate limit. While driving it watches the path ahead: a blocked
path first gets a stop line, and if the obstacle stays, a replan. Route mode follows a surveyed centreline
received as `nav_msgs/Path`.

A behaviour layer (`behaviour.hpp`, ROS-free, unit-tested) caps the speed profile from what perception
reports:
- **dust:** slows to 3 m/s (2 m/s in heavy dust) while the lane ahead is full of dust echoes, and keeps going;
- **follow:** keeps a 10 m + 2 s gap behind a slower vehicle in the lane, never closer than 8 m;
- **passing:** passes an oncoming vehicle in the other lane at 4 m/s;
- **yield:** stops before the crossing line of a vehicle about to cross the path.

It acts only on movers that look like vehicles: long side 2–14 m, 1–12 m/s, and consistent in position and
velocity over 4 scans. The tracker gives dust puffs and berm fragments random velocities too. Moving
objects are left out of the static stop/replan costmap, and a blockage further away than the stopping distance
must persist for 0.6 s before the truck stops.

| | Topic | Type |
|---|---|---|
| in | `costmap_file` (nav2 map YAML/PGM, raw mode) + optional `elevation_file` (PCD) | |
| in | `/perception/occupancy_grid` (live, near field) | `nav_msgs/OccupancyGrid` |
| in | `/localization/kinematic_state` | `nav_msgs/Odometry` |
| in | `/planning/goal` | `geometry_msgs/PoseStamped` |
| in | `/planning/route` | `nav_msgs/Path` |
| in | `/perception/objects` (tracked, with velocity) | `autoware_perception_msgs/DetectedObjects` |
| in | `/perception/points/dust` | `sensor_msgs/PointCloud2` |
| out | `/planning/trajectory` (`longitudinal_velocity_mps < 0` = reverse) | `autoware_planning_msgs/Trajectory` |
| out | `/planning/path`, `/planning/markers`, `/planning/search`, `/planning/metrics`, `/map/occupancy_grid` | |

```bash
ros2 launch iad_planning planning.launch.py map_dir:=data/maps/port vehicle:=container_agv   # goals from RViz
colcon test --packages-select iad_planning
```
