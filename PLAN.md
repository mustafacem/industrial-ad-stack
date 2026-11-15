# Plan v3: Gazebo Harmonic simulation + standalone ROS 2 subsystems

Target role: Senior Autonomous Driving Software Engineer, industrial and off-road mobility (ports,
factories, construction sites, logistics hubs). The JD asks for deep work in at least one of
Perception / Localization / Calibration / Planning / Control, plus C++ and ROS 2 on Linux.

v2 (done) built all five subsystems in C++17 / ROS 2 Humble on a custom ray-casting simulator and
recorded six RViz videos. v3 makes two changes:

1. **Simulation moves to Gazebo Harmonic + ros_gz.** The quarry and the port become SDF worlds with
   physics. The LiDARs, IMU and GNSS are real Gazebo sensors (`gpu_lidar` rendered on the GPU,
   `imu`, `navsat`), bridged to ROS 2 by `ros_gz_bridge`. The trucks are articulated wheeled models
   driven through a drive-by-wire plugin.
2. **Every subsystem is an independent ROS 2 package.** Perception, localization, calibration,
   planning and control share no code and no custom messages. Each talks only through standard
   ROS 2 / Autoware interface messages. Each has its own parameters, launch files, RViz view and
   tests, and runs alone from a rosbag2 recording, without the simulator.

## 1. Deliverables

| # | Video (media/) | Site | What it shows | Headline metric |
|---|---|---|---|---|
| 1 | `01_perception.mp4` | Quarry | Gazebo GPU LiDAR in particle dust on rough terrain: dust filter + ground segmentation + clustering vs. naive RANSAC | dust removed, ground recall, false obstacles, ms/frame |
| 2 | `02_localization.mp4` | Port | GNSS multipath under cranes and stacks; EKF (IMU + wheel odom + gated GNSS) + NDT vs. GNSS-only and dead reckoning | position error (cm) |
| 3 | `03_planning.mp4` | Port + Quarry | Hybrid A* + Reeds-Shepp, reverse docking, replanning around an unmapped container | plan time, length, cusps |
| 4 | `04_control.mp4` | Quarry | Physics truck with actuator dead time and slow steering; delay-compensated MPC vs. pure pursuit | lateral RMS / max, dock error |
| 5 | `05_calibration.mp4` | Port | Four corner LiDARs with wrong extrinsics; ground plane + mapping-based registration | rotation / translation error before → after |
| 6 | `06_full_mission.mp4` | Quarry | Whole stack closed loop in Gazebo: haul → reverse into the dump bay with dust and traffic | mission time, docking error |
| 7 | `showcase_reel.mp4` | both | Short cut of 1–6 | – |

Each video combines a Gazebo chase camera (rendered on the GPU, frames timed by sim time) with the
subsystem's RViz view. Also delivered: rosbag2 recordings per subsystem (`data/bags/`), a per-package
"isolated build" check, unit and launch tests, and an updated README.

## 2. Packages and the dependency rule

```
src/
  iad_gz_sim        Gazebo side: world compiler (quarry, port → SDF + meshes + map files), vehicle
                    models, gz-sim system plugins (drive-by-wire, scripted traffic), ros_gz_bridge
                    config, GNSS sky-visibility effects, sim-side route driver (stand-in driver)
  iad_perception    dust filter → ground segmentation → clustering/tracking → occupancy grid
  iad_localization  NDT (multi-resolution, point-to-plane) + EKF; GNSS + NDT yaw-search initialisation
  iad_calibration   ground-plane (roll, pitch, z) + mapping-based multi-LiDAR registration (x, y, yaw)
  iad_planning      Hybrid A* + Reeds-Shepp, velocity profile, replanning, map loader (PGM/YAML)
  iad_control       LTV MPC (ADMM QP) with delay compensation + PI/feed-forward longitudinal; pure pursuit
  iad_eval          ground-truth evaluators (needs the sim world; never a dependency of a subsystem)
  iad_tools         HUD overlay, metrics logger, mission sequencer (python)
  iad_bringup       integration launch files that compose the packages above
```

Rule: the five subsystem packages depend only on ROS 2 / Autoware interface packages
(`sensor_msgs`, `nav_msgs`, `geometry_msgs`, `visualization_msgs`, `ackermann_msgs`,
`autoware_planning_msgs`, `autoware_perception_msgs`, `autoware_vehicle_msgs`), on `rclcpp`,
`tf2_ros`, and on Eigen. Small helpers such as angle wrapping are copied into each package.
`iad_common`, `iad_msgs` and the v2 simulator are removed.

## 3. Interface contract

| Topic | Type | Producer → consumer |
|---|---|---|
| `/sensing/lidar/top/points` | `sensor_msgs/PointCloud2` | Gazebo `gpu_lidar` → perception, localization, calibration |
| `/sensing/lidar/{front_left,…}/points` | `sensor_msgs/PointCloud2` | Gazebo corner LiDARs → calibration |
| `/sensing/imu/data` | `sensor_msgs/Imu` | Gazebo `imu` → localization |
| `/sensing/gnss/fix` | `sensor_msgs/NavSatFix` | Gazebo `navsat` + sky-visibility effects → localization |
| `/vehicle/status/velocity` | `geometry_msgs/TwistWithCovarianceStamped` | drive-by-wire plugin (wheel odometry) → localization, control |
| `/vehicle/status/steering` | `autoware_vehicle_msgs/SteeringReport` | drive-by-wire plugin → control |
| `/tf_static` | `tf2_msgs/TFMessage` | vehicle description (nominal extrinsics) → all |
| `/localization/kinematic_state` | `nav_msgs/Odometry` (+ TF map→base_link) | localization → perception, planning, control |
| `/perception/occupancy_grid` | `nav_msgs/OccupancyGrid` | perception → planning |
| `/perception/objects` | `autoware_perception_msgs/DetectedObjects` | perception → (HUD, eval) |
| `/planning/goal` | `geometry_msgs/PoseStamped` | mission / RViz → planning |
| `/planning/trajectory` | `autoware_planning_msgs/Trajectory` (v < 0 = reverse) | planning → control |
| `/control/command` | `ackermann_msgs/AckermannDriveStamped` | control → drive-by-wire plugin |
| `/sim/ground_truth/odom` | `nav_msgs/Odometry` | Gazebo → eval only |

Map files (written by the world compiler, standing in for a site survey): `pointcloud_map.pcd`
(localization), `costmap.pgm` + `costmap.yaml` (planning), `site.yaml` (named poses, routes).

## 4. Simulation in Gazebo (`iad_gz_sim`)

- **Worlds** are compiled from the seeded v2 scenario code: the terrain becomes a textured OBJ mesh
  (0.5 m) with a collision mesh; containers, cranes, berms and boulders become SDF primitives; the
  same geometry produces the PCD map and the costmap, so maps and simulation agree.
- **Vehicles**: haul truck and container AGV as wheeled models (chassis, steered front wheels,
  driven wheels) simulated by DART. A drive-by-wire system plugin turns `/control/command` into
  wheel torques and steering-joint targets, with dead time, steering lag and rate limit, power and
  traction limits, brakes, hill hold and rolling resistance by surface. Slip comes from physics.
- **Sensors**: `gpu_lidar` 32 × 1200 at 10 Hz (top) + four corner LiDARs, `imu`, `navsat`.
  The particle emitters for haul-road and dump dust also scatter `gpu_lidar` rays.
  GNSS gets multipath bias and dropouts from sky visibility near cranes and stacks.
- **Traffic**: light vehicle, haul truck, straddle carriers and terminal trucks follow scripted
  paths (system plugin).
- **Ground truth** (`/sim/ground_truth/odom`, model poses) feeds only `iad_eval` and the stand-in
  route driver.

## 5. Independence: how it is proven

1. **Isolated build**: `scripts/isolation_check.sh` copies each subsystem package alone into an
   empty workspace, then builds and tests it. It fails if any `iad_*` dependency remains.
2. **Bag replay**: every subsystem has `launch/<pkg>.launch.py` (live) and a `replay` mode that runs
   `ros2 bag play --clock` on its own bag, with its node and RViz view, and no Gazebo.
3. **Tests**: gtest for the algorithm cores, plus a launch test per node that feeds synthetic
   standard messages and checks the outputs.

## 6. Video pipeline

GPU chase camera in Gazebo → recorder plugin → ffmpeg (frames stamped by sim time, so the video is
smooth whatever the real-time factor). RViz2 on Xvfb → ffmpeg x11grab. Then host-side compositing
(`tools/video.py`) with title and result cards.

## 7. Milestones

- [x] G0 Image: Gazebo Harmonic + ros_gz + Autoware msgs; headless GPU (EGL) rendering checked
- [x] G1 `iad_gz_sim`: world compiler, quarry + port SDF, vehicle models, drive-by-wire, bridge, traffic, dust
- [x] G2 Standalone packages: remove `iad_common` / `iad_msgs`; standard interfaces; per-package params, launch, RViz, tests
- [x] G3 `iad_eval` (ground-truth evaluators) + HUD on the new topics
- [x] G4 Bags per subsystem + replay launches + isolation check passing
- [x] G5 Demos re-run on Gazebo, metrics re-measured, videos re-recorded
- [x] G6 README / PLAN updated

## 8. Risks

| Risk | Handling |
|---|---|
| DART stability with a 30–70 t truck on a big terrain mesh | Sphere wheel collisions, 2 ms step, tested first in a small world |
| Real-time factor below 1 with GPU LiDAR + RViz software rendering | Chase-camera frames are timed by sim time; RViz runs at a fixed lower RTF and is re-timed |
| No per-point labels from `gpu_lidar` | `iad_eval` labels points from the known world geometry and the traffic poses |
| No dual returns in Gazebo | Dust filter switches to range-image isolation + temporal consistency + intensity |
| Disk | 50 GB free; bags in MCAP with zstd; raw captures deleted after encoding |
