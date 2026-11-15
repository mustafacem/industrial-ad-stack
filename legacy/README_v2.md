# Industrial & off-road autonomous driving stack

A ROS 2 Humble / C++17 autonomy stack for **ports and quarries**: perception, localization, calibration,
planning and control, running closed loop on a 3D sensor-level simulator. The problems it targets are the
ones that separate industrial sites from public roads: **dust, rough terrain, GNSS multipath, no lanes,
and heavy vehicles with slow, delayed actuators**. Every number below is measured against the
simulator's ground truth, and every video is a screen recording of the running system in RViz2.

**▶ [Showcase reel, 1:44](media/showcase_reel.mp4)**  ·  six full demos below  ·  18 unit tests passing (`colcon test`: 24/24)

![Results at a glance](media/results_at_a_glance.png)

## Demos

| # | Demo | Preview | Headline |
|---|---|---|---|
| 01 | [Perception: 3D LiDAR on rough terrain, inside dust](media/01_perception.mp4) | ![](media/gif/01_perception.gif) | 98 % of dust echoes removed, 29× fewer false obstacles than a RANSAC baseline, 4.8 ms/frame |
| 02 | [Localization: GNSS-denied container terminal](media/02_localization.mp4) | ![](media/gif/02_localization.gif) | EKF + NDT 8.4 cm RMS where GNSS-based filters reach 2.4–2.6 m |
| 03 | [Planning: free space, no lanes](media/03_planning.mp4) | ![](media/gif/03_planning.gif) | 160–230 m Hybrid A* + Reeds-Shepp paths in 0.1–0.6 s; stop and replan around an unmapped container |
| 04 | [Control: 0.5 s dead time, 72 t truck](media/04_control.mp4) | ![](media/gif/04_control.gif) | MPC 0.13 m lateral RMS vs 0.90 m for pure pursuit; docks within 0.41 m / 1.3° |
| 05 | [Calibration: 4 corner LiDARs, no targets](media/05_calibration.mp4) | ![](media/gif/05_calibration.gif) | up to 3.6° / 31 cm off → 0.04–0.08° / 0.8–2.6 cm |
| 06 | [Full stack: one quarry haul cycle](media/06_full_mission.mp4) | ![](media/gif/06_full_mission.gif) | load, 330 m haul, dump in 157 s, all subsystems closed loop |

## 1. Perception (`iad_perception`)

The same 32-beam dual-return LiDAR stream goes through a baseline (one RANSAC ground plane, no dust filter)
and through this stack, side by side, on a quarry haul road with a 9 % ramp, berms, mud, crossing traffic
and wind-blown dust.

- **Dust filter**: a candidate is the first of two echoes, or a weak echo. It is dropped unless it is
  continuous with its neighbours along the scan line in the range image. Intensity alone is not enough,
  because wet mud is as dark as dust.
- **Ground segmentation**: a concentric-zone model (Patchwork-style) with PCA planes per polar bin, in a
  gravity-aligned frame. The search is seeded with the plane under the wheels, and a bin is accepted only
  if its plane meets the neighbouring plane at their shared border. Ramps stay ground. Cabin roofs and
  berm crests do not.
- **Objects and occupancy**: grid clustering, minimum-area boxes, an α-β tracker, and a log-odds grid with
  free-space ray tracing. In the full mission the grid is cluster-validated: isolated residual dust never
  marks a cell.

| 610 frames, 61 s haul | baseline | this stack |
|---|---|---|
| false obstacle points per frame | 5340 | **182** |
| phantom occupied grid cells | 1928 | **45** |
| dust echoes removed | 0 % | **98.1 %** |
| drivable ground kept as ground | 58.4 % | **91.7 %** |
| latency per ~40k-point frame (1 core) | 5.8 ms | 4.8 ms |

## 2. Localization (`iad_localization`)

An AGV drives a 0.9 km loop through a container terminal: open sky, a canyon of 13 m stacks, the truck lane
under an RTG, and the quay lane under three ship-to-shore cranes. Raw GNSS fixes are off by up to 9.8 m
while the receiver claims 1.2 m. Four estimators run on the same data.

- **EKF** state: x, y, yaw, **gyro bias** and **odometry scale**. Online it learned the injected
  0.0035 rad/s bias exactly, and a wheel scale of 0.985 against a true 0.988. In mud the scale state
  absorbs wheel slip (0.77–1.05 during the recorded haul cycle).
- **Gating**: GNSS and NDT updates pass a χ² Mahalanobis gate. A lost-filter guard re-opens the covariance
  after a run of rejections, but only when no other source (NDT) is holding the filter.
- **Delays**: late measurements are applied at their own timestamp by replaying the input history.
- **NDT**: a coarse-to-fine voxel-Gaussian pyramid, point-to-plane on planar voxels, rotation linearised
  about the vehicle, and a **degeneracy-aware covariance** (a corridor reports that it cannot see sliding
  along it). About 10 ms per scan.

| 127 s loop | RMS | max |
|---|---|---|
| **EKF + NDT (this stack)** | **8.4 cm** | 25 cm |
| EKF + gated GNSS | 2.63 m | 8.0 m |
| EKF + raw GNSS | 2.40 m | 8.0 m |
| dead reckoning (IMU + odometry) | 41.8 m | 67.0 m |

Gating alone barely beats raw GNSS here, because multipath bias drifts slowly and every fix looks
plausible on its own. It takes the map to see through it.

## 3. Planning (`iad_planning`)

Hybrid A* on the site costmap: a surveyed terrain and structure map with slope, roughness and material
costs, fused with near-field live occupancy from perception. It is the same family as Autoware's
`freespace_planner`, and builds on v1 of this project (`offroad_av`).

- **Reeds-Shepp analytic expansion** and a heuristic of max(obstacle-aware Dijkstra, Reeds-Shepp
  distance). Gear changes are allowed only where the goal needs a turnaround (docking).
- **Reeds-Shepp shortcutting** removes lattice meander, so a 100 m aisle drive is a straight line.
- **Time parameterisation**: gear, curvature, terrain and **steering-rate** limits (slow down where
  curvature changes faster than hydraulic steering can follow), plus a stop at every cusp.
- **Blocked path ahead**: put down a stop line, hold 2.5 s (crossing traffic usually clears), then replan.

| query | path | cusps | nodes | time |
|---|---|---|---|---|
| start → RTG truck lane | 228 m | 0 | 3984 | 0.57 s |
| replan around an unmapped container | 161 m | 1 | 822 | 0.20 s |
| RTG lane → under quay crane 2 | 216 m | 0 | 618 | 0.14 s |
| quarry: haul road → reverse into dump bay | 89 m | 1 | 48 | 0.03 s |

## 4. Control (`iad_control`)

A loaded 72 t haul truck backs into the dump bay. The actuator has 0.5 s dead time, a 0.35 rad/s steering
rate limit, steering lag and rolling resistance on loose ground. Both runs follow the same plan.

- **Lateral MPC**: a linear time-varying error model with steering lag and a 3 s horizon. The QP has
  steering and steering-rate limits, solved by a small dense **ADMM** solver (the OSQP iteration) in about
  0.5 ms.
- **Dead-time compensation**: the commands still in the actuator pipe are replayed, so the MPC solves from
  where the truck *will be*.
- **Longitudinal**: PI with planned-acceleration, rolling-resistance and grade feed-forward, plus
  breakaway at standstill.

| dump manoeuvre, 72 t, 0.5 s dead time | lateral RMS | max | dock error | heading |
|---|---|---|---|---|
| pure pursuit (baseline) | 0.90 m | 1.88 m | 2.01 m | 3.8° |
| pure pursuit + delay compensation | 0.51 m | 1.21 m | 1.23 m | 3.0° |
| **MPC + delay compensation** | **0.13 m** | **0.55 m** | **0.41 m** | **1.3°** |

![Lateral error over time](media/04_control_lateral_error.png)

## 5. Calibration (`iad_calibration`)

Large vehicles need corner LiDARs for the blind spots the roof LiDAR cannot see; demo 01 shows one. Here
the TF tree claims corner-LiDAR extrinsics that are off by up to 3.6° and 31 cm.

1. **Ground plane, parked**: RANSAC + PCA in each sensor frame gives roll, pitch and mounting height.
2. **Mapping-based registration, driving**: a reference map is built from the roof LiDAR. All keyframes of
   each corner LiDAR are registered to it jointly (point-to-plane Gauss-Newton on yaw, x, y). The result is
   written as YAML ([`media/05_calibration_extrinsics.yaml`](media/05_calibration_extrinsics.yaml)).

| sensor | rotation before → after | translation before → after |
|---|---|---|
| front left | 3.41° → 0.05° | 27.3 → 0.8 cm |
| front right | 3.61° → 0.04° | 31.5 → 0.8 cm |
| rear left | 2.76° → 0.07° | 24.2 → 1.0 cm |
| rear right | 3.47° → 0.08° | 24.4 → 2.6 cm |

Observability matters. A straight drive past parallel walls left the longitudinal offset unconstrained
(4–8 cm error). Adding a 90° turn past perpendicular stack faces brought it to 1–3 cm.

![Before and after](media/05_calibration_before_after.png)

## 6. Full stack

Perception, localization (EKF + NDT), planning and MPC drive one loaded truck through a full haul cycle.
It backs in beside the excavator, hauls 330 m up the ramp through dust, mud and oncoming traffic, and
reverses into the dump bay. Recorded run: **157 s**, dust removed 97.8 %, localization **5 cm RMS**
(18 cm max), lateral tracking 24 cm RMS, 2 replans, docking 0.39 m / 9.5°. The docking heading in this run
is the weakest number in the repo, and it is reported as measured.

## The simulator (`iad_sim`)

A custom C++ sensor-level simulator was chosen over Gazebo, CARLA or AWSIM. One reason is practical: about
10 GB of free disk and no GPU passthrough. The other is that it provides **per-point ground-truth labels**
(ground, object, dust, ego body, steep terrain), true extrinsics and the true pose, which is what makes the
numbers above measurable.

- **Worlds** (procedural, seeded):
  - a 400 × 300 m quarry: benched pit, grade-limited haul road with berms and cuts, mud, a dump edge
    with safety berm, boulders, an overhead conveyor, an excavator, traffic
  - a 520 × 360 m container terminal: 3 × 4 blocks stacked 1–5 high, RTGs, ship-to-shore cranes, the quay
    edge, straddle carriers
- **LiDAR**: OpenMP ray casting with an azimuth broad-phase (about 4 ms per 38k-ray scan). Dust is
  Beer-Lambert backscatter sampled by thinning, with **dual return**. Intensity follows reflectivity and
  incidence angle.
- **GNSS**: quality follows sky visibility, ray-tested against the 3D world: RTK fix → float → single with
  multipath bias → no fix. RTK takes seconds of clear sky to re-converge.
- **Vehicle**: bicycle model with dead time, rate-limited lagged steering, power- and traction-limited
  drive, grade and rolling resistance, gears, slip and understeer on mud, and attitude from the terrain
  under the wheels.
- Publishes `/clock`, so the whole stack runs in sim time, plus the prior maps (point cloud and site
  costmap).

## Architecture

```
iad_common        header-only core: math, grids, vehicle model, PointCloud2 conversion, voxels
iad_msgs          Trajectory / TrajectoryPoint
iad_sim           worlds, vehicle dynamics, sensors, ground truth, prior maps, RViz meshes
iad_perception    dust filter -> CZM ground -> clustering/tracking -> log-odds occupancy
iad_localization  NDT pyramid + EKF (gating, delay replay), 4 estimators for comparison
iad_planning      Hybrid A* + Reeds-Shepp, shortcutting, smoothing, velocity profile, stop-and-replan
iad_control       LTV MPC (ADMM QP) + longitudinal PI/FF; pure-pursuit baseline
iad_calibration   ground plane + mapping-based multi-LiDAR registration
iad_eval          offline harnesses (perception confusion by range, planning queries) without ROS
iad_bringup       launch files, generated RViz configs
iad_tools         HUD overlays, metrics logger (JSONL), mission manager
```

```
iad_sim ─► /sensing/lidar/*/points, /sensing/imu/data, /sensing/gnss/*, /vehicle/status, /map/*, /clock
   ▲        │
   │        ├─► iad_perception ─► /perception/occupancy_grid, /perception/objects
   │        ├─► iad_localization ─► /localization/odometry, TF map→base_link
   │        └─► iad_planning ◄── /planning/goal (mission) ─► /planning/trajectory
   └──── /control/command ◄── iad_control
```

Algorithm cores are ROS-free headers, unit-tested without ROS, behind thin nodes. This is the same
core/node split Autoware uses.

## Run it

Everything runs in Docker (ROS 2 Humble + RViz2 + Xvfb + ffmpeg). No ROS install is needed on the host.

```bash
docker build -t iad_stack:humble -f docker/Dockerfile docker
scripts/build.sh                                       # colcon build (Release)
scripts/dev.sh colcon test && scripts/dev.sh colcon test-result --all

# demos (headless; add rviz:=true on a machine with a display)
scripts/dev.sh ros2 launch iad_bringup perception_demo.launch.py rviz:=false
scripts/dev.sh ros2 launch iad_bringup localization_demo.launch.py rviz:=false
scripts/dev.sh ros2 launch iad_bringup mission.launch.py scenario:=port goals:=rtg_lane_end,crane2_lane \
    perception:=true planning_obstacles:=true rviz:=false
scripts/dev.sh ros2 launch iad_bringup mission.launch.py start_pose:=dump_approach goals:=dump loaded:=true \
    actuator_delay:=0.5 algorithm:=pure_pursuit delay_compensation:=false rviz:=false
scripts/dev.sh ros2 launch iad_bringup calibration_demo.launch.py rviz:=false
scripts/dev.sh ros2 launch iad_bringup mission.launch.py goals:=load,dump perception:=true localization:=true rviz:=false

# record and compose a video: virtual X screen -> RViz -> ffmpeg, then title/result cards on the host
scripts/record.sh perception_raw 62 perception_demo.launch.py    # (REC_ENV="-e REC_LAYOUT=split")
python3 tools/video.py perception                               # -> media/01_perception.mp4

# offline evaluation without ROS transport
scripts/dev.sh ./build/iad_eval/perception_eval --step 6
scripts/dev.sh ./build/iad_eval/planning_eval port 55 -15 0 262 57 0
```

Metrics stream as JSON on `*/metrics` topics and are logged to `output/logs/*.jsonl`. The RViz configs are
generated by `tools/make_rviz.py`.

## Engineering notes: things that broke and what fixed them

- **Silent point-cloud loss.** Two perception nodes processed only 150–190 of 400 scans. Default Fast DDS
  shared memory (512 KB segments) pushed 0.5 MB clouds to fragmented best-effort UDP, where they dropped.
  A larger shared-memory segment fixed it ([`docker/fastdds.xml`](docker/fastdds.xml)). Parallel
  containers on one Docker bridge also discovered each other over DDS multicast until each got its own
  `ROS_DOMAIN_ID`.
- **The truck "stalled" on a 9 % ramp** because the model gave it less traction than gravity's pull. Real
  haul trucks have rimpull for 15 %. A rolling-resistance and breakaway feed-forward fixed a second stall
  on loose dirt.
- **An overconfident EKF in mud.** Wheel slip (15–30 %) is a correlated error, not white noise. The filter
  gated out perfectly good RTK fixes and NDT matches until the odometry-scale state was allowed to follow
  the ground.
- **A 4 m GNSS mast tilts.** A flat lever arm is off by 0.36 m at 5° pitch, so the lever arm is now
  rotated by the estimated roll and pitch.
- **Ground truth needed care.** "Terrain" is not "drivable": berm crests and cut faces are labelled
  separately so that ground recall means something.
- **Lattice meander.** The weighted Hybrid A* produced ±5 m S-curves in a straight aisle that no smoother
  could remove. Gear-restricted Reeds-Shepp shortcutting fixed it.
- **A rare start-up segfault** in the simulator was found by the unit tests: an unclamped height read at
  the map border, inside a non-const member function that silently picked the reference overload.

## Limitations and what I would do next

- Synthetic sensors. There is no motion distortion (deskewing), no camera, and no learned perception; the
  real next step is replaying recorded bags (and Autoware's sample data) through the same nodes.
- Objects are classified by size; tracking is greedy nearest-neighbour with α-β filters.
- The vehicle model is a planar bicycle with load, grade and slip, with no articulation (an articulated
  dump truck or tractor-trailer needs its own kinematics in both planner and MPC).
- Docking heading in the full mission (9.5°) is worse than in the isolated control demo (1.3°). The
  replanned approach to the bay is shorter and tighter. A terminal heading cost tied to docking and a
  final straight approach segment in the planner are the next fix.

## Repository layout

```
docker/        Dockerfile, Fast DDS profile
src/           ROS 2 packages (see Architecture)
scripts/       dev.sh (run in container), build.sh, record.sh (+ record_in_container.sh)
tools/         make_rviz.py, video.py (cards, compositing, reel), perception_report.py, control_trace.py
media/         final videos, GIFs, result cards, calibrated extrinsics
PLAN.md        the plan and milestone checklist this was built against
```
