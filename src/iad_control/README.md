# iad_control

Trajectory tracking for heavy vehicles with actuator dead time.
- **Lateral:** a linear time-varying MPC (ADMM QP). Before solving, it replays the steering commands still
  in the actuator pipe ("delay compensation").
- **Longitudinal:** PI plus feed-forward for planned acceleration, rolling resistance and grade.
- **Gear changes:** finish the segment, stop, steer at standstill, then go.

Pure pursuit is included as a baseline. The package also ships a software-in-the-loop plant, so it runs
closed loop with no simulator at all.

| | Topic | Type |
|---|---|---|
| in | `/planning/trajectory` | `autoware_planning_msgs/Trajectory` |
| in | `/localization/kinematic_state` (`odom_topic`) | `nav_msgs/Odometry` |
| in | `/vehicle/status/velocity` | `geometry_msgs/TwistWithCovarianceStamped` |
| in | `/vehicle/status/steering` | `autoware_vehicle_msgs/SteeringReport` |
| out | `/control/command`: `steering_angle`, `acceleration` along the gear, `sign(speed)` = gear | `ackermann_msgs/AckermannDriveStamped` |
| out | `/control/reference`, `/control/predicted`, `/control/trail`, `/control/metrics` | |

```bash
ros2 launch iad_control control.launch.py sil:=true          # closed loop on the SIL plant, demo trajectory
ros2 launch iad_control control.launch.py sil:=true algorithm:=pure_pursuit delay_compensation:=false
colcon test --packages-select iad_control                    # gtest + closed-loop SIL launch test
```
