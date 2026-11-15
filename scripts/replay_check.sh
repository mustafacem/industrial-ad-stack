#!/usr/bin/env bash
# Independence check, part 2: every subsystem runs ALONE from its rosbag2 recording, through its own launch
# file (no Gazebo, no other iad_* node, no evaluator), and must produce its outputs.
#   scripts/replay_check.sh [perception localization planning control calibration]
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CHECKS=("$@")
[ ${#CHECKS[@]} -eq 0 ] && CHECKS=(perception localization planning control calibration)
status=0
for c in "${CHECKS[@]}"; do
  case $c in
    perception)   L="iad_perception perception.launch.py bag:=/ws/data/bags/perception_quarry"
                  T="/perception/occupancy_grid nav_msgs/msg/OccupancyGrid 300" ;;
    localization) L="iad_localization localization.launch.py bag:=/ws/data/bags/localization_port map_dir:=/ws/data/maps/port"
                  T="/localization/kinematic_state nav_msgs/msg/Odometry 3000" ;;
    planning)     L="iad_planning planning.launch.py bag:=/ws/data/bags/planning_port map_dir:=/ws/data/maps/port vehicle:=container_agv odom_topic:=/sim/ground_truth/odom"
                  T="/planning/trajectory autoware_planning_msgs/msg/Trajectory 2" ;;
    control)      L="iad_control control.launch.py bag:=/ws/data/bags/control_quarry odom_topic:=/sim/ground_truth/odom"
                  T="/control/command ackermann_msgs/msg/AckermannDriveStamped 500" ;;
    calibration)  L="iad_calibration calibration.launch.py bag:=/ws/data/bags/calibration_port odom_topic:=/sim/ground_truth/odom"
                  T="/calibration/metrics std_msgs/msg/String 50" ;;
    *) echo "unknown $c"; continue ;;
  esac
  set -- $T
  out=$(IAD_GPU=0 "$ROOT/scripts/dev.sh" bash -c "
    ros2 launch $L rviz:=false rate:=${REPLAY_RATE:-2.0} > /ws/output/logs/replay_$c.log 2>&1 &
    python3 - $1 $2 <<'PY'
import importlib, sys, time, rclpy
from rclpy.node import Node
topic, typ = sys.argv[1], sys.argv[2]
pkg, _, name = typ.split('/')
cls = getattr(importlib.import_module(pkg + '.msg'), name)
rclpy.init(); n = Node('replay_probe'); cnt = [0]; last = [None]
def cb(m): cnt[0] += 1; last[0] = m
n.create_subscription(cls, topic, cb, 10)  # reliable/volatile: matches volatile and latched publishers
import subprocess
t0 = time.time()
while time.time() - t0 < 600:  # until the bag has been played
    rclpy.spin_once(n, timeout_sec=0.2)
    if time.time() - t0 > 15 and subprocess.run(['pgrep', '-f', '[b]in/ros2 bag play'], capture_output=True).returncode != 0:
        break
extra = ''
if typ.endswith('String') and last[0] is not None:
    extra = ' ' + last[0].data[:160]
print(cnt[0], extra)
PY
    pkill -INT -f 'ros2 launch' ; sleep 2" 2>/dev/null | tail -n 1)
  n=${out%% *}
  if [ -n "$n" ] && [ "$n" -ge "$3" ] 2>/dev/null; then echo "PASS $c: $n messages on $1 ${out#* }"
  else echo "FAIL $c: '${out}' messages on $1 (expected >= $3), see output/logs/replay_$c.log"; status=1; fi
done
exit $status
