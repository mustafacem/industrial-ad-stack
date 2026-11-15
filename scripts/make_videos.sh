#!/usr/bin/env bash
# Record every demo (RViz capture + Gazebo GPU chase camera + clock log) into output/raw/, one after the
# other (RViz renders in software, so runs must not overlap). Composition: tools/video.py v3-demo.
#   scripts/make_videos.sh [demo ...]      demos: perception localization planning control calibration mission
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
DEMOS=("$@")
[ ${#DEMOS[@]} -eq 0 ] && DEMOS=(perception localization planning control calibration mission)
rec() { echo "== $1 ($2 s sim)"; "$ROOT/scripts/record.sh" "$@" 2>&1 | tail -n 1; }
for d in "${DEMOS[@]}"; do
  case $d in
    perception)   REC_LAYOUT=split rec 01_perception 90 perception_demo.launch.py split:=true ;;
    localization) rec 02_localization 160 localization_demo.launch.py ;;
    planning)     rec 03_planning 230 planning_demo.launch.py ;;
    control)      rec 04_control_mpc 80 control_demo.launch.py algorithm:=mpc delay_compensation:=true
                  rec 04_control_pp 80 control_demo.launch.py algorithm:=pure_pursuit delay_compensation:=false ;;
    calibration)  rec 05_calibration 105 calibration_demo.launch.py ;;
    mission)      rec 06_mission 170 mission_demo.launch.py ;;
    *) echo "unknown demo $d" ;;
  esac
done
ls -la output/raw/*.mp4 2>/dev/null
