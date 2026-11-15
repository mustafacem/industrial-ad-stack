#!/usr/bin/env bash
# Record one demo run: RViz capture + Gazebo chase camera + clock log, into output/raw/<name>*.
#   scripts/record.sh <name> <sim seconds> <launch file> [launch args...]
#   REC_LAYOUT=split for the two-window views, REC_GZ=0 to skip the Gazebo camera.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NAME=$1; DUR=$2; LAUNCH=$3; shift 3
IAD_DOCKER_ARGS="--name iad_rec_${NAME} -e REC_LAYOUT=${REC_LAYOUT:-single} -e REC_GZ=${REC_GZ:-1} -e REC_WARMUP=${REC_WARMUP:-14}" \
  "$ROOT/scripts/dev.sh" bash /ws/scripts/record_in_container.sh "$NAME" "$DUR" iad_bringup "$LAUNCH" "$@"
