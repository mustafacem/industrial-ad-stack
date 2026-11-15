#!/usr/bin/env bash
# Build the workspace inside the container. Extra args go to colcon (e.g. --packages-select iad_sim).
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
exec scripts/dev.sh colcon build --symlink-install --parallel-workers 4 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON "$@"
