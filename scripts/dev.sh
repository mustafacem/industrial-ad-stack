#!/usr/bin/env bash
# Run a command inside the ROS 2 Humble container with this repo mounted at /ws (as the host user).
# Each container gets its own ROS_DOMAIN_ID: containers share the docker bridge, so parallel runs would
# otherwise discover each other over DDS multicast and cross-talk.
#   scripts/dev.sh colcon build --symlink-install
#   scripts/dev.sh ros2 launch iad_bringup perception_demo.launch.py
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${IAD_IMAGE:-iad_stack:humble}"
TTY=""; [ -t 0 ] && [ -t 1 ] && TTY="-it"
# GPU (NVIDIA container runtime) for Gazebo's gpu_lidar / cameras; IAD_GPU=0 disables it.
GPU=""; [ "${IAD_GPU:-1}" = "1" ] && GPU="--runtime=nvidia -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all"
mkdir -p "$ROOT/.home"
exec docker run --rm $TTY $GPU ${IAD_DOCKER_ARGS:-} \
  --user "$(id -u):$(id -g)" -e HOME=/ws/.home -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-$((RANDOM % 200 + 1))} -e FASTRTPS_DEFAULT_PROFILES_FILE=/ws/docker/fastdds.xml -e RCUTILS_COLORIZED_OUTPUT=0 \
  -e GZ_IP=127.0.0.1 -e GZ_PARTITION=iad \
  --shm-size=12g -v "$ROOT":/ws -w /ws "$IMAGE" \
  bash -c 'source /opt/ros/humble/setup.bash; if [ -f /ws/install/setup.bash ]; then source /ws/install/setup.bash; fi; exec "$@"' _ "$@"
