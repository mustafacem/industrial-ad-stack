# Dev / demo image: ROS 2 Humble + Gazebo Harmonic (ros_gz) + RViz2 + headless X (Xvfb) + ffmpeg,
# so the whole stack can be built, simulated and recorded without a ROS install on the host.
#   docker build -t iad_stack:humble -f docker/Dockerfile docker
# GPU rendering (gpu_lidar, cameras) needs:  --runtime=nvidia -e NVIDIA_DRIVER_CAPABILITIES=all
FROM ros:humble-ros-base
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-rviz2 \
      ros-humble-rviz-2d-overlay-plugins \
      ros-humble-rviz-2d-overlay-msgs \
      ros-humble-ackermann-msgs \
      ros-humble-tf2-eigen \
      libeigen3-dev \
      python3-numpy \
      xvfb ffmpeg fonts-dejavu-core \
    && rm -rf /var/lib/apt/lists/*

# Gazebo Harmonic (OSRF repo) + the Humble/Harmonic ros_gz build, Autoware interface messages
# (the de-facto standard AD message set), rosbag2 MCAP storage.
RUN apt-get update && apt-get install -y --no-install-recommends curl gnupg lsb-release \
    && curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
         -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" \
         > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends \
      gz-harmonic \
      ros-humble-ros-gzharmonic \
      ros-humble-autoware-planning-msgs \
      ros-humble-autoware-perception-msgs \
      ros-humble-autoware-vehicle-msgs \
      ros-humble-rosbag2-storage-mcap \
      ros-humble-launch-testing-ament-cmake \
      python3-pil python3-yaml \
    && rm -rf /var/lib/apt/lists/*

# The NVIDIA container runtime mounts libEGL_nvidia but not its GLVND vendor file: without it EGL falls
# back to Mesa and Gazebo's headless (EGL) rendering fails.
COPY 10_nvidia.json /usr/share/glvnd/egl_vendor.d/10_nvidia.json

ENV QT_X11_NO_MITSHM=1 \
    RCUTILS_COLORIZED_OUTPUT=1 \
    GZ_VERSION=harmonic
WORKDIR /ws
