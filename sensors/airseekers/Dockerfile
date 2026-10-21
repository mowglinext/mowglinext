# syntax=docker/dockerfile:1.7
# =============================================================================
# Airseekers Tron hardware-bridge sidecar image (HARDWARE_BACKEND=airseekers).
#
# Speaks the Tron's vendor chassis/cutter MCU, reads its WIT JY61P IMU, its
# panel keys and its UM960 GNSS receiver, and publishes MowgliNext's
# hardware_bridge contract plus /gps/fix and /gps/status. The vendor firmware
# is NOT modified; this container replaces mowgli-ros2's hardware_bridge_node
# on a Tron.
#
# IMPORTANT: build context = monorepo root (copies ros2/src/mowgli_interfaces
# and the robot-config template):
#
#   docker build -t mowgli-airseekers -f sensors/airseekers/Dockerfile .
# =============================================================================

FROM ros:lyrical-ros-base AS builder

ARG DEBIAN_FRONTEND=noninteractive

RUN sed -i 's|http://archive.ubuntu.com/ubuntu|http://azure.archive.ubuntu.com/ubuntu|g' /etc/apt/sources.list.d/*.sources 2>/dev/null || true

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      python3-colcon-common-extensions \
      ros-lyrical-ament-cmake \
      ros-lyrical-geometry-msgs \
      ros-lyrical-nav-msgs \
      ros-lyrical-rosidl-default-generators \
      ros-lyrical-rosidl-default-runtime \
      ros-lyrical-sensor-msgs \
      ros-lyrical-std-msgs \
      ros-lyrical-std-srvs

WORKDIR /ws/src
COPY ros2/src/mowgli_interfaces /ws/src/mowgli_interfaces
COPY sensors/airseekers/um960_gps_driver /ws/src/um960_gps_driver
COPY sensors/airseekers/mowgli_airseekers_bridge /ws/src/mowgli_airseekers_bridge

WORKDIR /ws
RUN chown -R ubuntu:ubuntu /ws
USER ubuntu
RUN /bin/bash -c 'source /opt/ros/lyrical/setup.bash \
 && MAKEFLAGS=-j3 colcon build --merge-install --parallel-workers 2 \
      --packages-up-to mowgli_airseekers_bridge \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
      -DBUILD_TESTING=OFF -Wno-dev \
 && rm -rf /ws/build /ws/log /ws/src'

FROM ros:lyrical-ros-base

ARG DEBIAN_FRONTEND=noninteractive

RUN sed -i 's|http://archive.ubuntu.com/ubuntu|http://azure.archive.ubuntu.com/ubuntu|g' /etc/apt/sources.list.d/*.sources 2>/dev/null || true

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
      python3-yaml \
      ros-lyrical-geometry-msgs \
      ros-lyrical-launch \
      ros-lyrical-launch-ros \
      ros-lyrical-nav-msgs \
      ros-lyrical-rclpy \
      ros-lyrical-rmw-cyclonedds-cpp \
      ros-lyrical-rosidl-default-runtime \
      ros-lyrical-sensor-msgs \
      ros-lyrical-std-msgs \
      ros-lyrical-std-srvs

COPY --from=builder /ws/install /opt/airseekers_bridge
# The robot-config TEMPLATE (Invariant 15) + the Airseekers backend defaults:
# the launch file layers the operator's sparse mowgli_robot.yaml over them,
# exactly like mowgli-ros2, so "default" means the same value here.
COPY ros2/src/mowgli_bringup/config/mowgli_robot.yaml /opt/airseekers_bridge/mowgli_robot.template.yaml
COPY ros2/src/mowgli_bringup/config/backends/airseekers.yaml /opt/airseekers_bridge/backend_defaults.yaml

COPY sensors/airseekers/ros2_entrypoint.sh /ros2_entrypoint.sh
COPY sensors/airseekers/start_airseekers.sh /start_airseekers.sh
RUN chmod +x /ros2_entrypoint.sh /start_airseekers.sh

ENTRYPOINT ["/ros2_entrypoint.sh"]
CMD ["/start_airseekers.sh"]
