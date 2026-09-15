# syntax=docker/dockerfile:1.7
# =============================================================================
# OpenMower hardware-bridge sidecar image (HARDWARE_BACKEND=openmower).
#
# Speaks the stock OpenMower v1 electronics — LowLevel (Pico) board + xESC
# motor controllers — and publishes MowgliNext's hardware_bridge contract.
# The OpenMower firmware is NOT modified; this container replaces
# mowgli-ros2's hardware_bridge_node on those robots.
#
# IMPORTANT: this Dockerfile expects the monorepo root as its build context
# (it copies ros2/src/mowgli_interfaces and ros2/src/mowgli_hardware):
#
#   docker build -t mowgli-openmower -f sensors/openmower/Dockerfile .
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
      ros-lyrical-launch \
      ros-lyrical-launch-ros \
      ros-lyrical-nav-msgs \
      ros-lyrical-rclcpp \
      ros-lyrical-rosidl-default-generators \
      ros-lyrical-rosidl-default-runtime \
      ros-lyrical-sensor-msgs \
      ros-lyrical-std-msgs \
      ros-lyrical-std-srvs

WORKDIR /ws/src
COPY ros2/src/mowgli_interfaces /ws/src/mowgli_interfaces
COPY ros2/src/mowgli_hardware /ws/src/mowgli_hardware
COPY sensors/openmower/mowgli_openmower_bridge /ws/src/mowgli_openmower_bridge

WORKDIR /ws
RUN chown -R ubuntu:ubuntu /ws
USER ubuntu
RUN /bin/bash -c 'source /opt/ros/lyrical/setup.bash \
 && MAKEFLAGS=-j3 colcon build --merge-install --parallel-workers 2 \
      --packages-up-to mowgli_openmower_bridge \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
      -DCMAKE_CXX_STANDARD=20 -DBUILD_TESTING=OFF -Wno-dev \
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
      ros-lyrical-rclcpp \
      ros-lyrical-rmw-cyclonedds-cpp \
      ros-lyrical-rosidl-default-runtime \
      ros-lyrical-sensor-msgs \
      ros-lyrical-std-msgs \
      ros-lyrical-std-srvs

COPY --from=builder /ws/install /opt/openmower_bridge

COPY sensors/openmower/ros2_entrypoint.sh /ros2_entrypoint.sh
COPY sensors/openmower/start_openmower.sh /start_openmower.sh
RUN chmod +x /ros2_entrypoint.sh /start_openmower.sh

ENTRYPOINT ["/ros2_entrypoint.sh"]
CMD ["/start_openmower.sh"]
