# syntax=docker/dockerfile:1.7
# =============================================================================
# Universal GNSS sidecar image.
#
# IMPORTANT: this Dockerfile expects the monorepo root as its build context:
#
#   docker build -t mowgli-gps -f sensors/gps/Dockerfile .
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
      ros-lyrical-diagnostic-msgs \
      ros-lyrical-geometry-msgs \
      ros-lyrical-launch \
      ros-lyrical-launch-ros \
      ros-lyrical-nav-msgs \
      ros-lyrical-rclcpp \
      ros-lyrical-rclpy \
      ros-lyrical-rosidl-default-generators \
      ros-lyrical-rosidl-default-runtime \
      ros-lyrical-rtcm-msgs \
      ros-lyrical-sensor-msgs \
      ros-lyrical-std-msgs

WORKDIR /ws/src
COPY ros2/src/mowgli_interfaces /ws/src/mowgli_interfaces
COPY sensors/gps/mowgli_gnss_bridge /ws/src/mowgli_gnss_bridge
COPY ros2/src/external/universal-gnss/gnss_ros2 /ws/src/gnss_ros2
COPY ros2/src/external/universal-gnss/gnss_core /ws/src/gnss_core
COPY ros2/src/external/universal-gnss/gnss_protocols /ws/src/gnss_protocols
COPY ros2/src/external/universal-gnss/gnss_transport /ws/src/gnss_transport
COPY ros2/src/external/universal-gnss/gnss_driver /ws/src/gnss_driver
COPY ros2/src/external/universal-gnss/gnss_ntrip /ws/src/gnss_ntrip
COPY ros2/src/external/universal-gnss/gnss_runtime /ws/src/gnss_runtime
COPY ros2/src/external/universal-gnss/gnss_tools /ws/src/gnss_tools
COPY ros2/src/external/universal-gnss/CMakeLists.txt /ws/gnss_tools_root/CMakeLists.txt

WORKDIR /ws
RUN chown -R ubuntu:ubuntu /ws
USER ubuntu
# The pinned GNSS package does not install its intermediate driver libraries.
# Keep them static under Lyrical's ament defaults so installed nodes are self-contained.
RUN /bin/bash -c 'source /opt/ros/lyrical/setup.bash \
 && MAKEFLAGS=-j3 colcon build --merge-install --parallel-workers 2 \
      --packages-select mowgli_interfaces universal_gnss_ros2 mowgli_gnss_bridge \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
      -DCMAKE_CXX_STANDARD=20 -DBUILD_SHARED_LIBS=OFF -DBUILD_TESTING=OFF -Wno-dev \
 && for component in gnss_core gnss_protocols gnss_driver gnss_transport gnss_ntrip gnss_runtime gnss_tools; do \
      ln -s "/ws/src/$component" "/ws/gnss_tools_root/$component"; \
    done \
 && cmake -S /ws/gnss_tools_root -B /ws/gnss_tools_build \
      -DBUILD_TESTING=OFF -DUNIVERSAL_GNSS_BUILD_ROS2=OFF \
 && cmake --build /ws/gnss_tools_build --parallel 3 \
 && cmake --install /ws/gnss_tools_build --prefix /ws/install \
 && rm -rf /ws/build /ws/log /ws/gnss_tools_build /ws/gnss_tools_root /ws/src'

FROM ros:lyrical-ros-base

ARG DEBIAN_FRONTEND=noninteractive

RUN sed -i 's|http://archive.ubuntu.com/ubuntu|http://azure.archive.ubuntu.com/ubuntu|g' /etc/apt/sources.list.d/*.sources 2>/dev/null || true

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
      ros-lyrical-diagnostic-msgs \
      ros-lyrical-rmw-cyclonedds-cpp \
      ros-lyrical-rclpy \
      ros-lyrical-rosidl-default-runtime \
      ros-lyrical-rtcm-msgs

COPY --from=builder /ws/install /opt/gnss_sidecar

ENV PATH="/opt/gnss_sidecar/bin:${PATH}"

COPY sensors/gps/ros2_entrypoint.sh /ros2_entrypoint.sh
COPY sensors/gps/start_gps.sh /start_gps.sh
COPY sensors/gps/universal_gnss_topic_bridge.py /universal_gnss_topic_bridge.py
RUN chmod +x /ros2_entrypoint.sh /start_gps.sh /universal_gnss_topic_bridge.py

ENTRYPOINT ["/ros2_entrypoint.sh"]
CMD ["/start_gps.sh"]
