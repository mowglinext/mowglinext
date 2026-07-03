# syntax=docker/dockerfile:1.7
# =============================================================================
# Universal GNSS sidecar image.
#
# IMPORTANT: this Dockerfile expects the monorepo root as its build context:
#
#   docker build -t mowgli-gps -f sensors/gps/Dockerfile .
# =============================================================================

FROM ros:kilted-ros-base AS builder

ARG DEBIAN_FRONTEND=noninteractive

RUN sed -i 's|http://archive.ubuntu.com/ubuntu|http://azure.archive.ubuntu.com/ubuntu|g' /etc/apt/sources.list.d/*.sources 2>/dev/null || true

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      python3-colcon-common-extensions \
      ros-kilted-ament-cmake \
      ros-kilted-diagnostic-msgs \
      ros-kilted-geometry-msgs \
      ros-kilted-launch \
      ros-kilted-launch-ros \
      ros-kilted-nav-msgs \
      ros-kilted-rclcpp \
      ros-kilted-rclpy \
      ros-kilted-rosidl-default-generators \
      ros-kilted-rosidl-default-runtime \
      ros-kilted-rtcm-msgs \
      ros-kilted-sensor-msgs \
      ros-kilted-std-msgs

WORKDIR /ws/src
COPY ros2/src/mowgli_interfaces /ws/src/mowgli_interfaces
COPY ros2/src/external/universal-gnss/gnss_ros2 /ws/src/gnss_ros2
COPY ros2/src/external/universal-gnss/gnss_core /ws/src/gnss_core
COPY ros2/src/external/universal-gnss/gnss_protocols /ws/src/gnss_protocols
COPY ros2/src/external/universal-gnss/gnss_transport /ws/src/gnss_transport
COPY ros2/src/external/universal-gnss/gnss_driver /ws/src/gnss_driver
COPY ros2/src/external/universal-gnss/gnss_ntrip /ws/src/gnss_ntrip
COPY ros2/src/external/universal-gnss/gnss_tools /ws/src/gnss_tools
COPY ros2/src/external/universal-gnss/CMakeLists.txt /ws/gnss_tools_root/CMakeLists.txt

WORKDIR /ws
RUN . /opt/ros/kilted/setup.sh \
 && colcon build --merge-install \
      --packages-select mowgli_interfaces universal_gnss_ros2 \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev \
 && ln -s /ws/src/gnss_core /ws/gnss_tools_root/gnss_core \
 && ln -s /ws/src/gnss_protocols /ws/gnss_tools_root/gnss_protocols \
 && ln -s /ws/src/gnss_driver /ws/gnss_tools_root/gnss_driver \
 && ln -s /ws/src/gnss_transport /ws/gnss_tools_root/gnss_transport \
 && ln -s /ws/src/gnss_ntrip /ws/gnss_tools_root/gnss_ntrip \
 && ln -s /ws/src/gnss_tools /ws/gnss_tools_root/gnss_tools \
 && cmake -S /ws/gnss_tools_root -B /ws/gnss_tools_build \
      -DBUILD_TESTING=OFF \
      -DUNIVERSAL_GNSS_BUILD_ROS2=OFF \
 && cmake --build /ws/gnss_tools_build -j"$(nproc)" \
 && cmake --install /ws/gnss_tools_build --prefix /ws/install \
 && rm -rf /ws/build /ws/log /ws/gnss_tools_build /ws/gnss_tools_root /ws/src

FROM ros:kilted-ros-base

ARG DEBIAN_FRONTEND=noninteractive

RUN sed -i 's|http://archive.ubuntu.com/ubuntu|http://azure.archive.ubuntu.com/ubuntu|g' /etc/apt/sources.list.d/*.sources 2>/dev/null || true

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
      ros-kilted-diagnostic-msgs \
      ros-kilted-rmw-cyclonedds-cpp \
      ros-kilted-rclpy \
      ros-kilted-rosidl-default-runtime \
      ros-kilted-rtcm-msgs

COPY --from=builder /ws/install /opt/gnss_sidecar

ENV PATH="/opt/gnss_sidecar/bin:${PATH}"

COPY sensors/gps/ros2_entrypoint.sh /ros2_entrypoint.sh
COPY sensors/gps/start_gps.sh /start_gps.sh
COPY sensors/gps/universal_gnss_topic_bridge.py /universal_gnss_topic_bridge.py
RUN chmod +x /ros2_entrypoint.sh /start_gps.sh /universal_gnss_topic_bridge.py

ENTRYPOINT ["/ros2_entrypoint.sh"]
CMD ["/start_gps.sh"]
