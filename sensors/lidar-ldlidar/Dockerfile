# syntax=docker/dockerfile:1.7
# =============================================================================
# LDRobot LD19 LiDAR driver for ROS 2 — Myzhar/ldrobot-lidar-ros2
#
# This replaces ldrobotSensorTeam/ldlidar_stl_ros2 because the upstream
# driver emits a jittery scan length (499-503 readings per revolution),
# which slam_toolbox's Karto backend rejects with "expected N range
# readings" warnings. Myzhar's driver has a `lidar.bins` parameter that
# forces a fixed output length — default 455.
#
# Publishes sensor_msgs/LaserScan on /scan (remapped from ~/scan).
# Frame id: lidar_link.
#
# The Myzhar driver is a lifecycle ComposableNode; it auto-transitions
# through configure → activate via the bundled nav2_lifecycle_manager
# launch (ldlidar_with_mgr.launch.py).
# =============================================================================

FROM ros:kilted-ros-base AS build

ARG DEBIAN_FRONTEND=noninteractive
ARG LDLIDAR_REF=v0.3.0
ARG LDLIDAR_SHA=4ee53a8b176037cf418a54b25007075bb2b1d3a0

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      git \
      libudev-dev \
      python3-colcon-common-extensions \
      ros-kilted-ament-cmake \
      ros-kilted-bond \
      ros-kilted-bondcpp \
      ros-kilted-diagnostic-msgs \
      ros-kilted-diagnostic-updater \
      ros-kilted-launch-ros \
      ros-kilted-nav2-lifecycle-manager \
      ros-kilted-nav2-msgs \
      ros-kilted-nav2-util \
      ros-kilted-rclcpp \
      ros-kilted-rclcpp-components \
      ros-kilted-rclcpp-lifecycle \
      ros-kilted-robot-state-publisher \
      ros-kilted-sensor-msgs \
      ros-kilted-std-msgs

WORKDIR /ros2_ws/src
RUN git clone --depth 1 --branch ${LDLIDAR_REF} \
      https://github.com/Myzhar/ldrobot-lidar-ros2.git \
 && cd ldrobot-lidar-ros2 \
 && git checkout ${LDLIDAR_SHA} \
 && git submodule update --init --recursive

# Upstream CMakeLists has a FATAL_ERROR when the ROS distro isn't humble
# or jazzy. Downgrade every FATAL_ERROR to WARNING in the driver's cmake
# so Kilted builds — the FATAL_ERROR on the distro check is multi-line
# so a targeted sed can't anchor on the string. The driver has no
# distro-specific calls; the check is a courtesy.
RUN find ldrobot-lidar-ros2 -name CMakeLists.txt -exec \
      sed -i 's/message(FATAL_ERROR/message(WARNING/g' {} \;

# Patch the subscriber-count gating bug: the driver calls
# count_subscribers(_scanTopic) with the UNRESOLVED relative topic name
# (e.g. "~/scan"), which always returns 0 — even when real subscribers
# exist on the resolved /scan. Replace the relative-name lookup with
# the publisher's own get_subscription_count(), which uses the resolved
# name. Without this patch the driver reports "NO SUBSCRIBERS" and
# skips publishing even though it reads the LiDAR at 10 Hz.
RUN sed -i 's|count_subscribers(_scanTopic)|_scanPub->get_subscription_count()|g' \
      ldrobot-lidar-ros2/ldlidar_component/component/src/ldlidar_component.cpp

WORKDIR /ros2_ws
RUN . /opt/ros/kilted/setup.sh \
 && colcon build --merge-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
      --parallel-workers $(nproc)

# =============================================================================
# Runtime
# =============================================================================
FROM ros:kilted-ros-base AS runtime

ARG DEBIAN_FRONTEND=noninteractive

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
      libudev1 \
      ros-kilted-bond \
      ros-kilted-bondcpp \
      ros-kilted-diagnostic-updater \
      ros-kilted-launch-ros \
      ros-kilted-nav2-lifecycle-manager \
      ros-kilted-rclcpp-components \
      ros-kilted-rclcpp-lifecycle \
      ros-kilted-rmw-cyclonedds-cpp \
      ros-kilted-robot-state-publisher

COPY --from=build /ros2_ws/install /opt/ldlidar

COPY ros2_entrypoint.sh /ros2_entrypoint.sh
COPY ldlidar.yaml /ldlidar.yaml
COPY ldlidar_scan.launch.py /ldlidar_scan.launch.py
COPY start_lidar.sh /start_lidar.sh
RUN chmod +x /ros2_entrypoint.sh /start_lidar.sh

ENTRYPOINT ["/ros2_entrypoint.sh"]
CMD ["/start_lidar.sh"]
