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

FROM ros:lyrical-ros-base AS build

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
      ros-lyrical-ament-cmake \
      ros-lyrical-bond \
      ros-lyrical-bondcpp \
      ros-lyrical-diagnostic-msgs \
      ros-lyrical-diagnostic-updater \
      ros-lyrical-launch-ros \
      ros-lyrical-nav2-lifecycle-manager \
      ros-lyrical-nav2-msgs \
      ros-lyrical-nav2-util \
      ros-lyrical-nav2-common \
      ros-lyrical-nav2-ros-common \
      ros-lyrical-rclcpp \
      ros-lyrical-rclcpp-components \
      ros-lyrical-rclcpp-lifecycle \
      ros-lyrical-robot-state-publisher \
      ros-lyrical-sensor-msgs \
      ros-lyrical-std-msgs

WORKDIR /ros2_ws/src
RUN git clone --depth 1 --branch ${LDLIDAR_REF} \
      https://github.com/Myzhar/ldrobot-lidar-ros2.git \
 && cd ldrobot-lidar-ros2 \
 && git checkout ${LDLIDAR_SHA} \
 && git submodule update --init --recursive

# Upstream CMakeLists has a FATAL_ERROR when the ROS distro isn't humble
# or jazzy. Downgrade every FATAL_ERROR to WARNING in the driver's cmake
# so Lyrical builds — the FATAL_ERROR on the distro check is multi-line
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

# Adapt the pinned upstream driver to Nav2's Lyrical lifecycle API.
RUN python3 - <<'PY_PATCH'
from pathlib import Path
root = Path('/ros2_ws/src/ldrobot-lidar-ros2/ldlidar_component')
for p in root.rglob('*'):
    if p.suffix not in {'.cpp', '.hpp', '.xml', '.txt'}:
        continue
    s = p.read_text().replace('nav2_util', 'nav2_ros_common').replace('nav2_ros_common::', 'nav2::')
    s = s.replace('  nav2::LifecycleNode::integer_range limits_int;\n', '')
    s = s.replace('(_scanTopic, qos, pub_opt)', '(_scanTopic, qos)')
    s = s.replace('ament_target_dependencies(${PROJECT_NAME}\n    ${DEPENDENCIES}\n)',
                  'foreach(dep IN LISTS DEPENDENCIES)\n'
                  '  target_link_libraries(${PROJECT_NAME} ${${dep}_TARGETS})\n'
                  'endforeach()')
    p.write_text(s)
PY_PATCH

WORKDIR /ros2_ws
RUN chown -R ubuntu:ubuntu /ros2_ws
USER ubuntu
RUN /bin/bash -c 'source /opt/ros/lyrical/setup.bash \
 && MAKEFLAGS=-j3 colcon build --merge-install --parallel-workers 2 \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
      -DCMAKE_CXX_STANDARD=20 -DBUILD_TESTING=OFF'

# Upstream ldlidar_component builds the LDROBOT SDK as a SHARED library
# target named `ldlidar` (libldlidar.so) and links the component against it,
# but its install() rule only installs the component — not the SDK lib. With
# --merge-install + a runtime stage that COPYs only /ros2_ws/install,
# libldlidar.so is left behind in the build tree and the component fails to
# dlopen at startup ("libldlidar.so: cannot open shared object file"). Install
# it explicitly next to the component (/opt/ldlidar/lib is on LD_LIBRARY_PATH
# via the sourced overlay).
RUN cp "$(find /ros2_ws/build -name libldlidar.so -print -quit)" \
       /ros2_ws/install/lib/ \
 && test -f /ros2_ws/install/lib/libldlidar.so

# =============================================================================
# Runtime
# =============================================================================
FROM ros:lyrical-ros-base AS runtime

ARG DEBIAN_FRONTEND=noninteractive

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
      libudev1 \
      ros-lyrical-bond \
      ros-lyrical-bondcpp \
      ros-lyrical-diagnostic-updater \
      ros-lyrical-launch-ros \
      ros-lyrical-nav2-lifecycle-manager \
      ros-lyrical-rclcpp-components \
      ros-lyrical-rclcpp-lifecycle \
      ros-lyrical-rmw-cyclonedds-cpp \
      ros-lyrical-robot-state-publisher

COPY --from=build /ros2_ws/install /opt/ldlidar

COPY ros2_entrypoint.sh /ros2_entrypoint.sh
COPY ldlidar.yaml /ldlidar.yaml
COPY ldlidar_scan.launch.py /ldlidar_scan.launch.py
COPY start_lidar.sh /start_lidar.sh
RUN chmod +x /ros2_entrypoint.sh /start_lidar.sh

ENTRYPOINT ["/ros2_entrypoint.sh"]
CMD ["/start_lidar.sh"]
