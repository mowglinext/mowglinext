# =============================================================================
# Slamtec RPLiDAR driver for ROS2 Lyrical
#
# Supports RPLiDAR A1, A2, A3, S1, S2, S3, C1.
# Publishes sensor_msgs/LaserScan on /scan with frame_id=lidar_link.
#
# To use this instead of the default LD19 driver, set in your .env:
#   LIDAR_IMAGE=ghcr.io/mowglinext/mowglinext/lidar-rplidar:main
# or override the build context in docker-compose.override.yml.
# =============================================================================
FROM ros:lyrical-ros-base AS build

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    ros-lyrical-ament-cmake \
    build-essential \
    cmake \
    git \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

RUN git init -q src/rplidar_ros \
 && git -C src/rplidar_ros fetch --depth 1 https://github.com/Slamtec/rplidar_ros.git 24cc9b6dea97e045bda1408eaa867ce730fd3fc3 \
 && git -C src/rplidar_ros checkout -q --detach FETCH_HEAD

RUN python3 - <<'PY_PATCH'
from pathlib import Path
import re
for p in Path('/ros2_ws/src').rglob('CMakeLists.txt'):
    s = p.read_text()
    def link(m):
        target, *deps = m[1].split()
        return 'target_link_libraries(' + target + ' ' + ' '.join('${' + d + '_TARGETS}' for d in deps) + ')'
    s = re.sub(r'ament_target_dependencies\((.*?)\)', link, s, flags=re.S)
    p.write_text(s)
PY_PATCH

RUN chown -R ubuntu:ubuntu /ros2_ws
USER ubuntu
RUN /bin/bash -c 'source /opt/ros/lyrical/setup.bash \
 && MAKEFLAGS=-j3 colcon build --parallel-workers 2 \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
      -DCMAKE_CXX_STANDARD=20 -DBUILD_TESTING=OFF'

# =============================================================================
# Runtime — minimal image
# =============================================================================
FROM ros:lyrical-ros-base AS runtime

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-lyrical-rmw-cyclonedds-cpp \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

COPY --from=build /ros2_ws/install/ install/

COPY ros2_entrypoint.sh /ros2_entrypoint.sh
RUN chmod +x /ros2_entrypoint.sh

ENTRYPOINT ["/ros2_entrypoint.sh"]
CMD ["ros2", "launch", "rplidar_ros", "rplidar_a2m8_launch.py", \
     "frame_id:=lidar_link", \
     "topic_name:=/scan", \
     "serial_port:=/dev/ttyUSB0", \
     "serial_baudrate:=115200"]
