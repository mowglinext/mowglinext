# =============================================================================
# LDRobot STL27L LiDAR driver for ROS2 Lyrical
#
# Publishes sensor_msgs/LaserScan on /scan with frame_id=lidar_link.
# Built from source — no apt package available for Lyrical.
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

RUN git init -q src/ldlidar_stl_ros2 \
 && git -C src/ldlidar_stl_ros2 fetch --depth 1 https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git bf668a89baf722a787dadc442860dcbf33a82f5a \
 && git -C src/ldlidar_stl_ros2 checkout -q --detach FETCH_HEAD

# Fix missing pthread.h include in upstream source
RUN sed -i '1i #include <pthread.h>' \
      src/ldlidar_stl_ros2/ldlidar_driver/src/logger/log_module.cpp

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
CMD ["ros2", "run", "ldlidar_stl_ros2", "ldlidar_stl_ros2_node", \
     "--ros-args", \
     "-p", "product_name:=LDLiDAR_STL27L", \
     "-p", "topic_name:=scan", \
     "-p", "frame_id:=lidar_link", \
     "-p", "port_name:=/dev/ttyS1", \
     "-p", "port_baudrate:=921600", \
     "-p", "laser_scan_dir:=true", \
     "-p", "enable_angle_crop_func:=false"]
