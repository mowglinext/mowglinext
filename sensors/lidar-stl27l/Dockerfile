# =============================================================================
# LDRobot STL27L LiDAR driver for ROS2 Jazzy
#
# Publishes sensor_msgs/LaserScan on /scan with frame_id=lidar_link.
# Built from source — no apt package available for Jazzy.
# =============================================================================
FROM ros:kilted-ros-base AS build

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    ros-kilted-ament-cmake \
    build-essential \
    cmake \
    git \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

RUN git clone --depth 1 \
      https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git \
      src/ldlidar_stl_ros2

# Fix missing pthread.h include in upstream source
RUN sed -i '1i #include <pthread.h>' \
      src/ldlidar_stl_ros2/ldlidar_driver/src/logger/log_module.cpp

RUN /bin/bash -c "\
    source /opt/ros/kilted/setup.bash && \
    colcon build \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
      --parallel-workers \$(nproc) \
    "

# =============================================================================
# Runtime — minimal image
# =============================================================================
FROM ros:kilted-ros-base AS runtime

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-kilted-rmw-cyclonedds-cpp \
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
