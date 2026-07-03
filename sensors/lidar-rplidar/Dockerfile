# =============================================================================
# Slamtec RPLiDAR driver for ROS2 Jazzy
#
# Supports RPLiDAR A1, A2, A3, S1, S2, S3, C1.
# Publishes sensor_msgs/LaserScan on /scan with frame_id=lidar_link.
#
# To use this instead of the default LD19 driver, set in your .env:
#   LIDAR_IMAGE=ghcr.io/mowglinext/mowglinext/lidar-rplidar:main
# or override the build context in docker-compose.override.yml.
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

RUN git clone --depth 1 -b ros2 \
      https://github.com/Slamtec/rplidar_ros.git \
      src/rplidar_ros

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
CMD ["ros2", "launch", "rplidar_ros", "rplidar_a2m8_launch.py", \
     "frame_id:=lidar_link", \
     "topic_name:=/scan", \
     "serial_port:=/dev/ttyUSB0", \
     "serial_baudrate:=115200"]
