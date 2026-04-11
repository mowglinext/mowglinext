# =============================================================================
# MAVROS for ROS2 Jazzy
#
# MAVLink bridge for ArduPilot/PX4 flight controllers.
# Publishes IMU, GPS, and other MAVLink data to ROS2 topics.
# =============================================================================
FROM ros:kilted-ros-base

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-kilted-mavros \
    ros-kilted-mavros-msgs \
    ros-kilted-mavros-extras \
    ros-kilted-rmw-cyclonedds-cpp \
    wget \
 && rm -rf /var/lib/apt/lists/*

# Install GeographicLib datasets required by MAVROS
RUN wget -q https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh -O /tmp/install_geo.sh \
 && chmod +x /tmp/install_geo.sh \
 && /tmp/install_geo.sh \
 && rm /tmp/install_geo.sh

COPY ros2_entrypoint.sh /ros2_entrypoint.sh
RUN chmod +x /ros2_entrypoint.sh

ENTRYPOINT ["/ros2_entrypoint.sh"]
CMD ["ros2", "launch", "mavros", "px4.launch", \
     "fcu_url:=/dev/ttyUSB0:921600"]
