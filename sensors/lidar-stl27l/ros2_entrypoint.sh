#!/bin/bash
set -e

# Source ROS2 and workspace
source /opt/ros/lyrical/setup.bash
source /ros2_ws/install/setup.bash

exec "$@"
