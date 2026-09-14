#!/bin/bash
set -e

# Source ROS2 and workspace
source /opt/ros/lyrical/setup.bash
source /opt/ldlidar/setup.bash

exec "$@"
