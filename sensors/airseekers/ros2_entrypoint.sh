#!/bin/bash
set -e

# Source ROS2 (setup.bash uses unset variables internally)
set +u
source /opt/ros/lyrical/setup.bash
if [ -f /opt/airseekers_bridge/setup.bash ]; then
  source /opt/airseekers_bridge/setup.bash
fi
set -u

exec "$@"
