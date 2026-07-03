#!/bin/bash
set -e

# Source ROS2 (setup.bash uses unset variables internally)
set +u
source /opt/ros/kilted/setup.bash
if [ -f /opt/gnss_sidecar/setup.bash ]; then
  source /opt/gnss_sidecar/setup.bash
fi
set -u

exec "$@"
