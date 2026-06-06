#!/bin/bash
set -e

# Source ROS2 (setup.bash uses unset variables internally)
set +u
source /opt/ros/kilted/setup.bash
# ublox_dgnss is built from source and installed to /opt/ublox_dgnss
if [ -f /opt/ublox_dgnss/setup.bash ]; then
  source /opt/ublox_dgnss/setup.bash
fi
if [ -f /opt/gnss_sidecar/setup.bash ]; then
  source /opt/gnss_sidecar/setup.bash
fi
set -u

exec "$@"
