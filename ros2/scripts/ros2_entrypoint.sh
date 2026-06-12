#!/bin/bash
# =============================================================================
# ros2_entrypoint.sh
#
# Container entrypoint. Sources ROS2 Kilted and the workspace overlay before
# exec'ing whatever command was passed as CMD (or via `docker run <cmd>`).
#
# NOTE: Make this script executable on the host before building:
#   chmod +x scripts/ros2_entrypoint.sh
# =============================================================================
set -e

# Source ROS2 base installation
# shellcheck source=/opt/ros/kilted/setup.bash
source /opt/ros/kilted/setup.bash

# Source the workspace overlay if it has been built (not present in dev before
# first colcon build, but always present in runtime/simulation images).
if [ -f /ros2_ws/install/setup.bash ]; then
    # shellcheck source=/ros2_ws/install/setup.bash
    source /ros2_ws/install/setup.bash
fi

exec "$@"
