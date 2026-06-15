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

# Source the patched nav2_mppi_controller overlay (arc-length path progress,
# upstream PR #6055 backport — see ros2/patches/README.md). Sourced right after
# the ROS base so it PREPENDS to AMENT_PREFIX_PATH/LD_LIBRARY_PATH and shadows
# the apt-installed system nav2_mppi_controller 1.4.2, whose Euclidean
# findPathFurthestReachedPoint() makes MPPI cut corners at coverage-path
# reversals. Guarded so the dev image (no /opt/nav2_mppi_patched) still starts.
# REMOVE this block once Kilted ships the fix (see ros2/patches/README.md).
if [ -f /opt/nav2_mppi_patched/setup.bash ]; then
    # shellcheck source=/opt/nav2_mppi_patched/setup.bash
    source /opt/nav2_mppi_patched/setup.bash
fi

# Source the ublox interface overlay (ublox_ubx_msgs + ublox_ubx_interfaces),
# built from the cedbossneo/ublox_dgnss fork into /opt/ublox_msgs. These are
# present only in runtime/simulation images so foxglove_bridge can resolve the
# schemas of the GPS driver's /ubx_* topics + ublox_dgnss services (the driver
# itself runs in the separate sensors/gps image). Guarded so the dev image,
# which has no /opt/ublox_msgs, still starts.
if [ -f /opt/ublox_msgs/setup.bash ]; then
    # shellcheck source=/opt/ublox_msgs/setup.bash
    source /opt/ublox_msgs/setup.bash
fi

# Source the workspace overlay if it has been built (not present in dev before
# first colcon build, but always present in runtime/simulation images).
if [ -f /ros2_ws/install/setup.bash ]; then
    # shellcheck source=/ros2_ws/install/setup.bash
    source /ros2_ws/install/setup.bash
fi

exec "$@"
