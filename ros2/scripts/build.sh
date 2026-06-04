#!/bin/bash
# =============================================================================
# build.sh — Workspace build helper
#
# Intended to be run inside the dev or build container where the source tree
# is available at /ros2_ws/src.
#
# Environment variables (all optional):
#   BUILD_TYPE   — CMake build type (default: Release)
#   PACKAGES     — Space-separated list of packages to build (default: all)
#
# Examples:
#   ./scripts/build.sh
#   BUILD_TYPE=Debug ./scripts/build.sh
#   PACKAGES="mowgli_behavior mowgli_nav2_plugins" ./scripts/build.sh
#
# NOTE: Make this script executable on the host before use:
#   chmod +x scripts/build.sh
# =============================================================================
set -euo pipefail

WORKSPACE=/ros2_ws
BUILD_TYPE="${BUILD_TYPE:-Release}"
PARALLEL_WORKERS=$(nproc)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SYNC_WORKSPACE_SCRIPT="${SCRIPT_DIR}/sync_workspace_packages.sh"

cd "${WORKSPACE}"

# shellcheck source=/opt/ros/kilted/setup.bash
set +u
source /opt/ros/${ROS_DISTRO:-kilted}/setup.bash
set -u

mapfile -t BUILD_PATHS < <("${SYNC_WORKSPACE_SCRIPT}" --print-base-paths)

if [ "${#BUILD_PATHS[@]}" -eq 0 ]; then
    echo "ERROR: No ROS2 package roots were linked into ${WORKSPACE}/src." >&2
    exit 1
fi

echo "======================================================="
echo " Mowgli ROS2 workspace build"
echo " Build type  : ${BUILD_TYPE}"
echo " Workers     : ${PARALLEL_WORKERS}"
echo " Workspace   : ${WORKSPACE}"
if [ -n "${PACKAGES}" ]; then
    echo " Packages    : ${PACKAGES}"
fi
echo "======================================================="

# Build selected packages or the entire workspace
if [ -n "${PACKAGES}" ]; then
    # shellcheck disable=SC2086
    colcon build \
        --base-paths "${BUILD_PATHS[@]}" \
        --packages-select ${PACKAGES} \
        --cmake-args -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
        --parallel-workers "${PARALLEL_WORKERS}" \
        --symlink-install \
        --event-handlers console_cohesion+
else
    colcon build \
        --base-paths "${BUILD_PATHS[@]}" \
        --cmake-args -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
        --parallel-workers "${PARALLEL_WORKERS}" \
        --symlink-install \
        --event-handlers console_cohesion+
fi

echo ""
echo "Build complete. Source the workspace overlay with:"
echo "  source ${WORKSPACE}/install/setup.bash"
