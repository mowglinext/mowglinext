#!/bin/bash
# =============================================================================
# test.sh — Workspace test runner
#
# Runs all colcon tests and prints a verbose result summary. Exits non-zero if
# any test fails, making it safe to use in CI pipelines.
#
# Environment variables (all optional):
#   PACKAGES   — Space-separated list of packages to test (default: all)
#
# Examples:
#   ./scripts/test.sh
#   PACKAGES="mowgli_behavior" ./scripts/test.sh
#
# NOTE: Make this script executable on the host before use:
#   chmod +x scripts/test.sh
# =============================================================================
set -euo pipefail

WORKSPACE=/ros2_ws
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SYNC_WORKSPACE_SCRIPT="${SCRIPT_DIR}/sync_workspace_packages.sh"

cd "${WORKSPACE}"

# shellcheck source=/opt/ros/kilted/setup.bash
set +u
source /opt/ros/${ROS_DISTRO:-kilted}/setup.bash
set -u

# The workspace must be built before running tests
if [ ! -f "${WORKSPACE}/install/setup.bash" ]; then
    echo "ERROR: No install/ directory found. Run build.sh first." >&2
    exit 1
fi

# shellcheck source=/ros2_ws/install/setup.bash
set +u
source "${WORKSPACE}/install/setup.bash"
set -u

mapfile -t BUILD_PATHS < <("${SYNC_WORKSPACE_SCRIPT}" --print-base-paths)

if [ "${#BUILD_PATHS[@]}" -eq 0 ]; then
    echo "ERROR: No ROS2 package roots were linked into ${WORKSPACE}/src." >&2
    exit 1
fi

echo "======================================================="
echo " Mowgli ROS2 workspace tests"
echo " Workspace : ${WORKSPACE}"
if [ -n "${PACKAGES}" ]; then
    echo " Packages  : ${PACKAGES}"
fi
echo "======================================================="

# Run tests for selected packages or the entire workspace
if [ -n "${PACKAGES}" ]; then
    # shellcheck disable=SC2086
    colcon test \
        --base-paths "${BUILD_PATHS[@]}" \
        --packages-select ${PACKAGES} \
        --return-code-on-test-failure \
        --event-handlers console_cohesion+
else
    colcon test \
        --base-paths "${BUILD_PATHS[@]}" \
        --return-code-on-test-failure \
        --event-handlers console_cohesion+
fi

# Always print full result table regardless of pass/fail
echo ""
echo "======================================================="
echo " Test results"
echo "======================================================="
colcon test-result --verbose

echo ""
echo "All tests passed."
