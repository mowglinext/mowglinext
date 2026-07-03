#!/bin/bash
# =============================================================================
# post-create.sh — Runs after the devcontainer is created.
#
# Symlinks the monorepo's ROS2 packages into the workspace and resolves
# dependencies. It intentionally does not build the full workspace by default:
# optional coverage packages need Fields2Cover and should not block opening the
# devcontainer.
# =============================================================================
set -euo pipefail

echo "=== MowgliNext: Setting up ROS2 workspace ==="

# Source ROS2
# shellcheck source=/opt/ros/kilted/setup.bash
set +u
source /opt/ros/kilted/setup.bash
set -u

cd /ros2_ws

echo "Cleaning stale workspace artifacts..."

# Never let generated colcon artifacts inside src/ be discovered as packages.
rm -rf src/install src/build src/log

# Fields2Cover is an optional/full-stack CMake dependency. Do not build a
# workspace copy if one is present from older tests.
rm -f src/fields2cover src/Fields2Cover

if git -C /ros2_ws/src/mowglinext rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    echo "Ensuring git submodules are present..."
    git -C /ros2_ws/src/mowglinext submodule update --init --recursive
fi

SYNC_WORKSPACE_SCRIPT="/ros2_ws/src/mowglinext/ros2/scripts/sync_workspace_packages.sh"
"${SYNC_WORKSPACE_SCRIPT}"
mapfile -t BUILD_PATHS < <("${SYNC_WORKSPACE_SCRIPT}" --print-base-paths)

if [ "${#BUILD_PATHS[@]}" -eq 0 ]; then
    echo "No ROS2 package roots were linked into /ros2_ws/src."
    exit 1
fi

# ---------------------------------------------------------------------------
# ARM / x86 warning. Firmware builds (PlatformIO stm32 toolchain) assume
# ARM host support or cross-compile, GUI builds cross arches fine, ROS2
# builds fine on both but the real robot is ARM.
# ---------------------------------------------------------------------------
ARCH=$(uname -m)
if [ "$ARCH" != "aarch64" ] && [ "$ARCH" != "arm64" ]; then
    echo ""
    echo "WARNING: Dev container arch is ${ARCH} (not ARM)."
    echo "   ROS2 / GUI / sim build fine; firmware (pio run) won't match the"
    echo "   real robot without a cross-compile step."
    echo ""
fi

# ---------------------------------------------------------------------------
# Resolve rosdep dependencies
# ---------------------------------------------------------------------------
echo "Resolving rosdep dependencies..."
ROSDEP_SKIP_KEYS=()

# universal_gnss_ros2 normally comes from the vendored submodule linked by
# sync_workspace_packages.sh. If that submodule or an override checkout is
# absent, keep rosdep from trying to resolve it as a system package.
if [ ! -f "/ros2_ws/src/universal_gnss_ros2/package.xml" ]; then
    ROSDEP_SKIP_KEYS+=(universal_gnss_ros2)
fi

rosdep_args=(
    install
    --from-paths "${BUILD_PATHS[@]}"
    --ignore-src
    --rosdistro kilted
    -y
)

if [ "${#ROSDEP_SKIP_KEYS[@]}" -gt 0 ]; then
    echo "Skipping rosdep keys not linked into this workspace: ${ROSDEP_SKIP_KEYS[*]}"
    rosdep_args+=(--skip-keys "${ROSDEP_SKIP_KEYS[*]}")
fi

rosdep "${rosdep_args[@]}" || true

# ---------------------------------------------------------------------------
# Optional focused development build.
# ---------------------------------------------------------------------------
DEV_PACKAGES="${DEV_PACKAGES:-mowgli_interfaces mowgli_localization universal_gnss_ros2 mowgli_bringup}"
MOWGLI_POST_CREATE_BUILD="${MOWGLI_POST_CREATE_BUILD:-0}"

if [ "${MOWGLI_POST_CREATE_BUILD}" = "1" ]; then
    echo "Building focused development package set..."
    echo "  ${DEV_PACKAGES}"

    PACKAGES="${DEV_PACKAGES}" BUILD_TYPE=Release \
        /ros2_ws/src/mowglinext/ros2/scripts/build.sh

    # Source the built workspace
    # shellcheck disable=SC1091
    source install/setup.bash
else
    echo "Skipping ROS2 build during post-create."
    echo "Set MOWGLI_POST_CREATE_BUILD=1 to build the focused development set during container creation."
fi

# ---------------------------------------------------------------------------
# Optional: install pre-commit hooks if the repo has a config (no-op today).
# ---------------------------------------------------------------------------
if [ -f "src/mowglinext/.pre-commit-config.yaml" ]; then
    (cd src/mowglinext && pre-commit install) || true
fi

echo ""
echo "=== MowgliNext workspace ready ==="
echo ""
echo "Quick start (from ros2/ directory):"
echo "  make sim          # Launch headless simulation (Foxglove ws://localhost:8765)"
echo "  make e2e-test     # Run E2E validation (sim must be running)"
echo "  make build-dev    # Build the focused dev package set"
echo "  make build-full   # Build the full linked workspace"
echo "  make format       # Format C++ code"
echo "  make help         # Show all targets"
echo ""
echo "GUI work (from gui/ directory):"
echo "  go build -o openmower-gui          # Build the Go backend"
echo "  cd web && yarn install && yarn dev # Start the React frontend"
echo ""
