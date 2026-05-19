#!/bin/bash
# =============================================================================
# post-create.sh — Runs after the devcontainer is created.
#
# Symlinks the monorepo's ROS2 packages into the workspace, resolves
# dependencies, and builds the full workspace.
# =============================================================================
set -e

echo "=== MowgliNext: Setting up ROS2 workspace ==="

# Source ROS2
source /opt/ros/kilted/setup.bash

cd /ros2_ws

echo "Cleaning stale workspace artifacts..."

# Never let generated colcon artifacts inside src/ be discovered as packages.
rm -rf src/install src/build src/log

# Fields2Cover is installed as a system CMake dependency in the devcontainer.
# Do not build a workspace copy if one is present from older tests.
rm -f src/fields2cover src/Fields2Cover

# ---------------------------------------------------------------------------
# Symlink ROS2 packages from monorepo into workspace src/
#
# The workspace is /ros2_ws, the monorepo is bind-mounted at
# /ros2_ws/src/mowglinext. We symlink each mowgli_* package into src/
# so colcon discovers them at the workspace root, plus fusion_graph.
# ---------------------------------------------------------------------------

echo "Linking ROS2 packages into workspace..."

mkdir -p src

for pkg_dir in src/mowglinext/ros2/src/mowgli_*/; do
    [ -d "$pkg_dir" ] || continue
    pkg_name=$(basename "$pkg_dir")
    ln -sfn "/ros2_ws/$pkg_dir" "src/$pkg_name"
    echo "  Linked: $pkg_name"
done

# OpenNav coverage packages.
for pkg_dir in src/mowglinext/ros2/src/opennav_coverage/*/; do
    if [ -f "${pkg_dir}/package.xml" ]; then
        pkg_name=$(basename "$pkg_dir")
        ln -sfn "/ros2_ws/$pkg_dir" "src/$pkg_name"
        echo "  Linked: $pkg_name"
    fi
done

if [ -d "src/mowglinext/ros2/src/fusion_graph" ]; then
    ln -sfn "/ros2_ws/src/mowglinext/ros2/src/fusion_graph" "src/fusion_graph"
    echo "  Linked: fusion_graph"
fi

# ---------------------------------------------------------------------------
# ARM / x86 warning. Firmware builds (PlatformIO stm32 toolchain) assume
# ARM host support or cross-compile, GUI builds cross arches fine, ROS2
# builds fine on both but the real robot is ARM.
# ---------------------------------------------------------------------------
ARCH=$(uname -m)
if [ "$ARCH" != "aarch64" ] && [ "$ARCH" != "arm64" ]; then
    echo ""
    echo "⚠  Dev container arch is ${ARCH} (not ARM)."
    echo "   ROS2 / GUI / sim build fine; firmware (pio run) won't match the"
    echo "   real robot without a cross-compile step."
    echo ""
fi

# ---------------------------------------------------------------------------
# Resolve rosdep dependencies
# ---------------------------------------------------------------------------
echo "Resolving rosdep dependencies..."
rosdep install \
    --from-paths src \
    --ignore-src \
    --rosdistro kilted \
    -y || true

# ---------------------------------------------------------------------------
# Build the workspace.
# ---------------------------------------------------------------------------
echo "Building workspace (this may take a few minutes on first run)..."

# unicore_gnss is a stub package.xml under sensors/unicore/ with no CMakeLists.
# fields2cover is provided as a system dependency, not built as a ROS package.
colcon build \
    --packages-ignore unicore_gnss fields2cover \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=OFF \
    --parallel-workers "$(nproc)" \
    --symlink-install \
    --event-handlers console_cohesion+

# Source the built workspace
# shellcheck disable=SC1091
source install/setup.bash

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
echo "  make build        # Rebuild after code changes"
echo "  make format       # Format C++ code"
echo "  make help         # Show all targets"
echo ""
echo "GUI work (from gui/ directory):"
echo "  go build -o openmower-gui          # Build the Go backend"
echo "  cd web && yarn install && yarn dev # Start the React frontend"
echo ""