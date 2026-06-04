#!/bin/bash
# =============================================================================
# start_dev_sim.sh
#
# Dev simulation entrypoint: builds the workspace from mounted sources if
# needed, then starts VNC + Gazebo GUI + full Nav2 stack.
#
# Usage (via docker compose):
#   docker compose up dev-sim
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

VNC_PORT="${VNC_PORT:-5901}"
NOVNC_PORT="${NOVNC_PORT:-6080}"
VNC_RESOLUTION="${VNC_RESOLUTION:-1280x720}"

set +u
source /opt/ros/kilted/setup.bash
set -u

# ---- Build workspace if install tree is empty or stale ----------------------
if [ ! -f /ros2_ws/install/setup.bash ]; then
    echo "=== First run: building full workspace from mounted sources ==="
    BUILD_TYPE=Release "${SCRIPT_DIR}/build.sh"
    echo "=== Build complete ==="
fi

set +u
source /ros2_ws/install/setup.bash
set -u

# ---- Start VNC server -------------------------------------------------------
echo "=== Starting VNC server on :1 (${VNC_RESOLUTION}) ==="

rm -f /tmp/.X1-lock /tmp/.X11-unix/X1

vncserver :1 \
    -geometry "${VNC_RESOLUTION}" \
    -depth 24 \
    -SecurityTypes None \
    --I-KNOW-THIS-IS-INSECURE \
    2>/dev/null

export DISPLAY=:1

openbox --sm-disable &

echo "=== Starting noVNC web proxy on port ${NOVNC_PORT} ==="
websockify \
    --web /usr/share/novnc/ \
    "${NOVNC_PORT}" \
    "localhost:${VNC_PORT}" &

echo ""
echo "========================================================"
echo "  DEV SIMULATION"
echo ""
echo "  Gazebo GUI:      http://localhost:${NOVNC_PORT}/vnc.html"
echo "  Foxglove Studio: ws://localhost:8765"
echo ""
echo "  Rebuild:  make dev-build"
echo "  Rebuild:  make dev-build-pkg PKG=<package>"
echo "  Restart:  make dev-restart"
echo "  Shell:    make dev-shell"
echo "========================================================"
echo ""

# Launch file is configurable via LAUNCH_FILE env var.
# Default: sim_full_system.launch.py with Gazebo GUI.
LAUNCH_FILE="${LAUNCH_FILE:-sim_full_system.launch.py}"
LAUNCH_ARGS="${LAUNCH_ARGS:-headless:=false use_rviz:=false}"

echo "  Launch: ros2 launch mowgli_bringup ${LAUNCH_FILE} ${LAUNCH_ARGS}"
echo ""

# shellcheck disable=SC2086
exec ros2 launch mowgli_bringup ${LAUNCH_FILE} ${LAUNCH_ARGS}
