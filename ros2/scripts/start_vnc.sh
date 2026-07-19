#!/bin/bash
# =============================================================================
# start_vnc.sh
#
# Starts a VNC server + noVNC web proxy inside the simulation container.
# Access the simulator GUI from your browser at http://localhost:6080/vnc.html
#
# This script is meant to be run inside the Docker container, typically via:
#   docker exec mowgli-sim /ros2_ws/scripts/start_vnc.sh
#
# Or as an alternative entrypoint for GUI mode:
#   docker run -p 6080:6080 -p 8765:8765 mowgli-ros2-sim /ros2_ws/scripts/start_vnc.sh
# =============================================================================
set -e

VNC_PORT="${VNC_PORT:-5901}"
NOVNC_PORT="${NOVNC_PORT:-6080}"
VNC_RESOLUTION="${VNC_RESOLUTION:-1280x720}"

echo "=== Starting VNC server on :1 (${VNC_RESOLUTION}) ==="

# Remove stale lock files
rm -f /tmp/.X1-lock /tmp/.X11-unix/X1

# Start VNC server (no password for local dev)
vncserver :1 \
    -geometry "${VNC_RESOLUTION}" \
    -depth 24 \
    -SecurityTypes None \
    --I-KNOW-THIS-IS-INSECURE \
    2>/dev/null

export DISPLAY=:1

# Start a minimal window manager
openbox --sm-disable &

echo "=== Starting noVNC web proxy on port ${NOVNC_PORT} ==="
websockify \
    --web /usr/share/novnc/ \
    "${NOVNC_PORT}" \
    "localhost:${VNC_PORT}" &

echo ""
echo "========================================================"
echo "  Simulator GUI available at:"
echo "    http://localhost:${NOVNC_PORT}/vnc.html"
echo ""
echo "  Foxglove Studio connects to:"
echo "    ws://localhost:8765"
echo "========================================================"
echo ""

# Source ROS2 and workspace
source /opt/ros/kilted/setup.bash
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

# Launch the full simulation (not headless). This script is the sole
# simulation launcher for the Compose GUI service.
exec ros2 launch mowgli_bringup sim_full_system.launch.py \
    headless:=false \
    use_rviz:=false
