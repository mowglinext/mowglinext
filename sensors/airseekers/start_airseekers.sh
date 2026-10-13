#!/bin/bash
# Start the Airseekers Tron hardware bridge (+ the UM960 GNSS node unless the
# robot config says gnss_stack: universal). Device paths come from the
# mounted sparse /config/mowgli_robot.yaml (GUI), then AIRSEEKERS_* from
# docker/.env, then the template + backend defaults.
set -euo pipefail

set +u
source /opt/ros/lyrical/setup.bash
source /opt/airseekers_bridge/setup.bash
set -u

for dev in "${AIRSEEKERS_MCU_PORT:-/dev/serial_mower}" "${AIRSEEKERS_IMU_PORT:-/dev/serial_imu}" \
           /dev/serial_rtk; do
  if [ ! -e "$dev" ]; then
    echo "[airseekers] WARNING: $dev is absent right now; the bridge keeps retrying" >&2
  fi
done

echo "[airseekers] MCU=${AIRSEEKERS_MCU_PORT:-config} IMU=${AIRSEEKERS_IMU_PORT:-config}" \
     "config=${AIRSEEKERS_ROBOT_CONFIG:-/config/mowgli_robot.yaml}"

exec ros2 launch mowgli_airseekers_bridge airseekers_bridge.launch.py
