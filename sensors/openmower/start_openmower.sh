#!/bin/bash
# Start the OpenMower hardware bridge. Device paths and the ESC type come
# from the OPENMOWER_* environment written by the installer into docker/.env;
# per-robot kinematics come from the mounted sparse /config/mowgli_robot.yaml.
set -euo pipefail

set +u
source /opt/ros/lyrical/setup.bash
source /opt/openmower_bridge/setup.bash
set -u

: "${OPENMOWER_LL_PORT:=/dev/ttyAMA0}"
: "${OPENMOWER_XESC_TYPE:=xesc_mini}"
: "${OPENMOWER_XESC_LEFT_PORT:=/dev/ttyAMA5}"
: "${OPENMOWER_XESC_RIGHT_PORT:=/dev/ttyAMA3}"
: "${OPENMOWER_XESC_MOW_PORT:=/dev/ttyAMA4}"
export OPENMOWER_LL_PORT OPENMOWER_XESC_TYPE OPENMOWER_XESC_LEFT_PORT \
  OPENMOWER_XESC_RIGHT_PORT OPENMOWER_XESC_MOW_PORT

case "$OPENMOWER_XESC_TYPE" in
  xesc_mini|xesc_2040) ;;
  *)
    echo "[openmower] OPENMOWER_XESC_TYPE='$OPENMOWER_XESC_TYPE' is not xesc_mini|xesc_2040" >&2
    exit 1
    ;;
esac

for dev in "$OPENMOWER_LL_PORT" "$OPENMOWER_XESC_LEFT_PORT" "$OPENMOWER_XESC_RIGHT_PORT" \
           "$OPENMOWER_XESC_MOW_PORT"; do
  if [ ! -e "$dev" ]; then
    echo "[openmower] WARNING: $dev is absent right now; the bridge keeps retrying" >&2
  fi
done

echo "[openmower] LowLevel=$OPENMOWER_LL_PORT xESC=$OPENMOWER_XESC_TYPE" \
     "L=$OPENMOWER_XESC_LEFT_PORT R=$OPENMOWER_XESC_RIGHT_PORT M=$OPENMOWER_XESC_MOW_PORT"

exec ros2 launch mowgli_openmower_bridge openmower_bridge.launch.py
