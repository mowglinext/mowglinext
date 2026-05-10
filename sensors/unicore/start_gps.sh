#!/bin/bash
# =============================================================================
# UM982 GNSS driver startup + NTRIP client
#
# Launches:
#   1. um982_node         — UM982 GNSS driver and RTCM injector on /dev/gps,
#                           publishes /gps/fix, /gps/azimuth, /gps/diagnostics
#                           and subscribes to /ntrip_client/rtcm
#   2. ntrip_client_node  — NTRIP caster client publishing /ntrip_client/rtcm
#
# Reads gps_port / gps_baudrate / ntrip_* from /config/mowgli_robot.yaml.
# Bind-mount docker/config/mowgli to /config (see docker-compose.unicore.yaml).
#
# We deliberately do NOT use `ros2 launch mowgli_unicore_gnss
# um982_launch.py` here: the upstream launch file accepts no arguments and
# always loads the package-share config/um982.yaml (port=/dev/gps,
# baudrate=921600, frame_id=gps). Driving the node via `ros2 run` with
# explicit -p overrides is the only way to honour the operator's
# gps_baudrate from mowgli_robot.yaml.
#
# NTRIP defaults override:
#   * use_https=false  — upstream default is true, but the public Centipede
#     caster (crtk.net:2101) speaks plain HTTP/RTCM3. Without this override
#     the connection silently fails on TLS.
# =============================================================================
set -euo pipefail

CONFIG="/config/mowgli_robot.yaml"
GPS_PID=""
NTRIP_PID=""

if [ ! -f "$CONFIG" ]; then
  echo "[start_gps.sh] ERROR: $CONFIG not found. Bind-mount docker/config/mowgli to /config."
  exit 1
fi

parse_yaml() {
  awk -F: -v key="$1" '
    $1 ~ "^[[:space:]]*" key "[[:space:]]*$" {
      value = substr($0, index($0, ":") + 1)
      sub(/^[[:space:]]+/, "", value)
      sub(/[[:space:]]+$/, "", value)
      gsub(/^["'"'"']|["'"'"']$/, "", value)
      print value
      exit
    }
  ' "$CONFIG"
}

is_truthy() {
  case "${1,,}" in
    true|1|yes|y|on)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

cleanup() {
  [ -n "$NTRIP_PID" ] && kill "$NTRIP_PID" 2>/dev/null || true
  [ -n "$GPS_PID" ] && kill "$GPS_PID" 2>/dev/null || true
}

trap cleanup EXIT INT TERM

GPS_PORT=$(parse_yaml gps_port)
GPS_PORT="${GPS_PORT:-/dev/gps}"
GPS_BAUD=$(parse_yaml gps_baudrate)
GPS_BAUD="${GPS_BAUD:-921600}"
UNICORE_TARGET_BAUD="${UNICORE_TARGET_BAUD:-921600}"

# Optional one-shot UM98x config blast — sends MODE ROVER + LOG
# directives via /configure_receiver.sh. SAVECONFIG persists in NVRAM,
# so this is a no-op on a receiver that's already correctly configured.
# Default true so the "just installed, never configured" case works
# out of the box; set to false in mowgli_robot.yaml after you've run
# Unicore Setup tool yourself.
UNICORE_AUTO_CONFIGURE=$(parse_yaml unicore_auto_configure)
UNICORE_AUTO_CONFIGURE="${UNICORE_AUTO_CONFIGURE:-true}"

NTRIP_ENABLED=$(parse_yaml ntrip_enabled)
NTRIP_ENABLED="${NTRIP_ENABLED:-false}"
NTRIP_HOST=$(parse_yaml ntrip_host)
NTRIP_PORT=$(parse_yaml ntrip_port)
NTRIP_USER=$(parse_yaml ntrip_user)
NTRIP_PASSWORD=$(parse_yaml ntrip_password)
NTRIP_MOUNTPOINT=$(parse_yaml ntrip_mountpoint)

if is_truthy "$UNICORE_AUTO_CONFIGURE"; then
  if [ -x /configure_receiver.sh ]; then
    if /configure_receiver.sh "$GPS_PORT" "$GPS_BAUD"; then
      GPS_BAUD="$UNICORE_TARGET_BAUD"
      echo "[start_gps.sh] Receiver auto-config succeeded; using baud=${GPS_BAUD} for runtime."
    else
      echo "[start_gps.sh] WARN: receiver auto-config failed — continuing anyway"
    fi
  else
    echo "[start_gps.sh] WARN: /configure_receiver.sh missing — skipping auto-config"
  fi
fi

echo "[start_gps.sh] Launching Unicore UM982 GNSS driver: port=${GPS_PORT} baud=${GPS_BAUD}"
# diagnostics_topic = /diagnostics — the ROS2 standard aggregator topic
# (matches gps_health_aggregator.py in the ublox path). The GUI's
# Diagnostics panel reads from /diagnostics; namespacing under
# /gps/diagnostics would hide the unicore status from the dashboard.
ros2 run mowgli_unicore_gnss um982_node --ros-args \
  -p "port:=${GPS_PORT}" \
  -p "baudrate:=${GPS_BAUD}" \
  -p "frame_id:=gps_link" \
  -p "fix_topic:=/gps/fix" \
  -p "heading_topic:=/gps/azimuth" \
  -p "diagnostics_topic:=/diagnostics" \
  -p "rtcm_topic:=/ntrip_client/rtcm" &
GPS_PID=$!

if is_truthy "$NTRIP_ENABLED"; then
  echo "[start_gps.sh] NTRIP enabled: ${NTRIP_HOST}:${NTRIP_PORT}/${NTRIP_MOUNTPOINT} (HTTP)"
  sleep 3
  ros2 run ntrip_client_node ntrip_client_node --ros-args \
    -p "use_https:=false" \
    -p "host:=${NTRIP_HOST}" \
    -p "port:=${NTRIP_PORT}" \
    -p "mountpoint:=${NTRIP_MOUNTPOINT}" \
    -p "username:=${NTRIP_USER}" \
    -p "password:=${NTRIP_PASSWORD}" &
  NTRIP_PID=$!
fi

wait -n || true
wait
