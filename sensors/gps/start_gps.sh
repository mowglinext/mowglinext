#!/bin/bash
# =============================================================================
# Universal GNSS sidecar startup.
#
# Public outputs:
#   /gps/fix         sensor_msgs/NavSatFix
#   /gps/status      mowgli_interfaces/GnssStatus
#   /diagnostics     diagnostic_msgs/DiagnosticArray
#   /rtcm            rtcm_msgs/Message
# =============================================================================
set -euo pipefail

CONFIG="/config/mowgli_robot.yaml"

if [ ! -f "$CONFIG" ]; then
  echo "[start_gps.sh] ERROR: $CONFIG not found. Bind-mount config/mowgli/ to /config."
  exit 1
fi

parse_yaml() {
  # Tolerate a missing key: under `set -euo pipefail` a non-matching grep makes
  # the pipeline exit non-zero, and a bare `NTRIP_HOST=$(parse_yaml ...)`
  # assignment then aborts the whole script BEFORE the `${VAR:-default}`
  # fallbacks below can apply. `|| true` keeps it returning empty + exit 0.
  { grep -E "^\s+${1}:" "$CONFIG" | head -1 | sed 's/.*:\s*//' | tr -d '"' | tr -d "'"; } || true
}

normalize_lower() {
  printf '%s' "${1:-}" | tr '[:upper:]' '[:lower:]'
}

normalize_bool() {
  case "$(normalize_lower "${1:-}")" in
    1|true|yes|y|on)
      printf 'true\n'
      ;;
    *)
      printf 'false\n'
      ;;
  esac
}

resolve_receiver_family() {
  if [ -n "${GNSS_RECEIVER_FAMILY:-}" ]; then
    printf '%s\n' "$(normalize_lower "${GNSS_RECEIVER_FAMILY}")"
    return 0
  fi

  local yaml_family
  yaml_family="$(parse_yaml gnss_receiver_family)"
  if [ -n "$yaml_family" ]; then
    printf '%s\n' "$(normalize_lower "$yaml_family")"
    return 0
  fi

  printf 'auto\n'
}

resolve_transport() {
  printf '%s\n' "${GNSS_TRANSPORT:-serial}"
}

resolve_serial_device() {
  if [ -n "${GNSS_SERIAL_DEVICE:-}" ]; then
    printf '%s\n' "$GNSS_SERIAL_DEVICE"
    return 0
  fi

  local yaml_serial_device
  yaml_serial_device="$(parse_yaml gnss_serial_device)"
  if [ -n "$yaml_serial_device" ]; then
    printf '%s\n' "$yaml_serial_device"
    return 0
  fi

  printf '/dev/ttyAMA4\n'
}

resolve_serial_baud() {
  if [ -n "${GNSS_SERIAL_BAUD:-}" ]; then
    printf '%s\n' "$GNSS_SERIAL_BAUD"
    return 0
  fi

  local yaml_serial_baud
  yaml_serial_baud="$(parse_yaml gnss_serial_baud)"
  if [ -n "$yaml_serial_baud" ]; then
    printf '%s\n' "$yaml_serial_baud"
    return 0
  fi

  printf '921600\n'
}

resolve_ntrip_enabled() {
  if [ -n "${GNSS_NTRIP_ENABLED:-}" ]; then
    normalize_bool "$GNSS_NTRIP_ENABLED"
    return 0
  fi

  normalize_bool "$(parse_yaml ntrip_enabled)"
}

resolve_ntrip_host() {
  if [ -n "${GNSS_NTRIP_HOST:-}" ]; then
    printf '%s\n' "$GNSS_NTRIP_HOST"
    return 0
  fi

  local host
  host="$(parse_yaml ntrip_host)"
  printf '%s\n' "${host:-crtk.net}"
}

resolve_ntrip_port() {
  if [ -n "${GNSS_NTRIP_PORT:-}" ]; then
    printf '%s\n' "$GNSS_NTRIP_PORT"
    return 0
  fi

  local port
  port="$(parse_yaml ntrip_port)"
  printf '%s\n' "${port:-2101}"
}

resolve_ntrip_username() {
  if [ -n "${GNSS_NTRIP_USERNAME:-}" ]; then
    printf '%s\n' "$GNSS_NTRIP_USERNAME"
    return 0
  fi

  local username
  username="$(parse_yaml ntrip_user)"
  printf '%s\n' "${username:-centipede}"
}

resolve_ntrip_password() {
  if [ -n "${GNSS_NTRIP_PASSWORD:-}" ]; then
    printf '%s\n' "$GNSS_NTRIP_PASSWORD"
    return 0
  fi

  local password
  password="$(parse_yaml ntrip_password)"
  printf '%s\n' "${password:-centipede}"
}

resolve_ntrip_mountpoint() {
  if [ -n "${GNSS_NTRIP_MOUNTPOINT:-}" ]; then
    printf '%s\n' "$GNSS_NTRIP_MOUNTPOINT"
    return 0
  fi

  local mountpoint
  mountpoint="$(parse_yaml ntrip_mountpoint)"
  printf '%s\n' "${mountpoint:-NEAR}"
}

resolve_ntrip_gga_enabled() {
  if [ -n "${GNSS_NTRIP_GGA_ENABLED:-}" ]; then
    normalize_bool "$GNSS_NTRIP_GGA_ENABLED"
    return 0
  fi

  local mountpoint
  mountpoint="$(resolve_ntrip_mountpoint)"
  case "$(normalize_lower "$mountpoint")" in
    near*)
      printf 'true\n'
      ;;
    *)
      printf 'false\n'
      ;;
  esac
}

resolve_ntrip_gga_interval_s() {
  printf '%s\n' "${GNSS_NTRIP_GGA_INTERVAL_S:-10}"
}

resolve_frame_id() {
  printf '%s\n' "${GNSS_FRAME_ID:-gps_link}"
}

if [ "$(normalize_lower "${GNSS_STACK:-universal}")" = "disabled" ]; then
  echo "[start_gps.sh] ERROR: GNSS_STACK=disabled is not valid for the GNSS sidecar."
  exit 1
fi

set +u
source /opt/ros/kilted/setup.bash
if [ -f /opt/gnss_sidecar/setup.bash ]; then
  source /opt/gnss_sidecar/setup.bash
fi
set -u

if [ ! -f /opt/gnss_sidecar/setup.bash ]; then
  echo "[start_gps.sh] ERROR: /opt/gnss_sidecar/setup.bash not found; Universal GNSS overlay missing."
  exit 1
fi

GPS_PID=""
NTRIP_PID=""
UNIVERSAL_BRIDGE_PID=""

cleanup() {
  [ -n "$GPS_PID" ] && kill "$GPS_PID" 2>/dev/null || true
  [ -n "$NTRIP_PID" ] && kill "$NTRIP_PID" 2>/dev/null || true
  [ -n "$UNIVERSAL_BRIDGE_PID" ] && kill "$UNIVERSAL_BRIDGE_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

internal_status_topic="/_gps_internal/universal/status"
internal_rtcm_topic="/_gps_internal/universal/rtcm"
receiver_family="$(resolve_receiver_family)"
transport="$(resolve_transport)"
serial_device="$(resolve_serial_device)"
serial_baud="$(resolve_serial_baud)"
frame_id="$(resolve_frame_id)"
ntrip_enabled="$(resolve_ntrip_enabled)"
ntrip_host="$(resolve_ntrip_host)"
ntrip_port="$(resolve_ntrip_port)"
ntrip_user="$(resolve_ntrip_username)"
ntrip_password="$(resolve_ntrip_password)"
ntrip_mountpoint="$(resolve_ntrip_mountpoint)"
ntrip_gga_enabled="$(resolve_ntrip_gga_enabled)"
ntrip_gga_interval_s="$(resolve_ntrip_gga_interval_s)"

echo "[start_gps.sh] Runtime=universal receiver_family=${receiver_family} transport=${transport} device=${serial_device} baud=${serial_baud}"

ros2 run universal_gnss_ros2 receiver_node --ros-args \
  -p "receiver_family:=${receiver_family}" \
  -p "transport:=${transport}" \
  -p "serial_device:=${serial_device}" \
  -p "serial_baud:=${serial_baud}" \
  -p "publish_rate_hz:=5.0" \
  -p "frame_id:=${frame_id}" \
  -r status:=${internal_status_topic} \
  -r diagnostics:=/diagnostics \
  -r fix:=/gps/fix \
  -r rtcm:=${internal_rtcm_topic} &
GPS_PID=$!

python3 /universal_gnss_topic_bridge.py --ros-args \
  -p "backend:=universal" \
  -p "receiver_family:=${receiver_family}" \
  -p "frame_id:=${frame_id}" \
  -p "input_status_topic:=${internal_status_topic}" \
  -p "output_status_topic:=/gps/status" \
  -p "input_rtcm_topic:=${internal_rtcm_topic}" \
  -p "output_rtcm_topic:=/rtcm" &
UNIVERSAL_BRIDGE_PID=$!

if [ "$ntrip_enabled" = "true" ]; then
  echo "[start_gps.sh] Runtime=universal NTRIP enabled: ${ntrip_host}:${ntrip_port}/${ntrip_mountpoint}"
  sleep 3
  ros2 run universal_gnss_ros2 ntrip_node --ros-args \
    -p "caster_host:=${ntrip_host}" \
    -p "caster_port:=${ntrip_port}" \
    -p "mountpoint:=${ntrip_mountpoint}" \
    -p "username:=${ntrip_user}" \
    -p "password:=${ntrip_password}" \
    -p "gga_enabled:=${ntrip_gga_enabled}" \
    -p "gga_interval_s:=${ntrip_gga_interval_s}" \
    -p "tls_enabled:=false" \
    -r status:=${internal_status_topic} \
    -r diagnostics:=/diagnostics \
    -r rtcm:=${internal_rtcm_topic} &
  NTRIP_PID=$!
fi

wait -n || true
cleanup
wait
