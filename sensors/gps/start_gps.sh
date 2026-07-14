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

CONFIG="${GNSS_CONFIG_PATH:-/config/mowgli_robot.yaml}"
ROS_SETUP_BASH="${ROS_SETUP_BASH:-/opt/ros/kilted/setup.bash}"
GNSS_SIDECAR_SETUP_BASH="${GNSS_SIDECAR_SETUP_BASH:-/opt/gnss_sidecar/setup.bash}"
UNIVERSAL_BRIDGE_SCRIPT="${UNIVERSAL_BRIDGE_SCRIPT:-/universal_gnss_topic_bridge.py}"
ROS2_BIN="${ROS2_BIN:-ros2}"
PYTHON3_BIN="${PYTHON3_BIN:-python3}"

if [ ! -f "$CONFIG" ]; then
  echo "[start_gps.sh] ERROR: $CONFIG not found. Bind-mount config/mowgli/ to /config."
  exit 1
fi

parse_yaml() {
  # Extract the scalar value of an indented `<key>:` entry. Strips only the
  # first `key:` prefix (so values containing ':' — NTRIP passwords, host:port —
  # survive) and only a single pair of surrounding quotes (so a value that
  # legitimately contains a quote is not corrupted). The `|| true` tolerates a
  # missing key: under `set -euo pipefail` a non-matching grep makes the pipeline
  # exit non-zero, and a bare `NTRIP_HOST=$(parse_yaml ...)` assignment would then
  # abort the whole script BEFORE the `${VAR:-default}` fallbacks can apply.
  { grep -E "^[[:space:]]+${1}:" "$CONFIG" | head -1 \
    | sed -E 's/^[[:space:]]*[^:]*:[[:space:]]*//' \
    | sed -E 's/^"(.*)"$/\1/; s/^'\''(.*)'\''$/\1/'; } || true
}

parse_yaml_any() {
  local key
  for key in "$@"; do
    local value
    value="$(parse_yaml "$key")"
    if [ -n "$value" ]; then
      printf '%s\n' "$value"
      return 0
    fi
  done
  printf '\n'
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
  local yaml_family
  yaml_family="$(parse_yaml gnss_receiver_family)"
  if [ -n "$yaml_family" ]; then
    printf '%s\n' "$(normalize_lower "$yaml_family")"
    return 0
  fi

  if [ -n "${GNSS_RECEIVER_FAMILY:-}" ]; then
    printf '%s\n' "$(normalize_lower "${GNSS_RECEIVER_FAMILY}")"
    return 0
  fi

  printf 'auto\n'
}

resolve_transport() {
  local yaml_transport
  yaml_transport="$(parse_yaml_any gnss_transport)"
  if [ -n "$yaml_transport" ]; then
    printf '%s\n' "$yaml_transport"
    return 0
  fi

  printf '%s\n' "${GNSS_TRANSPORT:-serial}"
}

resolve_serial_device() {
  local yaml_serial_device
  yaml_serial_device="$(parse_yaml gnss_serial_device)"
  if [ -n "$yaml_serial_device" ]; then
    printf '%s\n' "$yaml_serial_device"
    return 0
  fi

  if [ -n "${GNSS_SERIAL_DEVICE:-}" ]; then
    printf '%s\n' "$GNSS_SERIAL_DEVICE"
    return 0
  fi

  printf '/dev/ttyAMA4\n'
}

resolve_serial_baud() {
  local yaml_serial_baud
  yaml_serial_baud="$(parse_yaml gnss_serial_baud)"
  if [ -n "$yaml_serial_baud" ]; then
    printf '%s\n' "$yaml_serial_baud"
    return 0
  fi

  if [ -n "${GNSS_SERIAL_BAUD:-}" ]; then
    printf '%s\n' "$GNSS_SERIAL_BAUD"
    return 0
  fi

  printf '921600\n'
}

resolve_ntrip_enabled() {
  local yaml_enabled
  yaml_enabled="$(parse_yaml_any gnss_ntrip_enabled ntrip_enabled)"
  if [ -n "$yaml_enabled" ]; then
    normalize_bool "$yaml_enabled"
    return 0
  fi

  if [ -n "${GNSS_NTRIP_ENABLED:-}" ]; then
    normalize_bool "$GNSS_NTRIP_ENABLED"
    return 0
  fi

  # Default-on when wholly unconfigured (matches the prior compose default).
  printf 'true\n'
}

resolve_ntrip_host() {
  local host
  host="$(parse_yaml_any gnss_ntrip_host ntrip_host)"
  if [ -n "$host" ]; then
    printf '%s\n' "$host"
    return 0
  fi

  if [ -n "${GNSS_NTRIP_HOST:-}" ]; then
    printf '%s\n' "$GNSS_NTRIP_HOST"
    return 0
  fi

  printf 'crtk.net\n'
}

resolve_ntrip_port() {
  local port
  port="$(parse_yaml_any gnss_ntrip_port ntrip_port)"
  if [ -n "$port" ]; then
    printf '%s\n' "$port"
    return 0
  fi

  if [ -n "${GNSS_NTRIP_PORT:-}" ]; then
    printf '%s\n' "$GNSS_NTRIP_PORT"
    return 0
  fi

  printf '2101\n'
}

# crtk.net is the public Centipede caster (anonymous login "centipede/centipede").
# Only fall back to that login when the receiver is actually pointed at that
# caster — never inject it for a custom caster or override a cleared credential.
ntrip_uses_centipede_caster() {
  [ "$(normalize_lower "$(resolve_ntrip_host)")" = "crtk.net" ]
}

resolve_ntrip_username() {
  local username
  username="$(parse_yaml_any gnss_ntrip_username ntrip_user)"
  if [ -n "$username" ]; then
    printf '%s\n' "$username"
    return 0
  fi

  if [ -n "${GNSS_NTRIP_USERNAME:-}" ]; then
    printf '%s\n' "$GNSS_NTRIP_USERNAME"
    return 0
  fi

  if ntrip_uses_centipede_caster; then
    printf 'centipede\n'
  fi
}

resolve_ntrip_password() {
  local password
  password="$(parse_yaml_any gnss_ntrip_password ntrip_password)"
  if [ -n "$password" ]; then
    printf '%s\n' "$password"
    return 0
  fi

  if [ -n "${GNSS_NTRIP_PASSWORD:-}" ]; then
    printf '%s\n' "$GNSS_NTRIP_PASSWORD"
    return 0
  fi

  if ntrip_uses_centipede_caster; then
    printf 'centipede\n'
  fi
}

resolve_ntrip_mountpoint() {
  local mountpoint
  mountpoint="$(parse_yaml_any gnss_ntrip_mountpoint ntrip_mountpoint)"
  if [ -n "$mountpoint" ]; then
    printf '%s\n' "$mountpoint"
    return 0
  fi

  if [ -n "${GNSS_NTRIP_MOUNTPOINT:-}" ]; then
    printf '%s\n' "$GNSS_NTRIP_MOUNTPOINT"
    return 0
  fi

  printf 'NEAR\n'
}

resolve_ntrip_gga_enabled() {
  local yaml_gga_enabled
  yaml_gga_enabled="$(parse_yaml_any gnss_ntrip_gga_enabled)"
  if [ -n "$yaml_gga_enabled" ]; then
    normalize_bool "$yaml_gga_enabled"
    return 0
  fi

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
  local yaml_gga_interval_s
  yaml_gga_interval_s="$(parse_yaml_any gnss_ntrip_gga_interval_s)"
  if [ -n "$yaml_gga_interval_s" ]; then
    printf '%s\n' "$yaml_gga_interval_s"
    return 0
  fi

  printf '%s\n' "${GNSS_NTRIP_GGA_INTERVAL_S:-10}"
}

resolve_frame_id() {
  local yaml_frame_id
  yaml_frame_id="$(parse_yaml_any gnss_frame_id)"
  if [ -n "$yaml_frame_id" ]; then
    printf '%s\n' "$yaml_frame_id"
    return 0
  fi

  printf '%s\n' "${GNSS_FRAME_ID:-gps_link}"
}

resolve_publish_rate_hz() {
  local yaml_publish_rate
  yaml_publish_rate="$(parse_yaml gnss_profile_rate_hz)"
  if [ -n "$yaml_publish_rate" ]; then
    printf '%s\n' "$yaml_publish_rate"
    return 0
  fi

  if [ -n "${GNSS_PROFILE_RATE_HZ:-}" ]; then
    printf '%s\n' "$GNSS_PROFILE_RATE_HZ"
    return 0
  fi

  printf '5\n'
}

print_command() {
  local label="$1"
  shift

  printf '[start_gps.sh] %s' "$label"
  printf ' %q' "$@"
  printf '\n'
}

GNSS_DRY_RUN="$(normalize_bool "${GNSS_DRY_RUN:-false}")"

if [ "$(normalize_lower "${GNSS_STACK:-universal}")" = "disabled" ]; then
  echo "[start_gps.sh] ERROR: GNSS_STACK=disabled is not valid for the GNSS sidecar."
  exit 1
fi

set +u
source "$ROS_SETUP_BASH"
if [ -f "$GNSS_SIDECAR_SETUP_BASH" ]; then
  source "$GNSS_SIDECAR_SETUP_BASH"
fi
set -u

if [ ! -f "$GNSS_SIDECAR_SETUP_BASH" ]; then
  echo "[start_gps.sh] ERROR: $GNSS_SIDECAR_SETUP_BASH not found; Universal GNSS overlay missing."
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
normalize_ros_double() {
  value="$1"

  case "$value" in
    "")
      printf '%s\n' "10.0"
      ;;
    *.*)
      printf '%s\n' "$value"
      ;;
    *[!0-9]*)
      printf '%s\n' "$value"
      ;;
    *)
      printf '%s\n' "${value}.0"
      ;;
  esac
}

publish_rate_hz="$(normalize_ros_double "$(resolve_publish_rate_hz)")"
frame_id="$(resolve_frame_id)"
ntrip_enabled="$(resolve_ntrip_enabled)"
ntrip_host="$(resolve_ntrip_host)"
ntrip_port="$(resolve_ntrip_port)"
ntrip_user="$(resolve_ntrip_username)"
ntrip_password="$(resolve_ntrip_password)"
ntrip_mountpoint="$(resolve_ntrip_mountpoint)"
ntrip_gga_enabled="$(resolve_ntrip_gga_enabled)"
ntrip_gga_interval_s="$(resolve_ntrip_gga_interval_s)"

if [ "$transport" = "serial" ] && [ ! -e "$serial_device" ]; then
  echo "[start_gps.sh] ERROR: selected GNSS serial device does not exist: ${serial_device}"
  exit 1
fi

receiver_node_cmd=(
  "$ROS2_BIN" run universal_gnss_ros2 receiver_node --ros-args
  -p "receiver_family:=${receiver_family}"
  -p "transport:=${transport}"
  -p "serial_device:=${serial_device}"
  -p "serial_baud:=${serial_baud}"
  -p "publish_rate_hz:=${publish_rate_hz}"
  -p "frame_id:=${frame_id}"
  -r "status:=${internal_status_topic}"
  -r "diagnostics:=/diagnostics"
  -r "fix:=/gps/fix"
  -r "rtcm:=${internal_rtcm_topic}"
)

bridge_cmd=(
  "$PYTHON3_BIN" "$UNIVERSAL_BRIDGE_SCRIPT" --ros-args
  -p "backend:=universal"
  -p "receiver_family:=${receiver_family}"
  -p "frame_id:=${frame_id}"
  -p "input_status_topic:=${internal_status_topic}"
  -p "output_status_topic:=/gps/status"
  -p "input_diagnostics_topic:=/diagnostics"
  -p "input_rtcm_topic:=${internal_rtcm_topic}"
  -p "output_rtcm_topic:=/rtcm"
)

ntrip_cmd=(
  "$ROS2_BIN" run universal_gnss_ros2 ntrip_node --ros-args
  -p "caster_host:=${ntrip_host}"
  -p "caster_port:=${ntrip_port}"
  -p "mountpoint:=${ntrip_mountpoint}"
  -p "username:=${ntrip_user}"
  -p "password:=${ntrip_password}"
  -p "gga_enabled:=${ntrip_gga_enabled}"
  -p "gga_interval_s:=${ntrip_gga_interval_s}"
  -p "tls_enabled:=false"
  -r "status:=${internal_status_topic}"
  -r "diagnostics:=/diagnostics"
  -r "rtcm:=${internal_rtcm_topic}"
)

echo "[start_gps.sh] Runtime=universal receiver_family=${receiver_family} transport=${transport} device=${serial_device} baud=${serial_baud} rate_hz=${publish_rate_hz}"

if [ "$GNSS_DRY_RUN" = "true" ]; then
  print_command "receiver_node" "${receiver_node_cmd[@]}"
  print_command "topic_bridge" "${bridge_cmd[@]}"
  if [ "$ntrip_enabled" = "true" ]; then
    print_command "ntrip_node" "${ntrip_cmd[@]}"
  fi
  exit 0
fi

"${receiver_node_cmd[@]}" &
GPS_PID=$!

"${bridge_cmd[@]}" &
UNIVERSAL_BRIDGE_PID=$!

if [ "$ntrip_enabled" = "true" ]; then
  echo "[start_gps.sh] Runtime=universal NTRIP enabled: ${ntrip_host}:${ntrip_port}/${ntrip_mountpoint}"
  sleep 3
  "${ntrip_cmd[@]}" &
  NTRIP_PID=$!
fi

wait -n || true
cleanup
wait
