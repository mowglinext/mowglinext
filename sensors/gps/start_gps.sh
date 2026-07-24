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

# Automatic receiver-profile apply (issue #395). MowgliNext's runtime receiver
# node never configures the receiver, so a UM980 whose persisted config isn't a
# high-precision rover sits at DGPS even while valid RTCM is being forwarded —
# unlike OpenMower, whose driver pushes the rover config on every boot. Applying
# the `rover_high_precision` profile in runtime-only mode at startup restores
# that behaviour (re-applied each boot, no persistent NVM changes).
resolve_config_apply_enabled() {
  local yaml_enabled
  yaml_enabled="$(parse_yaml_any gnss_config_apply_enabled)"
  if [ -n "$yaml_enabled" ]; then
    normalize_bool "$yaml_enabled"
    return 0
  fi

  if [ -n "${GNSS_CONFIG_APPLY_ENABLED:-}" ]; then
    normalize_bool "$GNSS_CONFIG_APPLY_ENABLED"
    return 0
  fi

  printf 'true\n'
}

resolve_config_profile() {
  local yaml_profile
  yaml_profile="$(parse_yaml_any gnss_config_profile)"
  if [ -n "$yaml_profile" ]; then
    printf '%s\n' "$yaml_profile"
    return 0
  fi

  printf '%s\n' "${GNSS_CONFIG_PROFILE:-rover_high_precision}"
}

resolve_signal_profile() {
  local yaml_signal_profile
  yaml_signal_profile="$(parse_yaml_any gnss_signal_profile)"
  if [ -n "$yaml_signal_profile" ]; then
    printf '%s\n' "$yaml_signal_profile"
    return 0
  fi

  printf '%s\n' "${GNSS_SIGNAL_PROFILE:-balanced}"
}

resolve_receiver_model() {
  local yaml_model
  yaml_model="$(parse_yaml_any gnss_receiver_model)"
  if [ -n "$yaml_model" ]; then
    printf '%s\n' "$yaml_model"
    return 0
  fi

  printf '%s\n' "${GNSS_RECEIVER_MODEL:-}"
}

resolve_signal_group() {
  local yaml_signal_group
  yaml_signal_group="$(parse_yaml_any gnss_signal_group)"
  if [ -n "$yaml_signal_group" ]; then
    printf '%s\n' "$yaml_signal_group"
    return 0
  fi

  printf '%s\n' "${GNSS_SIGNAL_GROUP:-}"
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
config_apply_enabled="$(resolve_config_apply_enabled)"
config_profile="$(resolve_config_profile)"
signal_profile="$(resolve_signal_profile)"
receiver_model="$(resolve_receiver_model)"
signal_group="$(resolve_signal_group)"

if [ "$transport" = "serial" ] && [ ! -e "$serial_device" ]; then
  echo "[start_gps.sh] ERROR: selected GNSS serial device does not exist: ${serial_device}"
  exit 1
fi

# Build the receiver-profile apply command (issue #395). Runs synchronously
# BEFORE receiver_node opens the serial device — the receiver can only be held
# open by one process at a time, and the apply must finish and release the port.
GNSS_CONFIG_APPLY_BIN="${GNSS_CONFIG_APPLY_BIN:-gnss_config_apply}"
GNSS_CONFIG_APPLY_TIMEOUT_MS="${GNSS_CONFIG_APPLY_TIMEOUT_MS:-5000}"
config_apply_cmd=(
  "$GNSS_CONFIG_APPLY_BIN"
  --json
  --family "$receiver_family"
  --device "$serial_device"
  --baud "$serial_baud"
  --config-baud "$serial_baud"
  --profile "$config_profile"
  --apply-mode runtime-only
  --rate-hz "$publish_rate_hz"
  --timeout-ms "$GNSS_CONFIG_APPLY_TIMEOUT_MS"
  --signal-profile "$signal_profile"
  --confirm
)
if [ -n "$receiver_model" ]; then
  config_apply_cmd+=(--model "$receiver_model")
fi
if [ -n "$signal_group" ]; then
  config_apply_cmd+=(--signal-group "$signal_group")
fi

# Apply the receiver profile synchronously (serial transport only). A failure is
# non-fatal: the receiver keeps whatever config it already holds, so a
# pre-configured receiver still runs. The apply must complete before
# receiver_node opens the device.
apply_receiver_profile() {
  if [ "$config_apply_enabled" != "true" ]; then
    echo "[start_gps.sh] Receiver profile auto-apply disabled (gnss_config_apply_enabled=false); leaving receiver config unchanged."
    return 0
  fi
  if [ "$transport" != "serial" ]; then
    echo "[start_gps.sh] Receiver profile auto-apply skipped for non-serial transport '${transport}'."
    return 0
  fi

  echo "[start_gps.sh] Applying receiver profile '${config_profile}' (runtime-only, family=${receiver_family}, signal_profile=${signal_profile}) before starting receiver_node"
  if "${config_apply_cmd[@]}"; then
    echo "[start_gps.sh] Receiver profile apply succeeded"
  else
    echo "[start_gps.sh] WARNING: receiver profile apply failed; continuing with the receiver's existing configuration"
  fi
}

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

# Topic bridge ("topic manager"): C++ (mowgli_gnss_bridge) by default — a
# behaviour-exact, lower-CPU port of universal_gnss_topic_bridge.py. Set
# GNSS_BRIDGE_IMPL=python to fall back to the retained Python script (identical
# --ros-args), e.g. for A/B comparison or if the C++ build is unavailable.
if [ "$(normalize_lower "${GNSS_BRIDGE_IMPL:-cpp}")" = "python" ]; then
  bridge_cmd=(
    "$PYTHON3_BIN" "$UNIVERSAL_BRIDGE_SCRIPT" --ros-args
  )
else
  bridge_cmd=(
    "$ROS2_BIN" run mowgli_gnss_bridge universal_gnss_topic_bridge --ros-args
  )
fi
bridge_cmd+=(
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
  if [ "$config_apply_enabled" = "true" ] && [ "$transport" = "serial" ]; then
    print_command "config_apply" "${config_apply_cmd[@]}"
  fi
  print_command "receiver_node" "${receiver_node_cmd[@]}"
  print_command "topic_bridge" "${bridge_cmd[@]}"
  if [ "$ntrip_enabled" = "true" ]; then
    print_command "ntrip_node" "${ntrip_cmd[@]}"
  fi
  exit 0
fi

apply_receiver_profile

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
