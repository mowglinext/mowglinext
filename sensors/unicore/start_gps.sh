#!/bin/bash
# =============================================================================
# Unicore GNSS driver startup + NTRIP client
#
# Launches:
#   1. unicore_node       — Unicore GNSS driver and RTCM injector on /dev/gps,
#                           publishes /gps/fix, /gps/azimuth, /diagnostics
#                           and subscribes to /ntrip_client/rtcm
#   2. ntrip_client_node  — NTRIP caster client publishing /ntrip_client/rtcm
#
# Reads gps_port / gps_baudrate / ntrip_* from /config/mowgli_robot.yaml.
# Bind-mount docker/config/mowgli to /config (see docker-compose.unicore.yaml).
#
# We deliberately do NOT use `ros2 launch mowgli_unicore_gnss
# unicore_launch.py` here: the upstream launch file accepts no arguments and
# always loads the package-share config/unicore.yaml (port=/dev/gps,
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

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/configure_receiver.sh"

CONFIG="${MOWGLI_CONFIG_PATH:-/config/mowgli_robot.yaml}"
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

resolve_bool_setting() {
  local var_name="${1:?resolve_bool_setting: missing var name}"
  local default_value="${2:-false}"
  local raw_value="${!var_name-}"

  if [ -z "$raw_value" ]; then
    unicore_bool_string "$default_value"
  else
    unicore_bool_string "$raw_value"
  fi
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
UNICORE_PROFILE="${UNICORE_PROFILE:-normal}"
UNICORE_OUTPUT_FORMAT="${UNICORE_OUTPUT_FORMAT:-}"

# Optional one-shot UM98x config blast — sends MODE ROVER + LOG
# directives via /configure_receiver.sh. SAVECONFIG persists in NVRAM,
# so this is a no-op on a receiver that's already correctly configured.
# Default true so the "just installed, never configured" case works
# out of the box; set to false in mowgli_robot.yaml after you've run
# Unicore Setup tool yourself.
UNICORE_AUTO_CONFIGURE=$(parse_yaml unicore_auto_configure)
UNICORE_AUTO_CONFIGURE="${UNICORE_AUTO_CONFIGURE:-true}"

unicore_apply_profile_defaults

UNICORE_BINARY_REQUESTED_BY_OUTPUT="$(unicore_binary_enabled_from_output_format "$UNICORE_OUTPUT_FORMAT")"
UNICORE_ENABLE_UNICORE_BINARY="$(
  resolve_bool_setting UNICORE_ENABLE_UNICORE_BINARY "$UNICORE_BINARY_REQUESTED_BY_OUTPUT"
)"

if is_truthy "$UNICORE_BINARY_REQUESTED_BY_OUTPUT" && ! is_truthy "$UNICORE_ENABLE_UNICORE_BINARY"; then
  echo "[start_gps.sh] WARN: output_format=${UNICORE_OUTPUT_FORMAT} requires binary transport; forcing enable_unicore_binary=true"
  UNICORE_ENABLE_UNICORE_BINARY="true"
elif ! is_truthy "$UNICORE_BINARY_REQUESTED_BY_OUTPUT" && is_truthy "$UNICORE_ENABLE_UNICORE_BINARY"; then
  echo "[start_gps.sh] WARN: enable_unicore_binary=true requested with ascii output; falling back to ascii transport only"
  UNICORE_ENABLE_UNICORE_BINARY="false"
fi

UNICORE_USE_BINARY_NAV="$(
  resolve_bool_setting UNICORE_USE_BINARY_NAV \
    "$(unicore_profile_default_binary_consumer "$UNICORE_PROFILE" nav "$UNICORE_OUTPUT_FORMAT")"
)"
UNICORE_USE_BINARY_SATELLITE_DIAG="$(
  resolve_bool_setting UNICORE_USE_BINARY_SATELLITE_DIAG \
    "$(unicore_profile_default_binary_consumer "$UNICORE_PROFILE" satellite "$UNICORE_OUTPUT_FORMAT")"
)"
UNICORE_USE_BINARY_RTCM_DIAG="$(
  resolve_bool_setting UNICORE_USE_BINARY_RTCM_DIAG \
    "$(unicore_profile_default_binary_consumer "$UNICORE_PROFILE" rtcm "$UNICORE_OUTPUT_FORMAT")"
)"
UNICORE_USE_BINARY_RTK_DIAG="$(
  resolve_bool_setting UNICORE_USE_BINARY_RTK_DIAG \
    "$(unicore_profile_default_binary_consumer "$UNICORE_PROFILE" rtk "$UNICORE_OUTPUT_FORMAT")"
)"
UNICORE_USE_BINARY_RF_DIAG="$(
  resolve_bool_setting UNICORE_USE_BINARY_RF_DIAG \
    "$(unicore_profile_default_binary_consumer "$UNICORE_PROFILE" rf "$UNICORE_OUTPUT_FORMAT")"
)"
UNICORE_USE_BINARY_HW_DIAG="$(
  resolve_bool_setting UNICORE_USE_BINARY_HW_DIAG \
    "$(unicore_profile_default_binary_consumer "$UNICORE_PROFILE" hw "$UNICORE_OUTPUT_FORMAT")"
)"
UNICORE_USE_BINARY_JAMMING_DIAG="$(
  resolve_bool_setting UNICORE_USE_BINARY_JAMMING_DIAG \
    "$(unicore_profile_default_binary_consumer "$UNICORE_PROFILE" jamming "$UNICORE_OUTPUT_FORMAT")"
)"

UNICORE_ENABLE_RAW_OBSERVATIONS="${UNICORE_ENABLE_RAW_OBSERVATIONS:-false}"
UNICORE_RAW_DEFAULT="false"
if is_truthy "$UNICORE_ENABLE_RAW_OBSERVATIONS" &&
   unicore_profile_supports_raw "$UNICORE_PROFILE" &&
   unicore_output_has_binary "$UNICORE_OUTPUT_FORMAT"; then
  UNICORE_RAW_DEFAULT="true"
fi
UNICORE_ENABLE_RAW_OBSERVATION_DIAG="$(
  resolve_bool_setting UNICORE_ENABLE_RAW_OBSERVATION_DIAG "$UNICORE_RAW_DEFAULT"
)"
UNICORE_USE_BINARY_RAW_OBSERVATIONS="$(
  resolve_bool_setting UNICORE_USE_BINARY_RAW_OBSERVATIONS "$UNICORE_RAW_DEFAULT"
)"

if ! is_truthy "$UNICORE_ENABLE_UNICORE_BINARY"; then
  for var_name in \
    UNICORE_USE_BINARY_NAV \
    UNICORE_USE_BINARY_SATELLITE_DIAG \
    UNICORE_USE_BINARY_RTCM_DIAG \
    UNICORE_USE_BINARY_RTK_DIAG \
    UNICORE_USE_BINARY_RF_DIAG \
    UNICORE_USE_BINARY_HW_DIAG \
    UNICORE_USE_BINARY_JAMMING_DIAG \
    UNICORE_USE_BINARY_RAW_OBSERVATIONS; do
    if is_truthy "${!var_name}"; then
      echo "[start_gps.sh] WARN: ${var_name}=true requested without binary transport; forcing false"
      printf -v "$var_name" '%s' "false"
    fi
  done
fi

if is_truthy "$UNICORE_ENABLE_RAW_OBSERVATIONS" && ! unicore_profile_supports_raw "$UNICORE_PROFILE"; then
  echo "[start_gps.sh] WARN: raw observations are refused in profile=${UNICORE_PROFILE}; disabling raw diagnostics"
  UNICORE_ENABLE_RAW_OBSERVATION_DIAG="false"
  UNICORE_USE_BINARY_RAW_OBSERVATIONS="false"
elif is_truthy "$UNICORE_ENABLE_RAW_OBSERVATIONS" && ! unicore_output_has_binary "$UNICORE_OUTPUT_FORMAT"; then
  echo "[start_gps.sh] WARN: raw observations in ascii mode are capture-only; binary raw diagnostics stay disabled"
  UNICORE_ENABLE_RAW_OBSERVATION_DIAG="false"
  UNICORE_USE_BINARY_RAW_OBSERVATIONS="false"
fi

if [ "$UNICORE_OUTPUT_FORMAT" = "binary" ]; then
  if ! is_truthy "$UNICORE_USE_BINARY_NAV"; then
    echo "[start_gps.sh] WARN: binary output requires binary nav; forcing use_binary_nav=true"
    UNICORE_USE_BINARY_NAV="true"
  fi
  if is_truthy "$UNICORE_ENABLE_SATELLITES" && ! is_truthy "$UNICORE_USE_BINARY_SATELLITE_DIAG"; then
    echo "[start_gps.sh] WARN: binary output requires binary satellite diagnostics when satellites are enabled; forcing true"
    UNICORE_USE_BINARY_SATELLITE_DIAG="true"
  fi
  if ! is_truthy "$UNICORE_USE_BINARY_RTCM_DIAG"; then
    echo "[start_gps.sh] WARN: binary output requires binary RTCM diagnostics; forcing true"
    UNICORE_USE_BINARY_RTCM_DIAG="true"
  fi
  if ! is_truthy "$UNICORE_USE_BINARY_RTK_DIAG"; then
    echo "[start_gps.sh] WARN: binary output requires binary RTK diagnostics; forcing true"
    UNICORE_USE_BINARY_RTK_DIAG="true"
  fi
  if is_truthy "$UNICORE_ENABLE_RF" && ! is_truthy "$UNICORE_USE_BINARY_RF_DIAG"; then
    echo "[start_gps.sh] WARN: binary output requires binary RF diagnostics when RF is enabled; forcing true"
    UNICORE_USE_BINARY_RF_DIAG="true"
  fi
  if is_truthy "$UNICORE_ENABLE_HARDWARE" && ! is_truthy "$UNICORE_USE_BINARY_HW_DIAG"; then
    echo "[start_gps.sh] WARN: binary output requires binary hardware diagnostics when hardware is enabled; forcing true"
    UNICORE_USE_BINARY_HW_DIAG="true"
  fi
  if is_truthy "$UNICORE_ENABLE_JAMMING" && ! is_truthy "$UNICORE_USE_BINARY_JAMMING_DIAG"; then
    echo "[start_gps.sh] WARN: binary output requires binary jamming diagnostics when jamming is enabled; forcing true"
    UNICORE_USE_BINARY_JAMMING_DIAG="true"
  fi
fi

UNICORE_BINARY_COMPARE_ASCII="false"
if unicore_output_has_ascii "$UNICORE_OUTPUT_FORMAT" &&
   unicore_output_has_binary "$UNICORE_OUTPUT_FORMAT"; then
  UNICORE_BINARY_COMPARE_ASCII="true"
fi

echo "[start_gps.sh] Resolved Unicore backend: output=${UNICORE_OUTPUT_FORMAT} binary_transport=${UNICORE_ENABLE_UNICORE_BINARY} binary_nav=${UNICORE_USE_BINARY_NAV} binary_rtk=${UNICORE_USE_BINARY_RTK_DIAG} binary_rtcm=${UNICORE_USE_BINARY_RTCM_DIAG} binary_sat=${UNICORE_USE_BINARY_SATELLITE_DIAG} binary_rf=${UNICORE_USE_BINARY_RF_DIAG} binary_hw=${UNICORE_USE_BINARY_HW_DIAG} binary_jam=${UNICORE_USE_BINARY_JAMMING_DIAG} raw_diag=${UNICORE_ENABLE_RAW_OBSERVATION_DIAG} raw_binary=${UNICORE_USE_BINARY_RAW_OBSERVATIONS}"
echo "[start_gps.sh] Auto-config uses a per-message UM98x/N4 syntax table (LOG ONTIME, direct period, ONCHANGED) with compatibility fallbacks when needed."

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

UNICORE_ROS_PACKAGE="${UNICORE_ROS_PACKAGE:-unicore_gnss}"
UNICORE_ROS_EXECUTABLE="${UNICORE_ROS_EXECUTABLE:-unicore_node}"

echo "[start_gps.sh] Launching Unicore GNSS driver: port=${GPS_PORT} baud=${GPS_BAUD} profile=${UNICORE_PROFILE} pkg=${UNICORE_ROS_PACKAGE} exec=${UNICORE_ROS_EXECUTABLE}"
# diagnostics_topic = /diagnostics — the ROS2 standard aggregator topic
# (matches gps_health_aggregator.py in the ublox path). The GUI's
# Diagnostics panel reads from /diagnostics; namespacing under
# /gps/diagnostics would hide the unicore status from the dashboard.
ros2 run "${UNICORE_ROS_PACKAGE}" "${UNICORE_ROS_EXECUTABLE}" --ros-args \
  -p "port:=${GPS_PORT}" \
  -p "baudrate:=${GPS_BAUD}" \
  -p "frame_id:=gps_link" \
  -p "fix_topic:=/gps/fix" \
  -p "heading_topic:=/gps/azimuth" \
  -p "diagnostics_topic:=/diagnostics" \
  -p "rtcm_topic:=/ntrip_client/rtcm" \
  -p "enable_satellite_status:=$(unicore_bool_string "$UNICORE_ENABLE_SATELLITES")" \
  -p "enable_satsinfo:=$(unicore_bool_string "$UNICORE_ENABLE_SATELLITES")" \
  -p "satellite_diag_timeout_sec:=5.0" \
  -p "enable_rtk_status:=true" \
  -p "enable_rtcm_status:=true" \
  -p "enable_rf_status:=$(unicore_bool_string "$UNICORE_ENABLE_RF")" \
  -p "enable_hw_status:=$(unicore_bool_string "$UNICORE_ENABLE_HARDWARE")" \
  -p "enable_jamming_status:=$(unicore_bool_string "$UNICORE_ENABLE_JAMMING")" \
  -p "rf_diag_timeout_sec:=5.0" \
  -p "enable_unicore_binary:=${UNICORE_ENABLE_UNICORE_BINARY}" \
  -p "use_binary_nav:=${UNICORE_USE_BINARY_NAV}" \
  -p "use_binary_satellite_diag:=${UNICORE_USE_BINARY_SATELLITE_DIAG}" \
  -p "use_binary_rtcm_diag:=${UNICORE_USE_BINARY_RTCM_DIAG}" \
  -p "use_binary_rtk_diag:=${UNICORE_USE_BINARY_RTK_DIAG}" \
  -p "use_binary_rf_diag:=${UNICORE_USE_BINARY_RF_DIAG}" \
  -p "use_binary_hw_diag:=${UNICORE_USE_BINARY_HW_DIAG}" \
  -p "use_binary_jamming_diag:=${UNICORE_USE_BINARY_JAMMING_DIAG}" \
  -p "enable_raw_observation_diag:=${UNICORE_ENABLE_RAW_OBSERVATION_DIAG}" \
  -p "use_binary_raw_observations:=${UNICORE_USE_BINARY_RAW_OBSERVATIONS}" \
  -p "raw_observation_timeout_sec:=5.0" \
  -p "raw_observation_max_debug_entries:=0" \
  -p "binary_compare_ascii:=${UNICORE_BINARY_COMPARE_ASCII}" &
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
