#!/usr/bin/env bash

upsert_env_key() {
  local file="$1"
  local key="$2"
  local value="$3"
  local escaped_value="$value"

  escaped_value="${escaped_value//\\/\\\\}"
  escaped_value="${escaped_value//&/\\&}"
  escaped_value="${escaped_value//|/\\|}"

  if grep -q "^${key}=" "$file" 2>/dev/null; then
    sed -i "s|^${key}=.*|${key}=${escaped_value}|" "$file"
  else
    echo "${key}=${value}" >> "$file"
  fi
}

remove_env_key() {
  local file="$1"
  local key="$2"

  [ -f "$file" ] || return 0
  sed -i "/^${key}=.*/d" "$file"
}

remove_env_keys_with_prefix() {
  local file="$1"
  local prefix="$2"

  [ -f "$file" ] || return 0
  sed -i "/^${prefix}[A-Z0-9_]*=.*/d" "$file"
}

ensure_env_comment_line() {
  local file="$1"
  local comment="$2"

  [ -f "$file" ] || return 0
  grep -Fqx "$comment" "$file" 2>/dev/null && return 0
  printf '%s\n' "$comment" >> "$file"
}

remove_legacy_gnss_env_keys() {
  local file="$1"
  local legacy_keys=(
    GPS_CONNECTION
    GPS_""RUNTIME_MODE
    GPS_""PROTOCOL
    GPS_""PORT
    GPS_""BY_ID
    GPS_""UART_DEVICE
    GPS_""BAUD
    GPS_""DEBUG_ENABLED
    GPS_""DEBUG_PORT
    GPS_""DEBUG_UART_DEVICE
    GPS_""DEBUG_BAUD
    GPS_""UART_RULE
    GPS_""DEBUG_UART_RULE
    UBLOX_""DEVICE_FAMILY
    UBLOX_""DEVICE_SERIAL_STRING
    UNICORE_""ROS_PACKAGE
    UNICORE_""ROS_EXECUTABLE
    UNICORE_COM_PORT
    UNICORE_IMAGE
  )
  local key

  for key in "${legacy_keys[@]}"; do
    remove_env_key "$file" "$key"
  done
}

env_yaml_value() {
  local file="$1"
  local key="$2"
  local line value

  line="$(grep -m1 -E "^[[:space:]]+${key}:" "$file" 2>/dev/null || true)"
  value="${line#*:}"
  value="${value#"${value%%[![:space:]]*}"}"
  value="${value%%#*}"
  value="${value%"${value##*[![:space:]]}"}"
  value="${value#\"}"
  value="${value%\"}"
  printf '%s\n' "$value"
}

load_gnss_ntrip_runtime_defaults() {
  local yaml_file="$DOCKER_DIR/config/mowgli/mowgli_robot.yaml"
  local yaml_enabled yaml_host yaml_port yaml_user yaml_password yaml_mountpoint

  if [[ ! -f "$yaml_file" ]]; then
    return 0
  fi

  yaml_enabled="$(env_yaml_value "$yaml_file" ntrip_enabled)"
  yaml_host="$(env_yaml_value "$yaml_file" ntrip_host)"
  yaml_port="$(env_yaml_value "$yaml_file" ntrip_port)"
  yaml_user="$(env_yaml_value "$yaml_file" ntrip_user)"
  yaml_password="$(env_yaml_value "$yaml_file" ntrip_password)"
  yaml_mountpoint="$(env_yaml_value "$yaml_file" ntrip_mountpoint)"

  if [[ "${CONFIG_NTRIP_ENABLED_EXPLICIT:-false}" != "true" && -n "$yaml_enabled" ]]; then
    GNSS_NTRIP_ENABLED="$yaml_enabled"
  elif [[ -z "${GNSS_NTRIP_ENABLED:-}" ]]; then
    GNSS_NTRIP_ENABLED="$yaml_enabled"
  fi

  if [[ "${CONFIG_NTRIP_HOST_EXPLICIT:-false}" != "true" && -n "$yaml_host" ]]; then
    GNSS_NTRIP_HOST="$yaml_host"
  elif [[ -z "${GNSS_NTRIP_HOST:-}" ]]; then
    GNSS_NTRIP_HOST="$yaml_host"
  fi

  if [[ "${CONFIG_NTRIP_PORT_EXPLICIT:-false}" != "true" && -n "$yaml_port" ]]; then
    GNSS_NTRIP_PORT="$yaml_port"
  elif [[ -z "${GNSS_NTRIP_PORT:-}" ]]; then
    GNSS_NTRIP_PORT="$yaml_port"
  fi

  if [[ "${CONFIG_NTRIP_USER_EXPLICIT:-false}" != "true" && -n "$yaml_user" ]]; then
    GNSS_NTRIP_USERNAME="$yaml_user"
  elif [[ -z "${GNSS_NTRIP_USERNAME:-}" ]]; then
    GNSS_NTRIP_USERNAME="$yaml_user"
  fi

  if [[ "${CONFIG_NTRIP_PASSWORD_EXPLICIT:-false}" != "true" && -n "$yaml_password" ]]; then
    GNSS_NTRIP_PASSWORD="$yaml_password"
  elif [[ -z "${GNSS_NTRIP_PASSWORD:-}" ]]; then
    GNSS_NTRIP_PASSWORD="$yaml_password"
  fi

  if [[ "${CONFIG_NTRIP_MOUNTPOINT_EXPLICIT:-false}" != "true" && -n "$yaml_mountpoint" ]]; then
    GNSS_NTRIP_MOUNTPOINT="$yaml_mountpoint"
  elif [[ -z "${GNSS_NTRIP_MOUNTPOINT:-}" ]]; then
    GNSS_NTRIP_MOUNTPOINT="$yaml_mountpoint"
  fi
}

sync_gnss_env_contract_values() {
  local normalized_status_source

  load_gnss_ntrip_runtime_defaults

  GNSS_STACK="$(effective_gnss_stack 2>/dev/null || default_gnss_stack)"
  normalized_status_source="$(normalize_gnss_status_source "${GNSS_STATUS_SOURCE:-$(default_gnss_status_source)}")"

  case "$GNSS_STACK" in
    disabled)
      GNSS_STATUS_SOURCE="external"
      ;;
    *)
      if [[ "$normalized_status_source" == "external" && "${HARDWARE_BACKEND:-mowgli}" == "mavros" ]]; then
        GNSS_STATUS_SOURCE="external"
      else
        GNSS_STATUS_SOURCE="universal"
      fi
      ;;
  esac

  GNSS_RECEIVER_FAMILY="$(gnss_receiver_family_from_state)"
  GNSS_TRANSPORT="$(gnss_transport_from_state)"
  GNSS_SERIAL_DEVICE="$(gnss_serial_device_from_state)"
  GNSS_SERIAL_BAUD="$(gnss_serial_baud_from_state)"
  GNSS_FRAME_ID="${GNSS_FRAME_ID:-gps_link}"

  : "${GNSS_NTRIP_ENABLED:=${CONFIG_NTRIP_ENABLED:-true}}"
  : "${GNSS_NTRIP_HOST:=${CONFIG_NTRIP_HOST:-crtk.net}}"
  : "${GNSS_NTRIP_PORT:=${CONFIG_NTRIP_PORT:-2101}}"
  GNSS_NTRIP_USERNAME="${GNSS_NTRIP_USERNAME:-${CONFIG_NTRIP_USER:-}}"
  GNSS_NTRIP_PASSWORD="${GNSS_NTRIP_PASSWORD:-${CONFIG_NTRIP_PASSWORD:-}}"
  # crtk.net is the public Centipede caster, whose anonymous login is
  # "centipede/centipede". Only fall back to it when actually pointed at that
  # caster — a custom caster must never receive the Centipede credentials, and a
  # credential the operator deliberately cleared must not be silently restored.
  if [[ "${GNSS_NTRIP_HOST,,}" == "crtk.net" ]]; then
    : "${GNSS_NTRIP_USERNAME:=centipede}"
    : "${GNSS_NTRIP_PASSWORD:=centipede}"
  fi
  : "${GNSS_NTRIP_MOUNTPOINT:=${CONFIG_NTRIP_MOUNTPOINT:-NEAR}}"
  : "${GNSS_RTCM_FORWARDING:=true}"
  : "${GNSS_NTRIP_GGA_ENABLED:=$(if [[ "${GNSS_NTRIP_MOUNTPOINT:-}" =~ ^[Nn][Ee][Aa][Rr] ]]; then printf 'true\n'; else printf 'false\n'; fi)}"
  : "${GNSS_NTRIP_GGA_INTERVAL_S:=10}"
}

write_gnss_env_contract_keys() {
  local env_file="${1:?write_gnss_env_contract_keys: missing env file}"

  ensure_env_comment_line "$env_file" "# GNSS_* values below are fallback-only first-boot defaults."
  ensure_env_comment_line "$env_file" "# Active operator GNSS settings live in docker/config/mowgli/mowgli_robot.yaml and the GUI."
  upsert_env_key "$env_file" "GNSS_STACK" "$GNSS_STACK"
  upsert_env_key "$env_file" "GNSS_RECEIVER_FAMILY" "$GNSS_RECEIVER_FAMILY"
  upsert_env_key "$env_file" "GNSS_TRANSPORT" "$GNSS_TRANSPORT"
  upsert_env_key "$env_file" "GNSS_SERIAL_DEVICE" "$GNSS_SERIAL_DEVICE"
  upsert_env_key "$env_file" "GNSS_SERIAL_BAUD" "$GNSS_SERIAL_BAUD"
  upsert_env_key "$env_file" "GNSS_FRAME_ID" "$GNSS_FRAME_ID"
  upsert_env_key "$env_file" "GNSS_NTRIP_ENABLED" "$GNSS_NTRIP_ENABLED"
  upsert_env_key "$env_file" "GNSS_NTRIP_HOST" "$GNSS_NTRIP_HOST"
  upsert_env_key "$env_file" "GNSS_NTRIP_PORT" "$GNSS_NTRIP_PORT"
  upsert_env_key "$env_file" "GNSS_NTRIP_MOUNTPOINT" "$GNSS_NTRIP_MOUNTPOINT"
  upsert_env_key "$env_file" "GNSS_NTRIP_USERNAME" "$GNSS_NTRIP_USERNAME"
  upsert_env_key "$env_file" "GNSS_NTRIP_PASSWORD" "$GNSS_NTRIP_PASSWORD"
  upsert_env_key "$env_file" "GNSS_RTCM_FORWARDING" "$GNSS_RTCM_FORWARDING"
  upsert_env_key "$env_file" "GNSS_NTRIP_GGA_ENABLED" "$GNSS_NTRIP_GGA_ENABLED"
  upsert_env_key "$env_file" "GNSS_NTRIP_GGA_INTERVAL_S" "$GNSS_NTRIP_GGA_INTERVAL_S"
}

setup_env() {
  step "Environment (.env)"

  local env_file="$REPO_DIR/docker/.env"
  mkdir -p "$REPO_DIR/docker"

  : "${ROS_DOMAIN_ID:=0}"
  : "${MOWER_IP:=10.0.0.161}"
  : "${DISABLE_BLUETOOTH:=true}"
  : "${ENABLE_FOXGLOVE:=true}"

  # Main GNSS receiver.
  # docker/.env now carries fallback-only GNSS defaults for first boot and
  # headless recovery. The active operator configuration lives in YAML/GUI and
  # is resolved at runtime before these env values are consulted.
  : "${GNSS_BACKEND:=universal}"
  : "${GNSS_STATUS_SOURCE:=$(default_gnss_status_source)}"
  : "${GNSS_STACK:=$(default_gnss_stack)}"
  : "${GNSS_RECEIVER_FAMILY:=auto}"
  : "${GNSS_TRANSPORT:=serial}"
  : "${GNSS_SERIAL_DEVICE:=}"
  : "${GNSS_SERIAL_BAUD:=}"
  : "${GNSS_FRAME_ID:=gps_link}"
  : "${GNSS_NTRIP_ENABLED:=}"
  : "${GNSS_NTRIP_HOST:=}"
  : "${GNSS_NTRIP_PORT:=}"
  : "${GNSS_NTRIP_MOUNTPOINT:=}"
  : "${GNSS_NTRIP_USERNAME:=}"
  : "${GNSS_NTRIP_PASSWORD:=}"
  : "${GNSS_RTCM_FORWARDING:=}"
  : "${GNSS_NTRIP_GGA_ENABLED:=}"
  : "${GNSS_NTRIP_GGA_INTERVAL_S:=}"

  # LiDAR
  : "${LIDAR_ENABLED:=true}"
  : "${LIDAR_TYPE:=ldlidar}"
  : "${LIDAR_MODEL:=LDLiDAR_LD19}"
  : "${LIDAR_CONNECTION:=uart}"
  : "${LIDAR_PORT:=/dev/lidar}"
  : "${LIDAR_UART_DEVICE:=/dev/ttyAMA5}"
  : "${LIDAR_BAUD:=230400}"

  # TF-Luna
  : "${TFLUNA_FRONT_ENABLED:=false}"
  : "${TFLUNA_FRONT_PORT:=/dev/tfluna_front}"
  : "${TFLUNA_FRONT_UART_DEVICE:=/dev/ttyAMA3}"
  : "${TFLUNA_FRONT_BAUD:=115200}"

  : "${TFLUNA_EDGE_ENABLED:=false}"
  : "${TFLUNA_EDGE_PORT:=/dev/tfluna_edge}"
  : "${TFLUNA_EDGE_UART_DEVICE:=/dev/ttyAMA2}"
  : "${TFLUNA_EDGE_BAUD:=115200}"

  # Image tag — re-validate IMAGE_TAG (might have been loaded from .env
  # by load_env_defaults_file or set by --image-tag=/preset) and rebuild the
  # *_IMAGE_DEFAULT vars to match. mowglinext.sh unsets all *_IMAGE values
  # before this step, so the defaults below are what gets written.
  : "${IMAGE_TAG:=main}"
  if ! is_valid_image_tag "$IMAGE_TAG"; then
    warn "Invalid IMAGE_TAG=${IMAGE_TAG} in environment — defaulting to main"
    IMAGE_TAG="main"
  fi
  recompute_image_defaults

  # Images — select LiDAR image based on type
  : "${MOWGLI_ROS2_IMAGE:=${MOWGLI_ROS2_IMAGE_DEFAULT}}"
  : "${GPS_IMAGE:=${GPS_IMAGE_DEFAULT}}"
  : "${GUI_IMAGE:=${GUI_IMAGE_DEFAULT}}"
  : "${MAVROS_IMAGE:=${MAVROS_IMAGE_DEFAULT}}"
  if [[ -z "${LIDAR_IMAGE:-}" ]]; then
    case "${LIDAR_TYPE:-ldlidar}" in
      rplidar) LIDAR_IMAGE="${LIDAR_RPLIDAR_IMAGE_DEFAULT}" ;;
      stl27l)  LIDAR_IMAGE="${LIDAR_STL27L_IMAGE_DEFAULT}" ;;
      *)       LIDAR_IMAGE="${LIDAR_LDLIDAR_IMAGE_DEFAULT}" ;;
    esac
  fi

  # MAVROS / backend
  : "${HARDWARE_BACKEND:=mowgli}"
  : "${MAVROS_AUTOPILOT:=ardupilot}"
  : "${MAVROS_BY_ID:=}"
  : "${MAVROS_PORT:=/dev/mavros}"
  : "${MAVROS_BAUD:=921600}"
  : "${MAVROS_GCS_URL:=udp-b://@255.255.255.255:14550}" # udp-b = broadcast, udp = unicast, empty = disabled
  : "${MAVROS_TGT_SYSTEM:=1}"
  : "${MAVROS_TGT_COMPONENT:=1}"

  # Si MAVROS → GNSS backend désactivé
  if [[ "$HARDWARE_BACKEND" == "mavros" ]]; then
    GNSS_BACKEND="disabled"
    GNSS_STACK="disabled"
  elif [[ "${GNSS_BACKEND:-universal}" == "nmea" ]]; then
    warn_legacy_nmea_backend_once
    GNSS_BACKEND="universal"
    GNSS_RECEIVER_FAMILY="nmea"
  elif ! is_supported_gnss_backend "${GNSS_BACKEND:-universal}"; then
    warn "Unknown GNSS_BACKEND=${GNSS_BACKEND:-unset} — defaulting to universal"
    GNSS_BACKEND="universal"
  fi

  sync_gnss_env_contract_values

  local enable_mavros="false"
  if [[ "$HARDWARE_BACKEND" == "mavros" ]]; then
    enable_mavros="true"
  fi
  MAVROS_ENABLED="$enable_mavros"

  touch "$env_file"

  upsert_env_key "$env_file" "ROS_DOMAIN_ID" "$ROS_DOMAIN_ID"
  upsert_env_key "$env_file" "MOWER_IP" "$MOWER_IP"
  upsert_env_key "$env_file" "DISABLE_BLUETOOTH" "$DISABLE_BLUETOOTH"
  upsert_env_key "$env_file" "ENABLE_FOXGLOVE" "$ENABLE_FOXGLOVE"
  upsert_env_key "$env_file" "IMAGE_TAG" "$IMAGE_TAG"

  upsert_env_key "$env_file" "GNSS_BACKEND" "$(effective_gnss_backend 2>/dev/null || printf 'universal\n')"
  upsert_env_key "$env_file" "GNSS_STATUS_SOURCE" "$GNSS_STATUS_SOURCE"
  write_gnss_env_contract_keys "$env_file"

  upsert_env_key "$env_file" "LIDAR_ENABLED" "$LIDAR_ENABLED"
  upsert_env_key "$env_file" "LIDAR_TYPE" "$LIDAR_TYPE"
  upsert_env_key "$env_file" "LIDAR_MODEL" "$LIDAR_MODEL"
  upsert_env_key "$env_file" "LIDAR_CONNECTION" "$LIDAR_CONNECTION"
  upsert_env_key "$env_file" "LIDAR_PORT" "$LIDAR_PORT"
  upsert_env_key "$env_file" "LIDAR_UART_DEVICE" "$LIDAR_UART_DEVICE"
  upsert_env_key "$env_file" "LIDAR_BAUD" "$LIDAR_BAUD"

  upsert_env_key "$env_file" "TFLUNA_FRONT_ENABLED" "$TFLUNA_FRONT_ENABLED"
  upsert_env_key "$env_file" "TFLUNA_FRONT_PORT" "$TFLUNA_FRONT_PORT"
  upsert_env_key "$env_file" "TFLUNA_FRONT_UART_DEVICE" "$TFLUNA_FRONT_UART_DEVICE"
  upsert_env_key "$env_file" "TFLUNA_FRONT_BAUD" "$TFLUNA_FRONT_BAUD"

  upsert_env_key "$env_file" "TFLUNA_EDGE_ENABLED" "$TFLUNA_EDGE_ENABLED"
  upsert_env_key "$env_file" "TFLUNA_EDGE_PORT" "$TFLUNA_EDGE_PORT"
  upsert_env_key "$env_file" "TFLUNA_EDGE_UART_DEVICE" "$TFLUNA_EDGE_UART_DEVICE"
  upsert_env_key "$env_file" "TFLUNA_EDGE_BAUD" "$TFLUNA_EDGE_BAUD"

  upsert_env_key "$env_file" "MOWGLI_ROS2_IMAGE" "$MOWGLI_ROS2_IMAGE"
  upsert_env_key "$env_file" "GPS_IMAGE" "$GPS_IMAGE"
  upsert_env_key "$env_file" "LIDAR_IMAGE" "$LIDAR_IMAGE"
  upsert_env_key "$env_file" "MAVROS_IMAGE" "$MAVROS_IMAGE"
  upsert_env_key "$env_file" "GUI_IMAGE" "$GUI_IMAGE"

  upsert_env_key "$env_file" "HARDWARE_BACKEND" "$HARDWARE_BACKEND"
  upsert_env_key "$env_file" "MAVROS_ENABLED" "$MAVROS_ENABLED"
  upsert_env_key "$env_file" "MAVROS_BY_ID" "$MAVROS_BY_ID"
  upsert_env_key "$env_file" "MAVROS_PORT" "$MAVROS_PORT"
  upsert_env_key "$env_file" "MAVROS_BAUD" "$MAVROS_BAUD"
  upsert_env_key "$env_file" "MAVROS_GCS_URL" "$MAVROS_GCS_URL"
  upsert_env_key "$env_file" "MAVROS_TGT_SYSTEM" "$MAVROS_TGT_SYSTEM"
  upsert_env_key "$env_file" "MAVROS_TGT_COMPONENT" "$MAVROS_TGT_COMPONENT"
  upsert_env_key "$env_file" "MAVROS_AUTOPILOT" "$MAVROS_AUTOPILOT"

  remove_legacy_gnss_env_keys "$env_file"
  remove_env_key "$env_file" "NMEA_IMAGE"
  
  info "Backend selection : HARDWARE_BACKEND=$HARDWARE_BACKEND GNSS_BACKEND=$GNSS_BACKEND GNSS_STACK=$GNSS_STACK"
  info "Updated $env_file"
}
