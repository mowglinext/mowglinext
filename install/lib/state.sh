#!/usr/bin/env bash

STATE_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STATE_INSTALL_DIR="$(cd "${STATE_LIB_DIR}/.." && pwd)"

STATE_PARSED_KEYS=()
STATE_PARSED_VALUES=()
STATE_ACTIVE_PRESET_FILE=""
STATE_ACTIVE_PRESET_COUNT=0

is_allowed_installer_key() {
  local key="${1:-}"

  case "$key" in
    ROS_DOMAIN_ID|MOWER_IP|DISABLE_BLUETOOTH|ENABLE_FOXGLOVE|\
    GNSS_BACKEND|GPS_CONNECTION|GPS_PROTOCOL|GPS_PORT|GPS_BY_ID|GPS_UART_DEVICE|GPS_BAUD|UNICORE_COM_PORT|\
    UBLOX_DEVICE_SERIAL_STRING|\
    GPS_DEBUG_ENABLED|GPS_DEBUG_PORT|GPS_DEBUG_UART_DEVICE|GPS_DEBUG_BAUD|\
    LIDAR_ENABLED|LIDAR_TYPE|LIDAR_MODEL|LIDAR_CONNECTION|LIDAR_PORT|LIDAR_UART_DEVICE|LIDAR_BAUD|\
    TFLUNA_FRONT_ENABLED|TFLUNA_FRONT_PORT|TFLUNA_FRONT_UART_DEVICE|TFLUNA_FRONT_BAUD|\
    TFLUNA_EDGE_ENABLED|TFLUNA_EDGE_PORT|TFLUNA_EDGE_UART_DEVICE|TFLUNA_EDGE_BAUD|\
    MOWGLI_ROS2_IMAGE|GPS_IMAGE|UNICORE_IMAGE|LIDAR_IMAGE|MAVROS_IMAGE|NMEA_IMAGE|GUI_IMAGE|\
    HARDWARE_BACKEND|MAVROS_ENABLED|MAVROS_BY_ID|MAVROS_PORT|MAVROS_BAUD|MAVROS_GCS_URL|\
    MAVROS_TGT_SYSTEM|MAVROS_TGT_COMPONENT|MAVROS_AUTOPILOT|\
    UBLOX_DEVICE_FAMILY|ENABLE_VESC|VESC_IMAGE|VESC_CAN_INTERFACE|RANGE_IMAGE)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

_normalize_kv_value() {
  local value="${1-}"

  if [[ "$value" =~ ^\".*\"$ ]]; then
    value="${value:1:${#value}-2}"
  elif [[ "$value" =~ ^\'.*\'$ ]]; then
    value="${value:1:${#value}-2}"
  fi

  printf '%s' "$value"
}

preset_file_path() {
  local install_dir="${SCRIPT_DIR:-$STATE_INSTALL_DIR}"
  printf '%s/.preset\n' "$install_dir"
}

consumed_preset_file_path() {
  local preset_file="${1:-$(preset_file_path)}"
  printf '%s.consumed\n' "$preset_file"
}

parse_kv_file_strict() {
  local file="${1:?parse_kv_file_strict: missing file path}"
  local line trimmed key value
  local lineno=0

  STATE_PARSED_KEYS=()
  STATE_PARSED_VALUES=()

  [ -f "$file" ] || return 1

  while IFS= read -r line || [ -n "$line" ]; do
    lineno=$((lineno + 1))
    line="${line%$'\r'}"

    if [ "$lineno" -eq 1 ]; then
      line="${line#$'\xEF\xBB\xBF'}"
    fi

    trimmed="${line#"${line%%[![:space:]]*}"}"

    [ -n "$trimmed" ] || continue
    [[ "$trimmed" == \#* ]] && continue

    if [[ "$trimmed" == export[[:space:]]* ]]; then
      trimmed="${trimmed#export}"
      trimmed="${trimmed#"${trimmed%%[![:space:]]*}"}"
    fi

    if [[ ! "$trimmed" =~ ^([A-Za-z_][A-Za-z0-9_]*)=(.*)$ ]]; then
      warn "Ignoring invalid KEY=VALUE line in ${file}:${lineno}"
      continue
    fi

    key="${BASH_REMATCH[1]}"
    value="${BASH_REMATCH[2]}"

    if ! is_allowed_installer_key "$key"; then
      warn "Ignoring unknown installer key '${key}' in ${file}:${lineno}"
      continue
    fi

    value="$(_normalize_kv_value "$value")"
    STATE_PARSED_KEYS+=("$key")
    STATE_PARSED_VALUES+=("$value")
  done < "$file"
}

load_env_defaults_file() {
  local file="${1:-$REPO_DIR/docker/.env}"
  local i key value

  if ! parse_kv_file_strict "$file"; then
    return 1
  fi

  for i in "${!STATE_PARSED_KEYS[@]}"; do
    key="${STATE_PARSED_KEYS[$i]}"
    value="${STATE_PARSED_VALUES[$i]}"
    printf -v "$key" '%s' "$value"
    export "$key"
  done

  info "Loaded previous configuration from ${file}"
}

backup_env_defaults_file() {
  local file="${1:-$REPO_DIR/docker/.env}"
  local backup

  [ -f "$file" ] || return 0

  backup="${file}.old.$(date +%Y%m%d_%H%M%S)"
  mv "$file" "$backup"
  info "Moved previous runtime environment aside: ${file} -> ${backup}"
}

load_preset_file() {
  local file="${1:-$(preset_file_path)}"
  local i key value

  STATE_ACTIVE_PRESET_FILE=""
  STATE_ACTIVE_PRESET_COUNT=0

  if ! parse_kv_file_strict "$file"; then
    return 1
  fi

  for i in "${!STATE_PARSED_KEYS[@]}"; do
    key="${STATE_PARSED_KEYS[$i]}"
    value="${STATE_PARSED_VALUES[$i]}"
    printf -v "$key" '%s' "$value"
  done

  STATE_ACTIVE_PRESET_COUNT="${#STATE_PARSED_KEYS[@]}"

  if [ "$STATE_ACTIVE_PRESET_COUNT" -gt 0 ]; then
    STATE_ACTIVE_PRESET_FILE="$file"
    info "Loaded preset values from ${file}"
  else
    warn "No valid preset keys found in ${file}"
  fi
}

mark_preset_consumed() {
  local file="${1:-${STATE_ACTIVE_PRESET_FILE:-}}"
  local consumed_file

  [ -n "$file" ] || return 0
  [ -f "$file" ] || return 0
  [ "${STATE_ACTIVE_PRESET_COUNT:-0}" -gt 0 ] || return 0

  consumed_file="$(consumed_preset_file_path "$file")"
  mv -f "$file" "$consumed_file"
  info "Consumed preset: ${consumed_file}"
  STATE_ACTIVE_PRESET_FILE=""
  STATE_ACTIVE_PRESET_COUNT=0
}
