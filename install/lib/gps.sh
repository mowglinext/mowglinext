#!/usr/bin/env bash

pick_serial_by_id() {
  local default_device="${1:-}"
  local by_id_dir="${SERIAL_BY_ID_DIR:-/dev/serial/by-id}"
  local candidates=()
  local choice
  local i=1

  if [ -d "$by_id_dir" ]; then
    while IFS= read -r path; do
      candidates+=("$path")
    done < <(find "$by_id_dir" -maxdepth 1 -type l | sort)
  fi

  if [ "${#candidates[@]}" -eq 0 ]; then
    error "No USB serial device found in $by_id_dir"
    return 1
  fi

  echo ""
  info "Detected USB serial device(s):"
  for path in "${candidates[@]}"; do
    if [ -L "$path" ]; then
      echo "  ${i}) $path -> $(readlink -f "$path")"
    else
      echo "  ${i}) $path"
    fi
    i=$((i + 1))
  done

  local default_idx=""
  if [ -n "$default_device" ]; then
    for i in "${!candidates[@]}"; do
      if [ "${candidates[$i]}" = "$default_device" ]; then
        default_idx="$((i + 1))"
        break
      fi
    done
  fi

  prompt "$MSG_CHOICE" "${default_idx:-1}"
  choice="$REPLY"

  if ! [[ "$choice" =~ ^[0-9]+$ ]] || [ "$choice" -lt 1 ] || [ "$choice" -gt "${#candidates[@]}" ]; then
    error "Invalid USB serial device selection"
    return 1
  fi

  REPLY="${candidates[$((choice - 1))]}"
}

preset_key_loaded() {
  local wanted="${1:?preset_key_loaded: missing key}"
  local key

  [ "${STATE_ACTIVE_PRESET_COUNT:-0}" -gt 0 ] || return 1

  for key in "${STATE_PARSED_KEYS[@]}"; do
    [ "$key" = "$wanted" ] && return 0
  done

  return 1
}

configure_gps() {
  step "Universal GNSS configuration"

  GPS_UART_RULE=""
  GPS_DEBUG_UART_RULE=""
  GPS_DEBUG_ENABLED="false"
  GPS_DEBUG_UART_DEVICE=""
  : "${GPS_DEBUG_PORT:=/dev/gps_debug}"
  : "${GPS_DEBUG_BAUD:=115200}"
  : "${GNSS_STATUS_SOURCE:=universal}"
  : "${GNSS_STACK:=universal}"
  : "${GNSS_TRANSPORT:=serial}"
  : "${GNSS_RECEIVER_FAMILY:=auto}"
  : "${GNSS_SERIAL_BAUD:=}"
  : "${GPS_CONNECTION:=}"
  : "${GPS_BAUD:=}"
  : "${GPS_BY_ID:=}"
  : "${GPS_UART_DEVICE:=/dev/ttyAMA4}"
  : "${GPS_PORT:=/dev/gps}"
  : "${UBLOX_DEVICE_SERIAL_STRING:=}"

  local serial_preconfigured=false
  local baud_preconfigured=false
  local probe_mode="ask"
  local receiver_family
  local compat_backend
  local compat_protocol
  local connection
  local probe_port=""
  local default_baud="921600"

  if [[ "${GNSS_BACKEND:-}" == "nmea" ]]; then
    warn_legacy_nmea_backend_once
    GNSS_BACKEND="gps"
    GNSS_RECEIVER_FAMILY="nmea"
    GPS_PROTOCOL="NMEA"
  elif [[ "${GNSS_BACKEND:-}" == "ublox" ]] && [[ -z "${GNSS_RECEIVER_FAMILY:-}" || "${GNSS_RECEIVER_FAMILY:-}" == "auto" ]]; then
    GNSS_RECEIVER_FAMILY="ublox"
  elif [[ "${GNSS_BACKEND:-}" == "unicore" ]] && [[ -z "${GNSS_RECEIVER_FAMILY:-}" || "${GNSS_RECEIVER_FAMILY:-}" == "auto" ]]; then
    GNSS_RECEIVER_FAMILY="unicore"
  elif [[ "${GPS_PROTOCOL:-UBX}" == "NMEA" ]] && [[ -z "${GNSS_RECEIVER_FAMILY:-}" || "${GNSS_RECEIVER_FAMILY:-}" == "auto" ]]; then
    GNSS_RECEIVER_FAMILY="nmea"
  fi

  if [[ "$(effective_gnss_backend "${GNSS_BACKEND:-gps}")" == "disabled" ]]; then
    info "Direct GNSS configuration disabled for HARDWARE_BACKEND=${HARDWARE_BACKEND:-mowgli}"
    return 0
  fi

  if [[ "${PRESET_LOADED:-false}" == "true" ]]; then
    if [ "${STATE_ACTIVE_PRESET_COUNT:-0}" -gt 0 ]; then
      if preset_key_loaded GNSS_SERIAL_DEVICE \
        || preset_key_loaded GPS_BY_ID \
        || preset_key_loaded GPS_UART_DEVICE \
        || preset_key_loaded GPS_PORT; then
        serial_preconfigured=true
      fi
      if preset_key_loaded GNSS_SERIAL_BAUD || preset_key_loaded GPS_BAUD; then
        baud_preconfigured=true
      fi
    else
      if [[ -n "${GNSS_SERIAL_DEVICE:-}" || -n "${GPS_BY_ID:-}" ]]; then
        serial_preconfigured=true
      elif [[ "${GPS_CONNECTION:-}" == "uart" && -n "${GPS_UART_DEVICE:-}" ]]; then
        serial_preconfigured=true
      elif [[ "${GPS_CONNECTION:-}" == "usb" && ( -n "${GPS_BY_ID:-}" || "${GPS_PORT:-}" == /dev/serial/by-id/* ) ]]; then
        serial_preconfigured=true
      fi
      if [[ -n "${GNSS_SERIAL_BAUD:-}" || -n "${GPS_BAUD:-}" ]]; then
        baud_preconfigured=true
      fi
    fi
  fi

  GNSS_STACK="universal"
  GNSS_STATUS_SOURCE="universal"
  GNSS_TRANSPORT="serial"
  receiver_family="$(normalize_gnss_receiver_family "${GNSS_RECEIVER_FAMILY:-auto}")"
  GNSS_RECEIVER_FAMILY="$receiver_family"
  compat_backend="$(gnss_receiver_family_to_compat_backend "$receiver_family")"
  compat_protocol="$(gnss_receiver_family_to_gps_protocol "$receiver_family")"

  if [[ -z "${GNSS_SERIAL_DEVICE:-}" ]]; then
    case "${GPS_CONNECTION:-$(gnss_connection_from_serial_device)}" in
      usb)
        if [[ -n "${GPS_BY_ID:-}" ]]; then
          GNSS_SERIAL_DEVICE="$GPS_BY_ID"
        elif [[ "${GPS_PORT:-}" == /dev/serial/by-id/* ]]; then
          GNSS_SERIAL_DEVICE="$GPS_PORT"
        elif [[ "${UBLOX_DEVICE_SERIAL_STRING:-}" == /dev/serial/by-id/* ]]; then
          GNSS_SERIAL_DEVICE="$UBLOX_DEVICE_SERIAL_STRING"
        fi
        ;;
      uart)
        if [[ -n "${GPS_UART_DEVICE:-}" && "${GPS_UART_DEVICE:-}" != "/dev/gps" ]]; then
          GNSS_SERIAL_DEVICE="$GPS_UART_DEVICE"
        elif [[ -n "${GPS_PORT:-}" && "${GPS_PORT:-}" != "/dev/gps" ]]; then
          GNSS_SERIAL_DEVICE="$GPS_PORT"
        fi
        ;;
    esac
  fi

  if [[ -z "${GNSS_SERIAL_BAUD:-}" && -n "${GPS_BAUD:-}" ]]; then
    GNSS_SERIAL_BAUD="$GPS_BAUD"
  fi

  connection="$(gnss_connection_from_serial_device "${GNSS_SERIAL_DEVICE:-}")"

  if [[ "$serial_preconfigured" != "true" || -z "${GNSS_SERIAL_DEVICE:-}" ]]; then
    local connection_default="2"
    [[ "$connection" == "usb" ]] && connection_default="1"
    echo ""
    echo "$MSG_GPS_CONNECTION"
    echo "  1) USB"
    echo "  2) UART"
    prompt "$MSG_CHOICE" "$connection_default"

    case "$REPLY" in
      1)
        connection="usb"
        pick_serial_by_id "${GNSS_SERIAL_DEVICE:-${GPS_BY_ID:-}}" || return 1
        GNSS_SERIAL_DEVICE="$REPLY"
        ;;
      2)
        connection="uart"
        pick_uart_port "${GNSS_SERIAL_DEVICE:-${GPS_UART_DEVICE:-/dev/ttyAMA4}}"
        GNSS_SERIAL_DEVICE="$REPLY"
        ;;
      *)
        error "$MSG_GPS_INVALID_CONNECTION"
        return 1
        ;;
    esac
  else
    info "Universal GNSS device pre-configured: ${GNSS_SERIAL_DEVICE}"
    probe_mode="auto"
  fi

  default_baud="${GNSS_SERIAL_BAUD:-${GPS_BAUD:-921600}}"
  probe_port="${GNSS_SERIAL_DEVICE:-}"
  if [[ "$baud_preconfigured" != "true" && -n "$probe_port" ]]; then
    GNSS_BACKEND="$compat_backend"
    GPS_PROTOCOL="$compat_protocol"
    prompt_or_probe_baud "$probe_port" "$compat_backend" "$compat_protocol" "$default_baud" "$probe_mode"
    GNSS_SERIAL_BAUD="$REPLY"
    GPS_BAUD="$GNSS_SERIAL_BAUD"
    maybe_upgrade_unicore_baud "$probe_port" "$GNSS_SERIAL_BAUD" "$probe_mode"
    maybe_upgrade_ublox_baud "$probe_port" "$GNSS_SERIAL_BAUD" "$probe_mode"
    GNSS_SERIAL_BAUD="${GPS_BAUD:-$GNSS_SERIAL_BAUD}"
  elif [[ -z "${GNSS_SERIAL_BAUD:-}" ]]; then
    GNSS_SERIAL_BAUD="$default_baud"
  fi

  sync_legacy_gps_compat_from_gnss

  if [ "$GPS_CONNECTION" = "uart" ] && [ -n "${GPS_UART_DEVICE:-}" ]; then
    local gps_kernel
    gps_kernel="$(basename "$GPS_UART_DEVICE")"
    GPS_UART_RULE="KERNEL==\"${gps_kernel}\", SYMLINK+=\"gps\", MODE=\"0666\""
  fi

  echo ""
  info "Universal GNSS : receiver_family=$GNSS_RECEIVER_FAMILY transport=$GNSS_TRANSPORT device=$GNSS_SERIAL_DEVICE baud=$GNSS_SERIAL_BAUD"
  info "Compatibility  : GNSS_BACKEND=$GNSS_BACKEND GPS_CONNECTION=$GPS_CONNECTION GPS_PROTOCOL=$GPS_PROTOCOL GPS_PORT=$GPS_PORT"
  if [ -n "${GPS_BY_ID:-}" ]; then
    info "USB by-id path  : $GPS_BY_ID"
  fi
  return 0
}

run_gps_configuration_step() {
  configure_gps
}
