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
  step "GPS configuration"

  # Reset generated rules
  GPS_UART_RULE=""
  GPS_DEBUG_UART_RULE=""
  : "${GNSS_BACKEND:=gps}"
  local gnss_preconfigured=false
  local gps_preconfigured=false
  local gps_baud_preconfigured=false
  local ublox_serial_preconfigured=false

  if [[ "${GNSS_BACKEND:-}" == "nmea" ]]; then
    warn_legacy_nmea_backend_once
    GNSS_BACKEND="gps"
    GPS_PROTOCOL="NMEA"
  fi

  if [[ "${PRESET_LOADED:-false}" == "true" ]]; then
    if [ "${STATE_ACTIVE_PRESET_COUNT:-0}" -gt 0 ]; then
      preset_key_loaded GNSS_BACKEND && gnss_preconfigured=true
      if preset_key_loaded GPS_CONNECTION && preset_key_loaded GPS_PROTOCOL; then
        gps_preconfigured=true
      fi
      preset_key_loaded GPS_BAUD && gps_baud_preconfigured=true
      preset_key_loaded UBLOX_DEVICE_SERIAL_STRING && ublox_serial_preconfigured=true
    else
      [ -n "${GNSS_BACKEND:-}" ] && gnss_preconfigured=true
      if [ -n "${GPS_CONNECTION:-}" ] && [ -n "${GPS_PROTOCOL:-}" ]; then
        gps_preconfigured=true
      fi
      [ -n "${GPS_BAUD:-}" ] && gps_baud_preconfigured=true
      [ -n "${UBLOX_DEVICE_SERIAL_STRING:-}" ] && ublox_serial_preconfigured=true
    fi
  fi

  # If preset values exist (from web composer or CLI), skip interactive prompts
  if [[ "$gnss_preconfigured" == "true" ]]; then
    if ! is_supported_gnss_backend "${GNSS_BACKEND}"; then
      error "Invalid GNSS_BACKEND preset: ${GNSS_BACKEND} (expected: $(list_supported_gnss_backends))"
      return 1
    fi

    if [[ "$(effective_gnss_backend "${GNSS_BACKEND}")" == "disabled" ]]; then
      info "Direct GNSS configuration disabled for HARDWARE_BACKEND=${HARDWARE_BACKEND:-mowgli}"
      return 0
    fi
  fi

  # Skip GPS prompts only when the current preset provided the GPS details.
  # Stale GPS_* values loaded from docker/.env must not turn --gnss=unicore
  # into an incomplete USB preset without GPS_BY_ID.
  if [[ "$gnss_preconfigured" == "true" && "$gps_preconfigured" == "true" ]]; then

    : "${GPS_PORT:=/dev/gps}"
    : "${GPS_BY_ID:=}"
    : "${UBLOX_DEVICE_SERIAL_STRING:=}"
    : "${GPS_DEBUG_ENABLED:=false}"
    : "${GPS_DEBUG_PORT:=/dev/gps_debug}"
    : "${GPS_DEBUG_BAUD:=115200}"

    if [[ "${GNSS_BACKEND}" == "ublox" ]]; then
      if [[ "${GPS_CONNECTION}" != "usb" ]]; then
        error "GNSS_BACKEND=ublox only supports the dedicated USB/libusb driver. Use GNSS_BACKEND=gps with GPS_PROTOCOL=UBX for UART receivers."
        return 1
      fi
      if [[ "${GPS_PROTOCOL}" != "UBX" ]]; then
        error "GNSS_BACKEND=ublox requires GPS_PROTOCOL=UBX."
        return 1
      fi
      if [[ "$ublox_serial_preconfigured" != "true" || -z "${UBLOX_DEVICE_SERIAL_STRING:-}" ]]; then
        error "UBLOX_DEVICE_SERIAL_STRING is required for GNSS_BACKEND=ublox. The dedicated USB/libusb driver does not use GPS_PORT=/dev/gps."
        return 1
      fi
    fi

    if [[ "${GNSS_BACKEND}" == "unicore" && "${GPS_CONNECTION}" == "usb" && -z "${GPS_BY_ID:-}" ]]; then
      error "GPS_BY_ID is required for GNSS_BACKEND=unicore with GPS_CONNECTION=usb"
      return 1
    fi

    info "GNSS backend pre-configured: ${GNSS_BACKEND}"
    info "GPS pre-configured (skipping prompts)"

    # For UART connections, always let user confirm/change the port
    if [[ "${GPS_CONNECTION}" == "uart" ]]; then
      pick_uart_port "${GPS_UART_DEVICE:-/dev/ttyAMA4}"
      GPS_UART_DEVICE="$REPLY"
    fi

    if [[ "${GNSS_BACKEND}" != "ublox" && "$gps_baud_preconfigured" != "true" ]]; then
      local probe_port=""
      if [[ "${GPS_CONNECTION}" == "uart" ]]; then
        probe_port="${GPS_UART_DEVICE:-}"
      elif [[ "${GPS_CONNECTION}" == "usb" ]]; then
        probe_port="${GPS_BY_ID:-${GPS_PORT:-}}"
      fi

      if [ -n "$probe_port" ]; then
        prompt_or_probe_baud "$probe_port" "${GNSS_BACKEND:-gps}" "${GPS_PROTOCOL:-UBX}" "${GPS_BAUD:-921600}" "auto"
        GPS_BAUD="$REPLY"
        maybe_upgrade_unicore_baud "$probe_port" "$GPS_BAUD" "auto"
        maybe_upgrade_ublox_baud "$probe_port" "$GPS_BAUD" "auto"
      fi
    fi

    if [[ "${GPS_DEBUG_ENABLED}" == "true" ]]; then
      pick_uart_port "${GPS_DEBUG_UART_DEVICE:-/dev/ttyS0}"
      GPS_DEBUG_UART_DEVICE="$REPLY"
    fi
  else
    if [[ "$(effective_gnss_backend)" == "disabled" ]]; then
      info "Direct GNSS configuration disabled for HARDWARE_BACKEND=${HARDWARE_BACKEND:-mowgli}"
      return 0
    fi

    if [[ "$gnss_preconfigured" == "true" ]]; then
      info "GNSS backend pre-configured: ${GNSS_BACKEND}"
    else
      echo ""
      echo "Select GNSS backend:"
      echo "  1) Generic GPS (legacy container, UBX or NMEA)"
      echo "  2) u-blox (F9P, UBX HP + NTRIP bundled)"
      echo "  3) Unicore (UM98x)"
      prompt "$MSG_CHOICE" "1"
      local gnss_choice="$REPLY"

      case "$gnss_choice" in
        1)
          GNSS_BACKEND="gps"
          ;;
        2)
          GNSS_BACKEND="ublox"
          ;;
        3)
          GNSS_BACKEND="unicore"
          ;;
        *)
          error "Invalid GNSS backend choice"
          return 1
          ;;
      esac
    fi

    # Defaults based on PCB / GUI-ready
    : "${GPS_PROTOCOL:=UBX}"
    : "${GPS_CONNECTION:=uart}"
    : "${GPS_PORT:=/dev/gps}"
    : "${GPS_UART_DEVICE:=/dev/ttyAMA4}"
    : "${UBLOX_DEVICE_SERIAL_STRING:=}"
    # GPS_BAUD is the single runtime baud target for the main GNSS receiver.
    : "${GPS_BAUD:=921600}"

    # Debug only on miniUART
    : "${GPS_DEBUG_ENABLED:=false}"
    : "${GPS_DEBUG_PORT:=/dev/gps_debug}"
    : "${GPS_DEBUG_UART_DEVICE:=/dev/ttyS0}"
    : "${GPS_DEBUG_BAUD:=115200}"

    if [[ "${GNSS_BACKEND}" == "ublox" ]]; then
      GPS_CONNECTION="usb"
      GPS_PROTOCOL="UBX"
      GPS_UART_DEVICE=""
      GPS_BY_ID=""
      GPS_BAUD="921600"

      echo ""
      info "GNSS_BACKEND=ublox uses the dedicated USB/libusb u-blox driver."
      info "UART u-blox receivers should use GNSS_BACKEND=gps with GPS_PROTOCOL=UBX."
      prompt "u-blox USB serial string?" "${UBLOX_DEVICE_SERIAL_STRING:-}"
      UBLOX_DEVICE_SERIAL_STRING="$REPLY"
      if [[ -z "${UBLOX_DEVICE_SERIAL_STRING:-}" ]]; then
        error "UBLOX_DEVICE_SERIAL_STRING is required for GNSS_BACKEND=ublox"
        return 1
      fi
    else
      echo ""
      echo "$MSG_GPS_CONNECTION"
      echo "  1) USB"
      echo "  2) UART"
      prompt "$MSG_CHOICE" "2"
      local conn_choice="$REPLY"

      case "$conn_choice" in
        1)
          GPS_CONNECTION="usb"
          GPS_UART_DEVICE=""
          if [ "$GNSS_BACKEND" = "unicore" ]; then
            pick_serial_by_id "${GPS_BY_ID:-}" || return 1
            GPS_BY_ID="$REPLY"
          fi
          ;;
        2)
          GPS_CONNECTION="uart"
          GPS_BY_ID=""
          pick_uart_port "/dev/ttyAMA4"
          GPS_UART_DEVICE="$REPLY"
          ;;
        *)
          error "$MSG_GPS_INVALID_CONNECTION"
          return 1
          ;;
      esac

      echo ""
      echo "$MSG_GPS_PROTOCOL"
      echo "  1) UBX"
      echo "  2) NMEA"
      prompt "$MSG_CHOICE" "1"
      local proto_choice="$REPLY"

      case "$proto_choice" in
        1)
          GPS_PROTOCOL="UBX"
          GPS_BAUD="921600"
          ;;
        2)
          GPS_PROTOCOL="NMEA"
          GPS_BAUD="921600"
          ;;
        *)
          error "$MSG_GPS_INVALID_PROTOCOL"
          return 1
          ;;
      esac
    fi

    local probe_port=""
    local default_baud="${GPS_BAUD:-921600}"

    if [[ "${GNSS_BACKEND}" == "ublox" ]]; then
      probe_port=""
    elif [[ "${GPS_CONNECTION}" == "uart" ]]; then
      probe_port="${GPS_UART_DEVICE:-}"
    elif [[ "${GPS_CONNECTION}" == "usb" ]]; then
      probe_port="${GPS_BY_ID:-${GPS_PORT:-}}"
    fi

    if [ -n "$probe_port" ]; then
      prompt_or_probe_baud "$probe_port" "${GNSS_BACKEND:-gps}" "${GPS_PROTOCOL:-UBX}" "$default_baud" "ask"
      GPS_BAUD="$REPLY"
      maybe_upgrade_unicore_baud "$probe_port" "$GPS_BAUD" "ask"
      maybe_upgrade_ublox_baud "$probe_port" "$GPS_BAUD" "ask"
    fi

    echo ""
    if confirm "$MSG_GPS_DEBUG_CONFIRM"; then
      GPS_DEBUG_ENABLED="true"
      pick_uart_port "/dev/ttyS0"
      GPS_DEBUG_UART_DEVICE="$REPLY"
    else
      GPS_DEBUG_ENABLED="false"
      GPS_DEBUG_UART_DEVICE=""
    fi
  fi

  # Main GPS rule only if UART is selected
  if [ "$GPS_CONNECTION" = "uart" ] && [ -n "${GPS_UART_DEVICE:-}" ]; then
    local gps_kernel
    gps_kernel="$(basename "$GPS_UART_DEVICE")"
    GPS_UART_RULE="KERNEL==\"${gps_kernel}\", SYMLINK+=\"gps\", MODE=\"0666\""
  fi

  # Debug GPS rule only if enabled
  if [ "${GPS_DEBUG_ENABLED:-false}" = "true" ] && [ -n "${GPS_DEBUG_UART_DEVICE:-}" ]; then
    local gps_debug_kernel
    gps_debug_kernel="$(basename "$GPS_DEBUG_UART_DEVICE")"
    GPS_DEBUG_UART_RULE="KERNEL==\"${gps_debug_kernel}\", SYMLINK+=\"gps_debug\", MODE=\"0666\""
  fi

  echo ""
  info "$MSG_GPS_MAIN : backend=$GNSS_BACKEND connection=$GPS_CONNECTION protocol=$GPS_PROTOCOL port=$GPS_PORT uart=${GPS_UART_DEVICE:-none} baud=$GPS_BAUD"
  [ -n "${GPS_BY_ID:-}" ] && info "GPS USB by-id  : $GPS_BY_ID"
  [ -n "${UBLOX_DEVICE_SERIAL_STRING:-}" ] && info "u-blox USB serial string : $UBLOX_DEVICE_SERIAL_STRING"
  info "GPS debug     : enabled=$GPS_DEBUG_ENABLED port=$GPS_DEBUG_PORT uart=${GPS_DEBUG_UART_DEVICE:-none} baud=$GPS_DEBUG_BAUD"
}

run_gps_configuration_step() {
  configure_gps
}
