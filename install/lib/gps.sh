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

  echo ""

  if [ "${#candidates[@]}" -eq 0 ]; then
    warn "No USB serial by-id device found in $by_id_dir"
    warn "Falling back to manual USB serial port selection"

    echo "Enter USB serial device path:"
    echo "  examples: /dev/ttyACM0, /dev/ttyUSB0, /dev/gps"
    prompt "$MSG_CHOICE" "${default_device:-/dev/ttyACM0}"

    if [ ! -e "$REPLY" ] && [[ "$REPLY" != /dev/* ]]; then
      error "Invalid USB serial device path: $REPLY"
      return 1
    fi

    return 0
  fi

  info "Detected USB serial device(s):"
  for path in "${candidates[@]}"; do
    if [ -L "$path" ]; then
      echo "  ${i}) $path -> $(readlink -f "$path")"
    else
      echo "  ${i}) $path"
    fi
    i=$((i + 1))
  done

  echo "  m) Manual path"

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

  if [ "$choice" = "m" ] || [ "$choice" = "M" ]; then
    prompt "USB serial path" "${default_device:-/dev/ttyACM0}"
    return 0
  fi

  if ! [[ "$choice" =~ ^[0-9]+$ ]] || [ "$choice" -lt 1 ] || [ "$choice" -gt "${#candidates[@]}" ]; then
    error "Invalid USB serial device selection"
    return 1
  fi

  REPLY="${candidates[$((choice - 1))]}"
}

configure_gps() {
  step "GPS configuration"

  # Reset generated rules
  GPS_UART_RULE=""
  GPS_DEBUG_UART_RULE=""
  : "${GNSS_BACKEND:=gps}"

  # If preset values exist (from web composer or CLI), skip interactive prompts
  if [[ "${PRESET_LOADED:-false}" == "true" && -n "${GNSS_BACKEND:-}" && -n "${GPS_CONNECTION:-}" && -n "${GPS_PROTOCOL:-}" ]]; then
    case "${GNSS_BACKEND}" in
      gps|ublox|unicore|nmea) ;;
      *)
        error "Invalid GNSS_BACKEND preset: ${GNSS_BACKEND} (expected: gps, ublox, unicore, nmea)"
        return 1
        ;;
    esac

    : "${GPS_PORT:=/dev/gps}"
    : "${GPS_BY_ID:=}"
    : "${GPS_DEBUG_ENABLED:=false}"
    : "${GPS_DEBUG_PORT:=/dev/gps_debug}"
    : "${GPS_DEBUG_BAUD:=115200}"

    if [[ "${GNSS_BACKEND}" == "unicore" && "${GPS_CONNECTION}" == "usb" ]]; then
      : "${GPS_PORT:=/dev/gps}"

      if [ -n "${GPS_BY_ID:-}" ]; then
        info "Unicore USB by-id preset: ${GPS_BY_ID}"
      else
        warn "Unicore USB selected without GPS_BY_ID preset"
        warn "Using GPS_PORT=${GPS_PORT}"
      fi
    fi

    info "GNSS backend pre-configured: ${GNSS_BACKEND}"
    info "GPS pre-configured (skipping prompts)"

    # For UART connections, always let user confirm/change the port
    if [[ "${GPS_CONNECTION}" == "uart" ]]; then
      pick_uart_port "${GPS_UART_DEVICE:-/dev/ttyAMA4}"
      GPS_UART_DEVICE="$REPLY"
    fi
    if [[ "${GPS_DEBUG_ENABLED}" == "true" ]]; then
      pick_uart_port "${GPS_DEBUG_UART_DEVICE:-/dev/ttyS0}"
      GPS_DEBUG_UART_DEVICE="$REPLY"
    fi
  else
    echo ""
    echo "Select GNSS backend:"
    echo "  1) Generic GPS (legacy, UBX-only despite the name)"
    echo "  2) u-blox (F9P, UBX HP + NTRIP bundled)"
    echo "  3) Unicore (UM98x)"
    echo "  4) Generic NMEA (any NMEA-0183 receiver, UART or USB)"
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
      4)
        GNSS_BACKEND="nmea"
        ;;
      *)
        error "Invalid GNSS backend choice"
        return 1
        ;;
    esac

    # Defaults based on PCB / GUI-ready
    : "${GPS_PROTOCOL:=UBX}"
    : "${GPS_CONNECTION:=uart}"
    : "${GPS_PORT:=/dev/gps}"
    : "${GPS_UART_DEVICE:=/dev/ttyAMA4}"
    : "${GPS_BAUD:=460800}"

    # Debug only on miniUART
    : "${GPS_DEBUG_ENABLED:=false}"
    : "${GPS_DEBUG_PORT:=/dev/gps_debug}"
    : "${GPS_DEBUG_UART_DEVICE:=/dev/ttyS0}"
    : "${GPS_DEBUG_BAUD:=115200}"

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
        pick_usb_serial_device "GPS (${GNSS_BACKEND})" "${GPS_PORT:-/dev/ttyACM0}" || return 1

        if [ -n "${USB_SELECTED_BY_ID:-}" ]; then
          GPS_BY_ID="$USB_SELECTED_BY_ID"
          GPS_PORT="/dev/gps"
        else
          GPS_BY_ID=""
          GPS_PORT="$USB_SELECTED_PORT"
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
        GPS_BAUD="460800"
        ;;
      2)
        GPS_PROTOCOL="NMEA"
        GPS_BAUD="115200"
        ;;
      *)
        error "$MSG_GPS_INVALID_PROTOCOL"
        return 1
        ;;
    esac

    # Generic NMEA backend forces NMEA protocol and prompts for baud
    # since NMEA receivers vary widely (9600 default, 38400/115200 common
    # for higher rates).
    if [ "$GNSS_BACKEND" = "nmea" ]; then
      GPS_PROTOCOL="NMEA"
      echo ""
      echo "NMEA baud rate (typical: 9600, 38400, 115200):"
      prompt "$MSG_CHOICE" "9600"
      GPS_BAUD="$REPLY"
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

  # Unicore UART → 460800. The Mowgli PCB UART path (ttyAMA4) is wired
  # for 460800 (matches the F9P UBX rate), and the operator is expected
  # to configure the UM982 receiver to the same speed. The UBX/NMEA
  # protocol selector above is irrelevant for the Unicore driver — it
  # speaks Unicore-NMEA extensions natively — so we override whatever
  # baud the protocol path chose. USB is left alone (CDC-ACM doesn't
  # care about the configured rate).
  if [ "${GNSS_BACKEND:-}" = "unicore" ] && [ "${GPS_CONNECTION:-}" = "uart" ]; then
    GPS_BAUD="460800"
  fi

  echo ""
  info "$MSG_GPS_MAIN : backend=$GNSS_BACKEND connection=$GPS_CONNECTION protocol=$GPS_PROTOCOL port=$GPS_PORT uart=${GPS_UART_DEVICE:-none} baud=$GPS_BAUD"
  [ -n "${GPS_BY_ID:-}" ] && info "GPS USB by-id  : $GPS_BY_ID"
  info "GPS debug     : enabled=$GPS_DEBUG_ENABLED port=$GPS_DEBUG_PORT uart=${GPS_DEBUG_UART_DEVICE:-none} baud=$GPS_DEBUG_BAUD"
}

run_gps_configuration_step() {
  configure_gps
}
