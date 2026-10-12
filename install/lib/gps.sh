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

  if [[ "${NON_INTERACTIVE:-false}" == "true" && -z "$default_idx" && "${#candidates[@]}" -gt 1 ]]; then
    warn "Several USB serial devices found; taking the first — pass --gnss-device= to choose"
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

# The installer only wires the GNSS serial LINK: transport (uart/usb) and the
# device path, which drive the udev symlink and the UART overlay. Receiver
# family, baud upgrade, profile and NTRIP belong to the sidecar
# (gnss_config_apply at container start) and the GUI; the values recorded here
# are first-boot defaults only. There is deliberately no baud probing.
configure_gps() {
  step "GNSS serial link"

  if declare -F apply_existing_yaml_gnss_state >/dev/null 2>&1; then
    apply_existing_yaml_gnss_state
  fi

  : "${GNSS_BACKEND:=universal}"
  : "${GNSS_STATUS_SOURCE:=universal}"
  : "${GNSS_STACK:=universal}"
  : "${GNSS_TRANSPORT:=serial}"
  : "${GNSS_RECEIVER_FAMILY:=auto}"
  : "${GNSS_SERIAL_DEVICE:=}"
  : "${GNSS_SERIAL_BAUD:=}"
  : "${GNSS_CONNECTION_HINT:=}"

  local serial_preconfigured=false
  local connection

  if [[ "$(effective_gnss_backend "${GNSS_BACKEND:-universal}")" == "disabled" ]]; then
    info "Direct GNSS configuration disabled for HARDWARE_BACKEND=${HARDWARE_BACKEND:-mowgli}"
    return 0
  fi

  if [[ "${PRESET_LOADED:-false}" == "true" ]]; then
    if [ "${STATE_ACTIVE_PRESET_COUNT:-0}" -gt 0 ]; then
      preset_key_loaded GNSS_SERIAL_DEVICE && serial_preconfigured=true
    elif [[ -n "${GNSS_SERIAL_DEVICE:-}" ]]; then
      serial_preconfigured=true
    fi
  fi

  GNSS_BACKEND="universal"
  GNSS_STACK="universal"
  GNSS_STATUS_SOURCE="universal"
  GNSS_TRANSPORT="serial"
  GNSS_RECEIVER_FAMILY="$(normalize_gnss_receiver_family "${GNSS_RECEIVER_FAMILY:-auto}")"

  connection="$(gnss_connection_from_serial_device "${GNSS_SERIAL_DEVICE:-}")"
  [[ -n "$connection" ]] || connection="${GNSS_CONNECTION_HINT:-uart}"

  if [[ "$serial_preconfigured" != "true" || -z "${GNSS_SERIAL_DEVICE:-}" ]]; then
    local connection_default="2"
    [[ "$connection" == "usb" ]] && connection_default="1"
    echo ""
    echo "$MSG_GNSS_CONNECTION"
    echo "  1) USB"
    echo "  2) UART"
    prompt "$MSG_CHOICE" "$connection_default"

    case "$REPLY" in
      1)
        connection="usb"
        pick_serial_by_id "${GNSS_SERIAL_DEVICE:-}" || return 1
        GNSS_SERIAL_DEVICE="$REPLY"
        ;;
      2)
        connection="uart"
        local gnss_default="${GNSS_SERIAL_DEVICE:-}"
        openmower_uart_owner "$gnss_default" >/dev/null && gnss_default=""
        pick_free_uart_port "${gnss_default:-$(default_gnss_uart_device)}" "GNSS" || return 1
        GNSS_SERIAL_DEVICE="$REPLY"
        ;;
      *)
        error "$MSG_GPS_INVALID_CONNECTION"
        return 1
        ;;
    esac
  else
    info "GNSS device pre-configured: ${GNSS_SERIAL_DEVICE}"
    uart_port_is_free "$GNSS_SERIAL_DEVICE" "GNSS" || return 1
  fi

  # Explicit --gnss-baud wins, then the value already in mowgli_robot.yaml
  # (apply_existing_yaml_gnss_state), then the sidecar's canonical default.
  : "${GNSS_SERIAL_BAUD:=921600}"

  echo ""
  info "GNSS link : receiver_family=$GNSS_RECEIVER_FAMILY transport=$GNSS_TRANSPORT device=$GNSS_SERIAL_DEVICE baud=$GNSS_SERIAL_BAUD"
  return 0
}

run_gps_configuration_step() {
  configure_gps
}
