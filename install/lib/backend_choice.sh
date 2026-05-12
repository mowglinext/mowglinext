#!/usr/bin/env bash

sselect_hardware_backend() {
  echo ""
  echo "Select hardware backend:"
  echo "  [1] Mowgli STM32 board"
  echo "  [2] Pixhawk via MAVROS"
  echo ""
  printf "Choice [1-2] (default: 1): "
  read -r choice
  choice="${choice:-1}"

  case "$choice" in
    1)
      export HARDWARE_BACKEND="mowgli"
      export MAVROS_BY_ID=""
      export MAVROS_PORT=""

      info "Selected backend: Mowgli STM32 board"

      pick_usb_serial_device "Mowgli STM32 board" "${MOWGLI_PORT:-/dev/ttyACM0}" || return 1

      if [ -n "${USB_SELECTED_BY_ID:-}" ]; then
        export MOWGLI_BY_ID="$USB_SELECTED_BY_ID"
        export MOWGLI_PORT="/dev/mowgli"
      else
        export MOWGLI_BY_ID=""
        export MOWGLI_PORT="$USB_SELECTED_PORT"
      fi

      info "Selected Mowgli port: ${MOWGLI_PORT}"
      [ -n "${MOWGLI_BY_ID:-}" ] && info "Mowgli USB by-id: ${MOWGLI_BY_ID}"
      ;;

    2)
      export HARDWARE_BACKEND="mavros"
      export MOWGLI_BY_ID=""
      export MOWGLI_PORT=""

      info "Selected backend: Pixhawk via MAVROS"

      select_mavros_autopilot || return 1
      select_mavros_gcs_mode || return 1

      pick_usb_serial_device "MAVROS / Pixhawk" "${MAVROS_PORT:-/dev/ttyACM0}" || return 1

      if [ -n "${USB_SELECTED_BY_ID:-}" ]; then
        export MAVROS_BY_ID="$USB_SELECTED_BY_ID"
        export MAVROS_PORT="/dev/mavros"
      else
        export MAVROS_BY_ID=""
        export MAVROS_PORT="$USB_SELECTED_PORT"
      fi

      info "Selected MAVROS port: ${MAVROS_PORT}"
      [ -n "${MAVROS_BY_ID:-}" ] && info "MAVROS USB by-id: ${MAVROS_BY_ID}"
      ;;

    *)
      error "Invalid choice"
      return 1
      ;;
  esac
}

pick_usb_serial_device "MAVROS / Pixhawk" "/dev/ttyACM0" || return 1

if [ -n "${USB_SELECTED_BY_ID:-}" ]; then
  export MAVROS_BY_ID="$USB_SELECTED_BY_ID"
  export MAVROS_PORT="/dev/mavros"
else
  export MAVROS_BY_ID=""
  export MAVROS_PORT="$USB_SELECTED_PORT"
fi

  if [ -d "$byid_dir" ]; then
    while IFS= read -r path; do
      candidates+=("$path")
    done < <(find "$byid_dir" -maxdepth 1 -type l \
      \( -iname '*Pixhawk*' -o -iname '*Holybro*' \) | sort)
  fi

  if [ "${#candidates[@]}" -eq 0 ]; then
    warn "No Pixhawk entry found in /dev/serial/by-id, falling back to /dev/ttyACM* and /dev/ttyUSB*"
    for dev in /dev/ttyACM* /dev/ttyUSB*; do
      [ -e "$dev" ] && candidates+=("$dev")
    done
  fi

  if [ "${#candidates[@]}" -eq 0 ]; then
    warn "No serial device found for Pixhawk"
    return 1
  fi

  info "Detected MAVROS candidate device(s):"
  for c in "${candidates[@]}"; do
    if [ -L "$c" ]; then
      echo "  [$i] $c -> $(readlink -f "$c")"
    else
      echo "  [$i] $c"
    fi
    i=$((i + 1))
  done

  if [ "${#candidates[@]}" -eq 1 ]; then
    choice="1"
  else
    printf "Select MAVROS port [1-%d]: " "${#candidates[@]}"
    read -r choice
  fi

  if ! [[ "$choice" =~ ^[0-9]+$ ]] || [ "$choice" -lt 1 ] || [ "$choice" -gt "${#candidates[@]}" ]; then
    error "Invalid MAVROS device selection"
    return 1
  fi

  export MAVROS_BY_ID="${candidates[$((choice - 1))]}"
  info "Selected MAVROS device: ${MAVROS_BY_ID}"
}
select_mavros_autopilot() {
  echo ""
  echo "Select MAVROS autopilot stack:"
  echo "  [1] ArduPilot / ArduRover"
  echo "  [2] PX4"
  echo ""
  printf "Choice [1-2]: "
  read -r choice

  case "$choice" in
    1)
      export MAVROS_AUTOPILOT="ardupilot"
      info "Selected MAVROS autopilot: ArduPilot"
      ;;
    2)
      export MAVROS_AUTOPILOT="px4"
      info "Selected MAVROS autopilot: PX4"
      ;;
    *)
      error "Invalid choice"
      return 1
      ;;
  esac
}

select_mavros_gcs_mode() {
  echo ""
  echo "Select MAVROS GCS forwarding mode:"
  echo "  [1] Disabled"
  echo "  [2] UDP broadcast (PC + mobile on local network)"
  echo "  [3] UDP unicast (single remote PC)"
  echo ""
  printf "Choice [1-3]: "
  read -r choice

  case "$choice" in
    1)
      export MAVROS_GCS_URL=""
      info "MAVROS GCS forwarding disabled"
      ;;
    2)
      export MAVROS_GCS_URL="udp-b://@255.255.255.255:14550"
      info "Selected MAVROS GCS forwarding: UDP broadcast on port 14550"
      ;;
    3)
      select_mavros_gcs_ip || return 1
      ;;
    *)
      error "Invalid choice"
      return 1
      ;;
  esac
}

select_mavros_gcs_ip() {
  local ip=""
  local port="14550"

  echo ""
  printf "Remote GCS IP address (example: 192.168.10.100): "
  read -r ip

  if [ -z "$ip" ]; then
    error "IP address cannot be empty"
    return 1
  fi

  printf "Remote GCS UDP port [14550]: "
  read -r port
  port="${port:-14550}"

  if ! [[ "$port" =~ ^[0-9]+$ ]]; then
    error "Invalid UDP port"
    return 1
  fi

  export MAVROS_GCS_URL="udp://@${ip}:${port}"
  info "Selected MAVROS GCS forwarding: ${MAVROS_GCS_URL}"
}