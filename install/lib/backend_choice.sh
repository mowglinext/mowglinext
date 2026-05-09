#!/usr/bin/env bash

configure_mavros_backend_details() {
  local allow_existing="${1:-false}"

  if [[ "$allow_existing" == "true" && ( "${MAVROS_AUTOPILOT:-}" == "ardupilot" || "${MAVROS_AUTOPILOT:-}" == "px4" ) ]]; then
    info "Using MAVROS autopilot preset/default: ${MAVROS_AUTOPILOT}"
  else
    select_mavros_autopilot || return 1
  fi

  if [[ "$allow_existing" == "true" && -n "${MAVROS_GCS_URL+x}" ]]; then
    if [[ -n "${MAVROS_GCS_URL:-}" ]]; then
      info "Using MAVROS GCS forwarding preset/default: ${MAVROS_GCS_URL}"
    else
      info "Using MAVROS GCS forwarding preset/default: disabled"
    fi
  else
    select_mavros_gcs_mode || return 1
  fi

  if [[ "$allow_existing" == "true" && -n "${MAVROS_BY_ID:-}" && -e "${MAVROS_BY_ID}" ]]; then
    info "Using existing MAVROS device: ${MAVROS_BY_ID}"
    return 0
  fi

  if [[ "$allow_existing" == "true" && -n "${MAVROS_PORT:-}" && "${MAVROS_PORT}" == /dev/tty* && -e "${MAVROS_PORT}" ]]; then
    info "Using existing MAVROS device path: ${MAVROS_PORT}"
    MAVROS_BY_ID=""
    return 0
  fi

  echo ""
  warn "Please connect the Pixhawk to a USB port before continuing."
  warn "The installer will wait a few seconds for the serial device to appear."
  warn "If the device appears/disappears repeatedly, check the USB cable and power supply."
  echo ""

  if ! wait_for_mavros_device 15 3; then
    error "No stable Pixhawk serial device detected. Please connect it by USB and try again."
    return 1
  fi

  if ! detect_mavros_by_id; then
    error "Unable to detect a valid MAVROS serial device."
    return 1
  fi
}

select_hardware_backend() {
  if [[ "${PRESET_LOADED:-false}" == "true" && -n "${HARDWARE_BACKEND:-}" ]]; then
    case "${HARDWARE_BACKEND}" in
      mowgli)
        export HARDWARE_BACKEND="mowgli"
        export MAVROS_BY_ID=""
        info "Hardware backend pre-configured: Mowgli STM32 board"
        return 0
        ;;
      mavros)
        export HARDWARE_BACKEND="mavros"
        info "Hardware backend pre-configured: Pixhawk via MAVROS"
        configure_mavros_backend_details true || return 1
        return 0
        ;;
      *)
        error "Invalid HARDWARE_BACKEND preset: ${HARDWARE_BACKEND} (expected mowgli or mavros)"
        return 1
        ;;
    esac
  fi

  echo ""
  echo "Select hardware backend:"
  echo "  [1] Mowgli STM32 board"
  echo "  [2] Pixhawk via MAVROS"
  echo ""
  printf "Choice [1-2]: "
  read -r choice

  case "$choice" in
    1)
      export HARDWARE_BACKEND="mowgli"
      export MAVROS_BY_ID=""
      info "Selected backend: Mowgli STM32 board"
      ;;
    2)
      export HARDWARE_BACKEND="mavros"
      info "Selected backend: Pixhawk via MAVROS"
      configure_mavros_backend_details false || return 1
      ;;
    *)
      error "Invalid choice"
      return 1
      ;;
  esac
}

wait_for_mavros_device() {
  local timeout="${1:-15}"
  local stable_required="${2:-3}"
  local elapsed=0
  local stable_count=0
  local found=""

  info "Waiting for Pixhawk serial device to become stable..."

  while [ "$elapsed" -lt "$timeout" ]; do
    found=""

    # Prefer stable by-id link
    if [ -d /dev/serial/by-id ]; then
      found="$(find /dev/serial/by-id -maxdepth 1 -type l \
        \( -iname '*Pixhawk*' -o -iname '*Holybro*' \) | sort | head -n1)"
    fi

    # Fallback to ttyACM / ttyUSB
    if [ -z "$found" ]; then
      for dev in /dev/ttyACM* /dev/ttyUSB*; do
        if [ -e "$dev" ]; then
          found="$dev"
          break
        fi
      done
    fi

    if [ -n "$found" ]; then
      stable_count=$((stable_count + 1))
      info "Detected candidate: $found (${stable_count}/${stable_required})"
      if [ "$stable_count" -ge "$stable_required" ]; then
        info "Pixhawk serial device is stable"
        return 0
      fi
    else
      stable_count=0
    fi

    sleep 1
    elapsed=$((elapsed + 1))
  done

  return 1
}

detect_mavros_by_id() {
  local byid_dir="/dev/serial/by-id"
  local candidates=()
  local choice=""
  local i=1

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
