#!/usr/bin/env bash

USB_SERIAL_DEVICES=()
USB_SERIAL_USED=()

scan_usb_serial_devices() {
  local by_id_dir="${SERIAL_BY_ID_DIR:-/dev/serial/by-id}"
  USB_SERIAL_DEVICES=()
  USB_SERIAL_USED=()

  if [ -d "$by_id_dir" ]; then
    while IFS= read -r path; do
      USB_SERIAL_DEVICES+=("$path")
    done < <(find "$by_id_dir" -maxdepth 1 -type l | sort)
  fi

  # Fallback ttyACM/ttyUSB si pas de by-id
  if [ "${#USB_SERIAL_DEVICES[@]}" -eq 0 ]; then
    for dev in /dev/ttyACM* /dev/ttyUSB*; do
      [ -e "$dev" ] && USB_SERIAL_DEVICES+=("$dev")
    done
  fi

  if [ "${#USB_SERIAL_DEVICES[@]}" -eq 0 ]; then
    warn "No USB serial device detected"
  else
    info "USB serial devices detected:"
    local i=1
    for dev in "${USB_SERIAL_DEVICES[@]}"; do
      if [ -L "$dev" ]; then
        echo "  [$i] $dev -> $(readlink -f "$dev")"
      else
        echo "  [$i] $dev"
      fi
      i=$((i + 1))
    done
  fi
}

usb_serial_is_used() {
  local dev="$1"
  local used

  for used in "${USB_SERIAL_USED[@]}"; do
    [ "$used" = "$dev" ] && return 0
  done

  return 1
}

mark_usb_serial_used() {
  local dev="$1"
  USB_SERIAL_USED+=("$dev")
}

pick_usb_serial_device() {
  local label="${1:-USB serial device}"
  local default_manual="${2:-/dev/ttyACM0}"
  local available=()
  local dev
  local choice
  local i=1

  echo ""
  info "Select USB serial device for: $label"

  for dev in "${USB_SERIAL_DEVICES[@]}"; do
    if ! usb_serial_is_used "$dev"; then
      available+=("$dev")
    fi
  done

  if [ "${#available[@]}" -gt 0 ]; then
    for dev in "${available[@]}"; do
      if [ -L "$dev" ]; then
        echo "  [$i] $dev -> $(readlink -f "$dev")"
      else
        echo "  [$i] $dev"
      fi
      i=$((i + 1))
    done
  else
    warn "No unused USB serial device remaining"
  fi

  echo "  [m] Manual path"

  prompt "$MSG_CHOICE" "1"
  choice="$REPLY"

  if [ "$choice" = "m" ] || [ "$choice" = "M" ] || [ "${#available[@]}" -eq 0 ]; then
    prompt "USB serial path for $label" "$default_manual"

    if [[ "$REPLY" != /dev/* ]]; then
      error "Invalid USB serial path: $REPLY"
      return 1
    fi

    USB_SELECTED_DEVICE="$REPLY"
    USB_SELECTED_BY_ID=""
    USB_SELECTED_PORT="$REPLY"
    return 0
  fi

  if ! [[ "$choice" =~ ^[0-9]+$ ]] || [ "$choice" -lt 1 ] || [ "$choice" -gt "${#available[@]}" ]; then
    error "Invalid USB serial device selection"
    return 1
  fi

  USB_SELECTED_DEVICE="${available[$((choice - 1))]}"

  if [[ "$USB_SELECTED_DEVICE" == /dev/serial/by-id/* ]]; then
    USB_SELECTED_BY_ID="$USB_SELECTED_DEVICE"
    USB_SELECTED_PORT="$(readlink -f "$USB_SELECTED_DEVICE")"
  else
    USB_SELECTED_BY_ID=""
    USB_SELECTED_PORT="$USB_SELECTED_DEVICE"
  fi

  mark_usb_serial_used "$USB_SELECTED_DEVICE"
}