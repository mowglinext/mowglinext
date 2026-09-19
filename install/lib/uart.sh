#!/usr/bin/env bash

detect_rpi_model() {
  if [ -r /proc/device-tree/model ]; then
    tr -d '\0' < /proc/device-tree/model
    return 0
  fi
  echo "Unknown platform"
}

get_boot_config_file() {
  if ! platform_supports_pi_boot_config; then
    return 1
  fi

  local config_file="/boot/firmware/config.txt"
  [ -f "$config_file" ] || config_file="/boot/config.txt"
  echo "$config_file"
}

append_config_line_if_missing() {
  local line="$1"
  local config_file
  if ! config_file="$(get_boot_config_file)"; then
    info "Skipping Raspberry Pi boot config change (${line}) on this platform"
    return 0
  fi

  require_root_for "boot config"

  if ! grep -q "^${line}$" "$config_file" 2>/dev/null; then
    echo "$line" | $SUDO tee -a "$config_file" > /dev/null
    info "Enabled ${line}"
  else
    info "${line} already enabled"
  fi
}

upsert_config_line_by_prefix() {
  local prefix="$1"
  local line="$2"
  local config_file
  if ! config_file="$(get_boot_config_file)"; then
    info "Skipping Raspberry Pi boot config change (${line}) on this platform"
    return 0
  fi

  require_root_for "boot config"

  if grep -q "^${prefix}" "$config_file" 2>/dev/null; then
    $SUDO sed -i "s|^${prefix}.*|${line}|" "$config_file"
    info "Set ${line}"
  else
    echo "$line" | $SUDO tee -a "$config_file" > /dev/null
    info "Enabled ${line}"
  fi
}

remove_config_line_if_present() {
  local line="$1"
  local config_file
  if ! config_file="$(get_boot_config_file)"; then
    return 0
  fi

  require_root_for "boot config"

  if grep -q "^${line}$" "$config_file" 2>/dev/null; then
    $SUDO sed -i "\|^${line}$|d" "$config_file"
    info "Removed ${line}"
  fi
}

disable_bluetooth_for_uart() {
  if ! platform_supports_pi_uart_overlays; then
    info "Skipping Raspberry Pi Bluetooth/UART boot changes on this platform"
    return 0
  fi

  require_root_for "disable bluetooth"

  append_config_line_if_missing "dtoverlay=disable-bt"

  # Évite les conflits si une autre ancienne config traîne
  remove_config_line_if_present "dtoverlay=miniuart-bt"

  if command -v systemctl >/dev/null 2>&1; then
    $SUDO systemctl disable hciuart >/dev/null 2>&1 || true
    $SUDO systemctl stop hciuart >/dev/null 2>&1 || true
    info "Disabled hciuart service"
  fi
}

configure_raspberry_pi_5_hardware() {
  if ! platform_supports_pi_boot_config; then
    return 0
  fi

  if ! is_raspberry_pi_5; then
    return 0
  fi

  info "Applying Raspberry Pi 5 USB/fan boot settings"

  upsert_config_line_by_prefix "usb_max_current_enable=" "usb_max_current_enable=1"

  upsert_config_line_by_prefix "dtparam=fan_temp0=" "dtparam=fan_temp0=45000"
  upsert_config_line_by_prefix "dtparam=fan_temp0_hyst=" "dtparam=fan_temp0_hyst=5000"
  upsert_config_line_by_prefix "dtparam=fan_temp0_speed=" "dtparam=fan_temp0_speed=75"

  upsert_config_line_by_prefix "dtparam=fan_temp1=" "dtparam=fan_temp1=50000"
  upsert_config_line_by_prefix "dtparam=fan_temp1_hyst=" "dtparam=fan_temp1_hyst=5000"
  upsert_config_line_by_prefix "dtparam=fan_temp1_speed=" "dtparam=fan_temp1_speed=128"

  upsert_config_line_by_prefix "dtparam=fan_temp2=" "dtparam=fan_temp2=55000"
  upsert_config_line_by_prefix "dtparam=fan_temp2_hyst=" "dtparam=fan_temp2_hyst=5000"
  upsert_config_line_by_prefix "dtparam=fan_temp2_speed=" "dtparam=fan_temp2_speed=192"

  upsert_config_line_by_prefix "dtparam=fan_temp3=" "dtparam=fan_temp3=60000"
  upsert_config_line_by_prefix "dtparam=fan_temp3_hyst=" "dtparam=fan_temp3_hyst=5000"
  upsert_config_line_by_prefix "dtparam=fan_temp3_speed=" "dtparam=fan_temp3_speed=255"
}

# Maps a serial device path to the Raspberry Pi UART overlay number that
# must be enabled for it to exist. Prints nothing (and fails) for a path
# that needs no `dtoverlay=uartN` line: uart0 (`/dev/ttyAMA0`, the primary
# PL011, already covered by the unconditional `enable_uart=1`), the
# mini-uart (`/dev/ttyS0`), a USB `/dev/serial/by-id/...` path, or empty/unset.
uart_overlay_for_device() {
  case "${1:-}" in
    /dev/ttyAMA1) printf '1\n' ;;
    /dev/ttyAMA2) printf '2\n' ;;
    /dev/ttyAMA3) printf '3\n' ;;
    /dev/ttyAMA4) printf '4\n' ;;
    /dev/ttyAMA5) printf '5\n' ;;
    *) return 1 ;;
  esac
}

# Which of the five Raspberry Pi UART overlays the hardware CONFIGURED so far
# this run actually needs — derived from the exact port the operator picked
# for each peripheral (`pick_uart_port`, `GNSS_SERIAL_DEVICE` /
# `LIDAR_UART_DEVICE` / `TFLUNA_{FRONT,EDGE}_UART_DEVICE`), not a fixed
# per-peripheral assumption: real installs don't always land a given
# peripheral on the same header pin (e.g. a Pi 5 install that wired the LiDAR
# to ttyAMA2 instead of the common default ttyAMA5). Nothing here ever claims
# uart1 — no peripheral in this codebase is assigned to it today.
#
# Must run AFTER GPS/LiDAR/rangefinder configuration (mowglinext.sh calls
# this step after those, not before) so these variables are populated;
# `pick_uart_port` already tolerates picking a `/dev/ttyAMAn` that doesn't
# exist yet (marked `(*)`, meaning "exists after the dtoverlay + reboot this
# step performs"), so the ordering costs nothing at selection time.
#
# issue #631: blindly enabling all five regardless of configuration silently
# steals whatever GPIO a custom peripheral (e.g. an LED ring) is wired to.
required_uart_overlays() {
  local n

  n="$(uart_overlay_for_device "${GNSS_SERIAL_DEVICE:-}")" && printf '%s\n' "$n"

  if [[ "${LIDAR_ENABLED:-false}" == "true" ]]; then
    n="$(uart_overlay_for_device "${LIDAR_UART_DEVICE:-}")" && printf '%s\n' "$n"
  fi

  if [[ "${TFLUNA_FRONT_ENABLED:-false}" == "true" ]]; then
    n="$(uart_overlay_for_device "${TFLUNA_FRONT_UART_DEVICE:-}")" && printf '%s\n' "$n"
  fi

  if [[ "${TFLUNA_EDGE_ENABLED:-false}" == "true" ]]; then
    n="$(uart_overlay_for_device "${TFLUNA_EDGE_UART_DEVICE:-}")" && printf '%s\n' "$n"
  fi
}

enable_all_platform_uarts() {
  step "UART platform setup"

  local model
  model="$(detect_rpi_model)"
  info "Detected platform: ${model}"

  if ! platform_supports_pi_uart_overlays; then
    info "Raspberry Pi UART overlays are not applicable on this platform"
    return 0
  fi

  append_config_line_if_missing "enable_uart=1"

  # Enable only the overlays the configured hardware actually uses — not all
  # five unconditionally (issue #631). Only ADDS lines; never removes one,
  # so a `dtoverlay=uartN` the operator added by hand for something this
  # installer doesn't model (e.g. a custom LED ring) is left alone.
  local overlays n
  overlays="$(required_uart_overlays | sort -un)"
  if [[ -z "$overlays" ]]; then
    info "No configured peripheral needs a UART overlay — leaving uart1-uart5 untouched"
  else
    while IFS= read -r n; do
      [[ -n "$n" ]] || continue
      append_config_line_if_missing "dtoverlay=uart${n}"
    done <<< "$overlays"
  fi

  # Bluetooth toujours désactivé dans Mowgli II
  disable_bluetooth_for_uart
  configure_raspberry_pi_5_hardware

  info "UART overlay setup complete"
}
