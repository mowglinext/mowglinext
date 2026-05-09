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

  # Active tous les UART overlays utiles
  append_config_line_if_missing "dtoverlay=uart1"
  append_config_line_if_missing "dtoverlay=uart2"
  append_config_line_if_missing "dtoverlay=uart3"
  append_config_line_if_missing "dtoverlay=uart4"
  append_config_line_if_missing "dtoverlay=uart5"

  # Bluetooth toujours désactivé dans Mowgli II
  disable_bluetooth_for_uart
  configure_raspberry_pi_5_hardware

  info "All UART overlays requested and Bluetooth disabled"
}
