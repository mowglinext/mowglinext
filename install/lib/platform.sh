#!/usr/bin/env bash

detect_cpu_arch() {
  local arch=""

  if command -v uname >/dev/null 2>&1; then
    arch="$(uname -m 2>/dev/null || true)"
  fi

  case "$arch" in
    x86_64|amd64)
      printf 'amd64\n'
      ;;
    aarch64|arm64)
      printf 'arm64\n'
      ;;
    armv7l|armv7|armhf|armv6l|armv6|i386|i486|i586|i686)
      printf '32-bit\n'
      ;;
    *)
      if [ -n "$arch" ]; then
        printf '%s\n' "$arch"
      else
        printf 'unknown\n'
      fi
      ;;
  esac
}

is_32bit_arch() {
  local arch
  arch="$(detect_cpu_arch)"
  [[ "$arch" == "32-bit" ]]
}

detect_board_family() {
  local model=""
  local vendor=""

  if [ -r /proc/device-tree/model ]; then
    model="$(tr -d '\0' < /proc/device-tree/model 2>/dev/null || true)"
  fi

  if [ -z "$model" ] && [ -r /sys/devices/virtual/dmi/id/product_name ]; then
    model="$(cat /sys/devices/virtual/dmi/id/product_name 2>/dev/null || true)"
  fi

  if [ -r /sys/devices/virtual/dmi/id/sys_vendor ]; then
    vendor="$(cat /sys/devices/virtual/dmi/id/sys_vendor 2>/dev/null || true)"
  fi

  case "${model} ${vendor}" in
    *"Raspberry Pi"*)
      printf 'raspberry-pi\n'
      ;;
    *"Orange Pi"*|*"Xunlong"*)
      printf 'orange-pi\n'
      ;;
    *)
      case "$(detect_cpu_arch)" in
        amd64)
          printf 'x86\n'
          ;;
        *)
          printf 'generic-arm\n'
          ;;
      esac
      ;;
  esac
}

is_raspberry_pi() {
  [[ "$(detect_board_family)" == "raspberry-pi" ]]
}

is_raspberry_pi_5() {
  local model=""
  if [ -r /proc/device-tree/model ]; then
    model="$(tr -d '\0' < /proc/device-tree/model 2>/dev/null || true)"
  fi
  [[ "$model" == *"Raspberry Pi 5"* ]]
}

is_orange_pi() {
  [[ "$(detect_board_family)" == "orange-pi" ]]
}

is_x86_host() {
  [[ "$(detect_board_family)" == "x86" ]]
}

platform_supports_pi_boot_config() {
  is_raspberry_pi || return 1
  case "$(detect_cpu_arch)" in
    arm64) return 0 ;;
    *)     return 1 ;;
  esac
}

platform_supports_pi_uart_overlays() {
  platform_supports_pi_boot_config
}

assert_supported_platform() {
  local arch
  local family

  arch="$(detect_cpu_arch)"
  family="$(detect_board_family)"

  if is_32bit_arch; then
    error "32-bit platforms are not supported by MowgliNext (${arch})"
    error "Use a 64-bit OS on amd64 or arm64."
    return 1
  fi

  case "$arch" in
    amd64|arm64)
      ;;
    *)
      warn "Unrecognized CPU architecture: ${arch}"
      warn "Continuing, but only amd64 and arm64 are officially supported."
      ;;
  esac

  case "$family" in
    raspberry-pi|orange-pi|x86|generic-arm)
      return 0
      ;;
    *)
      warn "Unrecognized board family: ${family}"
      return 0
      ;;
  esac
}

print_platform_summary() {
  local arch
  local family
  local pi_boot="no"
  local pi_uart="no"

  arch="$(detect_cpu_arch)"
  family="$(detect_board_family)"

  if platform_supports_pi_boot_config; then
    pi_boot="yes"
  fi

  if platform_supports_pi_uart_overlays; then
    pi_uart="yes"
  fi

  info "Platform: arch=${arch} board=${family}"
  info "Pi boot config support: ${pi_boot}"
  info "Pi UART overlay support: ${pi_uart}"

  if is_orange_pi; then
    warn "Orange Pi detected: Raspberry Pi boot/UART overlay changes will be skipped on this platform."
  elif is_x86_host; then
    info "x86 host detected: Raspberry Pi-specific boot/UART changes will be skipped."
  fi
}
