#!/usr/bin/env bash

UBLOX_TARGET_BAUD="${UBLOX_TARGET_BAUD:-921600}"
UBLOX_CFG_PORT_ID="${UBLOX_CFG_PORT_ID:-1}"
UBLOX_HELPER="${UBLOX_HELPER:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/ublox_config_helper.py}"

ublox_backend_selected() {
  case "${GNSS_BACKEND:-}" in
    ublox)
      return 0
      ;;
    gps)
      [ "${GPS_PROTOCOL:-UBX}" = "UBX" ]
      return $?
      ;;
    *)
      return 1
      ;;
  esac
}

ublox_set_stty() {
  local port="${1:?ublox_set_stty: missing port}"
  local baud="${2:?ublox_set_stty: missing baud}"

  command -v stty >/dev/null 2>&1 || return 1
  stty -F "$port" "$baud" raw -echo -echoe -echok -ixon -ixoff
}

ublox_run_helper() {
  local port="${1:?ublox_run_helper: missing port}"
  shift

  command -v python3 >/dev/null 2>&1 || return 1
  [ -f "$UBLOX_HELPER" ] || return 1
  python3 "$UBLOX_HELPER" "$port" "$@"
}

detect_ublox_receiver() {
  local port="${1:?detect_ublox_receiver: missing port}"
  local baud="${2:?detect_ublox_receiver: missing baud}"

  serial_port_exists "$port" || return 1
  ublox_set_stty "$port" "$baud" || return 1
  ublox_run_helper "$port" identify
}

verify_ublox_baud_921600() {
  local port="${1:?verify_ublox_baud_921600: missing port}"

  detect_ublox_receiver "$port" "$UBLOX_TARGET_BAUD"
}

configure_ublox_baud_921600() {
  local port="${1:?configure_ublox_baud_921600: missing port}"
  local current_baud="${2:?configure_ublox_baud_921600: missing current baud}"

  serial_port_exists "$port" || return 1

  if ! detect_ublox_receiver "$port" "$current_baud" >/dev/null; then
    return 1
  fi

  if [ "$current_baud" = "$UBLOX_TARGET_BAUD" ]; then
    verify_ublox_baud_921600 "$port" >/dev/null
    return $?
  fi

  ublox_set_stty "$port" "$current_baud" || return 1
  ublox_run_helper "$port" set-baud "$UBLOX_CFG_PORT_ID" "$UBLOX_TARGET_BAUD" >/dev/null || return 1
  sleep 0.2

  ublox_set_stty "$port" "$UBLOX_TARGET_BAUD" || return 1
  if ! verify_ublox_baud_921600 "$port" >/dev/null; then
    return 1
  fi

  ublox_run_helper "$port" save >/dev/null || return 1
  sleep 0.2
}

maybe_upgrade_ublox_baud() {
  local port="${1:-}"
  local current_baud="${2:-}"
  local _mode="${3:-auto}"

  ublox_backend_selected || return 0
  [ -n "$port" ] || return 0
  [ -n "$current_baud" ] || return 0
  serial_port_exists "$port" || return 0

  if ! detect_ublox_receiver "$port" "$current_baud" >/dev/null 2>&1; then
    warn "Receiver on $port did not confirm as u-blox via UBX MON-VER; keeping GPS_BAUD=${current_baud}."
    GPS_BAUD="$current_baud"
    return 0
  fi

  if [ "$current_baud" = "$UBLOX_TARGET_BAUD" ]; then
    GPS_BAUD="$UBLOX_TARGET_BAUD"
    info "u-blox receiver already responds at ${UBLOX_TARGET_BAUD}; keeping GPS_BAUD=${GPS_BAUD}."
    return 0
  fi

  info "u-blox receiver detected at ${current_baud}; attempting UART${UBLOX_CFG_PORT_ID} -> ${UBLOX_TARGET_BAUD}."
  if configure_ublox_baud_921600 "$port" "$current_baud"; then
    GPS_BAUD="$UBLOX_TARGET_BAUD"
    info "u-blox receiver verified at ${UBLOX_TARGET_BAUD}; GPS_BAUD updated."
  else
    GPS_BAUD="$current_baud"
    warn "Unable to switch u-blox receiver to ${UBLOX_TARGET_BAUD}; keeping GPS_BAUD=${current_baud}."
  fi
}
