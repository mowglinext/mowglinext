#!/usr/bin/env bash

UNICORE_TARGET_BAUD="${UNICORE_TARGET_BAUD:-921600}"

unicore_com_port_default() {
  printf '%s\n' "${UNICORE_COM_PORT:-COM1}"
}

prompt_unicore_com_port() {
  local default_com
  default_com="$(unicore_com_port_default)"

  echo ""
  echo "Unicore receiver COM port used for baud configuration:"
  echo "  1) COM1 (default)"
  echo "  2) COM2"
  echo "  3) COM3"
  prompt "$MSG_CHOICE" "$default_com"

  case "${REPLY:-$default_com}" in
    1|COM1|com1) REPLY="COM1" ;;
    2|COM2|com2) REPLY="COM2" ;;
    3|COM3|com3) REPLY="COM3" ;;
    *)
      warn "Invalid Unicore COM port '${REPLY:-}'; using $default_com."
      REPLY="$default_com"
      ;;
  esac
}

unicore_set_stty() {
  local port="${1:?unicore_set_stty: missing port}"
  local baud="${2:?unicore_set_stty: missing baud}"

  command -v stty >/dev/null 2>&1 || return 1
  stty -F "$port" "$baud" raw -echo -echoe -echok -ixon -ixoff
}

unicore_send_command() {
  local port="${1:?unicore_send_command: missing port}"
  local command="${2:?unicore_send_command: missing command}"

  printf '%s\r\n' "$command" >"$port"
}

verify_unicore_baud_921600() {
  local port="${1:?verify_unicore_baud_921600: missing port}"

  serial_try_baud_nmea "$port" "$UNICORE_TARGET_BAUD"
}

configure_unicore_baud_921600() {
  local port="${1:?configure_unicore_baud_921600: missing port}"
  local current_baud="${2:?configure_unicore_baud_921600: missing current baud}"
  local com_port="${3:-$(unicore_com_port_default)}"

  if [ "$current_baud" = "$UNICORE_TARGET_BAUD" ]; then
    verify_unicore_baud_921600 "$port"
    return $?
  fi

  serial_port_exists "$port" || return 1

  unicore_set_stty "$port" "$current_baud" || return 1
  unicore_send_command "$port" "CONFIG ${com_port} ${UNICORE_TARGET_BAUD}" || return 1
  sleep 0.5

  unicore_set_stty "$port" "$UNICORE_TARGET_BAUD" || return 1
  if ! verify_unicore_baud_921600 "$port"; then
    return 1
  fi

  unicore_send_command "$port" "SAVECONFIG" || return 1
  sleep 0.2
}

maybe_upgrade_unicore_baud() {
  local port="${1:-}"
  local current_baud="${2:-}"
  local mode="${3:-auto}"
  local com_port

  case "${GNSS_RECEIVER_FAMILY:-${GNSS_BACKEND:-}}" in
    unicore) ;;
    *) return 0 ;;
  esac
  [ -n "$port" ] || return 0
  [ -n "$current_baud" ] || return 0
  [ "$current_baud" != "$UNICORE_TARGET_BAUD" ] || return 0

  if [ "$mode" = "ask" ] && [ -z "${UNICORE_COM_PORT:-}" ]; then
    prompt_unicore_com_port
    UNICORE_COM_PORT="$REPLY"
  else
    UNICORE_COM_PORT="$(unicore_com_port_default)"
  fi
  com_port="$UNICORE_COM_PORT"

  info "Unicore baud detected at ${current_baud}; attempting ${com_port} -> ${UNICORE_TARGET_BAUD}."
  if configure_unicore_baud_921600 "$port" "$current_baud" "$com_port"; then
    GNSS_SERIAL_BAUD="$UNICORE_TARGET_BAUD"
    info "Unicore receiver verified at ${UNICORE_TARGET_BAUD}; GNSS_SERIAL_BAUD updated."
  else
    GNSS_SERIAL_BAUD="$current_baud"
    warn "Unable to switch Unicore receiver to ${UNICORE_TARGET_BAUD}; keeping GNSS_SERIAL_BAUD=${current_baud}."
  fi
}
