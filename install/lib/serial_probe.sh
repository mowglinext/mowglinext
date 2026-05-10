#!/usr/bin/env bash

serial_port_exists() {
  local port="${1:-}"
  [ -n "$port" ] && [ -e "$port" ]
}

serial_try_baud_nmea() {
  local port="${1:?serial_try_baud_nmea: missing port}"
  local baud="${2:?serial_try_baud_nmea: missing baud}"
  local output

  serial_port_exists "$port" || return 1
  command -v stty >/dev/null 2>&1 || return 1
  command -v timeout >/dev/null 2>&1 || return 1

  output="$(
    {
      stty -F "$port" "$baud" raw -echo -echoe -echok -ixon -ixoff &&
        timeout 2 cat "$port"
    } 2>/dev/null || true
  )"

  printf '%s\n' "$output" | grep -Eq '\$(GN|GP)(GGA|RMC|VTG)(,|$)'
}

serial_probe_baud() {
  local port="${1:?serial_probe_baud: missing port}"
  local backend="${2:-gps}"
  local protocol="${3:-UBX}"
  local baud
  local bauds=()

  case "$backend" in
    unicore)
      bauds=(921600 460800 115200 230400 57600 38400 9600)
      ;;
    ublox)
      bauds=(460800 115200 9600 230400 57600 38400 921600)
      ;;
    *)
      case "$protocol" in
        NMEA) bauds=(115200 9600 38400 57600 230400 460800 921600) ;;
        *)    bauds=(460800 115200 9600 230400 57600 38400 921600) ;;
      esac
      ;;
  esac

  for baud in "${bauds[@]}"; do
    if serial_try_baud_nmea "$port" "$baud"; then
      printf '%s\n' "$baud"
      return 0
    fi
  done

  return 1
}

prompt_or_probe_baud() {
  local port="${1:?prompt_or_probe_baud: missing port}"
  local backend="${2:-gps}"
  local protocol="${3:-UBX}"
  local default_baud="${4:-921600}"
  local mode="${5:-ask}"
  local detected=""
  local choice=""

  if ! serial_port_exists "$port"; then
    warn "Serial port $port does not exist; falling back to manual baud selection."
  elif [ "$mode" = "auto" ]; then
    if detected="$(serial_probe_baud "$port" "$backend" "$protocol" 2>/dev/null)"; then
      info "Detected GPS/GNSS baud on $port: $detected"
      REPLY="$detected"
      return 0
    fi
    warn "Could not auto-detect GPS/GNSS baud on $port; falling back to manual selection."
  else
    echo ""
    echo "GPS/GNSS baud rate:"
    echo "  1) Auto-detect baud"
    echo "  2) Manual baud selection"
    prompt "$MSG_CHOICE" "1"
    choice="$REPLY"

    if [ "$choice" = "1" ] && serial_port_exists "$port"; then
      if detected="$(serial_probe_baud "$port" "$backend" "$protocol" 2>/dev/null)"; then
        info "Detected GPS/GNSS baud on $port: $detected"
        REPLY="$detected"
        return 0
      fi
      warn "Could not auto-detect GPS/GNSS baud on $port; falling back to manual selection."
    fi
  fi

  echo ""
  echo "Manual GPS/GNSS baud rate:"
  echo "  Common: 921600, 460800, 230400, 115200, 57600, 38400, 9600"
  prompt "$MSG_CHOICE" "$default_baud"
  REPLY="${REPLY:-$default_baud}"

  if ! [[ "$REPLY" =~ ^[0-9]+$ ]]; then
    warn "Invalid baud '$REPLY'; using default $default_baud."
    REPLY="$default_baud"
  fi
}
