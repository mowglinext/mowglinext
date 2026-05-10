#!/bin/bash
# =============================================================================
# Send a one-time UM98x rover configuration over the serial port.
#
# Symptoms this fixes:
#   * accuracy stuck at 10 m, RTK never validates, even with RTCM flowing
#   * /diagnostics carr_soln stays "none"
#
# Root cause: out-of-the-box / FRESET-state UM98x receivers default to
# something close to "rover, no NMEA enabled, no PVTSLNA" — and the
# default RTK timeout is short. Without an explicit MODE ROVER + LOG
# directives, the receiver outputs only minimal NMEA and the operator
# sees a single-point fix even though RTCM is being injected.
#
# We send commands using the port-agnostic short form
# (`LOG <msg> ONTIME <rate>`) so the receiver outputs on whichever
# physical UART the host is connected to.
#
# Reference: https://github.com/CentipedeRTK/docs-centipedeRTK
#            (assets/param_files/UM98x/UM980_aog_rover_last_CONFIG.txt)
#
# Usage: configure_receiver.sh [device] [baudrate]
#
# Persisted via SAVECONFIG in receiver NVRAM, so this only meaningfully
# changes anything on the first run; subsequent calls are idempotent.
# =============================================================================
set -euo pipefail

# NOTE:
# The serial device path (usually /dev/gps) is managed by Linux udev rules
# and must remain stable across all GNSS backends.
#
# DEVICE is the Linux serial path, normally /dev/gps.
# UNICORE_COM_PORT is ONLY the internal receiver-side logical COM port
# used in commands such as:
#   CONFIG COM1 921600
#
# Do NOT derive the Linux serial device path from UNICORE_COM_PORT.
TARGET_BAUD="${UNICORE_TARGET_BAUD:-921600}"
UNICORE_COM_PORT="${UNICORE_COM_PORT:-COM1}"

log() {
  echo "[configure_receiver.sh] $*" >&2
}

require_serial_port() {
  local port="${1:?require_serial_port: missing port}"

  if [ ! -c "$port" ]; then
    log "ERROR: $port is not a character device" >&2
    return 1
  fi
}

wait_for_serial_port() {
  local port="${1:?wait_for_serial_port: missing port}"
  local attempt

  for attempt in $(seq 1 20); do
    [ -w "$port" ] && return 0
    sleep 0.1
  done

  log "ERROR: $port is not writable after waiting" >&2
  return 1
}

serial_set_baud() {
  local port="${1:?serial_set_baud: missing port}"
  local baud="${2:?serial_set_baud: missing baud}"

  stty -F "$port" "$baud" raw -echo -echoe -echok -ixon -ixoff || {
    log "ERROR: stty failed on $port @ $baud" >&2
    return 1
  }
}

open_serial() {
  local port="${1:?open_serial: missing port}"

  exec 3<>"$port"
}

close_serial() {
  exec 3>&- || true
}

drain_serial() {
  local line

  while IFS= read -r -t 0.05 -u 3 line; do
    :
  done
}

send_serial_command() {
  local command="${1:?send_serial_command: missing command}"

  printf '%s\r\n' "$command" >&3
}

collect_serial_output() {
  local rounds="${1:?collect_serial_output: missing rounds}"
  local line output=""
  local i

  for i in $(seq 1 "$rounds"); do
    if IFS= read -r -t 0.15 -u 3 line; then
      output+="${line}"$'\n'
    fi
  done

  printf '%s' "$output"
}

query_receiver_identification() {
  local response=""

  drain_serial

  # `VERSION` is the documented query command; `VERSIONA` is a compatible
  # fallback on receivers that accept the log/message name directly.
  send_serial_command "VERSION"
  sleep 0.1
  response="$(collect_serial_output 10)"

  if [[ "$response" != *"UM980"* && "$response" != *"UM982"* && "$response" != *"#VERSIONA"* ]]; then
    send_serial_command "VERSIONA"
    sleep 0.1
    response+=$(collect_serial_output 10)
  fi

  printf '%s' "$response"
}

model_from_response() {
  local response="${1:-}"

  if [[ "$response" == *"UM980"* ]]; then
    printf '%s\n' "UM980"
  elif [[ "$response" == *"UM982"* ]]; then
    printf '%s\n' "UM982"
  else
    printf '%s\n' "unknown"
  fi
}

detect_receiver_at_baud() {
  local port="${1:?detect_receiver_at_baud: missing port}"
  local baud="${2:?detect_receiver_at_baud: missing baud}"
  local response model

  serial_set_baud "$port" "$baud" || return 1
  open_serial "$port"
  response="$(query_receiver_identification)"
  close_serial

  if [ -z "$response" ]; then
    return 1
  fi

  model="$(model_from_response "$response")"
  printf '%s|%s\n' "$baud" "$model"
}

detect_baud_candidates() {
  local preferred="${1:-}"
  local seen=" "
  local candidate
  local ordered=()

  for candidate in "$preferred" "$TARGET_BAUD" 460800 115200 230400 57600 38400 9600; do
    [ -n "$candidate" ] || continue
    case "$seen" in
      *" $candidate "*) continue ;;
    esac
    ordered+=("$candidate")
    seen+="$(printf '%s ' "$candidate")"
  done

  printf '%s\n' "${ordered[@]}"
}

detect_receiver_baud_and_model() {
  local port="${1:?detect_receiver_baud_and_model: missing port}"
  local preferred_baud="${2:-}"
  local candidate result baud model

  while IFS= read -r candidate; do
    [ -n "$candidate" ] || continue
    log "Probing receiver on ${port} @ ${candidate}..."

    if result="$(detect_receiver_at_baud "$port" "$candidate")"; then
      baud="${result%%|*}"
      model="${result#*|}"
      log "Receiver responded on ${baud}."
      printf '%s|%s\n' "$baud" "$model"
      return 0
    fi
  done < <(detect_baud_candidates "$preferred_baud")

  return 1
}

signalgroup_for_model() {
  local model="${1:?signalgroup_for_model: missing model}"

  case "$model" in
    UM980) printf '%s\n' "CONFIG SIGNALGROUP 2" ;;
    UM982) printf '%s\n' "CONFIG SIGNALGROUP 3 6" ;;
    *) return 1 ;;
  esac
}

verify_receiver_at_baud() {
  local port="${1:?verify_receiver_at_baud: missing port}"
  local baud="${2:?verify_receiver_at_baud: missing baud}"
  local response

  serial_set_baud "$port" "$baud" || return 1
  open_serial "$port"
  response="$(query_receiver_identification)"
  close_serial

  [ -n "$response" ]
}

send_config_batch() {
  local port="${1:?send_config_batch: missing port}"
  shift
  local commands=("$@")
  local command

  serial_set_baud "$port" "$TARGET_BAUD"
  open_serial "$port"
  drain_serial

  for command in "${commands[@]}"; do
    send_serial_command "$command"
    log "  -> $command"
    sleep 0.2
  done

  close_serial
}

apply_receiver_configuration() {
  local port="${1:?apply_receiver_configuration: missing port}"
  local detected_baud="${2:?apply_receiver_configuration: missing detected baud}"
  local model="${3:?apply_receiver_configuration: missing model}"
  local signalgroup
  local cmds=()

  signalgroup="$(signalgroup_for_model "$model")" || {
    log "WARN: model '${model}' not clearly identified; refusing to apply SIGNALGROUP." >&2
    return 1
  }

  if [ "$detected_baud" != "$TARGET_BAUD" ]; then
    log "Switching ${UNICORE_COM_PORT} from ${detected_baud} to ${TARGET_BAUD}..."
    serial_set_baud "$port" "$detected_baud"
    open_serial "$port"
    send_serial_command "CONFIG ${UNICORE_COM_PORT} ${TARGET_BAUD}"
    close_serial
    sleep 0.5
  else
    log "Receiver already running at ${TARGET_BAUD}."
  fi

  if ! verify_receiver_at_baud "$port" "$TARGET_BAUD"; then
    log "ERROR: receiver did not answer after switch to ${TARGET_BAUD}." >&2
    return 1
  fi

  log "Receiver verified at ${TARGET_BAUD}; detected model ${model}."

  cmds=(
    # Rover mode + NMEA version + RTK timeouts.
    # SURVEY MOW = low-dynamics survey-grade rover with the mowing-specific
    # dynamics preset. UAV (which we tried first) is for drones — assumes
    # high vertical accelerations and hurts ambiguity resolution on a
    # ground robot that mostly translates. AUTOMOTIVE is the next-best
    # alternative if SURVEY MOW isn't supported on a given firmware rev.
    "MODE ROVER SURVEY MOW"
    "CONFIG NMEAVERSION V411"
    # 180 s tolerates short NTRIP outages without dropping back to single
    # point. Default is 60 s on most firmware revisions.
    "CONFIG RTK TIMEOUT 180"
    "CONFIG RTK RELIABILITY 4 3"
    "CONFIG DGPS TIMEOUT 300"
    "${signalgroup}"
    "CONFIG AGNSS ENABLE"

    # Constellation enables. Mowgli mowers are typically in mid-latitude
    # open sky, so we want everything except QZSS (regional, doesn't help
    # in EU/NA).
    "UNMASK GPS"
    "UNMASK GLO"
    "UNMASK GAL"
    "UNMASK BDS"
    "MASK QZSS"

    # Output messages — port-agnostic LOG form, rate is period in seconds.
    # Unicore log-name convention: base name + 'A' suffix = ASCII format.
    # PVTSLN + A = PVTSLNA (the message header is `#PVTSLNA,...` — note
    # SINGLE trailing A — and our parser keys on that string).
    # BESTNAV + A = BESTNAVA (message header `#BESTNAVA,...` — double A
    # because the base name itself ends in V, not A).
    "LOG GPGGA ONTIME 1"
    "LOG PVTSLNA ONTIME 0.1"
    "LOG BESTNAVA ONTIME 0.1"
    "LOG GNHPR ONTIME 0.1"
    "LOG GPGSV ONTIME 1"
    "LOG GLGSV ONTIME 1"
    "LOG GAGSV ONTIME 1"
    "LOG GBGSV ONTIME 1"
  )

  log "Applying UM98x rover config to ${port} @ ${TARGET_BAUD}..."
  send_config_batch "$port" "${cmds[@]}"

  if ! verify_receiver_at_baud "$port" "$TARGET_BAUD"; then
    log "ERROR: receiver stopped responding at ${TARGET_BAUD} before SAVECONFIG." >&2
    return 1
  fi

  send_config_batch "$port" "SAVECONFIG"
  log "Done. Receiver config persisted via SAVECONFIG."
}

main() {
  local port="${1:-/dev/gps}"
  local preferred_baud="${2:-$TARGET_BAUD}"
  local detection detected_baud detected_model

  require_serial_port "$port"
  wait_for_serial_port "$port"

  detection="$(detect_receiver_baud_and_model "$port" "$preferred_baud")" || {
    log "ERROR: unable to detect a responding Unicore receiver on ${port}." >&2
    return 1
  }

  detected_baud="${detection%%|*}"
  detected_model="${detection#*|}"

  if [ "$detected_model" = "unknown" ]; then
    log "WARN: receiver model could not be identified from VERSION response." >&2
    log "WARN: no SIGNALGROUP will be applied and SAVECONFIG is skipped." >&2
    return 1
  fi

  apply_receiver_configuration "$port" "$detected_baud" "$detected_model"
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
