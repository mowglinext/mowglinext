#!/bin/bash
# =============================================================================
# post-start.sh — Runs each time the devcontainer starts.
#
# Exposes the host's stable USB serial symlinks inside the container when the
# host provides them. Docker's generated /dev often omits /dev/serial/by-id,
# which makes GNSS receiver selection fall back to volatile tty numbering.
# =============================================================================
set -euo pipefail

XVFB_DISPLAY=":99"
XVFB_LOCK="/tmp/.X99-lock"
XVFB_SOCKET="/tmp/.X11-unix/X99"
XVFB_LOG="/tmp/mowgli-xvfb.log"

xvfb_pid() {
  local pid

  [ -r "$XVFB_LOCK" ] || return 1
  pid="$(tr -d '[:space:]' < "$XVFB_LOCK")"
  case "$pid" in
    ''|*[!0-9]*) return 1 ;;
  esac

  [ -r "/proc/${pid}/comm" ] || return 1
  [ "$(cat "/proc/${pid}/comm")" = "Xvfb" ] || return 1
  printf '%s\n' "$pid"
}

if ! xvfb_pid >/dev/null || [ ! -S "$XVFB_SOCKET" ]; then
  # Docker preserves /tmp across container restarts. Xvfb does not always
  # remove its lock and socket when the container stops, so validate the
  # process rather than treating a leftover socket as a live display.
  sudo install -d -m 1777 -o root -g root /tmp/.X11-unix
  sudo rm -f "$XVFB_LOCK" "$XVFB_SOCKET"

  nohup Xvfb "$XVFB_DISPLAY" -screen 0 1280x720x24 \
    +extension GLX +render -noreset -nolisten tcp \
    >"$XVFB_LOG" 2>&1 &
  started_pid=$!

  for _ in $(seq 1 50); do
    if [ -S "$XVFB_SOCKET" ] && kill -0 "$started_pid" 2>/dev/null; then
      break
    fi
    sleep 0.1
  done

  if [ ! -S "$XVFB_SOCKET" ] || ! kill -0 "$started_pid" 2>/dev/null; then
    echo "Xvfb failed to start on ${XVFB_DISPLAY}:" >&2
    cat "$XVFB_LOG" >&2
    exit 1
  fi
fi

HOST_DEV_ROOT="${HOST_DEV_ROOT:-/host-dev}"
HOST_SERIAL_BY_ID="${HOST_SERIAL_BY_ID:-${HOST_DEV_ROOT}/serial/by-id}"
CONTAINER_SERIAL_DIR="${CONTAINER_SERIAL_DIR:-/dev/serial}"
CONTAINER_SERIAL_BY_ID="${CONTAINER_SERIAL_BY_ID:-${CONTAINER_SERIAL_DIR}/by-id}"

if [ ! -d "$HOST_SERIAL_BY_ID" ]; then
  exit 0
fi

mkdir -p "$CONTAINER_SERIAL_DIR"

if [ -L "$CONTAINER_SERIAL_BY_ID" ]; then
  current_target="$(readlink "$CONTAINER_SERIAL_BY_ID" || true)"
  if [ "$current_target" = "$HOST_SERIAL_BY_ID" ]; then
    exit 0
  fi
  rm -f "$CONTAINER_SERIAL_BY_ID"
elif [ -d "$CONTAINER_SERIAL_BY_ID" ]; then
  # A native by-id directory already exists in the container. Keep it.
  exit 0
elif [ -e "$CONTAINER_SERIAL_BY_ID" ]; then
  rm -f "$CONTAINER_SERIAL_BY_ID"
fi

ln -s "$HOST_SERIAL_BY_ID" "$CONTAINER_SERIAL_BY_ID"
