#!/bin/bash
# =============================================================================
# GPS startup — ublox_dgnss driver + NTRIP client.
#
# ublox_dgnss finds the F9P by USB VID:PID, so no /dev path is needed.
# Reads NTRIP credentials from /config/mowgli_robot.yaml (bind-mounted).
#
# Launches:
#   1. ublox_dgnss_node              — F9P driver (libusb)
#   2. ublox_nav_sat_fix_hp_node     — UBX HP → NavSatFix on /fix → /gps/fix
#   3. ntrip_client_node             — NTRIP → RTCM → driver (via libusb)
# =============================================================================
set -euo pipefail

CONFIG="/config/mowgli_robot.yaml"

if [ ! -f "$CONFIG" ]; then
  echo "[start_gps.sh] ERROR: $CONFIG not found. Bind-mount config/mowgli/ to /config."
  exit 1
fi

parse_yaml() {
  grep -E "^\s+${1}:" "$CONFIG" | head -1 | sed 's/.*:\s*//' | tr -d '"' | tr -d "'"
}

# Compose env wins over /config/mowgli_robot.yaml. The installer writes
# GPS_DEVICE_PATH / GPS_PROTOCOL / GPS_BAUD into docker/.env (see
# install/lib/env.sh) and the compose fragments forward them as container
# env vars; the YAML fallback keeps interactive `start_gps.sh` runs working
# inside a shelled-in container where only mowgli_robot.yaml is set.
GPS_PROTOCOL="${GPS_PROTOCOL:-$(parse_yaml gps_protocol)}"
GPS_PROTOCOL="${GPS_PROTOCOL:-UBX}"
if [ -z "${GNSS_ENABLE_FIX_DIAGNOSTICS:-}" ]; then
  case "$GPS_PROTOCOL" in
    NMEA) GNSS_ENABLE_FIX_DIAGNOSTICS="true" ;;
    *)    GNSS_ENABLE_FIX_DIAGNOSTICS="false" ;;
  esac
fi
GPS_PORT="${GPS_DEVICE_PATH:-$(parse_yaml gps_port)}"
GPS_PORT="${GPS_PORT:-/dev/gps}"
# gps_baudrate is the runtime baud for the main GNSS receiver.
GPS_BAUD="${GPS_BAUD:-$(parse_yaml gps_baudrate)}"
GPS_BAUD="${GPS_BAUD:-921600}"

NTRIP_ENABLED=$(parse_yaml ntrip_enabled)
NTRIP_HOST=$(parse_yaml ntrip_host)
NTRIP_PORT=$(parse_yaml ntrip_port)
NTRIP_USER=$(parse_yaml ntrip_user)
NTRIP_PASSWORD=$(parse_yaml ntrip_password)
NTRIP_MOUNTPOINT=$(parse_yaml ntrip_mountpoint)
NTRIP_ENABLED="${NTRIP_ENABLED:-false}"

set +u
source /opt/ros/kilted/setup.bash
source /opt/ublox_dgnss/setup.bash
set -u

is_truthy() {
  case "${1,,}" in
    true|1|yes|y|on)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

GPS_PID=""
HP_PID=""
NTRIP_PID=""
HEALTH_PID=""
FIX_DIAG_PID=""

cleanup() {
  [ -n "$GPS_PID" ] && kill "$GPS_PID" 2>/dev/null || true
  [ -n "$HP_PID" ] && kill "$HP_PID" 2>/dev/null || true
  [ -n "$NTRIP_PID" ] && kill "$NTRIP_PID" 2>/dev/null || true
  [ -n "$HEALTH_PID" ] && kill "$HEALTH_PID" 2>/dev/null || true
  [ -n "$FIX_DIAG_PID" ] && kill "$FIX_DIAG_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [ "$GPS_PROTOCOL" = "NMEA" ]; then
  # ── NMEA mode ──────────────────────────────────────────────────────────────
  # Generic NMEA GPS (LC29H, BN-220, etc.) — serial only.
  echo "[start_gps.sh] Protocol=NMEA, device=$GPS_PORT @ ${GPS_BAUD} baud"
  ros2 run nmea_navsat_driver nmea_serial_driver --ros-args \
    -p port:="${GPS_PORT}" \
    -p baud:=${GPS_BAUD} \
    -p frame_id:=gps_link \
    -r /fix:=/gps/fix \
    -r /vel:=/gps/vel \
    -r /heading:=/gps/azimuth \
    -r /time_reference:=/gps/time_reference &
  GPS_PID=$!
  if is_truthy "$GNSS_ENABLE_FIX_DIAGNOSTICS"; then
    GNSS_DIAGNOSTIC_BACKEND="nmea" python3 /gnss_fix_diagnostics.py &
    FIX_DIAG_PID=$!
  fi
else
  # ── UBX mode (default) ────────────────────────────────────────────────────
  # u-blox F9P via ublox_dgnss (libusb or serial).
  parse_driver_yaml() {
    grep -E "^\s+${1}:" /ublox_dgnss.yaml | head -1 | sed 's/.*:\s*//' | tr -d '"' | tr -d "'"
  }

  TRANSPORT=$(parse_driver_yaml TRANSPORT)
  TRANSPORT="${TRANSPORT:-usb}"

  if [ "$TRANSPORT" = "serial" ]; then
    # GPS_PORT is the host-side path the installer chose — usually a
    # /dev/serial/by-id/... symlink (always created by systemd-udev,
    # regardless of /etc/udev/rules.d/50-mowgli.rules). We override the
    # baked-in DEVICE_PATH from /ublox_dgnss.yaml via --param so we don't
    # need a /dev/gps udev symlink at all.
    DEVICE_PATH="$GPS_PORT"
    echo "[start_gps.sh] Transport=serial, device=$DEVICE_PATH"

    # Bind cdc_acm to any unbound u-blox VID:PID (1546:01a9) interfaces.
    # The kernel usually auto-binds on hotplug, but on some platforms the
    # F9P enumerates before cdc_acm is ready. The old code hardcoded
    # "6-1:1.0 / 6-1:1.1" which only matched a single Pi USB topology.
    if [ -d /sys/bus/usb/drivers/cdc_acm ]; then
      for dev in /sys/bus/usb/devices/*; do
        [ -e "$dev/idVendor" ] && [ -e "$dev/idProduct" ] || continue
        [ "$(cat "$dev/idVendor" 2>/dev/null)" = "1546" ] || continue
        # F9P=01a9, F9R=01a8, X20P=01aa — accept the whole u-blox range.
        case "$(cat "$dev/idProduct" 2>/dev/null)" in
          01a8|01a9|01aa) ;;
          *) continue ;;
        esac
        for iface in "$dev":*; do
          [ -e "$iface" ] || continue
          [ -L "$iface/driver" ] && continue
          ifname="$(basename "$iface")"
          echo "[start_gps.sh] binding cdc_acm to $ifname"
          echo "$ifname" > /sys/bus/usb/drivers/cdc_acm/bind 2>/dev/null || true
        done
      done
    fi

    # Wait for the device path to appear (up to 5 s)
    for i in $(seq 1 50); do
      [ -e "$DEVICE_PATH" ] && break
      sleep 0.1
    done
    # Resolve symlinks (e.g. /dev/serial/by-id/...) to a real char dev.
    REAL_DEVICE="$(readlink -f "$DEVICE_PATH" 2>/dev/null || echo "$DEVICE_PATH")"
    if [ ! -c "$REAL_DEVICE" ]; then
      echo "[start_gps.sh] ERROR: $DEVICE_PATH (-> $REAL_DEVICE) did not appear after 5s"
      echo "[start_gps.sh] Available serial-by-id entries:"
      ls -l /dev/serial/by-id/ 2>/dev/null || echo "  (none)"
      exit 1
    fi
  else
    DEVICE_PATH=""
    echo "[start_gps.sh] Transport=usb (libusb)"
  fi

  if [ "$TRANSPORT" = "serial" ]; then
    ros2 run ublox_dgnss_node ublox_dgnss_node --ros-args \
      --params-file /ublox_dgnss.yaml \
      -p "DEVICE_PATH:=${DEVICE_PATH}" &
  else
    ros2 run ublox_dgnss_node ublox_dgnss_node --ros-args \
      --params-file /ublox_dgnss.yaml &
  fi
  GPS_PID=$!

  # UBX HP → NavSatFix — remap /fix → /gps/fix for downstream consumers.
  ros2 run ublox_nav_sat_fix_hp_node ublox_nav_sat_fix_hp --ros-args \
    --params-file /ublox_dgnss.yaml \
    -r /fix:=/gps/fix &
  HP_PID=$!
fi

# Health aggregator — only useful in UBX mode (consumes /ubx_* topics).
if [ "$GPS_PROTOCOL" != "NMEA" ]; then
  python3 /gps_health_aggregator.py &
  HEALTH_PID=$!
  if is_truthy "$GNSS_ENABLE_FIX_DIAGNOSTICS"; then
    GNSS_DIAGNOSTIC_BACKEND="ublox" python3 /gnss_fix_diagnostics.py &
    FIX_DIAG_PID=$!
  fi
fi

if [ "$NTRIP_ENABLED" = "true" ]; then
  echo "[start_gps.sh] NTRIP enabled: ${NTRIP_HOST}:${NTRIP_PORT}/${NTRIP_MOUNTPOINT}"
  sleep 3
  ros2 run ntrip_client_node ntrip_client_node --ros-args \
    --params-file /ublox_dgnss.yaml \
    -p "host:=${NTRIP_HOST}" \
    -p "port:=${NTRIP_PORT}" \
    -p "mountpoint:=${NTRIP_MOUNTPOINT}" \
    -p "username:=${NTRIP_USER}" \
    -p "password:=${NTRIP_PASSWORD}" &
  NTRIP_PID=$!
fi

wait -n || true
cleanup
wait
