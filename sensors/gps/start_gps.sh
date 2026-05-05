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

GPS_PROTOCOL=$(parse_yaml gps_protocol)
GPS_PROTOCOL="${GPS_PROTOCOL:-UBX}"
GPS_PORT=$(parse_yaml gps_port)
GPS_PORT="${GPS_PORT:-/dev/gps}"
GPS_BAUD=$(parse_yaml gps_baudrate)
GPS_BAUD="${GPS_BAUD:-460800}"

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

GPS_PID=""
HP_PID=""
NTRIP_PID=""
HEALTH_PID=""
RTCM_BRIDGE_PID=""

cleanup() {
  [ -n "$GPS_PID" ] && kill "$GPS_PID" 2>/dev/null || true
  [ -n "$HP_PID" ] && kill "$HP_PID" 2>/dev/null || true
  [ -n "$NTRIP_PID" ] && kill "$NTRIP_PID" 2>/dev/null || true
  [ -n "$HEALTH_PID" ] && kill "$HEALTH_PID" 2>/dev/null || true
  [ -n "$RTCM_BRIDGE_PID" ] && kill "$RTCM_BRIDGE_PID" 2>/dev/null || true
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
    -r /fix:=/gps/fix &
  GPS_PID=$!
else
  # ── UBX mode (default) ────────────────────────────────────────────────────
  # u-blox F9P via ublox_dgnss (libusb or serial).
  parse_driver_yaml() {
    grep -E "^\s+${1}:" /ublox_dgnss.yaml | head -1 | sed 's/.*:\s*//' | tr -d '"' | tr -d "'"
  }

  TRANSPORT=$(parse_driver_yaml TRANSPORT)
  TRANSPORT="${TRANSPORT:-usb}"

  if [ "$TRANSPORT" = "serial" ]; then
    DEVICE_PATH=$(parse_driver_yaml DEVICE_PATH)
    DEVICE_PATH="${DEVICE_PATH:-/dev/gps}"
    echo "[start_gps.sh] Transport=serial, device=$DEVICE_PATH"

    # Ensure the kernel CDC ACM driver is bound to the F9P USB interfaces.
    for IF in 6-1:1.0 6-1:1.1; do
      if [ -e "/sys/bus/usb/devices/$IF" ] && [ ! -L "/sys/bus/usb/devices/$IF/driver" ]; then
        echo "[start_gps.sh] binding cdc_acm to $IF"
        echo "$IF" > /sys/bus/usb/drivers/cdc_acm/bind 2>/dev/null || true
      fi
    done

    # Wait for the device path to appear (up to 5 s)
    for i in $(seq 1 50); do
      [ -c "$DEVICE_PATH" ] && break
      sleep 0.1
    done
    if [ ! -c "$DEVICE_PATH" ]; then
      echo "[start_gps.sh] ERROR: $DEVICE_PATH did not appear after 5s"
      exit 1
    fi
  else
    echo "[start_gps.sh] Transport=usb (libusb)"
  fi

  ros2 run ublox_dgnss_node ublox_dgnss_node --ros-args \
    --params-file /ublox_dgnss.yaml &
  GPS_PID=$!

  # UBX HP → NavSatFix — remap /fix → /gps/fix for downstream consumers.
  ros2 run ublox_nav_sat_fix_hp_node ublox_nav_sat_fix_hp --ros-args \
    --params-file /ublox_dgnss.yaml \
    -r /fix:=/gps/fix &
  HP_PID=$!
fi

# Health aggregator — UBX mode reports fix/satellites/RTCM,
# NMEA mode reports the RTCM forwarding stream only.
python3 /gps_health_aggregator.py --ros-args -p "protocol:=${GPS_PROTOCOL}" &
HEALTH_PID=$!

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

  # NMEA receivers (LC29H, BN-220, …) need RTCM3 bytes written into the
  # serial port. ublox_dgnss handles this internally for UBX, but
  # nmea_navsat_driver is read-only — bridge the topic to the device.
  if [ "$GPS_PROTOCOL" = "NMEA" ]; then
    python3 /rtcm_serial_bridge.py --ros-args -p "device:=${GPS_PORT}" &
    RTCM_BRIDGE_PID=$!
  fi
fi

wait -n || true
cleanup
wait
