#!/bin/bash
# =============================================================================
# Generic NMEA driver launcher.
#
# Reads from a single serial device — UART (/dev/ttyS*, /dev/ttyAMA*) or
# USB (/dev/ttyACM*, /dev/ttyUSB*). The compose layer maps the host device
# to NMEA_PORT inside the container so this script does not care which.
#
# Env:
#   NMEA_PORT      device path inside container  (default /dev/gps)
#   NMEA_BAUD      serial baud rate              (default 9600)
#   NMEA_FRAME_ID  TF frame for NavSatFix        (default gps_link)
# =============================================================================
set -euo pipefail

NMEA_PORT="${NMEA_PORT:-/dev/gps}"
NMEA_BAUD="${NMEA_BAUD:-9600}"
NMEA_FRAME_ID="${NMEA_FRAME_ID:-gps_link}"

echo "[start_nmea.sh] port=${NMEA_PORT} baud=${NMEA_BAUD} frame=${NMEA_FRAME_ID}"

# Wait for the device path (up to 5 s) — udev/by-id symlinks may race
# the container start.
for i in $(seq 1 50); do
  [ -e "${NMEA_PORT}" ] && break
  sleep 0.1
done
if [ ! -e "${NMEA_PORT}" ]; then
  echo "[start_nmea.sh] ERROR: ${NMEA_PORT} did not appear after 5s"
  exit 1
fi

# nmea_serial_driver publishes /fix (NavSatFix), /vel (TwistStamped),
# /time_reference, /heading. Remap only the project-supported /gps/*
# topics here; generic NMEA heading is not exposed as /gps/azimuth because
# it is not a compass_msgs/Azimuth stream.
exec ros2 run nmea_navsat_driver nmea_serial_driver --ros-args \
  -p port:="${NMEA_PORT}" \
  -p baud:="${NMEA_BAUD}" \
  -p frame_id:="${NMEA_FRAME_ID}" \
  -p use_GNSS_time:=false \
  -p time_ref_source:="${NMEA_FRAME_ID}" \
  -r /fix:=/gps/fix \
  -r /vel:=/gps/vel \
  -r /time_reference:=/gps/time_reference
