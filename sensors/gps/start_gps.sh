#!/bin/bash
# =============================================================================
# GPS sidecar startup.
#
# Transitional compatibility path (default):
#   ublox_dgnss + nmea_navsat_driver + existing diagnostics/RTCM glue
#
# Future single GNSS runtime:
#   set GPS_RUNTIME_MODE=universal to launch the Universal GNSS receiver +
#   NTRIP nodes and bridge their native status/RTCM topics onto:
#     /gps/fix         sensor_msgs/NavSatFix
#     /gps/status      mowgli_interfaces/GnssStatus
#     /diagnostics     diagnostic_msgs/DiagnosticArray
#     /rtcm            rtcm_msgs/Message
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

normalize_lower() {
  printf '%s' "${1:-}" | tr '[:upper:]' '[:lower:]'
}

normalize_bool() {
  case "$(normalize_lower "${1:-}")" in
    1|true|yes|y|on)
      printf 'true\n'
      ;;
    *)
      printf 'false\n'
      ;;
  esac
}

resolve_receiver_family() {
  if [ -n "${GNSS_RECEIVER_FAMILY:-}" ]; then
    printf '%s\n' "$(normalize_lower "${GNSS_RECEIVER_FAMILY}")"
    return 0
  fi

  local yaml_family
  yaml_family="$(parse_yaml gnss_receiver_family)"
  if [ -n "$yaml_family" ]; then
    printf '%s\n' "$(normalize_lower "$yaml_family")"
    return 0
  fi

  case "$(normalize_lower "${GNSS_BACKEND:-gps}")" in
    unicore)
      printf 'unicore\n'
      return 0
      ;;
    ublox)
      printf 'ublox\n'
      return 0
      ;;
  esac

  case "$(normalize_lower "${GPS_PROTOCOL:-UBX}")" in
    nmea)
      printf 'nmea\n'
      ;;
    *)
      printf 'auto\n'
      ;;
  esac
}

resolve_transport() {
  printf '%s\n' "${GNSS_TRANSPORT:-serial}"
}

resolve_serial_device() {
  if [ -n "${GNSS_SERIAL_DEVICE:-}" ]; then
    printf '%s\n' "$GNSS_SERIAL_DEVICE"
    return 0
  fi

  local yaml_serial_device
  yaml_serial_device="$(parse_yaml gnss_serial_device)"
  if [ -n "$yaml_serial_device" ]; then
    printf '%s\n' "$yaml_serial_device"
    return 0
  fi

  if [ -n "${GPS_DEVICE_PATH:-}" ]; then
    printf '%s\n' "$GPS_DEVICE_PATH"
    return 0
  fi

  if [ -n "${GPS_BY_ID:-}" ]; then
    printf '%s\n' "$GPS_BY_ID"
    return 0
  fi

  printf '%s\n' "$GPS_PORT"
}

resolve_serial_baud() {
  if [ -n "${GNSS_SERIAL_BAUD:-}" ]; then
    printf '%s\n' "$GNSS_SERIAL_BAUD"
    return 0
  fi

  local yaml_serial_baud
  yaml_serial_baud="$(parse_yaml gnss_serial_baud)"
  if [ -n "$yaml_serial_baud" ]; then
    printf '%s\n' "$yaml_serial_baud"
    return 0
  fi

  printf '%s\n' "$GPS_BAUD"
}

resolve_ntrip_enabled() {
  if [ -n "${GNSS_NTRIP_ENABLED:-}" ]; then
    normalize_bool "$GNSS_NTRIP_ENABLED"
    return 0
  fi

  normalize_bool "$(parse_yaml ntrip_enabled)"
}

resolve_ntrip_host() {
  if [ -n "${GNSS_NTRIP_HOST:-}" ]; then
    printf '%s\n' "$GNSS_NTRIP_HOST"
    return 0
  fi
  parse_yaml ntrip_host
}

resolve_ntrip_port() {
  if [ -n "${GNSS_NTRIP_PORT:-}" ]; then
    printf '%s\n' "$GNSS_NTRIP_PORT"
    return 0
  fi

  local port
  port="$(parse_yaml ntrip_port)"
  printf '%s\n' "${port:-2101}"
}

resolve_ntrip_username() {
  if [ -n "${GNSS_NTRIP_USERNAME:-}" ]; then
    printf '%s\n' "$GNSS_NTRIP_USERNAME"
    return 0
  fi
  parse_yaml ntrip_user
}

resolve_ntrip_password() {
  if [ -n "${GNSS_NTRIP_PASSWORD:-}" ]; then
    printf '%s\n' "$GNSS_NTRIP_PASSWORD"
    return 0
  fi
  parse_yaml ntrip_password
}

resolve_ntrip_mountpoint() {
  if [ -n "${GNSS_NTRIP_MOUNTPOINT:-}" ]; then
    printf '%s\n' "$GNSS_NTRIP_MOUNTPOINT"
    return 0
  fi
  parse_yaml ntrip_mountpoint
}

resolve_ntrip_gga_enabled() {
  if [ -n "${GNSS_NTRIP_GGA_ENABLED:-}" ]; then
    normalize_bool "$GNSS_NTRIP_GGA_ENABLED"
    return 0
  fi

  local mountpoint
  mountpoint="$(resolve_ntrip_mountpoint)"
  case "$(normalize_lower "$mountpoint")" in
    near*)
      printf 'true\n'
      ;;
    *)
      printf 'false\n'
      ;;
  esac
}

resolve_ntrip_gga_interval_s() {
  printf '%s\n' "${GNSS_NTRIP_GGA_INTERVAL_S:-10}"
}

resolve_frame_id() {
  printf '%s\n' "${GNSS_FRAME_ID:-gps_link}"
}

gps_runtime_mode="$(normalize_lower "${GPS_RUNTIME_MODE:-compat}")"

# Compose env wins over /config/mowgli_robot.yaml. The installer writes
# GPS_DEVICE_PATH / GPS_PROTOCOL / GPS_BAUD into docker/.env (see
# install/lib/env.sh) and the compose fragments forward them as container
# env vars; the YAML fallback keeps interactive `start_gps.sh` runs working
# inside a shelled-in container where only mowgli_robot.yaml is set.
GPS_PROTOCOL="${GPS_PROTOCOL:-$(parse_yaml gps_protocol)}"
GPS_PROTOCOL="${GPS_PROTOCOL:-UBX}"
GPS_PORT="${GPS_DEVICE_PATH:-$(parse_yaml gps_port)}"
GPS_PORT="${GPS_PORT:-/dev/gps}"
# gps_baudrate is the runtime baud for the main GNSS receiver.
GPS_BAUD="${GPS_BAUD:-$(parse_yaml gps_baudrate)}"
GPS_BAUD="${GPS_BAUD:-921600}"

NTRIP_ENABLED="$(resolve_ntrip_enabled)"
NTRIP_HOST="$(resolve_ntrip_host)"
NTRIP_PORT="$(resolve_ntrip_port)"
NTRIP_USER="$(resolve_ntrip_username)"
NTRIP_PASSWORD="$(resolve_ntrip_password)"
NTRIP_MOUNTPOINT="$(resolve_ntrip_mountpoint)"

set +u
source /opt/ros/kilted/setup.bash
if [ -f /opt/ublox_dgnss/setup.bash ]; then
  source /opt/ublox_dgnss/setup.bash
fi
if [ -f /opt/gnss_sidecar/setup.bash ]; then
  source /opt/gnss_sidecar/setup.bash
fi
set -u

GPS_PID=""
HP_PID=""
NTRIP_PID=""
HEALTH_PID=""
RTCM_BRIDGE_PID=""
UNIVERSAL_BRIDGE_PID=""

cleanup() {
  [ -n "$GPS_PID" ] && kill "$GPS_PID" 2>/dev/null || true
  [ -n "$HP_PID" ] && kill "$HP_PID" 2>/dev/null || true
  [ -n "$NTRIP_PID" ] && kill "$NTRIP_PID" 2>/dev/null || true
  [ -n "$HEALTH_PID" ] && kill "$HEALTH_PID" 2>/dev/null || true
  [ -n "$RTCM_BRIDGE_PID" ] && kill "$RTCM_BRIDGE_PID" 2>/dev/null || true
  [ -n "$UNIVERSAL_BRIDGE_PID" ] && kill "$UNIVERSAL_BRIDGE_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

start_compat_runtime() {
  if [ "$GPS_PROTOCOL" = "NMEA" ]; then
    # ── NMEA compatibility path ──────────────────────────────────────────────
    # Generic NMEA GPS (LC29H, BN-220, etc.) — serial only.
    echo "[start_gps.sh] Runtime=compat protocol=NMEA, device=$GPS_PORT @ ${GPS_BAUD} baud"
    ros2 run nmea_navsat_driver nmea_serial_driver --ros-args \
      -p port:="${GPS_PORT}" \
      -p baud:=${GPS_BAUD} \
      -p frame_id:=gps_link \
      -r /fix:=/gps/fix &
    GPS_PID=$!
  else
    # ── UBX compatibility path (default) ────────────────────────────────────
    # u-blox F9P via ublox_dgnss (libusb or serial).
    parse_driver_yaml() {
      grep -E "^\s+${1}:" /ublox_dgnss.yaml | head -1 | sed 's/.*:\s*//' | tr -d '"' | tr -d "'"
    }

    local transport
    transport="$(parse_driver_yaml TRANSPORT)"
    transport="${transport:-usb}"

    if [ "$transport" = "serial" ]; then
      # GPS_PORT is the host-side path the installer chose — usually a
      # /dev/serial/by-id/... symlink (always created by systemd-udev,
      # regardless of /etc/udev/rules.d/50-mowgli.rules). We override the
      # baked-in DEVICE_PATH from /ublox_dgnss.yaml via --param so we don't
      # need a /dev/gps udev symlink at all.
      local device_path
      device_path="$GPS_PORT"
      echo "[start_gps.sh] Runtime=compat transport=serial, device=$device_path"

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
            local ifname
            ifname="$(basename "$iface")"
            echo "[start_gps.sh] binding cdc_acm to $ifname"
            echo "$ifname" > /sys/bus/usb/drivers/cdc_acm/bind 2>/dev/null || true
          done
        done
      fi

      # Wait for the device path to appear (up to 5 s)
      for _ in $(seq 1 50); do
        [ -e "$device_path" ] && break
        sleep 0.1
      done
      # Resolve symlinks (e.g. /dev/serial/by-id/...) to a real char dev.
      local real_device
      real_device="$(readlink -f "$device_path" 2>/dev/null || echo "$device_path")"
      if [ ! -c "$real_device" ]; then
        echo "[start_gps.sh] ERROR: $device_path (-> $real_device) did not appear after 5s"
        echo "[start_gps.sh] Available serial-by-id entries:"
        ls -l /dev/serial/by-id/ 2>/dev/null || echo "  (none)"
        exit 1
      fi

      ros2 run ublox_dgnss_node ublox_dgnss_node --ros-args \
        --params-file /ublox_dgnss.yaml \
        -p "DEVICE_PATH:=${device_path}" &
    else
      echo "[start_gps.sh] Runtime=compat transport=usb (libusb)"
      ros2 run ublox_dgnss_node ublox_dgnss_node --ros-args \
        --params-file /ublox_dgnss.yaml &
    fi
    GPS_PID=$!

    # UBX HP -> NavSatFix — remap /fix -> /gps/fix for downstream consumers.
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
    echo "[start_gps.sh] Runtime=compat NTRIP enabled: ${NTRIP_HOST}:${NTRIP_PORT}/${NTRIP_MOUNTPOINT}"
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
}

start_universal_runtime() {
  if [ ! -f /opt/gnss_sidecar/setup.bash ]; then
    echo "[start_gps.sh] ERROR: /opt/gnss_sidecar/setup.bash not found; Universal GNSS overlay missing."
    exit 1
  fi

  # Hidden ROS topics keep the sidecar's internal Universal GNSS transport
  # off the public graph while remaining consumable by the local bridge.
  local internal_status_topic="/_gps_internal/universal/status"
  local internal_rtcm_topic="/_gps_internal/universal/rtcm"
  local receiver_family
  local transport
  local serial_device
  local serial_baud
  local frame_id
  local ntrip_gga_enabled
  local ntrip_gga_interval_s

  receiver_family="$(resolve_receiver_family)"
  transport="$(resolve_transport)"
  serial_device="$(resolve_serial_device)"
  serial_baud="$(resolve_serial_baud)"
  frame_id="$(resolve_frame_id)"
  ntrip_gga_enabled="$(resolve_ntrip_gga_enabled)"
  ntrip_gga_interval_s="$(resolve_ntrip_gga_interval_s)"

  echo "[start_gps.sh] Runtime=universal receiver_family=${receiver_family} transport=${transport} device=${serial_device} baud=${serial_baud}"

  ros2 run universal_gnss_ros2 receiver_node --ros-args \
    -p "receiver_family:=${receiver_family}" \
    -p "transport:=${transport}" \
    -p "serial_device:=${serial_device}" \
    -p "serial_baud:=${serial_baud}" \
    -p "publish_rate_hz:=5.0" \
    -p "frame_id:=${frame_id}" \
    -r status:=${internal_status_topic} \
    -r diagnostics:=/diagnostics \
    -r fix:=/gps/fix \
    -r rtcm:=${internal_rtcm_topic} &
  GPS_PID=$!

  python3 /universal_gnss_topic_bridge.py --ros-args \
    -p "backend:=universal" \
    -p "receiver_family:=${receiver_family}" \
    -p "frame_id:=${frame_id}" \
    -p "input_status_topic:=${internal_status_topic}" \
    -p "output_status_topic:=/gps/status" \
    -p "input_rtcm_topic:=${internal_rtcm_topic}" \
    -p "output_rtcm_topic:=/rtcm" &
  UNIVERSAL_BRIDGE_PID=$!

  if [ "$NTRIP_ENABLED" = "true" ]; then
    echo "[start_gps.sh] Runtime=universal NTRIP enabled: ${NTRIP_HOST}:${NTRIP_PORT}/${NTRIP_MOUNTPOINT}"
    sleep 3
    ros2 run universal_gnss_ros2 ntrip_node --ros-args \
      -p "caster_host:=${NTRIP_HOST}" \
      -p "caster_port:=${NTRIP_PORT}" \
      -p "mountpoint:=${NTRIP_MOUNTPOINT}" \
      -p "username:=${NTRIP_USER}" \
      -p "password:=${NTRIP_PASSWORD}" \
      -p "gga_enabled:=${ntrip_gga_enabled}" \
      -p "gga_interval_s:=${ntrip_gga_interval_s}" \
      -p "tls_enabled:=false" \
      -r status:=${internal_status_topic} \
      -r diagnostics:=/diagnostics \
      -r rtcm:=${internal_rtcm_topic} &
    NTRIP_PID=$!
  fi
}

case "$gps_runtime_mode" in
  universal)
    start_universal_runtime
    ;;
  compat|legacy|"")
    start_compat_runtime
    ;;
  *)
    echo "[start_gps.sh] ERROR: unsupported GPS_RUNTIME_MODE=${gps_runtime_mode} (expected compat or universal)"
    exit 1
    ;;
esac

wait -n || true
cleanup
wait
