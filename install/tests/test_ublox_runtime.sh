#!/usr/bin/env bash
# =============================================================================
# Runtime wiring for GNSS_BACKEND=ublox
# =============================================================================

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=lib/framework.sh
source "$SCRIPT_DIR/lib/framework.sh"

compose_file="$REPO_ROOT/install/compose/docker-compose.ublox.yaml"
launch_file="$REPO_ROOT/ros2/src/mowgli_bringup/launch/ublox_gnss.launch.py"
config_file="$REPO_ROOT/ros2/src/mowgli_bringup/config/ublox_gnss.yaml"

section "Compose wiring"

compose_content="$(cat "$compose_file")"
assert_not_contains "compose does not pass GPS_BAUD to dedicated ublox backend" 'ublox_baud_rate:=' "$compose_content"
assert_contains "compose passes dedicated u-blox serial string" 'ublox_device_serial_string:=${UBLOX_DEVICE_SERIAL_STRING}' "$compose_content"
assert_contains "compose documents dedicated ublox backend transport" 'GNSS_BACKEND=ublox uses the dedicated USB/libusb u-blox driver.' "$compose_content"

section "Launch wiring"

launch_content="$(cat "$launch_file")"
assert_contains "launch still injects device serial string" '"DEVICE_SERIAL_STRING": device_serial_string' "$launch_content"
assert_not_contains "launch no longer declares ublox_baud_rate" '"ublox_baud_rate"' "$launch_content"
assert_not_contains "launch no longer injects BAUD_RATE" '"BAUD_RATE":' "$launch_content"

section "Config presence"

assert_file_exists "ublox runtime config file exists" "$config_file"
config_content="$(cat "$config_file")"
assert_contains "ublox runtime config declares ublox_dgnss section" 'ublox_dgnss:' "$config_content"
assert_contains "ublox runtime config declares adapter section" 'ublox_gnss_adapter:' "$config_content"
assert_contains "ublox runtime config documents serial ubx fallback" 'Serial UBX receivers should use GNSS_BACKEND=gps with GPS_PROTOCOL=UBX.' "$config_content"

test_summary
