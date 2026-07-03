#!/usr/bin/env bash
# =============================================================================
# Core installer coverage for install/mowglinext.sh and lib helpers
# =============================================================================

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=tests/lib/framework.sh
source "$SCRIPT_DIR/tests/lib/framework.sh"
# shellcheck source=tests/lib/mocks.sh
source "$SCRIPT_DIR/tests/lib/mocks.sh"
# shellcheck source=tests/lib/harness.sh
source "$SCRIPT_DIR/tests/lib/harness.sh"

setup_sandbox
install_all_mocks

section "Preset parsing uses the Universal GNSS contract"

repo_preset="$SANDBOX/repo_preset"
sandbox_repo "$repo_preset"
harness_init "$repo_preset"

cat > "$repo_preset/install/.preset" <<'EOF'
GNSS_RECEIVER_FAMILY=unicore
GNSS_SERIAL_DEVICE=/dev/serial/by-id/usb-gnss
GNSS_SERIAL_BAUD=921600
LIDAR_TYPE=ldlidar
EOF

unset GNSS_RECEIVER_FAMILY GNSS_SERIAL_DEVICE GNSS_SERIAL_BAUD LIDAR_TYPE 2>/dev/null || true
load_preset_file "$repo_preset/install/.preset"
assert_eq "preset sets GNSS_RECEIVER_FAMILY" "unicore" "${GNSS_RECEIVER_FAMILY:-}"
assert_eq "preset sets GNSS_SERIAL_DEVICE" "/dev/serial/by-id/usb-gnss" "${GNSS_SERIAL_DEVICE:-}"
assert_eq "preset sets GNSS_SERIAL_BAUD" "921600" "${GNSS_SERIAL_BAUD:-}"
assert_eq "preset keeps LiDAR settings" "ldlidar" "${LIDAR_TYPE:-}"

section "configure_gps preserves non-interactive GNSS presets"

GNSS_RECEIVER_FAMILY="unicore"
GNSS_SERIAL_DEVICE="/dev/ttyAMA4"
GNSS_SERIAL_BAUD="460800"
PRESET_LOADED=true
STATE_ACTIVE_PRESET_COUNT=3
STATE_PARSED_KEYS=(GNSS_RECEIVER_FAMILY GNSS_SERIAL_DEVICE GNSS_SERIAL_BAUD)

assert_exit_zero "configure_gps with GNSS-only preset" configure_gps
assert_eq "configure_gps keeps backend universal" "universal" "${GNSS_BACKEND:-}"
assert_eq "configure_gps keeps receiver family" "unicore" "${GNSS_RECEIVER_FAMILY:-}"
assert_eq "configure_gps keeps serial device" "/dev/ttyAMA4" "${GNSS_SERIAL_DEVICE:-}"
assert_eq "configure_gps keeps explicit baud" "460800" "${GNSS_SERIAL_BAUD:-}"

section "dynamic udev rules follow GNSS_SERIAL_DEVICE"

SERIAL_BY_ID_DIR="$SANDBOX/serial/by-id"
mkdir -p "$SERIAL_BY_ID_DIR" "$SANDBOX/dev"
touch "$SANDBOX/dev/ttyACM0" "$SANDBOX/dev/ttyUSB0"
ln -sf "$SANDBOX/dev/ttyACM0" "$SERIAL_BY_ID_DIR/usb-STMicroelectronics_Mowgli_test-if00"
ln -sf "$SANDBOX/dev/ttyUSB0" "$SERIAL_BY_ID_DIR/usb-GNSS-if00"

HARDWARE_BACKEND=mowgli
GNSS_BACKEND=universal
GNSS_RECEIVER_FAMILY=ublox
GNSS_SERIAL_DEVICE="$SERIAL_BY_ID_DIR/usb-GNSS-if00"
rules_usb="$(build_dynamic_udev_rules)"
assert_contains "USB GNSS udev rule emits /dev/gps symlink" 'SYMLINK+="gps"' "$rules_usb"

GNSS_SERIAL_DEVICE=/dev/ttyAMA4
rules_uart="$(build_dynamic_udev_rules)"
assert_contains "UART GNSS udev rule pins ttyAMA4" 'KERNEL=="ttyAMA4", SYMLINK+="gps", MODE="0666"' "$rules_uart"

section "end-to-end harness run writes clean Universal GNSS outputs"

repo_flow="$SANDBOX/repo_flow"
sandbox_repo "$repo_flow"
harness_init "$repo_flow"
harness_set_preset gnss=auto gnss_connection=uart lidar=ldlidar-uart tfluna=none

if harness_run; then
  pass "harness_run succeeds"
else
  fail "harness_run succeeds" "non-zero exit"
  test_summary
  exit 1
fi

env_content="$(cat "$repo_flow/docker/.env")"
legacy_protocol_key="GPS_""PROTOCOL="
legacy_port_key="GPS_""PORT="
legacy_baud_key="GPS_""BAUD="
legacy_by_id_key="GPS_""BY_ID="
legacy_ublox_key="UBLOX_""DEVICE_SERIAL_STRING="

assert_contains "runtime env keeps GNSS_BACKEND=universal" "GNSS_BACKEND=universal" "$env_content"
assert_contains "runtime env keeps GNSS_SERIAL_DEVICE=/dev/ttyAMA4" "GNSS_SERIAL_DEVICE=/dev/ttyAMA4" "$env_content"
assert_contains "runtime env keeps GNSS_NTRIP_ENABLED=true" "GNSS_NTRIP_ENABLED=true" "$env_content"
assert_contains "runtime env keeps GNSS_RTCM_FORWARDING=true" "GNSS_RTCM_FORWARDING=true" "$env_content"
assert_not_contains "runtime env omits legacy protocol key" "$legacy_protocol_key" "$env_content"
assert_not_contains "runtime env omits legacy port key" "$legacy_port_key" "$env_content"
assert_not_contains "runtime env omits legacy baud key" "$legacy_baud_key" "$env_content"
assert_not_contains "runtime env omits legacy by-id key" "$legacy_by_id_key" "$env_content"
assert_not_contains "runtime env omits legacy UBLOX serial key" "$legacy_ublox_key" "$env_content"

assert_file_exists "docker-compose.yaml materialised" "$repo_flow/docker/docker-compose.yaml"
assert_file_exists "mowgli_robot.yaml materialised" "$repo_flow/docker/config/mowgli/mowgli_robot.yaml"
assert_file_not_exists "legacy mower_config.sh omitted" "$repo_flow/docker/config/om/mower_config.sh"

section "Script syntax validation"

for script in \
  "$SCRIPT_DIR/mowglinext.sh" \
  "$SCRIPT_DIR/lib/gps.sh" \
  "$SCRIPT_DIR/lib/serial_probe.sh" \
  "$SCRIPT_DIR/lib/unicore_config.sh" \
  "$SCRIPT_DIR/lib/ublox_config.sh" \
  "$SCRIPT_DIR/lib/lidar.sh" \
  "$SCRIPT_DIR/lib/range.sh" \
  "$SCRIPT_DIR/lib/common.sh" \
  "$SCRIPT_DIR/lib/config.sh" \
  "$SCRIPT_DIR/lib/env.sh" \
  "$SCRIPT_DIR/lib/compose.sh"; do
  name=$(basename "$script")
  if bash -n "$script" 2>/dev/null; then
    pass "$name passes syntax check"
  else
    fail "$name has syntax errors"
  fi
done

test_summary
