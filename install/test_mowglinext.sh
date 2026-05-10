#!/usr/bin/env bash
# =============================================================================
# Tests for install/mowglinext.sh — preset loading and configuration functions
#
# Tests the core installer logic: preset file loading, GPS/LiDAR/rangefinder
# configuration with presets (skip prompts), and udev rule generation.
#
# Usage: bash install/test_mowglinext.sh
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── Test framework ─────────────────────────────────────────────────────────
TESTS_RUN=0
TESTS_PASSED=0
TESTS_FAILED=0

pass() {
  TESTS_PASSED=$((TESTS_PASSED + 1))
  TESTS_RUN=$((TESTS_RUN + 1))
  echo -e "  \033[0;32mPASS\033[0m  $1"
}

fail() {
  TESTS_FAILED=$((TESTS_FAILED + 1))
  TESTS_RUN=$((TESTS_RUN + 1))
  echo -e "  \033[0;31mFAIL\033[0m  $1"
  [ -n "${2:-}" ] && echo "        $2"
}

assert_eq() {
  local label="$1" expected="$2" actual="$3"
  if [ "$expected" = "$actual" ]; then
    pass "$label"
  else
    fail "$label" "expected='$expected' got='$actual'"
  fi
}

assert_not_empty() {
  local label="$1" value="$2"
  if [ -n "$value" ]; then
    pass "$label"
  else
    fail "$label" "expected non-empty value"
  fi
}

# ── Source common helpers (needed by configure_* functions) ─────────────────
source "$SCRIPT_DIR/lib/common.sh"
source "$SCRIPT_DIR/lib/i18n.sh"
source "$SCRIPT_DIR/lib/config.sh"
source "$SCRIPT_DIR/lib/state.sh"
load_locale

# Re-apply the test assertions after sourcing installer libs, which redefine
# pass()/fail() for user-facing status output only.
pass() {
  TESTS_PASSED=$((TESTS_PASSED + 1))
  TESTS_RUN=$((TESTS_RUN + 1))
  echo -e "  \033[0;32mPASS\033[0m  $1"
}

fail() {
  TESTS_FAILED=$((TESTS_FAILED + 1))
  TESTS_RUN=$((TESTS_RUN + 1))
  echo -e "  \033[0;31mFAIL\033[0m  $1"
  [ -n "${2:-}" ] && echo "        $2"
}

# ���─ Sandbox ────────────────────────────────────────────────────────────────
SANDBOX=$(mktemp -d)
trap 'rm -rf "$SANDBOX"' EXIT

# =============================================================================
# Test 1: load_preset — loads .preset file and sets PRESET_LOADED
# =============================================================================
echo ""
echo "── load_preset tests ──"

# Simulate load_preset (extracted from mowglinext.sh)
load_preset_test() {
  local test_dir="$1"
  local preset_file="${test_dir}/.preset"
  if [ -f "$preset_file" ]; then
    # shellcheck disable=SC1090
    source "$preset_file"
    PRESET_LOADED=true
  else
    PRESET_LOADED=false
  fi
}

# Test: no preset file
test_dir="$SANDBOX/no_preset"
mkdir -p "$test_dir"
PRESET_LOADED=""
load_preset_test "$test_dir"
assert_eq "No preset file: PRESET_LOADED=false" "false" "$PRESET_LOADED"

# Test: preset file exists with GPS config
test_dir="$SANDBOX/with_preset"
mkdir -p "$test_dir"
cat > "$test_dir/.preset" <<'EOF'
GPS_CONNECTION=usb
GPS_PROTOCOL=UBX
GPS_BAUD=921600
EOF
# Reset vars
unset GPS_CONNECTION GPS_PROTOCOL GPS_BAUD 2>/dev/null || true
PRESET_LOADED=""
load_preset_test "$test_dir"
assert_eq "With preset file: PRESET_LOADED=true" "true" "$PRESET_LOADED"
assert_eq "With preset file: GPS_CONNECTION=usb" "usb" "$GPS_CONNECTION"
assert_eq "With preset file: GPS_PROTOCOL=UBX" "UBX" "$GPS_PROTOCOL"
assert_eq "With preset file: GPS_BAUD=921600" "921600" "$GPS_BAUD"

# Test: preset file with all sections
test_dir="$SANDBOX/full_preset"
mkdir -p "$test_dir"
cat > "$test_dir/.preset" <<'EOF'
GPS_CONNECTION=uart
GPS_PROTOCOL=NMEA
GPS_BAUD=115200
GPS_UART_DEVICE=/dev/ttyAMA4
GPS_DEBUG_ENABLED=false
LIDAR_ENABLED=true
LIDAR_TYPE=rplidar
LIDAR_MODEL=RPLIDAR_A1
LIDAR_CONNECTION=usb
LIDAR_BAUD=115200
TFLUNA_FRONT_ENABLED=true
TFLUNA_EDGE_ENABLED=false
EOF
unset GPS_CONNECTION GPS_PROTOCOL GPS_BAUD LIDAR_ENABLED LIDAR_TYPE TFLUNA_FRONT_ENABLED TFLUNA_EDGE_ENABLED 2>/dev/null || true
load_preset_test "$test_dir"
assert_eq "Full preset: GPS_PROTOCOL=NMEA" "NMEA" "$GPS_PROTOCOL"
assert_eq "Full preset: LIDAR_TYPE=rplidar" "rplidar" "$LIDAR_TYPE"
assert_eq "Full preset: TFLUNA_FRONT_ENABLED=true" "true" "$TFLUNA_FRONT_ENABLED"
assert_eq "Full preset: TFLUNA_EDGE_ENABLED=false" "false" "$TFLUNA_EDGE_ENABLED"

# =============================================================================
# Test 2: configure_gps — preset mode (skips prompts)
# =============================================================================
echo ""
echo "── configure_gps with preset tests ──"

source "$SCRIPT_DIR/lib/serial_probe.sh"
source "$SCRIPT_DIR/lib/unicore_config.sh"
source "$SCRIPT_DIR/lib/gps.sh"

pick_uart_port() {
  REPLY="${1:-}"
}

# Test: GPS UBX-UART preset
unset GPS_BY_ID GPS_CONNECTION GPS_PROTOCOL GPS_PORT GPS_BAUD GPS_UART_DEVICE GPS_DEBUG_ENABLED GPS_UART_RULE GPS_DEBUG_UART_RULE 2>/dev/null || true
PRESET_LOADED=true
GPS_CONNECTION=uart
GPS_PROTOCOL=UBX
GPS_BAUD=921600
GPS_UART_DEVICE=/dev/ttyAMA4
GPS_DEBUG_ENABLED=false
configure_gps >/dev/null 2>&1
assert_eq "GPS preset uart: GPS_UART_RULE set" \
  'KERNEL=="ttyAMA4", SYMLINK+="gps", MODE="0666"' \
  "$GPS_UART_RULE"
assert_eq "GPS preset uart: no debug rule" "" "${GPS_DEBUG_UART_RULE:-}"

# Test: GPS UBX-USB preset (no udev rule needed)
unset GPS_BY_ID GPS_CONNECTION GPS_PROTOCOL GPS_PORT GPS_BAUD GPS_UART_DEVICE GPS_DEBUG_ENABLED GPS_UART_RULE GPS_DEBUG_UART_RULE 2>/dev/null || true
PRESET_LOADED=true
GPS_CONNECTION=usb
GPS_PROTOCOL=UBX
GPS_BAUD=921600
GPS_UART_DEVICE=
GPS_DEBUG_ENABLED=false
configure_gps >/dev/null 2>&1
assert_eq "GPS preset usb: no uart rule" "" "${GPS_UART_RULE:-}"

# Test: GPS with debug enabled
unset GPS_BY_ID GPS_CONNECTION GPS_PROTOCOL GPS_PORT GPS_BAUD GPS_UART_DEVICE GPS_DEBUG_ENABLED GPS_UART_RULE GPS_DEBUG_UART_RULE 2>/dev/null || true
PRESET_LOADED=true
GPS_CONNECTION=uart
GPS_PROTOCOL=UBX
GPS_BAUD=921600
GPS_UART_DEVICE=/dev/ttyAMA4
GPS_DEBUG_ENABLED=true
configure_gps >/dev/null 2>&1
assert_eq "GPS preset debug: debug uart rule set" \
  'KERNEL=="ttyS0", SYMLINK+="gps_debug", MODE="0666"' \
  "$GPS_DEBUG_UART_RULE"

# Test: invalid GNSS backend preset is rejected
unset GNSS_BACKEND GPS_CONNECTION GPS_PROTOCOL GPS_PORT GPS_BAUD GPS_UART_DEVICE GPS_DEBUG_ENABLED GPS_UART_RULE GPS_DEBUG_UART_RULE 2>/dev/null || true
PRESET_LOADED=true
GNSS_BACKEND=typo
GPS_CONNECTION=usb
GPS_PROTOCOL=UBX
GPS_BAUD=921600
GPS_DEBUG_ENABLED=false
if configure_gps >/dev/null 2>&1; then
  fail "GPS preset invalid GNSS_BACKEND rejected" "configure_gps unexpectedly succeeded"
else
  pass "GPS preset invalid GNSS_BACKEND rejected"
fi

# Test: Unicore USB preset requires an explicit by-id selection
unset GPS_BY_ID GNSS_BACKEND GPS_CONNECTION GPS_PROTOCOL GPS_PORT GPS_BAUD GPS_UART_DEVICE GPS_DEBUG_ENABLED GPS_UART_RULE GPS_DEBUG_UART_RULE 2>/dev/null || true
PRESET_LOADED=true
GNSS_BACKEND=unicore
GPS_CONNECTION=usb
GPS_PROTOCOL=UBX
GPS_BAUD=921600
GPS_DEBUG_ENABLED=false
if configure_gps >/dev/null 2>&1; then
  fail "GPS preset unicore USB requires GPS_BY_ID" "configure_gps unexpectedly succeeded"
else
  pass "GPS preset unicore USB requires GPS_BY_ID"
fi

# Test: invalid GNSS backend does not fall back to gps compose
REPO_DIR="$(dirname "$SCRIPT_DIR")"
INSTALL_DIR="$SCRIPT_DIR"
source "$SCRIPT_DIR/lib/compose.sh"
HARDWARE_BACKEND=mowgli
GNSS_BACKEND=typo
if build_compose_stack >/dev/null 2>&1; then
  fail "Compose invalid GNSS_BACKEND rejected" "build_compose_stack unexpectedly succeeded"
else
  pass "Compose invalid GNSS_BACKEND rejected"
fi

# Test: USB serial udev rules are generated from /dev/serial/by-id
source "$SCRIPT_DIR/lib/udev.sh"
SERIAL_BY_ID_DIR="$SANDBOX/serial/by-id"
mkdir -p "$SERIAL_BY_ID_DIR" "$SANDBOX/dev"
touch "$SANDBOX/dev/ttyACM0" "$SANDBOX/dev/ttyACM1" "$SANDBOX/dev/ttyACM2"
touch "$SANDBOX/dev/ttyUSB0"
ln -sf "$SANDBOX/dev/ttyACM0" "$SERIAL_BY_ID_DIR/usb-Mowgli-if00"
ln -sf "$SANDBOX/dev/ttyACM1" "$SERIAL_BY_ID_DIR/usb-u-blox_GNSS_receiver-if00"
ln -sf "$SANDBOX/dev/ttyACM2" "$SERIAL_BY_ID_DIR/usb-Pixhawk-if00"
ln -sf "$SANDBOX/dev/ttyUSB0" "$SERIAL_BY_ID_DIR/usb-1a86_USB_Serial-if00-port0"
HARDWARE_BACKEND=mavros
MAVROS_BY_ID="$SERIAL_BY_ID_DIR/usb-Pixhawk-if00"
GPS_CONNECTION=usb
GNSS_BACKEND=ublox
UBLOX_DEVICE_SERIAL_STRING="ublox-test-serial"
GPS_BY_ID=""
udev_rules="$(build_dynamic_udev_rules)"
case "$udev_rules" in
  *'KERNEL=="ttyACM0", SYMLINK+="mowgli", MODE="0666"'*) pass "udev by-id: Mowgli symlink rule" ;;
  *) fail "udev by-id: Mowgli symlink rule" "$udev_rules" ;;
esac
case "$udev_rules" in
  *'KERNEL=="ttyACM2", SYMLINK+="mavros", MODE="0666"'*) pass "udev by-id: MAVROS explicit symlink rule" ;;
  *) fail "udev by-id: MAVROS explicit symlink rule" "$udev_rules" ;;
esac

HARDWARE_BACKEND=mowgli
MAVROS_BY_ID=""
udev_rules="$(build_dynamic_udev_rules)"
case "$udev_rules" in
  *'SYMLINK+="gps"'*) fail "udev by-id: dedicated ublox backend does not create /dev/gps symlink" "$udev_rules" ;;
  *) pass "udev by-id: dedicated ublox backend does not create /dev/gps symlink" ;;
esac

GNSS_BACKEND=unicore
GPS_BY_ID=""
udev_rules="$(build_dynamic_udev_rules)"
case "$udev_rules" in
  *'SYMLINK+="gps"'*) fail "udev by-id: Unicore USB does not auto-detect generic adapter" "$udev_rules" ;;
  *) pass "udev by-id: Unicore USB does not auto-detect generic adapter" ;;
esac

GPS_BY_ID="$SERIAL_BY_ID_DIR/usb-1a86_USB_Serial-if00-port0"
udev_rules="$(build_dynamic_udev_rules)"
case "$udev_rules" in
  *'KERNEL=="ttyUSB0", SYMLINK+="gps", MODE="0666"'*) pass "udev by-id: explicit Unicore GPS_BY_ID symlink rule" ;;
  *) fail "udev by-id: explicit Unicore GPS_BY_ID symlink rule" "$udev_rules" ;;
esac

GPS_CONNECTION=uart
GPS_UART_DEVICE=/dev/ttyAMA4
udev_rules="$(build_dynamic_udev_rules)"
case "$udev_rules" in
  *'KERNEL=="ttyAMA4", SYMLINK+="gps", MODE="0666"'*) pass "udev uart: GPS kernel symlink rule preserved" ;;
  *) fail "udev uart: GPS kernel symlink rule preserved" "$udev_rules" ;;
esac
unset SERIAL_BY_ID_DIR MAVROS_BY_ID

# Test: emit_by_id_udev_rule prefers ATTRS{idVendor}/{idProduct}/{serial}
# (a stable match that survives USB re-enumeration) when udevadm exposes
# them. We mock udevadm to simulate a real USB tty and assert the emitted
# rule shape, then drop the mock to confirm the KERNEL== fallback path.
udevadm() {
  case "$1 $2" in
    "info --query=property")
      cat <<'PROPS'
ID_VENDOR_ID=1546
ID_MODEL_ID=01a9
ID_SERIAL_SHORT=DEADBEEF
PROPS
      ;;
  esac
}
export -f udevadm
mkdir -p "$SANDBOX/dev"
touch "$SANDBOX/dev/ttyACM9"
mkdir -p "$SANDBOX/serial-attrs"
ln -sf "$SANDBOX/dev/ttyACM9" "$SANDBOX/serial-attrs/usb-u-blox_GNSS-if00"
attrs_rule="$(emit_by_id_udev_rule "$SANDBOX/serial-attrs/usb-u-blox_GNSS-if00" gps)"
case "$attrs_rule" in
  *'SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", ATTRS{serial}=="DEADBEEF", SYMLINK+="gps", MODE="0666"'*)
    pass "udev attrs: emits stable VID/PID/serial rule when udevadm available" ;;
  *) fail "udev attrs: emits stable VID/PID/serial rule when udevadm available" "$attrs_rule" ;;
esac
unset -f udevadm

# =============================================================================
# Test 3: configure_lidar — preset mode (skips prompts)
# =============================================================================
echo ""
echo "── configure_lidar with preset tests ──"

source "$SCRIPT_DIR/lib/lidar.sh"

# Test: LDLiDAR UART preset
unset LIDAR_ENABLED LIDAR_TYPE LIDAR_MODEL LIDAR_CONNECTION LIDAR_PORT LIDAR_UART_DEVICE LIDAR_BAUD LIDAR_UART_RULE 2>/dev/null || true
PRESET_LOADED=true
LIDAR_ENABLED=true
LIDAR_TYPE=ldlidar
LIDAR_MODEL=LDLiDAR_LD19
LIDAR_CONNECTION=uart
LIDAR_UART_DEVICE=/dev/ttyAMA5
LIDAR_BAUD=230400
configure_lidar >/dev/null 2>&1
assert_eq "LiDAR preset ldlidar-uart: rule set" \
  'KERNEL=="ttyAMA5", SYMLINK+="lidar", MODE="0666"' \
  "$LIDAR_UART_RULE"
assert_eq "LiDAR preset ldlidar-uart: port defaulted" "/dev/lidar" "$LIDAR_PORT"

# Test: LDLiDAR USB preset (no udev rule)
unset LIDAR_ENABLED LIDAR_TYPE LIDAR_MODEL LIDAR_CONNECTION LIDAR_PORT LIDAR_UART_DEVICE LIDAR_BAUD LIDAR_UART_RULE 2>/dev/null || true
PRESET_LOADED=true
LIDAR_ENABLED=true
LIDAR_TYPE=ldlidar
LIDAR_MODEL=LDLiDAR_LD19
LIDAR_CONNECTION=usb
LIDAR_UART_DEVICE=
LIDAR_BAUD=230400
configure_lidar >/dev/null 2>&1
assert_eq "LiDAR preset ldlidar-usb: no uart rule" "" "${LIDAR_UART_RULE:-}"

# Test: No LiDAR preset
unset LIDAR_ENABLED LIDAR_TYPE LIDAR_MODEL LIDAR_CONNECTION LIDAR_PORT LIDAR_UART_DEVICE LIDAR_BAUD LIDAR_UART_RULE 2>/dev/null || true
PRESET_LOADED=true
LIDAR_ENABLED=false
LIDAR_TYPE=none
LIDAR_MODEL=
LIDAR_CONNECTION=
LIDAR_UART_DEVICE=
LIDAR_BAUD=0
configure_lidar >/dev/null 2>&1
assert_eq "LiDAR preset none: no uart rule" "" "${LIDAR_UART_RULE:-}"

# Test: RPLidar UART preset
unset LIDAR_ENABLED LIDAR_TYPE LIDAR_MODEL LIDAR_CONNECTION LIDAR_PORT LIDAR_UART_DEVICE LIDAR_BAUD LIDAR_UART_RULE 2>/dev/null || true
PRESET_LOADED=true
LIDAR_ENABLED=true
LIDAR_TYPE=rplidar
LIDAR_MODEL=RPLIDAR_A1
LIDAR_CONNECTION=uart
LIDAR_UART_DEVICE=/dev/ttyAMA5
LIDAR_BAUD=115200
configure_lidar >/dev/null 2>&1
assert_eq "LiDAR preset rplidar-uart: type preserved" "rplidar" "$LIDAR_TYPE"
assert_eq "LiDAR preset rplidar-uart: baud preserved" "115200" "$LIDAR_BAUD"
assert_not_empty "LiDAR preset rplidar-uart: rule set" "$LIDAR_UART_RULE"

# =============================================================================
# Test 4: configure_rangefinders — preset mode (skips prompts)
# =============================================================================
echo ""
echo "── configure_rangefinders with preset tests ──"

source "$SCRIPT_DIR/lib/range.sh"

# Test: No rangefinders
unset TFLUNA_FRONT_ENABLED TFLUNA_EDGE_ENABLED TFLUNA_FRONT_UART_RULE TFLUNA_EDGE_UART_RULE 2>/dev/null || true
PRESET_LOADED=true
TFLUNA_FRONT_ENABLED=false
TFLUNA_EDGE_ENABLED=false
configure_rangefinders >/dev/null 2>&1
assert_eq "Range preset none: no front rule" "" "${TFLUNA_FRONT_UART_RULE:-}"
assert_eq "Range preset none: no edge rule" "" "${TFLUNA_EDGE_UART_RULE:-}"

# Test: Front only
unset TFLUNA_FRONT_ENABLED TFLUNA_EDGE_ENABLED TFLUNA_FRONT_UART_RULE TFLUNA_EDGE_UART_RULE 2>/dev/null || true
PRESET_LOADED=true
TFLUNA_FRONT_ENABLED=true
TFLUNA_EDGE_ENABLED=false
configure_rangefinders >/dev/null 2>&1
assert_eq "Range preset front: feature unavailable disables front sensor" "false" "${TFLUNA_FRONT_ENABLED:-}"
assert_eq "Range preset front: no front rule when unavailable" "" "${TFLUNA_FRONT_UART_RULE:-}"
assert_eq "Range preset front: no edge rule" "" "${TFLUNA_EDGE_UART_RULE:-}"

# Test: Edge only
unset TFLUNA_FRONT_ENABLED TFLUNA_EDGE_ENABLED TFLUNA_FRONT_UART_RULE TFLUNA_EDGE_UART_RULE 2>/dev/null || true
PRESET_LOADED=true
TFLUNA_FRONT_ENABLED=false
TFLUNA_EDGE_ENABLED=true
configure_rangefinders >/dev/null 2>&1
assert_eq "Range preset edge: no front rule" "" "${TFLUNA_FRONT_UART_RULE:-}"
assert_eq "Range preset edge: feature unavailable disables edge sensor" "false" "${TFLUNA_EDGE_ENABLED:-}"
assert_eq "Range preset edge: no edge rule when unavailable" "" "${TFLUNA_EDGE_UART_RULE:-}"

# Test: Both
unset TFLUNA_FRONT_ENABLED TFLUNA_EDGE_ENABLED TFLUNA_FRONT_UART_RULE TFLUNA_EDGE_UART_RULE 2>/dev/null || true
PRESET_LOADED=true
TFLUNA_FRONT_ENABLED=true
TFLUNA_EDGE_ENABLED=true
configure_rangefinders >/dev/null 2>&1
assert_eq "Range preset both: feature unavailable disables front sensor" "false" "${TFLUNA_FRONT_ENABLED:-}"
assert_eq "Range preset both: feature unavailable disables edge sensor" "false" "${TFLUNA_EDGE_ENABLED:-}"
assert_eq "Range preset both: no front rule when unavailable" "" "${TFLUNA_FRONT_UART_RULE:-}"
assert_eq "Range preset both: no edge rule when unavailable" "" "${TFLUNA_EDGE_UART_RULE:-}"

# =============================================================================
# Test 5: Script syntax validation
# =============================================================================
echo ""
echo "── Script syntax tests ──"

for script in \
  "$SCRIPT_DIR/mowglinext.sh" \
  "$SCRIPT_DIR/lib/gps.sh" \
  "$SCRIPT_DIR/lib/serial_probe.sh" \
  "$SCRIPT_DIR/lib/unicore_config.sh" \
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

# =============================================================================
# Test 6: Preset mode does NOT prompt (functional check)
# =============================================================================
echo ""
echo "── Non-interactive mode tests ──"

# When PRESET_LOADED=true and vars are set, configure_gps should not call prompt()
# We test this by overriding prompt to fail
prompt() {
  fail "prompt() was called in preset mode — should be skipped"
  REPLY=""
}
confirm() {
  fail "confirm() was called in preset mode — should be skipped"
  return 0
}

# GPS with preset — should NOT prompt
unset GNSS_BACKEND GPS_CONNECTION GPS_PROTOCOL GPS_PORT GPS_BAUD GPS_UART_DEVICE GPS_DEBUG_ENABLED GPS_UART_RULE GPS_DEBUG_UART_RULE UBLOX_DEVICE_SERIAL_STRING 2>/dev/null || true
PRESET_LOADED=true
GNSS_BACKEND=ublox
GPS_CONNECTION=usb
GPS_PROTOCOL=UBX
GPS_BAUD=115200
GPS_UART_DEVICE=
UBLOX_DEVICE_SERIAL_STRING=ublox-test-serial
GPS_DEBUG_ENABLED=false
configure_gps >/dev/null 2>&1
assert_eq "GPS preset mode: keeps dedicated ublox serial string" "ublox-test-serial" "$UBLOX_DEVICE_SERIAL_STRING"
pass "GPS preset mode: no interactive prompts"

# LiDAR with preset — should NOT prompt
unset LIDAR_ENABLED LIDAR_TYPE LIDAR_MODEL LIDAR_CONNECTION LIDAR_PORT LIDAR_UART_DEVICE LIDAR_BAUD LIDAR_UART_RULE 2>/dev/null || true
PRESET_LOADED=true
LIDAR_ENABLED=true
LIDAR_TYPE=stl27l
LIDAR_MODEL=STL27L
LIDAR_CONNECTION=uart
LIDAR_UART_DEVICE=/dev/ttyAMA5
LIDAR_BAUD=230400
configure_lidar >/dev/null 2>&1
pass "LiDAR preset mode: no interactive prompts"

# Rangefinders with preset — should NOT prompt
unset TFLUNA_FRONT_ENABLED TFLUNA_EDGE_ENABLED TFLUNA_FRONT_UART_RULE TFLUNA_EDGE_UART_RULE 2>/dev/null || true
PRESET_LOADED=true
TFLUNA_FRONT_ENABLED=true
TFLUNA_EDGE_ENABLED=true
configure_rangefinders >/dev/null 2>&1
pass "Rangefinder preset mode: no interactive prompts"

# =============================================================================
# Summary
# =============================================================================
echo ""
echo "══════════════════════════════════════════"
echo "  Tests: $TESTS_RUN  Passed: $TESTS_PASSED  Failed: $TESTS_FAILED"
echo "══════════════════════════════════════════"

[ "$TESTS_FAILED" -eq 0 ] && exit 0 || exit 1
