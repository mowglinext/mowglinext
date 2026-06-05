#!/usr/bin/env bash
# =============================================================================
# Universal GNSS installer coverage
# =============================================================================

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=lib/framework.sh
source "$SCRIPT_DIR/lib/framework.sh"
# shellcheck source=lib/mocks.sh
source "$SCRIPT_DIR/lib/mocks.sh"
# shellcheck source=lib/harness.sh
source "$SCRIPT_DIR/lib/harness.sh"

env_value() {
  grep -E "^${2}=" "$1/docker/.env" | head -1 | cut -d= -f2-
}

selected_fragments_in_current_run() {
  local fragment
  for fragment in "${COMPOSE_FILES[@]}"; do
    basename "$fragment"
  done | sort
}

assert_no_legacy_gnss_fragment() {
  local label="$1"
  local fragments
  fragments="$(selected_fragments_in_current_run)"
  case "$fragments" in
    *docker-compose.gps.yml*|*docker-compose.unicore.yaml*)
      fail "$label" "got: $fragments"
      ;;
    *)
      pass "$label"
      ;;
  esac
}

setup_sandbox
install_all_mocks

section "Universal GNSS default UART path"

repo="$SANDBOX/repo_universal_uart"
sandbox_repo "$repo"
harness_init "$repo"
harness_set_preset gnss=auto lidar=none tfluna=none
if harness_run; then pass "harness_run universal uart"; else fail "harness_run universal uart"; fi
assert_eq "uart: GNSS_STACK=universal" "universal" "$(env_value "$repo" GNSS_STACK)"
assert_eq "uart: GNSS_STATUS_SOURCE=universal" "universal" "$(env_value "$repo" GNSS_STATUS_SOURCE)"
assert_eq "uart: GNSS_RECEIVER_FAMILY=auto" "auto" "$(env_value "$repo" GNSS_RECEIVER_FAMILY)"
assert_eq "uart: GNSS_TRANSPORT=serial" "serial" "$(env_value "$repo" GNSS_TRANSPORT)"
assert_eq "uart: GNSS_SERIAL_DEVICE=/dev/ttyAMA4" "/dev/ttyAMA4" "$(env_value "$repo" GNSS_SERIAL_DEVICE)"
assert_eq "uart: GNSS_SERIAL_BAUD=921600" "921600" "$(env_value "$repo" GNSS_SERIAL_BAUD)"
assert_eq "uart: compatibility GPS_CONNECTION=uart" "uart" "$(env_value "$repo" GPS_CONNECTION)"
assert_eq "uart: compatibility GPS_PROTOCOL=UBX" "UBX" "$(env_value "$repo" GPS_PROTOCOL)"
assert_eq "uart: compatibility GPS_UART_DEVICE=/dev/ttyAMA4" "/dev/ttyAMA4" "$(env_value "$repo" GPS_UART_DEVICE)"
assert_no_legacy_gnss_fragment "uart: no legacy GNSS sidecar in default compose"

section "Universal GNSS USB by-id path"

repo="$SANDBOX/repo_universal_usb"
sandbox_repo "$repo"
harness_init "$repo"
GPS_CONNECTION=usb
harness_set_preset gnss=auto lidar=none tfluna=none
if harness_run; then pass "harness_run universal usb"; else fail "harness_run universal usb"; fi
assert_eq "usb: GNSS_STACK=universal" "universal" "$(env_value "$repo" GNSS_STACK)"
assert_eq "usb: GNSS_RECEIVER_FAMILY=auto" "auto" "$(env_value "$repo" GNSS_RECEIVER_FAMILY)"
assert_eq "usb: GNSS_SERIAL_DEVICE uses by-id path" "/dev/serial/by-id/usb-stub" "$(env_value "$repo" GNSS_SERIAL_DEVICE)"
assert_eq "usb: GNSS_SERIAL_BAUD=921600" "921600" "$(env_value "$repo" GNSS_SERIAL_BAUD)"
assert_eq "usb: compatibility GPS_CONNECTION=usb" "usb" "$(env_value "$repo" GPS_CONNECTION)"
assert_eq "usb: compatibility GPS_BY_ID uses by-id path" "/dev/serial/by-id/usb-stub" "$(env_value "$repo" GPS_BY_ID)"
assert_eq "usb: compatibility GPS_PORT uses by-id path" "/dev/serial/by-id/usb-stub" "$(env_value "$repo" GPS_PORT)"
assert_no_legacy_gnss_fragment "usb: no legacy GNSS sidecar in default compose"

echo ""
echo "══════════════════════════════════════════"
echo "  Tests: $TESTS_RUN  Passed: $TESTS_PASSED  Failed: $TESTS_FAILED"
echo "══════════════════════════════════════════"

[ "$TESTS_FAILED" -eq 0 ] && exit 0 || exit 1
