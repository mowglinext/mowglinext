#!/usr/bin/env bash
# =============================================================================
# issue #631 — enable_all_platform_uarts() must only claim the UART overlays
# the CONFIGURED hardware actually uses, derived from the exact device path
# each peripheral was assigned (not a fixed per-peripheral assumption), so it
# stops silently stealing a GPIO pin a custom peripheral (e.g. an LED ring)
# may be wired to. This tests the pure selection logic in install/lib/uart.sh
# directly — no sandboxed installer run needed, and no Raspberry Pi platform
# mocking (enable_all_platform_uarts itself is a no-op on any non-Pi test
# runner via platform_supports_pi_uart_overlays, so a full harness_run cannot
# exercise this logic at all — see install/lib/platform.sh).
# =============================================================================

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=lib/framework.sh
source "$SCRIPT_DIR/lib/framework.sh"
# shellcheck source=../lib/uart.sh
source "$SCRIPT_DIR/../lib/uart.sh"

# Each case resets every variable required_uart_overlays() reads, so cases
# cannot leak state into each other.
reset_hardware_env() {
  unset GNSS_SERIAL_DEVICE LIDAR_ENABLED LIDAR_UART_DEVICE \
        TFLUNA_FRONT_ENABLED TFLUNA_FRONT_UART_DEVICE \
        TFLUNA_EDGE_ENABLED TFLUNA_EDGE_UART_DEVICE
}

section "uart_overlay_for_device() maps ttyAMA1-5, and nothing else"

assert_eq "ttyAMA1 -> overlay 1" "1" "$(uart_overlay_for_device /dev/ttyAMA1)"
assert_eq "ttyAMA5 -> overlay 5" "5" "$(uart_overlay_for_device /dev/ttyAMA5)"
assert_exit_nonzero "ttyAMA0 (primary UART) needs no overlay" uart_overlay_for_device /dev/ttyAMA0
assert_exit_nonzero "ttyS0 (mini-uart) needs no overlay" uart_overlay_for_device /dev/ttyS0
assert_exit_nonzero "a USB by-id path needs no overlay" uart_overlay_for_device "/dev/serial/by-id/usb-u-blox_ZED-F9P-if00"
assert_exit_nonzero "empty path needs no overlay" uart_overlay_for_device ""

section "required_uart_overlays() with nothing configured yet"

reset_hardware_env
assert_eq "nothing configured -> no overlays needed" "" "$(required_uart_overlays)"

section "required_uart_overlays() only claims what's actually enabled+on a header pin"

reset_hardware_env
GNSS_SERIAL_DEVICE="/dev/ttyAMA4"
LIDAR_ENABLED="true"
LIDAR_UART_DEVICE="/dev/ttyAMA5"
TFLUNA_FRONT_ENABLED="false"
TFLUNA_FRONT_UART_DEVICE="/dev/ttyAMA3"   # set (range.sh's own `:=` default) but NOT enabled
TFLUNA_EDGE_ENABLED="false"
TFLUNA_EDGE_UART_DEVICE="/dev/ttyAMA2"    # same — must not leak into the result
assert_eq "typical GNSS+LiDAR install needs exactly uart4+uart5" \
  "4
5" "$(required_uart_overlays | sort -un)"

section "LiDAR over USB claims nothing, even if LIDAR_ENABLED=true"

reset_hardware_env
GNSS_SERIAL_DEVICE="/dev/ttyAMA4"
LIDAR_ENABLED="true"
LIDAR_UART_DEVICE=""   # lidar.sh clears this when LIDAR_CONNECTION=usb
assert_eq "USB LiDAR does not claim a uart overlay" "4" "$(required_uart_overlays)"

section "LiDAR disabled entirely claims nothing, even if a stale device path lingers"

reset_hardware_env
GNSS_SERIAL_DEVICE="/dev/ttyAMA4"
LIDAR_ENABLED="false"
LIDAR_UART_DEVICE="/dev/ttyAMA5"
assert_eq "disabled LiDAR does not claim a uart overlay" "4" "$(required_uart_overlays)"

section "TF-Luna front+edge enabled claims uart2+uart3 in addition to GNSS+LiDAR"

reset_hardware_env
GNSS_SERIAL_DEVICE="/dev/ttyAMA4"
LIDAR_ENABLED="true"
LIDAR_UART_DEVICE="/dev/ttyAMA5"
TFLUNA_FRONT_ENABLED="true"
TFLUNA_FRONT_UART_DEVICE="/dev/ttyAMA3"
TFLUNA_EDGE_ENABLED="true"
TFLUNA_EDGE_UART_DEVICE="/dev/ttyAMA2"
assert_eq "all four peripherals enabled -> uart2,3,4,5, never uart1" \
  "2
3
4
5" "$(required_uart_overlays | sort -un)"

section "A non-default wiring choice (e.g. LiDAR on ttyAMA2 on a Pi 5) is honoured, not assumed"

reset_hardware_env
LIDAR_ENABLED="true"
LIDAR_UART_DEVICE="/dev/ttyAMA2"   # not the common-default ttyAMA5
assert_eq "overlay follows the ACTUAL picked port, not a fixed per-peripheral table" \
  "2" "$(required_uart_overlays)"

test_summary
