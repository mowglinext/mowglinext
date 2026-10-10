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
# shellcheck source=../lib/common.sh
source "$SCRIPT_DIR/../lib/common.sh"
# shellcheck source=../lib/config.sh
source "$SCRIPT_DIR/../lib/config.sh"   # default_gnss_uart_device
# shellcheck source=../lib/uart.sh
source "$SCRIPT_DIR/../lib/uart.sh"

# Each case resets every variable required_uart_overlays() reads, so cases
# cannot leak state into each other.
reset_hardware_env() {
  unset GNSS_SERIAL_DEVICE LIDAR_ENABLED LIDAR_UART_DEVICE HARDWARE_BACKEND \
    OPENMOWER_LL_PORT OPENMOWER_XESC_LEFT_PORT OPENMOWER_XESC_RIGHT_PORT OPENMOWER_XESC_MOW_PORT
}

# uart.sh reports through common.sh's helpers; only their output matters here.
error() { printf 'ERROR: %s\n' "$*"; }
info() { :; }

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

section "A non-default wiring choice (e.g. LiDAR on ttyAMA2 on a Pi 5) is honoured, not assumed"

reset_hardware_env
LIDAR_ENABLED="true"
LIDAR_UART_DEVICE="/dev/ttyAMA2"   # not the common-default ttyAMA5
assert_eq "overlay follows the ACTUAL picked port, not a fixed per-peripheral table" \
  "2" "$(required_uart_overlays)"

section "OpenMower v1: the three xESC ports claim their overlays, the LowLevel port none"

reset_hardware_env
HARDWARE_BACKEND="openmower"
GNSS_SERIAL_DEVICE="$(default_gnss_uart_device)"
assert_eq "OpenMower GNSS defaults to ttyAMA2 (open_mower_ros, kernel >= 6.1.28)" \
  "/dev/ttyAMA2" "$GNSS_SERIAL_DEVICE"
assert_eq "OpenMower install needs uart2 (GPS) + uart3/4/5 (xESC)" \
  "2
3
4
5" "$(required_uart_overlays | sort -un)"

reset_hardware_env
HARDWARE_BACKEND="openmower"
OPENMOWER_XESC_LEFT_PORT="/dev/ttyAMA1"
assert_eq "a re-wired xESC claims the overlay of its actual port" \
  "1
3
4" "$(required_uart_overlays | sort -un)"

reset_hardware_env
assert_eq "another backend keeps the usual GNSS default" "/dev/ttyAMA4" "$(default_gnss_uart_device)"
GNSS_SERIAL_DEVICE="/dev/ttyAMA4"
assert_eq "another backend claims no xESC overlay" "4" "$(required_uart_overlays)"

section "A peripheral cannot take a port an OpenMower controller is wired to"

reset_hardware_env
HARDWARE_BACKEND="openmower"
assert_eq "ttyAMA4 belongs to the mow xESC" "OpenMower-xESC-mow" "$(openmower_uart_owner /dev/ttyAMA4)"
assert_eq "ttyAMA0 belongs to the LowLevel board" "OpenMower-LowLevel" "$(openmower_uart_owner /dev/ttyAMA0)"
assert_exit_nonzero "ttyAMA2 is free for the GPS" openmower_uart_owner /dev/ttyAMA2
assert_exit_nonzero "GNSS on the mow xESC port is refused" uart_port_is_free /dev/ttyAMA4 GNSS
assert_contains "the refusal names the controller" "OpenMower-xESC-mow" "$(uart_port_is_free /dev/ttyAMA4 GNSS)"
assert_exit_zero "a USB GNSS is never refused" uart_port_is_free /dev/serial/by-id/usb-u-blox-if00 GNSS
reset_hardware_env
assert_exit_zero "without OpenMower every port is free" uart_port_is_free /dev/ttyAMA4 GNSS

section "configured_uart_devices() lists every UART the stack will open"

reset_hardware_env
HARDWARE_BACKEND="openmower"
GNSS_SERIAL_DEVICE="/dev/ttyAMA2"
assert_eq "GNSS + LowLevel + three xESC" \
  "/dev/ttyAMA0
/dev/ttyAMA2
/dev/ttyAMA3
/dev/ttyAMA4
/dev/ttyAMA5" "$(configured_uart_devices | sort -u)"

section "strip_serial_console_args() frees the primary UART, keeps everything else"

assert_eq "Ubuntu cmdline: serial0 console removed, tty1 kept" \
  "dwc_otg.lpm_enable=0 console=tty1 root=LABEL=writable rootwait fixrtc" \
  "$(printf 'console=serial0,115200 dwc_otg.lpm_enable=0 console=tty1 root=LABEL=writable rootwait fixrtc\n' | strip_serial_console_args)"
assert_eq "ttyAMA0 with a mode suffix and ttyS0 removed" \
  "console=tty1 root=/dev/mmcblk0p2" \
  "$(printf 'console=ttyAMA0,115200n8 console=ttyS0 console=tty1 root=/dev/mmcblk0p2\n' | strip_serial_console_args)"
assert_eq "no serial console -> line unchanged" \
  "console=tty1 root=/dev/mmcblk0p2 quiet" \
  "$(printf 'console=tty1 root=/dev/mmcblk0p2 quiet\n' | strip_serial_console_args)"
assert_eq "a USB serial console is left alone" \
  "console=ttyUSB0,115200 root=/dev/sda2" \
  "$(printf 'console=ttyUSB0,115200 root=/dev/sda2\n' | strip_serial_console_args)"

test_summary
