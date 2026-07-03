#!/usr/bin/env bash
# =============================================================================
# install/lib/udev.sh coverage
# =============================================================================

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=lib/framework.sh
source "$SCRIPT_DIR/lib/framework.sh"
# shellcheck source=lib/mocks.sh
source "$SCRIPT_DIR/lib/mocks.sh"

setup_sandbox
install_all_mocks

section "install_udev_rules works with set -u"

repo="$SANDBOX/repo"
sandbox_repo "$repo"

# shellcheck source=/dev/null
source "$repo/install/lib/common.sh"
# shellcheck source=/dev/null
source "$repo/install/lib/i18n.sh"
# shellcheck source=/dev/null
source "$repo/install/lib/config.sh"
# shellcheck source=/dev/null
source "$repo/install/lib/platform.sh"
# shellcheck source=/dev/null
source "$repo/install/lib/docker.sh"
# shellcheck source=/dev/null
source "$repo/install/lib/backend_choice.sh"
# shellcheck source=/dev/null
source "$repo/install/lib/range.sh"
# shellcheck source=/dev/null
source "$repo/install/lib/env.sh"
# shellcheck source=/dev/null
source "$repo/install/lib/udev.sh"

load_locale
reapply_test_assertions

export UDEV_RULES_FILE="$SANDBOX/99-mowgli.rules"
export SUDO=""

export HARDWARE_BACKEND="mowgli"
export GNSS_BACKEND="universal"
export GNSS_STATUS_SOURCE="universal"
export GNSS_STACK="universal"
export GNSS_RECEIVER_FAMILY="unicore"
export GNSS_TRANSPORT="serial"
export GNSS_CONNECTION_HINT="uart"
unset GNSS_SERIAL_DEVICE GNSS_SERIAL_BAUD 2>/dev/null || true
export LIDAR_ENABLED="false"
export TFLUNA_FRONT_ENABLED="false"
export TFLUNA_EDGE_ENABLED="false"

mkdir -p "$SANDBOX/dev"
mkdir -p "$SANDBOX/serial-by-id"
touch "$SANDBOX/dev/ttyACM0"
ln -s "$SANDBOX/dev/ttyACM0" "$SANDBOX/serial-by-id/usb-Mowgli_STM32-if00"
export SERIAL_BY_ID_DIR="$SANDBOX/serial-by-id"

# Reproduce the Universal GNSS Unicore path:
# --backend=mowgli --gnss=unicore --gnss-connection=uart --lidar=ldlidar-uart --tfluna=none
setup_env >/dev/null 2>&1
touch "$(gnss_serial_device_from_state)"

set -u
if output="$(install_udev_rules 2>&1)"; then
  pass "install_udev_rules handles unicore Universal GNSS preset under set -u"
else
  fail "install_udev_rules handles unicore Universal GNSS preset under set -u" "$output"
fi
set +u

assert_file_exists "udev rules file written" "$UDEV_RULES_FILE"
rules="$(cat "$UDEV_RULES_FILE")"
assert_contains "Mowgli by-id rule generated" 'SYMLINK+="mowgli"' "$rules"
assert_contains "Mowgli by-id rule uses resolved kernel" 'KERNEL=="ttyACM0"' "$rules"
assert_contains "gps uart rule generated" 'SYMLINK+="gps"' "$rules"
assert_eq "GNSS backend remains universal" "universal" "$GNSS_BACKEND"
assert_eq "GNSS receiver family remains unicore" "unicore" "$GNSS_RECEIVER_FAMILY"

test_summary
