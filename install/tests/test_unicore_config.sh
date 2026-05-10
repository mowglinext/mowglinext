#!/usr/bin/env bash
# =============================================================================
# install/lib/unicore_config.sh coverage
# =============================================================================

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=lib/framework.sh
source "$SCRIPT_DIR/lib/framework.sh"

setup_sandbox

# shellcheck source=/dev/null
source "$REPO_ROOT/install/lib/serial_probe.sh"
# shellcheck source=/dev/null
source "$REPO_ROOT/install/lib/unicore_config.sh"

info() { :; }
warn() { :; }
prompt() { REPLY="${2:-}"; }
MSG_CHOICE="Choice"

port="$SANDBOX/fake-tty"
touch "$port"

section "Unicore baud command sequence"

commands_file="$SANDBOX/unicore-commands"
stty_file="$SANDBOX/unicore-stty"
verify_result=0

serial_port_exists() { [ -e "$1" ]; }
unicore_set_stty() {
  printf '%s %s\n' "$1" "$2" >> "$stty_file"
}
unicore_send_command() {
  printf '%s\n' "$2" >> "$commands_file"
}
verify_unicore_baud_921600() {
  return "$verify_result"
}
sleep() { :; }

if configure_unicore_baud_921600 "$port" 460800 COM1; then
  pass "configure_unicore_baud_921600 succeeds when verify succeeds"
else
  fail "configure_unicore_baud_921600 succeeds when verify succeeds"
fi

assert_eq "Unicore command sequence" $'CONFIG COM1 921600\nSAVECONFIG' "$(cat "$commands_file")"
assert_eq "Unicore stty sequence" "$port 460800"$'\n'"$port 921600" "$(cat "$stty_file")"

section "GPS_BAUD changes only on verified success"

GNSS_BACKEND=unicore
GPS_BAUD=460800
UNICORE_COM_PORT=COM1
verify_result=0
: > "$commands_file"
: > "$stty_file"
maybe_upgrade_unicore_baud "$port" "$GPS_BAUD" auto
assert_eq "GPS_BAUD becomes 921600 after verified Unicore upgrade" "921600" "$GPS_BAUD"

GPS_BAUD=460800
verify_result=1
: > "$commands_file"
: > "$stty_file"
maybe_upgrade_unicore_baud "$port" "$GPS_BAUD" auto
assert_eq "GPS_BAUD remains detected baud after failed Unicore upgrade" "460800" "$GPS_BAUD"

section "No-op for other GNSS backends"

GPS_BAUD=460800
GNSS_BACKEND=gps
: > "$commands_file"
maybe_upgrade_unicore_baud "$port" "$GPS_BAUD" auto
assert_eq "generic gps backend does not send Unicore commands" "" "$(cat "$commands_file")"
assert_eq "generic gps backend keeps GPS_BAUD" "460800" "$GPS_BAUD"

GNSS_BACKEND=ublox
: > "$commands_file"
maybe_upgrade_unicore_baud "$port" "$GPS_BAUD" auto
assert_eq "ublox backend does not send Unicore commands" "" "$(cat "$commands_file")"

test_summary
