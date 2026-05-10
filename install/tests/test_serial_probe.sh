#!/usr/bin/env bash
# =============================================================================
# install/lib/serial_probe.sh coverage
# =============================================================================

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=lib/framework.sh
source "$SCRIPT_DIR/lib/framework.sh"

setup_sandbox

# shellcheck source=/dev/null
source "$REPO_ROOT/install/lib/serial_probe.sh"

warn() { :; }
info() { :; }
MSG_CHOICE="Choice"

section "NMEA baud detection"

port="$SANDBOX/fake-tty"
touch "$port"

SERIAL_PROBE_TEST_BAUD=""
SERIAL_PROBE_EMIT_NMEA_AT="460800"

stty() {
  SERIAL_PROBE_TEST_BAUD="${3:-}"
  return 0
}

timeout() {
  if [ "$SERIAL_PROBE_TEST_BAUD" = "$SERIAL_PROBE_EMIT_NMEA_AT" ]; then
    printf '%s\n' '$GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47'
  fi
}

if serial_try_baud_nmea "$port" 460800; then
  pass "serial_try_baud_nmea detects GNGGA"
else
  fail "serial_try_baud_nmea detects GNGGA"
fi

detected="$(serial_probe_baud "$port" gps UBX)"
assert_eq "serial_probe_baud returns detected baud" "460800" "$detected"

section "Detection failure"

SERIAL_PROBE_EMIT_NMEA_AT="none"
if serial_probe_baud "$port" gps UBX >/dev/null 2>&1; then
  fail "serial_probe_baud fails without NMEA frames"
else
  pass "serial_probe_baud fails without NMEA frames"
fi

section "Manual fallback"

prompt() {
  REPLY="${2:-}"
}

prompt_or_probe_baud "$port" gps NMEA 9600 auto
assert_eq "prompt_or_probe_baud falls back to default manual baud" "9600" "$REPLY"

test_summary
