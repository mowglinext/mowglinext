#!/usr/bin/env bash
# =============================================================================
# Static coverage for docs/install.sh public flags + preset defaults
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_SH="$SCRIPT_DIR/install.sh"

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

assert_contains() {
  local label="$1" needle="$2" haystack="$3"
  if grep -qF -- "$needle" <<<"$haystack"; then
    pass "$label"
  else
    fail "$label" "expected to contain '$needle'"
  fi
}

assert_not_contains() {
  local label="$1" needle="$2" haystack="$3"
  if ! grep -qF -- "$needle" <<<"$haystack"; then
    pass "$label"
  else
    fail "$label" "expected NOT to contain '$needle'"
  fi
}

echo ""
echo "── Bootstrap installer tests ──"

help_output="$(bash "$INSTALL_SH" --help 2>&1 || true)"
script_text="$(cat "$INSTALL_SH")"

assert_contains "help shows backend flag" "--backend=TYPE" "$help_output"
assert_contains "help shows gnss connection flag" "--gnss-connection" "$help_output"
assert_contains "help explains direct serial link selection" "Universal GNSS serial link: usb or uart" "$help_output"
assert_not_contains "help no longer advertises legacy gnss backend flag" "--gnss=BACKEND" "$help_output"
assert_not_contains "help no longer advertises legacy gps preset flag" "--gps=PRESET" "$help_output"

assert_contains "composer example uses gnss connection" "--gnss-connection=uart" "$script_text"
assert_contains "default direct-gnss preset forces universal stack" "GNSS_STACK=universal" "$script_text"
assert_contains "default direct-gnss preset forces universal status source" "GNSS_STATUS_SOURCE=universal" "$script_text"
assert_contains "default direct-gnss preset defaults receiver family to auto" "GNSS_RECEIVER_FAMILY=auto" "$script_text"
assert_contains "default direct-gnss preset keeps serial transport" "GNSS_TRANSPORT=serial" "$script_text"
assert_contains "connection preset writes GPS_CONNECTION compatibility key" 'echo "GPS_CONNECTION=${GNSS_CONNECTION_FLAG}" >> "$PRESET_FILE"' "$script_text"

echo ""
echo "══════════════════════════════════════════"
echo "  Tests: $TESTS_RUN  Passed: $TESTS_PASSED  Failed: $TESTS_FAILED"
echo "══════════════════════════════════════════"

[ "$TESTS_FAILED" -eq 0 ] && exit 0 || exit 1
