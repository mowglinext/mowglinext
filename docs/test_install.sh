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
legacy_connection_key="GPS_""CONNECTION="
legacy_protocol_key="GPS_""PROTOCOL="

assert_contains "help shows backend flag" "--backend=TYPE" "$help_output"
assert_contains "help shows gnss receiver family flag" "--gnss-receiver-family" "$help_output"
assert_contains "help shows gnss connection flag" "--gnss-connection" "$help_output"
assert_contains "help explains direct receiver family selection" "Universal GNSS receiver family: auto, ublox, unicore, nmea" "$help_output"
assert_contains "help explains direct serial link selection" "Universal GNSS serial link: usb or uart" "$help_output"
assert_not_contains "help no longer advertises legacy gnss backend flag" "--gnss=BACKEND" "$help_output"
assert_not_contains "help no longer advertises legacy gps preset flag" "--gps=PRESET" "$help_output"

assert_contains "composer example uses gnss receiver family" "--gnss-receiver-family=auto" "$script_text"
assert_contains "composer example uses gnss connection" "--gnss-connection=uart" "$script_text"
assert_contains "bootstrap forwards gnss connection to installer" 'INSTALLER_ARGS+=("--gnss-connection=$GNSS_CONNECTION_FLAG")' "$script_text"
assert_contains "bootstrap forwards gnss receiver selection to installer" 'INSTALLER_ARGS+=("--gnss=$GNSS_FLAG")' "$script_text"
assert_contains "bootstrap forwards gnss receiver family to installer" 'INSTALLER_ARGS+=("--gnss-receiver-family=$GNSS_RECEIVER_FAMILY_FLAG")' "$script_text"
assert_contains "bootstrap keeps branch aligned with installer" 'INSTALLER_ARGS+=("--branch=$REPO_BRANCH")' "$script_text"
assert_contains "deprecated gps flag is normalized instead of written to preset" "Deprecated bootstrap flag" "$script_text"
assert_not_contains "bootstrap no longer writes legacy GNSS compatibility keys" "$legacy_connection_key" "$script_text"
assert_not_contains "bootstrap no longer writes legacy GNSS protocol keys" "$legacy_protocol_key" "$script_text"
assert_not_contains "bootstrap no longer writes legacy stack presets" "GNSS_STACK=legacy" "$script_text"
assert_not_contains "bootstrap no longer writes receiver-specific backends" "GNSS_BACKEND=ublox" "$script_text"
assert_not_contains "bootstrap no longer writes legacy gps backend alias" "GNSS_BACKEND=gps" "$script_text"

echo ""
echo "══════════════════════════════════════════"
echo "  Tests: $TESTS_RUN  Passed: $TESTS_PASSED  Failed: $TESTS_FAILED"
echo "══════════════════════════════════════════"

[ "$TESTS_FAILED" -eq 0 ] && exit 0 || exit 1
