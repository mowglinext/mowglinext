#!/usr/bin/env bash
# =============================================================================
# Static coverage for docs/index.html web composer wiring
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INDEX_HTML="$SCRIPT_DIR/index.html"

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
echo "── Web composer tests ──"

html="$(cat "$INDEX_HTML")"

assert_contains "backend group exists" 'data-group="backend"' "$html"
assert_contains "backend option mowgli exists" 'data-value="mowgli"' "$html"
assert_contains "backend option mavros exists" 'data-value="mavros"' "$html"
assert_contains "backend state defaults to mowgli" "var state = { backend: 'mowgli'" "$html"
assert_contains "backend flag is generated" "parts.push('--backend=' + state.backend);" "$html"
assert_contains "mavros skips gnss/gps flags" "if (state.backend !== 'mavros') {" "$html"
assert_contains "gnss flag generation still exists" "parts.push('--gnss=' + state.gnss);" "$html"
assert_contains "unicore and ublox skip gps flag generation" "if (state.gnss !== 'unicore' && state.gnss !== 'ublox') {" "$html"
assert_contains "gps flag generation still exists" "parts.push('--gps=' + state.gps);" "$html"
assert_contains "gnss group has stable id" 'id="gnss-group"' "$html"
assert_contains "gps group has stable id" 'id="gps-group"' "$html"
assert_contains "gps hint has stable id" 'id="gps-group-hint"' "$html"
assert_contains "auto gnss option exists" 'data-value="auto"' "$html"
assert_contains "legacy gnss option exists" 'data-value="legacy"' "$html"
assert_contains "unicore installer handoff is explicit" "Universal Unicore keeps its real serial device selection inside the installer." "$html"
assert_contains "ublox hint documents installer by-id selection" "Universal u-blox asks for the receiver USB by-id device inside the installer" "$html"
assert_contains "legacy fallback hint exists" "Legacy fallback keeps the old direct GNSS container path." "$html"
assert_contains "mavros disables gnss group" "setGroupDisabled('gnss-group', mavrosSelected);" "$html"
assert_contains "unicore and ublox disable gps group" "setGroupDisabled('gps-group', mavrosSelected || unicoreSelected || ubloxSelected);" "$html"
assert_contains "tfluna group is disabled in ui" 'id="tfluna-group" aria-disabled="true"' "$html"
assert_contains "tfluna unavailability is explicit" "Temporarily disabled on this branch" "$html"
assert_not_contains "web composer never emits gnss=nmea" "--gnss=nmea" "$html"

assert_contains "channel group exists" 'data-group="channel"' "$html"
assert_contains "channel option main exists" 'data-value="main"' "$html"
assert_contains "channel option dev exists" 'data-value="dev"' "$html"
assert_contains "channel state defaults to main" "channel: 'main'" "$html"
assert_contains "channel flag emitted only when non-default" "if (state.channel !== 'main') {" "$html"
assert_contains "channel flag uses --branch" "parts.push('--branch=' + state.channel);" "$html"

echo ""
echo "══════════════════════════════════════════"
echo "  Tests: $TESTS_RUN  Passed: $TESTS_PASSED  Failed: $TESTS_FAILED"
echo "══════════════════════════════════════════"

[ "$TESTS_FAILED" -eq 0 ] && exit 0 || exit 1
