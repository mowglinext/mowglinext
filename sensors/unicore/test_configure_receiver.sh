#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/configure_receiver.sh"

failures=0

assert_eq() {
  local label="${1:?assert_eq: missing label}"
  local expected="${2-}"
  local actual="${3-}"

  if [ "$expected" != "$actual" ]; then
    echo "FAIL: $label"
    echo "  expected: $expected"
    echo "  actual:   $actual"
    failures=$((failures + 1))
  else
    echo "PASS: $label"
  fi
}

assert_contains() {
  local label="${1:?assert_contains: missing label}"
  local needle="${2:?assert_contains: missing needle}"
  local haystack="${3-}"

  if [[ "$haystack" != *"$needle"* ]]; then
    echo "FAIL: $label"
    echo "  missing: $needle"
    echo "  actual:  $haystack"
    failures=$((failures + 1))
  else
    echo "PASS: $label"
  fi
}

reset_mocks() {
  COMMAND_LOG=""
  STTY_LOG=""
  CURRENT_BAUD=""
  MOCK_MODEL="unknown"
  MOCK_RESPONDING_BAUDS=""
}

require_serial_port() { :; }
wait_for_serial_port() { :; }
open_serial() { :; }
close_serial() { :; }
drain_serial() { :; }
sleep() { :; }

serial_set_baud() {
  local _port="${1:?serial_set_baud: missing port}"
  local baud="${2:?serial_set_baud: missing baud}"

  CURRENT_BAUD="$baud"
  STTY_LOG+="${baud}"$'\n'
}

query_receiver_identification() {
  case " $MOCK_RESPONDING_BAUDS " in
    *" $CURRENT_BAUD "*) ;;
    *) return 0 ;;
  esac

  case "$MOCK_MODEL" in
    UM980) printf '%s\n' '#VERSIONA,"UM980","R4.10Build15434"' ;;
    UM982) printf '%s\n' '#VERSIONA,"UM982","R4.10Build15434"' ;;
    *) printf '%s\n' '#VERSIONA,"UNKNOWN","R4.10Build15434"' ;;
  esac
}

send_serial_command() {
  local command="${1:?send_serial_command: missing command}"

  COMMAND_LOG+="${command}"$'\n'
  if [[ "$command" =~ ^CONFIG[[:space:]]+COM1[[:space:]]+([0-9]+)$ ]]; then
    CURRENT_BAUD="${BASH_REMATCH[1]}"
  fi
}

run_scenario() {
  local label="${1:?run_scenario: missing label}"
  local port="/dev/mock"
  local preferred_baud="${2:?run_scenario: missing preferred baud}"
  local model="${3:?run_scenario: missing model}"
  local responding_bauds="${4:?run_scenario: missing responding bauds}"
  local expected_rc="${5:?run_scenario: missing expected rc}"
  local rc=0

  echo ""
  echo "Scenario: $label"
  reset_mocks
  MOCK_MODEL="$model"
  MOCK_RESPONDING_BAUDS="$responding_bauds"

  if main "$port" "$preferred_baud"; then
    rc=0
  else
    rc=$?
  fi

  assert_eq "$label rc" "$expected_rc" "$rc"
}

run_scenario "UM980 from 460800" "460800" "UM980" "460800 921600" "0"
assert_contains "UM980 switches to 921600" "CONFIG COM1 921600" "$COMMAND_LOG"
assert_contains "UM980 uses SIGNALGROUP 2" "CONFIG SIGNALGROUP 2" "$COMMAND_LOG"
assert_contains "UM980 saves config" "SAVECONFIG" "$COMMAND_LOG"

run_scenario "UM982 already at 921600" "921600" "UM982" "921600" "0"
assert_contains "UM982 uses SIGNALGROUP 3 6" "CONFIG SIGNALGROUP 3 6" "$COMMAND_LOG"
assert_eq "UM982 does not resend baud config" "" "$(printf '%s' "$COMMAND_LOG" | grep -F "CONFIG COM1 921600" || true)"

run_scenario "Unknown model is rejected" "115200" "unknown" "115200" "1"
assert_eq "Unknown model skips saveconfig" "" "$(printf '%s' "$COMMAND_LOG" | grep -F "SAVECONFIG" || true)"
assert_eq "Unknown model skips signalgroup" "" "$(printf '%s' "$COMMAND_LOG" | grep -F "CONFIG SIGNALGROUP" || true)"

if [ "$failures" -ne 0 ]; then
  echo ""
  echo "$failures test(s) failed."
  exit 1
fi

echo ""
echo "All tests passed."
