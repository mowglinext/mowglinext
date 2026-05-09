#!/usr/bin/env bash
# =============================================================================
# A.8 Negative paths — bad preset names, malformed flags, missing tools
#
# Drives mowglinext.sh and lib/*.sh through known-bad inputs and asserts
# they fail with non-zero exit codes (and ideally a recognisable error
# string in stderr).
# =============================================================================

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=lib/framework.sh
source "$SCRIPT_DIR/lib/framework.sh"
# shellcheck source=lib/mocks.sh
source "$SCRIPT_DIR/lib/mocks.sh"
# shellcheck source=lib/harness.sh
source "$SCRIPT_DIR/lib/harness.sh"

setup_sandbox
install_all_mocks

SANDBOX_REPO="$SANDBOX/repo"
sandbox_repo "$SANDBOX_REPO"

section "config.sh::parse_args rejects malformed flags"

# parse_args lives in lib/config.sh; source common+config in isolation.
# shellcheck source=/dev/null
source "$SANDBOX_REPO/install/lib/common.sh"
# shellcheck source=/dev/null
source "$SANDBOX_REPO/install/lib/i18n.sh"
load_locale
# shellcheck source=/dev/null
source "$SANDBOX_REPO/install/lib/config.sh"

# parse_args calls `exit 1` on bad input; run it in a subshell.
out=$( ( parse_args --gps=garbage-uart ) 2>&1 )
ec=$?
if [ "$ec" -ne 0 ]; then
  pass "--gps=garbage-uart fails (exit=$ec)"
  case "$out" in
    *"Unknown GPS protocol"*) pass "--gps error mentions 'Unknown GPS protocol'" ;;
    *)                        fail "--gps error mentions 'Unknown GPS protocol'" "got: $out" ;;
  esac
else
  fail "--gps=garbage-uart fails" "exit=0 unexpectedly"
fi

# Bad GPS connection
out=$( ( parse_args --gps=ubx-blue ) 2>&1 )
ec=$?
if [ "$ec" -ne 0 ]; then
  pass "--gps=ubx-blue fails (exit=$ec)"
else
  fail "--gps=ubx-blue fails" "exit=0 unexpectedly"
fi

# Bad LiDAR preset
out=$( ( parse_args --lidar=mythical-laser ) 2>&1 )
ec=$?
if [ "$ec" -ne 0 ]; then
  pass "--lidar=mythical-laser fails (exit=$ec)"
  case "$out" in
    *"Unknown lidar spec"*) pass "--lidar error mentions 'Unknown lidar spec'" ;;
    *)                      fail "--lidar error mentions 'Unknown lidar spec'" "got: $out" ;;
  esac
else
  fail "--lidar=mythical-laser fails" "exit=0 unexpectedly"
fi

# Bad TF-Luna preset
out=$( ( parse_args --tfluna=midnight ) 2>&1 )
ec=$?
if [ "$ec" -ne 0 ]; then
  pass "--tfluna=midnight fails (exit=$ec)"
else
  fail "--tfluna=midnight fails" "exit=0 unexpectedly"
fi

# Legacy pseudo-backend nmea must be rejected
out=$( ( parse_args --gnss=nmea ) 2>&1 )
ec=$?
if [ "$ec" -ne 0 ]; then
  pass "--gnss=nmea fails (exit=$ec)"
  case "$out" in
    *"Unknown GNSS backend"*) pass "--gnss=nmea error mentions 'Unknown GNSS backend'" ;;
    *)                        fail "--gnss=nmea error mentions 'Unknown GNSS backend'" "got: $out" ;;
  esac
else
  fail "--gnss=nmea fails" "exit=0 unexpectedly"
fi

section "configure_gps rejects invalid GNSS_BACKEND preset"

# Standalone test in a fresh subshell so we don't pollute env
( harness_init "$SANDBOX_REPO" >/dev/null 2>&1
  PRESET_LOADED=true
  GNSS_BACKEND=quantumgps
  GPS_CONNECTION=usb GPS_PROTOCOL=UBX GPS_BAUD=460800 GPS_DEBUG_ENABLED=false
  if configure_gps >/dev/null 2>&1; then
    exit 0
  else
    exit 1
  fi
)
ec=$?
if [ "$ec" -ne 0 ]; then
  pass "configure_gps rejects GNSS_BACKEND=quantumgps"
else
  fail "configure_gps rejects GNSS_BACKEND=quantumgps" "exit=0 unexpectedly"
fi

section "build_compose_stack rejects invalid GNSS_BACKEND"

repo_bad="$SANDBOX/repo_bad_compose"
sandbox_repo "$repo_bad"
( harness_init "$repo_bad" >/dev/null 2>&1
  HARDWARE_BACKEND=mowgli
  GNSS_BACKEND=typo
  if build_compose_stack >/dev/null 2>&1; then exit 0; else exit 1; fi
)
ec=$?
if [ "$ec" -ne 0 ]; then
  pass "build_compose_stack rejects GNSS_BACKEND=typo"
else
  fail "build_compose_stack rejects GNSS_BACKEND=typo" "exit=0 unexpectedly"
fi

section "Bootstrap script (docs/install.sh) rejects unknown args gracefully"

# docs/install.sh's --help should still exit 0; an unknown arg should
# warn but continue (it's defensive — accepts unknowns rather than
# aborting because curl|bash users can't easily fix typos).
help_out=$(bash "$BOOTSTRAP_SH" --help 2>&1)
help_ec=$?
assert_eq "bootstrap --help exits 0" "0" "$help_ec"
assert_contains "bootstrap --help advertises --gps" "--gps=PRESET" "$help_out"

# Syntax check the bootstrap and main installer
assert_exit_zero "bash -n on docs/install.sh"        bash -n "$BOOTSTRAP_SH"
assert_exit_zero "bash -n on install/mowglinext.sh"  bash -n "$INSTALLER_MAIN"

section "Missing required tooling"

# Confirm we can detect docker missing (the installer's tools.sh +
# docker.sh rely on this primitive).
empty_bin="$SANDBOX/empty_bin"
mkdir -p "$empty_bin"
( PATH="$empty_bin"
  command -v docker >/dev/null 2>&1
)
ec=$?
if [ "$ec" -ne 0 ]; then
  pass "absent docker is detected"
else
  fail "absent docker is detected" "PATH=$empty_bin still finds docker"
fi

test_summary
