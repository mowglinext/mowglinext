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
out=$( ( parse_args --gnss=garbage ) 2>&1 )
ec=$?
if [ "$ec" -ne 0 ]; then
  pass "--gnss=garbage fails (exit=$ec)"
  case "$out" in
    *"Unknown GNSS backend"*) pass "--gnss error mentions 'Unknown GNSS backend'" ;;
    *)                        fail "--gnss error mentions 'Unknown GNSS backend'" "got: $out" ;;
  esac
else
  fail "--gnss=garbage fails" "exit=0 unexpectedly"
fi

out=$( ( parse_args --gnss-connection=blue ) 2>&1 )
ec=$?
if [ "$ec" -ne 0 ]; then
  pass "--gnss-connection=blue fails (exit=$ec)"
  case "$out" in
    *"Unknown GNSS connection"*) pass "--gnss-connection error mentions 'Unknown GNSS connection'" ;;
    *)                           fail "--gnss-connection error mentions 'Unknown GNSS connection'" "got: $out" ;;
  esac
else
  fail "--gnss-connection=blue fails" "exit=0 unexpectedly"
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

# GNSS receiver-family nmea is now accepted on the Universal path
out=$( ( parse_args --gnss=nmea; printf 'GNSS_RECEIVER_FAMILY=%s\n' "$GNSS_RECEIVER_FAMILY" ) 2>&1 )
ec=$?
if [ "$ec" -eq 0 ]; then
  pass "--gnss=nmea succeeds"
  case "$out" in
    *"GNSS_RECEIVER_FAMILY=nmea"*) pass "--gnss=nmea selects the nmea receiver family" ;;
    *)                            fail "--gnss=nmea selects the nmea receiver family" "got: $out" ;;
  esac
else
  fail "--gnss=nmea succeeds" "exit=$ec"
fi

out=$( ( parse_args --image-tag=feat/universal-gnss-integration; printf 'IMAGE_TAG=%s\n' "$IMAGE_TAG" ) 2>&1 )
ec=$?
if [ "$ec" -eq 0 ]; then
  pass "--image-tag feature branch syntax is accepted"
  case "$out" in
    *"IMAGE_TAG=feat-universal-gnss-integration"*) pass "--image-tag sanitizes branch-style refs" ;;
    *)                                             fail "--image-tag sanitizes branch-style refs" "got: $out" ;;
  esac
else
  fail "--image-tag feature branch syntax is accepted" "exit=$ec"
fi

out=$( ( parse_args --branch=feat/universal-gnss-integration; printf 'REPO_BRANCH=%s\n' "$REPO_BRANCH" ) 2>&1 )
ec=$?
if [ "$ec" -eq 0 ]; then
  pass "--branch feature branch syntax is accepted"
  case "$out" in
    *"REPO_BRANCH=feat/universal-gnss-integration"*) pass "--branch keeps the git branch name intact" ;;
    *)                                               fail "--branch keeps the git branch name intact" "got: $out" ;;
  esac
else
  fail "--branch feature branch syntax is accepted" "exit=$ec"
fi

section "build_compose_stack normalizes invalid GNSS_BACKEND safely"

repo_bad="$SANDBOX/repo_bad_compose"
sandbox_repo "$repo_bad"
compose_state="$(
  harness_init "$repo_bad" >/dev/null 2>&1
  HARDWARE_BACKEND=mowgli
  GNSS_BACKEND=typo
  build_compose_stack >/dev/null 2>&1
  printf 'GNSS_BACKEND_EFFECTIVE=%s\n' "$(effective_gnss_backend)"
  printf 'COMPOSE_FILES=%s\n' "${COMPOSE_FILES[*]}"
)"
assert_contains "invalid GNSS_BACKEND falls back to universal" "GNSS_BACKEND_EFFECTIVE=universal" "$compose_state"
assert_contains "invalid GNSS_BACKEND still selects the gps sidecar fragment" "docker-compose.gps.yml" "$compose_state"

section "Bootstrap script (docs/install.sh) rejects unknown args gracefully"

# docs/install.sh's --help should still exit 0; an unknown arg should
# warn but continue (it's defensive — accepts unknowns rather than
# aborting because curl|bash users can't easily fix typos).
help_out=$(bash "$BOOTSTRAP_SH" --help 2>&1)
help_ec=$?
assert_eq "bootstrap --help exits 0" "0" "$help_ec"
assert_contains "bootstrap --help advertises --gnss-connection" "--gnss-connection" "$help_out"
assert_contains "bootstrap --help advertises --image-tag" "--image-tag" "$help_out"

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
