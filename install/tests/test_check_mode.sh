#!/usr/bin/env bash
# =============================================================================
# A.10 Check mode — `install/mowglinext.sh --check` should target the right
# runtime services for each backend and advertise the right rerun/restart
# commands.
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

assert_runtime_check_case() {
  local label="$1"
  local repo="$2"
  local expected="$3"
  local forbidden="${4:-}"
  local output
  local ec

  output="$(bash "$repo/install/mowglinext.sh" --check 2>&1)"
  ec=$?

  assert_eq "$label: --check exits 0" "0" "$ec"
  assert_contains "$label: runtime diagnostics banner" "Running diagnostics on runtime" "$output"
  assert_contains "$label: expected service marker" "$expected" "$output"
  if [ -n "$forbidden" ]; then
    assert_not_contains "$label: forbidden service marker absent" "$forbidden" "$output"
  fi
}

section "Command helpers use the real installer path"

repo_helpers="$SANDBOX/repo_helpers"
sandbox_repo "$repo_helpers"
harness_init "$repo_helpers"

cmd="$(installer_main_command)"
assert_contains "installer_main_command points to install/mowglinext.sh" "install/mowglinext.sh" "$cmd"

check_cmd="$(rerun_check_command)"
assert_contains "rerun_check_command points to install/mowglinext.sh --check" "install/mowglinext.sh --check" "$check_cmd"

restart_gps="$(compose_restart_services_for_backend | tr '\n' ' ' | sed 's/[[:space:]]*$//')"
assert_eq "restart services for legacy gps" "gps mowgli" "$restart_gps"

GNSS_BACKEND="gps"
GPS_PROTOCOL="NMEA"
restart_nmea="$(compose_restart_services_for_backend | tr '\n' ' ' | sed 's/[[:space:]]*$//')"
assert_eq "restart services for gps+nmea" "gps mowgli" "$restart_nmea"

HARDWARE_BACKEND="mavros"
GNSS_BACKEND="disabled"
restart_mavros="$(compose_restart_services_for_backend mavros | tr '\n' ' ' | sed 's/[[:space:]]*$//')"
assert_eq "restart services for mavros" "mavros ntrip mowgli" "$restart_mavros"

section "--check aligns with runtime backend selection"

repo_mowgli="$SANDBOX/repo_mowgli"
sandbox_repo "$repo_mowgli"
harness_init "$repo_mowgli"
harness_set_preset backend=mowgli gnss=gps gps=ubx-uart lidar=ldlidar-uart tfluna=none
harness_run >/dev/null 2>&1
assert_runtime_check_case "mowgli gps" "$repo_mowgli" "gps (mowgli-gps)" "mavros (mowgli-mavros)"

repo_nmea="$SANDBOX/repo_nmea"
sandbox_repo "$repo_nmea"
harness_init "$repo_nmea"
harness_set_preset backend=mowgli gnss=gps gps=nmea-uart lidar=ldlidar-uart tfluna=none
harness_run >/dev/null 2>&1
assert_runtime_check_case "mowgli gps+nmea" "$repo_nmea" "gps (mowgli-gps)" "mavros (mowgli-mavros)"

repo_mavros="$SANDBOX/repo_mavros"
sandbox_repo "$repo_mavros"
harness_init "$repo_mavros"
harness_set_preset backend=mavros gps=ubx-uart lidar=ldlidar-uart tfluna=none
harness_run >/dev/null 2>&1
output_mavros="$(bash "$repo_mavros/install/mowglinext.sh" --check 2>&1)"
ec=$?
assert_eq "mavros: --check exits 0" "0" "$ec"
assert_contains "mavros: expected mavros service" "mavros (mowgli-mavros)" "$output_mavros"
assert_contains "mavros: firmware check skipped" "MAVROS backend: skipping direct Mowgli firmware check" "$output_mavros"
assert_not_contains "mavros: no direct gps container expected" "gps (mowgli-gps)" "$output_mavros"

section "No stale check/restart paths remain"

assert_exit_nonzero "no stale ./install.sh --check path" rg -n '\./install\.sh --check' "$REPO_ROOT/install/lib" "$REPO_ROOT/docs"
assert_exit_nonzero "no hardcoded restart gps mowgli string" rg -n 'restart gps mowgli' "$REPO_ROOT/install/lib" "$REPO_ROOT/docs"

test_summary
