#!/usr/bin/env bash
# =============================================================================
# A.3 Hardware preset matrix — Mowgli STM32 vs Pixhawk MAVROS
#
# The installer's select_hardware_backend offers exactly two backends:
#   1. Mowgli STM32 board (HARDWARE_BACKEND=mowgli)
#   2. Pixhawk via MAVROS    (HARDWARE_BACKEND=mavros)
# The "Yardforce500*" labels mentioned in product copy are mower_model
# strings inside docker/config/mowgli/mowgli_robot.yaml, not separate
# install presets.  This test covers both backends end-to-end.
# =============================================================================

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=lib/framework.sh
source "$SCRIPT_DIR/lib/framework.sh"
# shellcheck source=lib/mocks.sh
source "$SCRIPT_DIR/lib/mocks.sh"
# shellcheck source=lib/harness.sh
source "$SCRIPT_DIR/lib/harness.sh"

env_value() {
  local repo="$1" key="$2"
  grep -E "^${key}=" "$repo/docker/.env" | head -1 | cut -d= -f2-
}

selected_fragments_in_current_run() {
  printf '%s\n' "${COMPOSE_FILES[@]}" | xargs -n1 basename | sort
}

setup_sandbox
install_all_mocks

# ── Mowgli STM32 backend ──────────────────────────────────────────────────
section "HARDWARE_BACKEND=mowgli (Mowgli STM32 board)"

mowgli_repo="$SANDBOX/repo_mowgli"
sandbox_repo "$mowgli_repo"
harness_init "$mowgli_repo"
harness_set_preset backend=mowgli gps=ubx-uart lidar=ldlidar-uart tfluna=none
if harness_run; then
  pass "mowgli backend: harness_run succeeds"
else
  fail "mowgli backend: harness_run succeeds"
fi
assert_eq "mowgli backend: HARDWARE_BACKEND=mowgli" "mowgli" "$(env_value "$mowgli_repo" HARDWARE_BACKEND)"
assert_eq "mowgli backend: GNSS_BACKEND=gps"       "gps"    "$(env_value "$mowgli_repo" GNSS_BACKEND)"
assert_eq "mowgli backend: MAVROS_ENABLED=false"   "false"  "$(env_value "$mowgli_repo" MAVROS_ENABLED)"

mowgli_fragments=$(selected_fragments_in_current_run)
for required in docker-compose.base.yml docker-compose.gui.yml docker-compose.gps.yml docker-compose.lidar-ldlidar.yml; do
  case "$mowgli_fragments" in
    *"$required"*) pass "mowgli backend: fragment $required present" ;;
    *)             fail "mowgli backend: fragment $required present" ;;
  esac
done
case "$mowgli_fragments" in
  *docker-compose.mavros.yml*) fail "mowgli backend: NO mavros fragment" "docker-compose.mavros.yml leaked into compose selection" ;;
  *)                           pass "mowgli backend: NO mavros fragment" ;;
esac
case "$mowgli_fragments" in
  *docker-compose.nmea.yaml*) fail "mowgli backend: NO nmea fragment by default" "unexpected nmea fragment selected" ;;
  *)                          pass "mowgli backend: NO nmea fragment by default" ;;
esac

# ── Pixhawk MAVROS backend ────────────────────────────────────────────────
section "HARDWARE_BACKEND=mavros (Pixhawk via MAVROS)"

mavros_repo="$SANDBOX/repo_mavros"
sandbox_repo "$mavros_repo"
harness_init "$mavros_repo"
harness_set_preset backend=mavros gps=ubx-uart lidar=ldlidar-uart tfluna=none
if harness_run; then
  pass "mavros backend: harness_run succeeds"
else
  fail "mavros backend: harness_run succeeds"
fi
assert_eq "mavros backend: HARDWARE_BACKEND=mavros" "mavros"   "$(env_value "$mavros_repo" HARDWARE_BACKEND)"
assert_eq "mavros backend: GNSS_BACKEND=disabled"   "disabled" "$(env_value "$mavros_repo" GNSS_BACKEND)"
assert_eq "mavros backend: MAVROS_ENABLED=true"     "true"     "$(env_value "$mavros_repo" MAVROS_ENABLED)"

mavros_fragments=$(selected_fragments_in_current_run)
for required in docker-compose.base.yml docker-compose.gui.yml docker-compose.mavros.yml docker-compose.lidar-ldlidar.yml; do
  case "$mavros_fragments" in
    *"$required"*) pass "mavros backend: fragment $required present" ;;
    *)             fail "mavros backend: fragment $required present" ;;
  esac
done
case "$mavros_fragments" in
  *docker-compose.gps.yml*|*docker-compose.unicore.yaml*|*docker-compose.nmea.yaml*)
    fail "mavros backend: NO direct GNSS fragment" "direct GNSS fragment leaked into mavros compose selection"
    ;;
  *)
    pass "mavros backend: NO direct GNSS fragment"
    ;;
esac

test_summary
