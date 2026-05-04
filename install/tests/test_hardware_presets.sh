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

mowgli_services=$(grep -E '^\s+container_name:' "$mowgli_repo/docker/docker-compose.yaml" \
  | awk '{print $2}' | sort)
for required in mowgli-ros2 mowgli-gui mowgli-gps mowgli-lidar; do
  case "$mowgli_services" in
    *"$required"*) pass "mowgli backend: service $required present" ;;
    *)             fail "mowgli backend: service $required present" ;;
  esac
done
case "$mowgli_services" in
  *mowgli-mavros*) fail "mowgli backend: NO mavros service" "mowgli-mavros leaked into compose" ;;
  *)               pass "mowgli backend: NO mavros service" ;;
esac
case "$mowgli_services" in
  *mowgli-ntrip*) fail "mowgli backend: NO standalone ntrip sidecar" "mowgli-ntrip leaked into compose" ;;
  *)              pass "mowgli backend: NO standalone ntrip sidecar" ;;
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

mavros_services=$(grep -E '^\s+container_name:' "$mavros_repo/docker/docker-compose.yaml" \
  | awk '{print $2}' | sort)
for required in mowgli-ros2 mowgli-gui mowgli-mavros mowgli-lidar; do
  case "$mavros_services" in
    *"$required"*) pass "mavros backend: service $required present" ;;
    *)             fail "mavros backend: service $required present" ;;
  esac
done
case "$mavros_services" in
  *mowgli-gps*) fail "mavros backend: NO direct GPS container" "mowgli-gps leaked into mavros compose" ;;
  *)            pass "mavros backend: NO direct GPS container" ;;
esac
case "$mavros_services" in
  *mowgli-ntrip*) fail "mavros backend: NO standalone ntrip sidecar" "mowgli-ntrip leaked into mavros compose" ;;
  *)              pass "mavros backend: NO standalone ntrip sidecar" ;;
esac

test_summary
