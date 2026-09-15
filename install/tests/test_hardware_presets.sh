#!/usr/bin/env bash
# =============================================================================
# A.3 Hardware preset matrix — Mowgli STM32 vs Pixhawk MAVROS
#
# The installer's select_hardware_backend offers three backends:
#   1. Mowgli STM32 board      (HARDWARE_BACKEND=mowgli)
#   2. Pixhawk via MAVROS      (HARDWARE_BACKEND=mavros)
#   3. OpenMower electronics   (HARDWARE_BACKEND=openmower — LowLevel + xESC
#      behind the mowgli-openmower bridge sidecar, GNSS stays universal)
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
harness_set_preset backend=mowgli gnss=auto gnss_connection=uart lidar=ldlidar-uart tfluna=none
if harness_run; then
  pass "mowgli backend: harness_run succeeds"
else
  fail "mowgli backend: harness_run succeeds"
fi
assert_eq "mowgli backend: HARDWARE_BACKEND=mowgli" "mowgli" "$(env_value "$mowgli_repo" HARDWARE_BACKEND)"
assert_eq "mowgli backend: GNSS_BACKEND=universal" "universal" "$(env_value "$mowgli_repo" GNSS_BACKEND)"
assert_eq "mowgli backend: GNSS_STACK=universal"   "universal" "$(env_value "$mowgli_repo" GNSS_STACK)"
assert_eq "mowgli backend: GNSS_STATUS_SOURCE=universal" "universal" "$(env_value "$mowgli_repo" GNSS_STATUS_SOURCE)"
assert_eq "mowgli backend: MAVROS_ENABLED=false"   "false"  "$(env_value "$mowgli_repo" MAVROS_ENABLED)"

mowgli_fragments=$(selected_fragments_in_current_run)
for required in docker-compose.base.yml docker-compose.gui.yml docker-compose.gps.yml docker-compose.lidar-ldlidar.yml; do
  case "$mowgli_fragments" in
    *"$required"*) pass "mowgli backend: fragment $required present" ;;
    *)             fail "mowgli backend: fragment $required present" ;;
  esac
done
case "$mowgli_fragments" in
  *docker-compose.mavros.yml*)
    fail "mowgli backend: no MAVROS fragment by default" "unexpected fragment selected"
    ;;
  *)
    pass "mowgli backend: no MAVROS fragment by default"
    ;;
esac
case "$mowgli_fragments" in
  *docker-compose.openmower.yml*)
    fail "mowgli backend: no OpenMower fragment by default" "unexpected fragment selected"
    ;;
  *)
    pass "mowgli backend: no OpenMower fragment by default"
    ;;
esac
assert_eq "mowgli backend: OPENMOWER_ENABLED=false" "false" "$(env_value "$mowgli_repo" OPENMOWER_ENABLED)"

# ── Pixhawk MAVROS backend ────────────────────────────────────────────────
section "HARDWARE_BACKEND=mavros (Pixhawk via MAVROS)"

mavros_repo="$SANDBOX/repo_mavros"
sandbox_repo "$mavros_repo"
harness_init "$mavros_repo"
harness_set_preset backend=mavros gnss=auto gnss_connection=uart lidar=ldlidar-uart tfluna=none
if harness_run; then
  pass "mavros backend: harness_run succeeds"
else
  fail "mavros backend: harness_run succeeds"
fi
assert_eq "mavros backend: HARDWARE_BACKEND=mavros" "mavros" "$(env_value "$mavros_repo" HARDWARE_BACKEND)"
assert_eq "mavros backend: GNSS_BACKEND remains universal" "universal" "$(env_value "$mavros_repo" GNSS_BACKEND)"
assert_eq "mavros backend: GNSS_STACK remains universal" "universal" "$(env_value "$mavros_repo" GNSS_STACK)"
assert_eq "mavros backend: GNSS_STATUS_SOURCE remains universal" "universal" "$(env_value "$mavros_repo" GNSS_STATUS_SOURCE)"
assert_eq "mavros backend: MAVROS_ENABLED=true" "true" "$(env_value "$mavros_repo" MAVROS_ENABLED)"

mavros_fragments=$(selected_fragments_in_current_run)
for required in docker-compose.base.yml docker-compose.gui.yml docker-compose.gps.yml docker-compose.mavros.yml docker-compose.lidar-ldlidar.yml; do
  case "$mavros_fragments" in
    *"$required"*) pass "mavros backend: fragment $required present" ;;
    *)             fail "mavros backend: fragment $required present" ;;
  esac
done
case "$mavros_fragments" in
  *docker-compose.gps.yml*) pass "mavros backend: independent Universal GNSS fragment present" ;;
  *) fail "mavros backend: independent Universal GNSS fragment present" ;;
esac

section "HARDWARE_BACKEND=mavros + GNSS_STACK=disabled"

mavros_no_gnss_repo="$SANDBOX/repo_mavros_no_gnss"
sandbox_repo "$mavros_no_gnss_repo"
harness_init "$mavros_no_gnss_repo"
harness_set_preset backend=mavros gnss=disabled lidar=none tfluna=none
if harness_run; then
  pass "mavros + disabled GNSS: harness_run succeeds"
else
  fail "mavros + disabled GNSS: harness_run succeeds"
fi
assert_eq "mavros + disabled GNSS: GNSS_BACKEND=disabled" "disabled" "$(env_value "$mavros_no_gnss_repo" GNSS_BACKEND)"
assert_eq "mavros + disabled GNSS: GNSS_STACK=disabled" "disabled" "$(env_value "$mavros_no_gnss_repo" GNSS_STACK)"
assert_eq "mavros + disabled GNSS: MAVROS_ENABLED=true" "true" "$(env_value "$mavros_no_gnss_repo" MAVROS_ENABLED)"

mavros_no_gnss_fragments=$(selected_fragments_in_current_run)
case "$mavros_no_gnss_fragments" in
  *docker-compose.mavros.yml*) pass "mavros + disabled GNSS: MAVROS fragment present" ;;
  *) fail "mavros + disabled GNSS: MAVROS fragment present" ;;
esac
case "$mavros_no_gnss_fragments" in
  *docker-compose.gps.yml*) fail "mavros + disabled GNSS: GPS fragment absent" ;;
  *) pass "mavros + disabled GNSS: GPS fragment absent" ;;
esac

# ── OpenMower electronics backend ────────────────────────────────────────
section "HARDWARE_BACKEND=openmower (LowLevel board + xESC behind the bridge sidecar)"

openmower_repo="$SANDBOX/repo_openmower"
sandbox_repo "$openmower_repo"
harness_init "$openmower_repo"
harness_set_preset backend=openmower gnss=auto gnss_connection=uart lidar=ldlidar-uart tfluna=none
if harness_run; then
  pass "openmower backend: harness_run succeeds"
else
  fail "openmower backend: harness_run succeeds"
fi
assert_eq "openmower backend: HARDWARE_BACKEND=openmower" "openmower" "$(env_value "$openmower_repo" HARDWARE_BACKEND)"
assert_eq "openmower backend: GNSS stays universal"     "universal" "$(env_value "$openmower_repo" GNSS_BACKEND)"
assert_eq "openmower backend: GNSS_STACK=universal"      "universal" "$(env_value "$openmower_repo" GNSS_STACK)"
assert_eq "openmower backend: OPENMOWER_ENABLED=true"    "true"      "$(env_value "$openmower_repo" OPENMOWER_ENABLED)"
assert_eq "openmower backend: MAVROS_ENABLED=false"      "false"     "$(env_value "$openmower_repo" MAVROS_ENABLED)"
assert_eq "openmower backend: default ESC type"          "xesc_mini" "$(env_value "$openmower_repo" OPENMOWER_XESC_TYPE)"
assert_eq "openmower backend: LowLevel port default"     "/dev/ttyAMA0" "$(env_value "$openmower_repo" OPENMOWER_LL_PORT)"
assert_eq "openmower backend: left xESC port default"    "/dev/ttyAMA5" "$(env_value "$openmower_repo" OPENMOWER_XESC_LEFT_PORT)"
assert_eq "openmower backend: right xESC port default"   "/dev/ttyAMA3" "$(env_value "$openmower_repo" OPENMOWER_XESC_RIGHT_PORT)"
assert_eq "openmower backend: mow xESC port default"     "/dev/ttyAMA4" "$(env_value "$openmower_repo" OPENMOWER_XESC_MOW_PORT)"
assert_match "openmower backend: OPENMOWER_IMAGE points at the openmower image" "/openmower:" "$(env_value "$openmower_repo" OPENMOWER_IMAGE)"

openmower_fragments=$(selected_fragments_in_current_run)
for required in docker-compose.base.yml docker-compose.gui.yml docker-compose.gps.yml docker-compose.openmower.yml docker-compose.lidar-ldlidar.yml; do
  case "$openmower_fragments" in
    *"$required"*) pass "openmower backend: fragment $required present" ;;
    *)             fail "openmower backend: fragment $required present" ;;
  esac
done
case "$openmower_fragments" in
  *docker-compose.mavros.yml*)
    fail "openmower backend: no MAVROS fragment" "mavros fragment leaked into openmower compose selection"
    ;;
  *)
    pass "openmower backend: no MAVROS fragment"
    ;;
esac

openmower_yaml="$openmower_repo/docker/config/mowgli/mowgli_robot.yaml"
assert_match "openmower backend: ticks_per_meter seeded for xESC hall ticks" "ticks_per_meter: 1600.0" "$(grep -E '^[[:space:]]+ticks_per_meter:' "$openmower_yaml")"
assert_match "openmower backend: the sparse config keeps lidar_enabled" "lidar_enabled: true" "$(grep -E '^[[:space:]]+lidar_enabled:' "$openmower_yaml")"

test_summary
