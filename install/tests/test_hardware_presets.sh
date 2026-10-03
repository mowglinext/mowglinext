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
harness_set_preset backend=mowgli gnss=auto gnss_connection=uart lidar=ldlidar-uart
if harness_run; then
  pass "mowgli backend: harness_run succeeds"
else
  fail "mowgli backend: harness_run succeeds"
fi
assert_eq "mowgli backend: HARDWARE_BACKEND=mowgli" "mowgli" "$(env_value "$mowgli_repo" HARDWARE_BACKEND)"
assert_eq "mowgli backend: GNSS_SOURCE=direct" "direct" "$(env_value "$mowgli_repo" GNSS_SOURCE)"
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

# ── Pixhawk MAVROS backend + GPS on Pixhawk ───────────────────────────────
section "HARDWARE_BACKEND=mavros + GNSS_SOURCE=mavros"

mavros_repo="$SANDBOX/repo_mavros"
sandbox_repo "$mavros_repo"
harness_init "$mavros_repo"
harness_set_preset backend=mavros gnss_source=mavros gnss=auto lidar=ldlidar-uart
if harness_run; then
  pass "mavros/pixhawk GNSS: harness_run succeeds"
else
  fail "mavros/pixhawk GNSS: harness_run succeeds"
fi

assert_eq "mavros/pixhawk GNSS: HARDWARE_BACKEND=mavros" \
  "mavros" "$(env_value "$mavros_repo" HARDWARE_BACKEND)"
assert_eq "mavros/pixhawk GNSS: GNSS_SOURCE=mavros" \
  "mavros" "$(env_value "$mavros_repo" GNSS_SOURCE)"
assert_eq "mavros/pixhawk GNSS: GNSS_BACKEND remains universal" \
  "universal" "$(env_value "$mavros_repo" GNSS_BACKEND)"
assert_eq "mavros/pixhawk GNSS: GNSS_STACK remains universal" \
  "universal" "$(env_value "$mavros_repo" GNSS_STACK)"
assert_eq "mavros/pixhawk GNSS: GNSS_STATUS_SOURCE=external" \
  "external" "$(env_value "$mavros_repo" GNSS_STATUS_SOURCE)"
assert_eq "mavros/pixhawk GNSS: direct serial device unused" \
  "" "$(env_value "$mavros_repo" GNSS_SERIAL_DEVICE)"
assert_eq "mavros/pixhawk GNSS: MAVROS_GPS1_CANONICAL=true" \
  "true" "$(env_value "$mavros_repo" MAVROS_GPS1_CANONICAL)"
assert_eq "mavros/pixhawk GNSS: MAVROS_ENABLED=true" \
  "true" "$(env_value "$mavros_repo" MAVROS_ENABLED)"

assert_eq "mavros/pixhawk GNSS: detected by-id path is the MAVROS port" \
  "/dev/serial/by-id/usb-Pixhawk-stub" \
  "$(env_value "$mavros_repo" MAVROS_PORT)"

mavros_fragments=$(selected_fragments_in_current_run)
for required in docker-compose.base.yml docker-compose.gui.yml docker-compose.gps-mavros.yml docker-compose.mavros.yml docker-compose.lidar-ldlidar.yml; do
  case "$mavros_fragments" in
    *"$required"*) pass "mavros/pixhawk GNSS: fragment $required present" ;;
    *)             fail "mavros/pixhawk GNSS: fragment $required present" ;;
  esac
done
case "$mavros_fragments" in
  *docker-compose.gps.yml*) fail "mavros/pixhawk GNSS: direct GPS fragment absent" ;;
  *) pass "mavros/pixhawk GNSS: direct GPS fragment absent" ;;
esac

# The legacy MAVROS_GPS1_CANONICAL variable is only a deprecated
# consistency guard for GPS1. GPS2 canonical ownership is selected by
# GNSS_SOURCE/GNSS_MAVROS_SOURCE and must leave the GPS1 guard false.
mavros_gps2_repo="$SANDBOX/repo_mavros_gps2"
sandbox_repo "$mavros_gps2_repo"
harness_init "$mavros_gps2_repo"
harness_set_preset backend=mavros gnss_source=mavros gnss=auto lidar=none
GNSS_MAVROS_SOURCE="gps2"
if harness_run; then
  pass "mavros/GPS2 GNSS: harness_run succeeds"
else
  fail "mavros/GPS2 GNSS: harness_run succeeds"
fi
assert_eq "mavros/GPS2 GNSS: GNSS_MAVROS_SOURCE=gps2" \
  "gps2" "$(env_value "$mavros_gps2_repo" GNSS_MAVROS_SOURCE)"
assert_eq "mavros/GPS2 GNSS: deprecated GPS1 guard remains false" \
  "false" "$(env_value "$mavros_gps2_repo" MAVROS_GPS1_CANONICAL)"

# ── Pixhawk MAVROS backend + GPS directly on SoC ──────────────────────────
section "HARDWARE_BACKEND=mavros + GNSS_SOURCE=direct"

mavros_direct_repo="$SANDBOX/repo_mavros_direct"
sandbox_repo "$mavros_direct_repo"
harness_init "$mavros_direct_repo"
harness_set_preset backend=mavros gnss_source=direct gnss=auto gnss_connection=uart lidar=none
if harness_run; then
  pass "mavros/direct GNSS: harness_run succeeds"
else
  fail "mavros/direct GNSS: harness_run succeeds"
fi
assert_eq "mavros/direct GNSS: GNSS_SOURCE=direct" "direct" "$(env_value "$mavros_direct_repo" GNSS_SOURCE)"
assert_eq "mavros/direct GNSS: GNSS_STATUS_SOURCE=universal" "universal" "$(env_value "$mavros_direct_repo" GNSS_STATUS_SOURCE)"
assert_eq "mavros/direct GNSS: serial device retained" "/dev/ttyAMA4" "$(env_value "$mavros_direct_repo" GNSS_SERIAL_DEVICE)"
assert_eq "mavros/direct GNSS: MAVROS_GPS1_CANONICAL=false" "false" "$(env_value "$mavros_direct_repo" MAVROS_GPS1_CANONICAL)"

mavros_direct_fragments=$(selected_fragments_in_current_run)
for required in docker-compose.base.yml docker-compose.gui.yml docker-compose.gps.yml docker-compose.mavros.yml; do
  case "$mavros_direct_fragments" in
    *"$required"*) pass "mavros/direct GNSS: fragment $required present" ;;
    *)             fail "mavros/direct GNSS: fragment $required present" ;;
  esac
done
case "$mavros_direct_fragments" in
  *docker-compose.gps-mavros.yml*) fail "mavros/direct GNSS: NTRIP-only fragment absent" ;;
  *) pass "mavros/direct GNSS: NTRIP-only fragment absent" ;;
esac

section "HARDWARE_BACKEND=mavros + GNSS_STACK=disabled"

mavros_no_gnss_repo="$SANDBOX/repo_mavros_no_gnss"
sandbox_repo "$mavros_no_gnss_repo"
harness_init "$mavros_no_gnss_repo"
harness_set_preset backend=mavros gnss_source=direct gnss=disabled lidar=none
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
  *docker-compose.gps.yml*|*docker-compose.gps-mavros.yml*) fail "mavros + disabled GNSS: GPS fragment absent" ;;
  *) pass "mavros + disabled GNSS: GPS fragment absent" ;;
esac

# ── Invalid backend values fail closed ────────────────────────────────────
section "invalid HARDWARE_BACKEND"

invalid_repo="$SANDBOX/repo_invalid"
sandbox_repo "$invalid_repo"
harness_init "$invalid_repo"
HARDWARE_BACKEND="unexpected"
assert_exit_nonzero "invalid backend: setup_env rejects the value" setup_env
assert_exit_nonzero "invalid backend: compose selection rejects the value" build_compose_stack
assert_exit_nonzero "invalid backend: restart selection rejects the value" \
  compose_restart_services_for_backend unexpected

test_summary
