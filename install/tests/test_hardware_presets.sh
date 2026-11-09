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
case "$mowgli_fragments" in
  *docker-compose.openmower.yml*)
    fail "mowgli backend: no OpenMower fragment by default" "unexpected fragment selected"
    ;;
  *)
    pass "mowgli backend: no OpenMower fragment by default"
    ;;
esac
assert_eq "mowgli backend: OPENMOWER_ENABLED=false" "false" "$(env_value "$mowgli_repo" OPENMOWER_ENABLED)"

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

# ── OpenMower electronics backend ────────────────────────────────────────
section "HARDWARE_BACKEND=openmower (LowLevel board + xESC behind the bridge sidecar)"

openmower_repo="$SANDBOX/repo_openmower"
sandbox_repo "$openmower_repo"
harness_init "$openmower_repo"
# Every Pi UART is wired on an OpenMower v1 board (LowLevel ttyAMA0, GPS
# ttyAMA2, xESC ttyAMA3/4/5), so its LiDAR can only be a USB one.
harness_set_preset backend=openmower gnss=auto gnss_connection=uart lidar=ldlidar-usb
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
assert_eq "openmower backend: GNSS on the board's GPS UART" "/dev/ttyAMA2" "$(env_value "$openmower_repo" GNSS_SERIAL_DEVICE)"
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
# OpenMower's own defaults come from the backend overlay
# (mowgli_bringup/config/backends/openmower.yaml), never from seeding.
if grep -qE '^[[:space:]]+ticks_per_meter:' "$openmower_yaml"; then
  fail "openmower backend: ticks_per_meter is not seeded" "$(grep -E '^[[:space:]]+ticks_per_meter:' "$openmower_yaml")"
else
  pass "openmower backend: ticks_per_meter is not seeded"
fi
assert_match "openmower backend: the sparse config keeps lidar_enabled" "lidar_enabled: true" "$(grep -E '^[[:space:]]+lidar_enabled:' "$openmower_yaml")"

section "HARDWARE_BACKEND=openmower refuses a UART LiDAR on an xESC port"

openmower_clash_repo="$SANDBOX/repo_openmower_clash"
sandbox_repo "$openmower_clash_repo"
harness_init "$openmower_clash_repo"
harness_set_preset backend=openmower gnss=auto gnss_connection=uart lidar=ldlidar-uart
if harness_run; then
  fail "openmower + UART LiDAR on ttyAMA5: install refused" "installer accepted the left xESC port for the LiDAR"
else
  pass "openmower + UART LiDAR on ttyAMA5: install refused"
fi
# harness_run discards the step output; replay the LiDAR step to read it.
assert_contains "the refusal names the xESC it would collide with" "OpenMower-xESC-left" "$(configure_lidar 2>&1)"

test_summary
