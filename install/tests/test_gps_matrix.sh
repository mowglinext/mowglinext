#!/usr/bin/env bash
# =============================================================================
# A.4 GPS backend matrix
#
# The installer supports three direct GNSS backends:
#   gps     — generic legacy GPS container       (docker-compose.gps.yml)
#   ublox   — u-blox F9P (ublox_dgnss launch)    (docker-compose.ublox.yaml)
#   unicore — Unicore UM98x (unicore_gnss launch)(docker-compose.unicore.yaml)
# Generic NMEA receivers are modeled as GNSS_BACKEND=gps with
# GPS_PROTOCOL=NMEA, not as a separate GNSS backend.
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
  grep -E "^${2}=" "$1/docker/.env" | head -1 | cut -d= -f2-
}

selected_fragments_in_current_run() {
  printf '%s\n' "${COMPOSE_FILES[@]}" | xargs -n1 basename | sort
}

setup_sandbox
install_all_mocks

# ── Default GPS backend: legacy "gps" container, UBX over UART ────────────
section "gnss=gps protocol=UBX connection=uart"

repo="$SANDBOX/repo_gps_ubx_uart"
sandbox_repo "$repo"
harness_init "$repo"
harness_set_preset gnss=gps gps=ubx-uart lidar=none tfluna=none
if harness_run; then pass "harness_run gps/ubx/uart"; else fail "harness_run gps/ubx/uart"; fi
assert_eq "gps/ubx/uart: GNSS_BACKEND=gps"  "gps"   "$(env_value "$repo" GNSS_BACKEND)"
assert_eq "gps/ubx/uart: GPS_PROTOCOL=UBX"  "UBX"   "$(env_value "$repo" GPS_PROTOCOL)"
assert_eq "gps/ubx/uart: GPS_BAUD=460800"   "460800" "$(env_value "$repo" GPS_BAUD)"
assert_eq "gps/ubx/uart: GPS_CONNECTION=uart" "uart" "$(env_value "$repo" GPS_CONNECTION)"
case "$(selected_fragments_in_current_run)" in
  *docker-compose.gps.yml*) pass "gps/ubx/uart: gps fragment present" ;;
  *)                        fail "gps/ubx/uart: gps fragment present" ;;
esac

# ── NMEA over UART (lower baud) ────────────────────────────────────────────
section "gnss=gps protocol=NMEA connection=uart"

repo="$SANDBOX/repo_gps_nmea_uart"
sandbox_repo "$repo"
harness_init "$repo"
harness_set_preset gnss=gps gps=nmea-uart lidar=none tfluna=none
harness_run >/dev/null 2>&1
assert_eq "nmea/uart: GPS_PROTOCOL=NMEA" "NMEA"   "$(env_value "$repo" GPS_PROTOCOL)"
assert_eq "nmea/uart: GPS_BAUD=115200"   "115200" "$(env_value "$repo" GPS_BAUD)"
gps_nmea_fragments=$(selected_fragments_in_current_run)
case "$gps_nmea_fragments" in
  *docker-compose.gps.yml*)  pass "nmea/uart: gps fragment present" ;;
  *)                         fail "nmea/uart: gps fragment present" "got: $gps_nmea_fragments" ;;
esac
case "$gps_nmea_fragments" in
  *docker-compose.nmea.yaml*) fail "nmea/uart: NO dormant nmea fragment" "nmea fragment leaked when protocol NMEA selected on gps backend" ;;
  *)                          pass "nmea/uart: NO dormant nmea fragment" ;;
esac

# ── UBX over USB ───────────────────────────────────────────────────────────
section "gnss=gps protocol=UBX connection=usb"

repo="$SANDBOX/repo_gps_ubx_usb"
sandbox_repo "$repo"
harness_init "$repo"
harness_set_preset gnss=gps gps=ubx-usb lidar=none tfluna=none
harness_run >/dev/null 2>&1
assert_eq "ubx/usb: GPS_CONNECTION=usb" "usb" "$(env_value "$repo" GPS_CONNECTION)"
# NOTE: env.sh::setup_env runs `: "${GPS_UART_DEVICE:=/dev/ttyAMA4}"` —
# the := expansion replaces empty values with the default, so the .env
# always has GPS_UART_DEVICE set. With GPS_CONNECTION=usb the compose
# fragments ignore that variable and bind GPS_PORT directly. The
# important invariant is that no UART udev rule is generated for USB.
assert_eq "ubx/usb: no UART udev rule (GPS_UART_RULE empty)" "" "${GPS_UART_RULE:-}"

# ── u-blox F9P backend (ublox_dgnss) ───────────────────────────────────────
section "gnss=ublox (F9P / ublox_dgnss launch)"

repo="$SANDBOX/repo_ublox"
sandbox_repo "$repo"
harness_init "$repo"
harness_set_preset gnss=ublox gps=ubx-uart lidar=none tfluna=none
if harness_run; then pass "harness_run ublox"; else fail "harness_run ublox"; fi
assert_eq "ublox: GNSS_BACKEND=ublox" "ublox" "$(env_value "$repo" GNSS_BACKEND)"

# Compose selection must include the ublox fragment and exclude the legacy gps fragment.
ublox_fragments=$(selected_fragments_in_current_run)
case "$ublox_fragments" in
  *docker-compose.ublox.yaml*) pass "ublox: ublox fragment present" ;;
  *)                           fail "ublox: ublox fragment present" "got: $ublox_fragments" ;;
esac
case "$ublox_fragments" in
  *docker-compose.gps.yml*) fail "ublox: NO legacy gps fragment" "legacy gps leaked when ublox selected" ;;
  *)                        pass "ublox: NO legacy gps fragment" ;;
esac

# ── Unicore UM98x backend ──────────────────────────────────────────────────
section "gnss=unicore (UM98x via unicore_gnss launch)"

repo="$SANDBOX/repo_unicore"
sandbox_repo "$repo"
harness_init "$repo"
# Unicore needs GPS_UART_DEVICE for its `devices: ${GPS_UART_DEVICE}:...`
# binding (see compose/docker-compose.unicore.yaml).
GPS_UART_DEVICE=/dev/ttyUSB0
harness_set_preset gnss=unicore gps=ubx-uart lidar=none tfluna=none
if harness_run; then pass "harness_run unicore"; else fail "harness_run unicore"; fi
assert_eq "unicore: GNSS_BACKEND=unicore" "unicore" "$(env_value "$repo" GNSS_BACKEND)"
# Unicore UART must run at 460800 — see configure_gps override in
# install/lib/gps.sh. The PCB-side UART (ttyAMA4) is fixed at this rate;
# any other baud means the receiver won't talk to the host.
assert_eq "unicore/uart: GPS_BAUD=460800" "460800" "$(env_value "$repo" GPS_BAUD)"

unicore_fragments=$(selected_fragments_in_current_run)
case "$unicore_fragments" in
  *docker-compose.unicore.yaml*) pass "unicore: unicore fragment present" ;;
  *)                             fail "unicore: unicore fragment present" "got: $unicore_fragments" ;;
esac
case "$unicore_fragments" in
  *docker-compose.gps.yml*) fail "unicore: NO legacy gps fragment" "legacy gps leaked when unicore selected" ;;
  *)                        pass "unicore: NO legacy gps fragment" ;;
esac

# ── Invalid GNSS backend name should fail ──────────────────────────────────
section "Invalid gnss backend is rejected"

repo="$SANDBOX/repo_bad"
sandbox_repo "$repo"
harness_init "$repo"
harness_set_preset gnss=quantumgps gps=ubx-uart lidar=none tfluna=none
if harness_run >/dev/null 2>&1; then
  fail "invalid gnss backend rejected" "harness_run unexpectedly succeeded"
else
  pass "invalid gnss backend rejected"
fi

test_summary
