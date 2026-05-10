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
assert_eq "gps/ubx/uart: GPS_BAUD=921600"   "921600" "$(env_value "$repo" GPS_BAUD)"
assert_eq "gps/ubx/uart: GPS_CONNECTION=uart" "uart" "$(env_value "$repo" GPS_CONNECTION)"
case "$(selected_fragments_in_current_run)" in
  *docker-compose.gps.yml*) pass "gps/ubx/uart: gps fragment present" ;;
  *)                        fail "gps/ubx/uart: gps fragment present" ;;
esac

# ── NMEA over UART (same single runtime GPS_BAUD target unless detected otherwise) ──
section "gnss=gps protocol=NMEA connection=uart"

repo="$SANDBOX/repo_gps_nmea_uart"
sandbox_repo "$repo"
harness_init "$repo"
harness_set_preset gnss=gps gps=nmea-uart lidar=none tfluna=none
harness_run >/dev/null 2>&1
assert_eq "nmea/uart: GPS_PROTOCOL=NMEA" "NMEA"   "$(env_value "$repo" GPS_PROTOCOL)"
assert_eq "nmea/uart: GPS_BAUD=921600"   "921600" "$(env_value "$repo" GPS_BAUD)"
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
harness_set_preset gnss=ublox lidar=none tfluna=none
if harness_run; then pass "harness_run ublox"; else fail "harness_run ublox"; fi
assert_eq "ublox: GNSS_BACKEND=ublox" "ublox" "$(env_value "$repo" GNSS_BACKEND)"
assert_eq "ublox: GPS_CONNECTION forced to usb" "usb" "$(env_value "$repo" GPS_CONNECTION)"
assert_eq "ublox: GPS_PROTOCOL forced to UBX" "UBX" "$(env_value "$repo" GPS_PROTOCOL)"
assert_eq "ublox: dedicated serial string stored" "ublox-test-serial" "$(env_value "$repo" UBLOX_DEVICE_SERIAL_STRING)"

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
# Web/CLI presets no longer bake a backend-specific intermediate baud.
assert_eq "unicore/uart: GPS_BAUD=921600" "921600" "$(env_value "$repo" GPS_BAUD)"

unicore_fragments=$(selected_fragments_in_current_run)
case "$unicore_fragments" in
  *docker-compose.unicore.yaml*) pass "unicore: unicore fragment present" ;;
  *)                             fail "unicore: unicore fragment present" "got: $unicore_fragments" ;;
esac
case "$unicore_fragments" in
  *docker-compose.gps.yml*) fail "unicore: NO legacy gps fragment" "legacy gps leaked when unicore selected" ;;
  *)                        pass "unicore: NO legacy gps fragment" ;;
esac

# ── Web composer Unicore preset must not reuse stale GPS USB defaults ─────
section "gnss=unicore web preset with stale GPS USB config"

repo="$SANDBOX/repo_unicore_web_stale_env"
sandbox_repo "$repo"
harness_init "$repo"

serial_dir="$SANDBOX/serial-unicore"
serial_target="$SANDBOX/ttyUSB-unicore"
mkdir -p "$serial_dir"
touch "$serial_target"
ln -s "$serial_target" "$serial_dir/usb-Unicore_UM980"

export SERIAL_BY_ID_DIR="$serial_dir"
export PRESET_LOADED=true
export STATE_ACTIVE_PRESET_COUNT=1
STATE_PARSED_KEYS=(GNSS_BACKEND)
STATE_PARSED_VALUES=(unicore)

GNSS_BACKEND=unicore
GPS_CONNECTION=usb
GPS_PROTOCOL=UBX
GPS_BY_ID=""
pick_serial_called=false

prompt_count=0
prompt() {
  prompt_count=$((prompt_count + 1))
  case "$prompt_count" in
    1) REPLY="1" ;; # connection: USB
    2) REPLY="1" ;; # by-id candidate
    3) REPLY="1" ;; # protocol: UBX
    *) REPLY="${2:-}" ;;
  esac
}
pick_serial_by_id() {
  pick_serial_called=true
  REPLY="$serial_dir/usb-Unicore_UM980"
}

if configure_gps >/dev/null 2>&1; then
  pass "unicore web preset ignores stale GPS USB preset values"
else
  fail "unicore web preset ignores stale GPS USB preset values"
fi
assert_eq "unicore web preset asks for by-id selection" "true" "$pick_serial_called"
assert_eq "unicore web preset selects by-id interactively" "$serial_dir/usb-Unicore_UM980" "${GPS_BY_ID:-}"
assert_eq "unicore web preset keeps backend" "unicore" "${GNSS_BACKEND:-}"
unset SERIAL_BY_ID_DIR

# ── Explicit GPS_BAUD must remain authoritative ───────────────────────────
section "explicit GPS_BAUD is not auto-replaced"

repo="$SANDBOX/repo_explicit_baud"
sandbox_repo "$repo"
harness_init "$repo"

export PRESET_LOADED=true
export STATE_ACTIVE_PRESET_COUNT=4
STATE_PARSED_KEYS=(GNSS_BACKEND GPS_CONNECTION GPS_PROTOCOL GPS_BAUD)
STATE_PARSED_VALUES=(gps uart NMEA 115200)

GNSS_BACKEND=gps
GPS_CONNECTION=uart
GPS_PROTOCOL=NMEA
GPS_BAUD=115200
GPS_UART_DEVICE=/dev/ttyAMA4

prompt_or_probe_baud() {
  fail "explicit GPS_BAUD is not auto-replaced" "prompt_or_probe_baud should not be called"
  return 1
}

if configure_gps >/dev/null 2>&1; then
  pass "explicit GPS_BAUD preset configures GPS"
else
  fail "explicit GPS_BAUD preset configures GPS"
fi
assert_eq "explicit GPS_BAUD remains unchanged" "115200" "${GPS_BAUD:-}"
unset -f prompt_or_probe_baud

# ── Generic GPS UBX can upgrade to 921600 when confirmed as u-blox ────────
section "generic gps UBX auto-detected baud can be upgraded"

repo="$SANDBOX/repo_gps_ubx_upgrade"
sandbox_repo "$repo"
harness_init "$repo"

export PRESET_LOADED=true
export STATE_ACTIVE_PRESET_COUNT=1
STATE_PARSED_KEYS=(GNSS_BACKEND)
STATE_PARSED_VALUES=(gps)

GNSS_BACKEND=gps
unset GPS_CONNECTION GPS_PROTOCOL GPS_BAUD GPS_BY_ID 2>/dev/null || true

prompt_count=0
prompt() {
  prompt_count=$((prompt_count + 1))
  case "$prompt_count" in
    1) REPLY="2" ;; # connection: UART
    2) REPLY="1" ;; # protocol: UBX
    3) REPLY="1" ;; # baud: auto-detect
    *) REPLY="${2:-}" ;;
  esac
}
serial_probe_baud() {
  printf '9600\n'
}
serial_port_exists() {
  return 0
}
ublox_upgrade_args_file="$SANDBOX/ublox-upgrade-args"
maybe_upgrade_ublox_baud() {
  printf '%s\n%s\n%s\n' "$1" "$2" "$3" > "$ublox_upgrade_args_file"
  GPS_BAUD="921600"
  return 0
}

if configure_gps >/dev/null 2>&1; then
  pass "generic gps UBX auto-detected upgrade configures GPS"
else
  fail "generic gps UBX auto-detected upgrade configures GPS"
fi
mapfile -t ublox_upgrade_args < "$ublox_upgrade_args_file"
assert_eq "generic gps UBX upgrade uses selected UART" "/dev/ttyAMA4" "${ublox_upgrade_args[0]:-}"
assert_eq "generic gps UBX upgrade uses detected baud" "9600" "${ublox_upgrade_args[1]:-}"
assert_eq "generic gps UBX upgrade keeps auto mode" "ask" "${ublox_upgrade_args[2]:-}"
assert_eq "generic gps UBX upgrade success writes GPS_BAUD=921600" "921600" "${GPS_BAUD:-}"
unset -f maybe_upgrade_ublox_baud serial_probe_baud serial_port_exists

# ── Unicore web preset without GPS_BAUD can auto-detect baud ──────────────
section "unicore web preset auto-detects GPS_BAUD"

repo="$SANDBOX/repo_unicore_web_auto_baud"
sandbox_repo "$repo"
harness_init "$repo"

export PRESET_LOADED=true
export STATE_ACTIVE_PRESET_COUNT=1
STATE_PARSED_KEYS=(GNSS_BACKEND)
STATE_PARSED_VALUES=(unicore)

GNSS_BACKEND=unicore
unset GPS_CONNECTION GPS_PROTOCOL GPS_BAUD GPS_BY_ID 2>/dev/null || true

prompt_count=0
prompt() {
  prompt_count=$((prompt_count + 1))
  case "$prompt_count" in
    1) REPLY="1" ;; # connection: USB
    2) REPLY="1" ;; # protocol: UBX
    *) REPLY="${2:-}" ;;
  esac
}
pick_serial_by_id() {
  REPLY="$serial_dir/usb-Unicore_UM980"
}
probe_args_file="$SANDBOX/unicore-probe-args"
serial_probe_baud() {
  printf '%s\n%s\n' "$1" "$2" > "$probe_args_file"
  printf '921600\n'
}

if configure_gps >/dev/null 2>&1; then
  pass "unicore web preset without GPS_BAUD configures GPS"
else
  fail "unicore web preset without GPS_BAUD configures GPS"
fi
mapfile -t probe_args < "$probe_args_file"
assert_eq "unicore baud probe uses selected by-id" "$serial_dir/usb-Unicore_UM980" "${probe_args[0]:-}"
assert_eq "unicore baud probe uses backend" "unicore" "${probe_args[1]:-}"
assert_eq "unicore web preset writes detected GPS_BAUD" "921600" "${GPS_BAUD:-}"

# ── Unicore detected baud is upgraded to 921600 only after verification ───
section "unicore auto-detected baud upgrade success"

repo="$SANDBOX/repo_unicore_upgrade_success"
sandbox_repo "$repo"
harness_init "$repo"

export PRESET_LOADED=true
export STATE_ACTIVE_PRESET_COUNT=1
STATE_PARSED_KEYS=(GNSS_BACKEND)
STATE_PARSED_VALUES=(unicore)

GNSS_BACKEND=unicore
unset GPS_CONNECTION GPS_PROTOCOL GPS_BAUD GPS_BY_ID UNICORE_COM_PORT 2>/dev/null || true

prompt_count=0
prompt() {
  prompt_count=$((prompt_count + 1))
  case "$prompt_count" in
    1) REPLY="1" ;;    # connection: USB
    2) REPLY="1" ;;    # protocol: UBX
    3) REPLY="1" ;;    # baud: auto-detect
    4) REPLY="COM1" ;; # Unicore COM port
    *) REPLY="${2:-}" ;;
  esac
}
pick_serial_by_id() {
  REPLY="$serial_dir/usb-Unicore_UM980"
}
serial_probe_baud() {
  printf '460800\n'
}
upgrade_args_file="$SANDBOX/unicore-upgrade-success-args"
configure_unicore_baud_921600() {
  printf '%s\n%s\n%s\n' "$1" "$2" "$3" > "$upgrade_args_file"
  return 0
}

if configure_gps >/dev/null 2>&1; then
  pass "unicore detected baud upgrade success configures GPS"
else
  fail "unicore detected baud upgrade success configures GPS"
fi
mapfile -t upgrade_args < "$upgrade_args_file"
assert_eq "unicore upgrade uses selected by-id" "$serial_dir/usb-Unicore_UM980" "${upgrade_args[0]:-}"
assert_eq "unicore upgrade uses detected baud" "460800" "${upgrade_args[1]:-}"
assert_eq "unicore upgrade uses COM1 default" "COM1" "${upgrade_args[2]:-}"
assert_eq "unicore upgrade success writes GPS_BAUD=921600" "921600" "${GPS_BAUD:-}"

section "unicore auto-detected baud upgrade failure keeps detected baud"

repo="$SANDBOX/repo_unicore_upgrade_failure"
sandbox_repo "$repo"
harness_init "$repo"

export PRESET_LOADED=true
export STATE_ACTIVE_PRESET_COUNT=1
STATE_PARSED_KEYS=(GNSS_BACKEND)
STATE_PARSED_VALUES=(unicore)

GNSS_BACKEND=unicore
unset GPS_CONNECTION GPS_PROTOCOL GPS_BAUD GPS_BY_ID UNICORE_COM_PORT 2>/dev/null || true

prompt_count=0
prompt() {
  prompt_count=$((prompt_count + 1))
  case "$prompt_count" in
    1) REPLY="1" ;;
    2) REPLY="1" ;;
    3) REPLY="1" ;;
    4) REPLY="COM1" ;;
    *) REPLY="${2:-}" ;;
  esac
}
pick_serial_by_id() {
  REPLY="$serial_dir/usb-Unicore_UM980"
}
serial_probe_baud() {
  printf '460800\n'
}
configure_unicore_baud_921600() {
  return 1
}

if configure_gps >/dev/null 2>&1; then
  pass "unicore detected baud upgrade failure still configures GPS"
else
  fail "unicore detected baud upgrade failure still configures GPS"
fi
assert_eq "unicore upgrade failure keeps detected GPS_BAUD" "460800" "${GPS_BAUD:-}"

unset -f configure_unicore_baud_921600 serial_probe_baud

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
