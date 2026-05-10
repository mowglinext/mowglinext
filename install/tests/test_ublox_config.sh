#!/usr/bin/env bash
# =============================================================================
# install/lib/ublox_config.sh coverage
# =============================================================================

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=lib/framework.sh
source "$SCRIPT_DIR/lib/framework.sh"

setup_sandbox

# shellcheck source=/dev/null
source "$REPO_ROOT/install/lib/serial_probe.sh"
# shellcheck source=/dev/null
source "$REPO_ROOT/install/lib/ublox_config.sh"

info() { :; }
warn() { :; }
sleep() { :; }

port="$SANDBOX/fake-tty"
touch "$port"

reset_ublox_mocks() {
  UBLOX_COMMANDS=""
  UBLOX_STTY=""
  UBLOX_CURRENT_BAUD=""
  UBLOX_IDENTIFY_BAUDS=""
  UBLOX_IDENTIFY_OK="true"
  UBLOX_SET_BAUD_OK="true"
  UBLOX_SAVE_OK="true"
  UBLOX_TARGET_ACTIVE=""
}

serial_port_exists() { [ -e "$1" ]; }

ublox_set_stty() {
  UBLOX_CURRENT_BAUD="$2"
  UBLOX_STTY+="$1 $2"$'\n'
}

ublox_run_helper() {
  local helper_port="${1:?missing port}"
  local command="${2:?missing command}"
  shift 2

  case "$command" in
    identify)
      UBLOX_COMMANDS+="identify ${helper_port} ${UBLOX_CURRENT_BAUD}"$'\n'
      [ "$UBLOX_IDENTIFY_OK" = "true" ] || return 1
      case " $UBLOX_IDENTIFY_BAUDS " in
        *" ${UBLOX_CURRENT_BAUD} "*) printf '%s\n' "u-blox ZED-F9P" ;;
        *) return 1 ;;
      esac
      ;;
    set-baud)
      UBLOX_COMMANDS+="set-baud ${helper_port} $1 $2"$'\n'
      [ "$UBLOX_SET_BAUD_OK" = "true" ] || return 1
      UBLOX_TARGET_ACTIVE="$2"
      UBLOX_IDENTIFY_BAUDS="${UBLOX_IDENTIFY_BAUDS} ${UBLOX_TARGET_ACTIVE}"
      ;;
    save)
      UBLOX_COMMANDS+="save ${helper_port}"$'\n'
      [ "$UBLOX_SAVE_OK" = "true" ]
      ;;
    *)
      return 1
      ;;
  esac
}

section "u-blox baud command sequence"

reset_ublox_mocks
UBLOX_IDENTIFY_BAUDS="9600"

if configure_ublox_baud_921600 "$port" 9600; then
  pass "configure_ublox_baud_921600 succeeds when verify succeeds"
else
  fail "configure_ublox_baud_921600 succeeds when verify succeeds"
fi

assert_contains "u-blox identifies before config" "identify $port 9600" "$UBLOX_COMMANDS"
assert_contains "u-blox sends CFG-PRT update" "set-baud $port 1 921600" "$UBLOX_COMMANDS"
assert_contains "u-blox saves config after verify" "save $port" "$UBLOX_COMMANDS"
assert_eq "u-blox stty sequence" "$port 9600"$'\n'"$port 9600"$'\n'"$port 921600"$'\n'"$port 921600" "$(printf '%s' "$UBLOX_STTY" | sed '/^$/d')"

section "already at 921600 is a no-op verify"

reset_ublox_mocks
UBLOX_IDENTIFY_BAUDS="921600"

if configure_ublox_baud_921600 "$port" 921600; then
  pass "configure_ublox_baud_921600 verifies existing 921600"
else
  fail "configure_ublox_baud_921600 verifies existing 921600"
fi

assert_not_contains "no CFG-PRT change when already at 921600" "set-baud" "$UBLOX_COMMANDS"
assert_not_contains "no save when already at 921600" "save" "$UBLOX_COMMANDS"

section "GPS_BAUD changes only on verified u-blox success"

reset_ublox_mocks
GNSS_BACKEND=ublox
GPS_PROTOCOL=UBX
GPS_BAUD=9600
UBLOX_IDENTIFY_BAUDS="9600"
maybe_upgrade_ublox_baud "$port" "$GPS_BAUD" auto
assert_eq "ublox backend becomes 921600 after verified upgrade" "921600" "$GPS_BAUD"

reset_ublox_mocks
GNSS_BACKEND=gps
GPS_PROTOCOL=UBX
GPS_BAUD=9600
UBLOX_IDENTIFY_BAUDS="9600"
maybe_upgrade_ublox_baud "$port" "$GPS_BAUD" auto
assert_eq "generic gps UBX becomes 921600 only after verified u-blox upgrade" "921600" "$GPS_BAUD"

section "unknown receiver is not forced"

reset_ublox_mocks
GNSS_BACKEND=gps
GPS_PROTOCOL=UBX
GPS_BAUD=9600
UBLOX_IDENTIFY_BAUDS=""
maybe_upgrade_ublox_baud "$port" "$GPS_BAUD" auto
assert_eq "unknown receiver keeps detected baud" "9600" "$GPS_BAUD"
assert_not_contains "unknown receiver skips CFG-PRT" "set-baud" "$UBLOX_COMMANDS"

section "verification failure keeps detected baud"

reset_ublox_mocks
GNSS_BACKEND=ublox
GPS_PROTOCOL=UBX
GPS_BAUD=9600
UBLOX_IDENTIFY_BAUDS="9600"
UBLOX_SET_BAUD_OK="false"
maybe_upgrade_ublox_baud "$port" "$GPS_BAUD" auto
assert_eq "failed upgrade keeps detected baud" "9600" "$GPS_BAUD"

section "No-op for other GNSS modes"

reset_ublox_mocks
GNSS_BACKEND=unicore
GPS_PROTOCOL=UBX
GPS_BAUD=460800
maybe_upgrade_ublox_baud "$port" "$GPS_BAUD" auto
assert_eq "unicore backend does not send u-blox commands" "" "$UBLOX_COMMANDS"

reset_ublox_mocks
GNSS_BACKEND=gps
GPS_PROTOCOL=NMEA
GPS_BAUD=115200
maybe_upgrade_ublox_baud "$port" "$GPS_BAUD" auto
assert_eq "generic NMEA backend does not send u-blox commands" "" "$UBLOX_COMMANDS"

test_summary
