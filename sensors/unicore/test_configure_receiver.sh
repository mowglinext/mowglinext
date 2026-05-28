#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/configure_receiver.sh"

failures=0
declare -A MOCK_COMMAND_RESPONSES=()

assert_eq() {
  local label="${1:?assert_eq: missing label}"
  local expected="${2-}"
  local actual="${3-}"

  if [ "$expected" != "$actual" ]; then
    echo "FAIL: $label"
    echo "  expected: $expected"
    echo "  actual:   $actual"
    failures=$((failures + 1))
  else
    echo "PASS: $label"
  fi
}

assert_contains() {
  local label="${1:?assert_contains: missing label}"
  local needle="${2:?assert_contains: missing needle}"
  local haystack="${3-}"

  if [[ "$haystack" != *"$needle"* ]]; then
    echo "FAIL: $label"
    echo "  missing: $needle"
    echo "  actual:  $haystack"
    failures=$((failures + 1))
  else
    echo "PASS: $label"
  fi
}

assert_not_contains() {
  local label="${1:?assert_not_contains: missing label}"
  local needle="${2:?assert_not_contains: missing needle}"
  local haystack="${3-}"

  if [[ "$haystack" == *"$needle"* ]]; then
    echo "FAIL: $label"
    echo "  unexpected: $needle"
    echo "  actual:     $haystack"
    failures=$((failures + 1))
  else
    echo "PASS: $label"
  fi
}

reset_mocks() {
  COMMAND_LOG=""
  STTY_LOG=""
  CURRENT_BAUD=""
  MOCK_MODEL="unknown"
  MOCK_RESPONDING_BAUDS=""
  LAST_COMMAND=""
  MOCK_COMMAND_RESPONSES=()
  UNICORE_ACCEPTED_LOG_COMMANDS=()
  UNICORE_REJECTED_LOG_COMMANDS=()
  UNICORE_LOG_COMMAND_SYNTAX_BY_MESSAGE=()
}

reset_profile_env() {
  UNICORE_PROFILE="normal"
  UNICORE_OUTPUT_FORMAT="ascii"
  UNICORE_SIGNALGROUP_OVERRIDE=""
  UNICORE_MAIN_LOG_PERIOD=""
  UNICORE_BESTNAV_LOG_PERIOD=""
  UNICORE_DIAGNOSTIC_LOG_PERIOD=""
  UNICORE_SATELLITE_LOG_PERIOD=""
  UNICORE_RF_LOG_PERIOD=""
  UNICORE_RAW_LOG_PERIOD=""
  UNICORE_ENABLE_SATELLITES=""
  UNICORE_ENABLE_RF=""
  UNICORE_ENABLE_JAMMING=""
  UNICORE_ENABLE_HARDWARE=""
  UNICORE_ENABLE_GGAH=""
  UNICORE_ENABLE_RAW_OBSERVATIONS=""
}

require_serial_port() { :; }
wait_for_serial_port() { :; }
open_serial() { :; }
close_serial() { :; }
drain_serial() { :; }
sleep() { :; }

serial_set_baud() {
  local _port="${1:?serial_set_baud: missing port}"
  local baud="${2:?serial_set_baud: missing baud}"

  CURRENT_BAUD="$baud"
  STTY_LOG+="${baud}"$'\n'
}

query_receiver_identification() {
  case " $MOCK_RESPONDING_BAUDS " in
    *" $CURRENT_BAUD "*) ;;
    *) return 0 ;;
  esac

  case "$MOCK_MODEL" in
    UM980) printf '%s\n' '#VERSIONA,"UM980","R4.10Build15434"' ;;
    UM982) printf '%s\n' '#VERSIONA,"UM982","R4.10Build15434"' ;;
    *) printf '%s\n' '#VERSIONA,"UNKNOWN","R4.10Build15434"' ;;
  esac
}

send_serial_command() {
  local command="${1:?send_serial_command: missing command}"

  LAST_COMMAND="$command"
  COMMAND_LOG+="${command}"$'\n'
  if [[ "$command" =~ ^CONFIG[[:space:]]+COM1[[:space:]]+([0-9]+)$ ]]; then
    CURRENT_BAUD="${BASH_REMATCH[1]}"
  fi
}

collect_serial_output() {
  local _rounds="${1:?collect_serial_output: missing rounds}"

  printf '%s' "${MOCK_COMMAND_RESPONSES[$LAST_COMMAND]-}"
}

run_scenario() {
  local label="${1:?run_scenario: missing label}"
  local port="/dev/mock"
  local preferred_baud="${2:?run_scenario: missing preferred baud}"
  local model="${3:?run_scenario: missing model}"
  local responding_bauds="${4:?run_scenario: missing responding bauds}"
  local expected_rc="${5:?run_scenario: missing expected rc}"
  local rc=0

  echo ""
  echo "Scenario: $label"
  reset_mocks
  MOCK_MODEL="$model"
  MOCK_RESPONDING_BAUDS="$responding_bauds"

  if main "$port" "$preferred_baud"; then
    rc=0
  else
    rc=$?
  fi

  assert_eq "$label rc" "$expected_rc" "$rc"
}

run_scenario "UM980 from 460800" "460800" "UM980" "460800 921600" "0"
assert_contains "UM980 switches to 921600" "CONFIG COM1 921600" "$COMMAND_LOG"
assert_contains "UM980 uses SIGNALGROUP 2" "CONFIG SIGNALGROUP 2" "$COMMAND_LOG"
assert_contains "UM980 saves config" "SAVECONFIG" "$COMMAND_LOG"

run_scenario "UM982 already at 921600" "921600" "UM982" "921600" "0"
assert_contains "UM982 uses SIGNALGROUP 3 6" "CONFIG SIGNALGROUP 3 6" "$COMMAND_LOG"
assert_eq "UM982 does not resend baud config" "" "$(printf '%s' "$COMMAND_LOG" | grep -F "CONFIG COM1 921600" || true)"

# An unidentified model must NOT abort the whole config: SIGNALGROUP is the
# only model-specific command, so the model-independent rover config (MODE
# ROVER, RTK timeouts, masks, LOG schedule) + SAVECONFIG still apply. Gating
# everything on detection left healthy UM98x receivers stuck in their default
# non-RTK state with every diagnostic stale.
run_scenario "Unknown model still applies base config" "921600" "unknown" "921600" "0"
assert_contains "Unknown model applies MODE ROVER" "MODE ROVER" "$COMMAND_LOG"
assert_contains "Unknown model still saves config" "SAVECONFIG" "$COMMAND_LOG"
assert_eq "Unknown model skips signalgroup without override" "" "$(printf '%s' "$COMMAND_LOG" | grep -F "CONFIG SIGNALGROUP" || true)"

UNICORE_SIGNALGROUP_OVERRIDE="CONFIG SIGNALGROUP 2"
run_scenario "Unknown model honours signalgroup override" "921600" "unknown" "921600" "0"
assert_contains "Override forces signalgroup despite unknown model" "CONFIG SIGNALGROUP 2" "$COMMAND_LOG"
UNICORE_SIGNALGROUP_OVERRIDE=""

reset_profile_env
UNICORE_PROFILE="normal"
unicore_apply_profile_defaults
NORMAL_LOGS="$(build_log_commands)"
assert_not_contains "normal excludes BESTSATA" "BESTSATA" "$NORMAL_LOGS"
assert_not_contains "normal excludes SATSINFOA" "SATSINFOA" "$NORMAL_LOGS"
assert_not_contains "normal excludes AGCA" "AGCA" "$NORMAL_LOGS"
assert_not_contains "normal excludes JAMSTATUSA" "JAMSTATUSA" "$NORMAL_LOGS"
assert_contains "normal keeps PVTSLNA" "PVTSLNA" "$NORMAL_LOGS"
assert_contains "normal uses BESTNAVA direct period" $'BESTNAVA 0.2' "$NORMAL_LOGS"
assert_contains "normal uses GPHPR direct period" $'GPHPR 0.2' "$NORMAL_LOGS"
assert_contains "normal uses GPHPR2 onchanged" $'GPHPR2 ONCHANGED' "$NORMAL_LOGS"
assert_contains "normal uses RTKSTATUSA direct period" $'RTKSTATUSA 1' "$NORMAL_LOGS"
assert_contains "normal uses RTCMSTATUSA onchanged" $'RTCMSTATUSA ONCHANGED' "$NORMAL_LOGS"
assert_not_contains "normal no LOG BESTNAVA" "LOG BESTNAVA ONTIME" "$NORMAL_LOGS"
assert_not_contains "normal no LOG GNHPR" "LOG GNHPR ONTIME" "$NORMAL_LOGS"
assert_not_contains "normal excludes PVTSLNB" "PVTSLNB" "$NORMAL_LOGS"

reset_profile_env
UNICORE_PROFILE="debug"
unicore_apply_profile_defaults
DEBUG_LOGS="$(build_log_commands)"
assert_contains "debug includes BESTSATA" "BESTSATA" "$DEBUG_LOGS"
assert_contains "debug includes SATSINFOA" "SATSINFOA" "$DEBUG_LOGS"
assert_contains "debug includes AGCA" "AGCA" "$DEBUG_LOGS"
assert_contains "debug includes HWSTATUSA" "HWSTATUSA" "$DEBUG_LOGS"
assert_contains "debug includes JAMSTATUSA" "JAMSTATUSA" "$DEBUG_LOGS"
assert_contains "debug includes FREQJAMSTATUSA" "FREQJAMSTATUSA" "$DEBUG_LOGS"
assert_not_contains "debug ascii excludes BESTSATB" "BESTSATB" "$DEBUG_LOGS"
assert_not_contains "debug ascii excludes PVTSLNB" "PVTSLNB" "$DEBUG_LOGS"

reset_profile_env
UNICORE_PROFILE="debug"
UNICORE_OUTPUT_FORMAT="hybrid"
unicore_apply_profile_defaults
DEBUG_HYBRID_LOGS="$(build_log_commands)"
assert_contains "debug hybrid includes PVTSLNA" "PVTSLNA" "$DEBUG_HYBRID_LOGS"
assert_contains "debug hybrid includes PVTSLNB" "PVTSLNB" "$DEBUG_HYBRID_LOGS"
assert_contains "debug hybrid uses BESTNAVB direct period" $'BESTNAVB 0.2' "$DEBUG_HYBRID_LOGS"
assert_contains "debug hybrid uses RTKSTATUSB direct period" $'RTKSTATUSB 1' "$DEBUG_HYBRID_LOGS"
assert_contains "debug hybrid uses RTCMSTATUSB onchanged" $'RTCMSTATUSB ONCHANGED' "$DEBUG_HYBRID_LOGS"
assert_contains "debug hybrid includes BESTSATA" "BESTSATA" "$DEBUG_HYBRID_LOGS"
assert_contains "debug hybrid includes BESTSATB" "BESTSATB" "$DEBUG_HYBRID_LOGS"
assert_contains "debug hybrid keeps GPGGA" "GPGGA" "$DEBUG_HYBRID_LOGS"

reset_profile_env
UNICORE_PROFILE="debug"
UNICORE_OUTPUT_FORMAT="binary"
unicore_apply_profile_defaults
DEBUG_BINARY_LOGS="$(build_log_commands)"
assert_not_contains "debug binary excludes PVTSLNA" "PVTSLNA" "$DEBUG_BINARY_LOGS"
assert_contains "debug binary includes PVTSLNB" "PVTSLNB" "$DEBUG_BINARY_LOGS"
assert_not_contains "debug binary excludes BESTSATA" "BESTSATA" "$DEBUG_BINARY_LOGS"
assert_contains "debug binary includes BESTSATB" "BESTSATB" "$DEBUG_BINARY_LOGS"
assert_not_contains "debug binary excludes GPGGA" "GPGGA" "$DEBUG_BINARY_LOGS"

reset_profile_env
UNICORE_PROFILE="survey"
unicore_apply_profile_defaults
SURVEY_LOGS="$(build_log_commands)"
assert_not_contains "survey excludes raw observations by default" "OBSVMCMP" "$SURVEY_LOGS"
assert_eq "survey main log period default" "1" "$UNICORE_MAIN_LOG_PERIOD"

reset_profile_env
UNICORE_PROFILE="survey"
UNICORE_ENABLE_RAW_OBSERVATIONS="true"
unicore_apply_profile_defaults
SURVEY_RAW_ASCII_LOGS="$(build_log_commands)"
assert_contains "survey raw ascii includes OBSVMCMPA" "OBSVMCMPA" "$SURVEY_RAW_ASCII_LOGS"

reset_profile_env
UNICORE_PROFILE="survey"
UNICORE_OUTPUT_FORMAT="hybrid"
UNICORE_ENABLE_RAW_OBSERVATIONS="true"
unicore_apply_profile_defaults
SURVEY_RAW_BINARY_LOGS="$(build_log_commands)"
assert_contains "survey raw hybrid includes OBSVMCMPB" "OBSVMCMPB" "$SURVEY_RAW_BINARY_LOGS"
assert_contains "survey raw hybrid keeps OBSVMCMPA" "OBSVMCMPA" "$SURVEY_RAW_BINARY_LOGS"

reset_profile_env
UNICORE_PROFILE="normal"
UNICORE_ENABLE_RAW_OBSERVATIONS="true"
unicore_apply_profile_defaults
assert_eq "normal refuses raw observations" "false" "$UNICORE_ENABLE_RAW_OBSERVATIONS"
assert_not_contains "normal raw refusal emits no OBSVMCMP" "OBSVMCMP" "$(build_log_commands)"

reset_profile_env
UNICORE_PROFILE="survey"
UNICORE_ENABLE_RAW_OBSERVATIONS="true"
UNICORE_RAW_LOG_PERIOD="0.2"
unicore_apply_profile_defaults
assert_eq "survey raw period is clamped" "1" "$UNICORE_RAW_LOG_PERIOD"

reset_profile_env
UNICORE_PROFILE="high_precision"
unicore_apply_profile_defaults
HIGH_PRECISION_COMMANDS="$(build_profile_commands)"
assert_contains "high_precision enables PVTALG MULTI" "CONFIG PVTALG MULTI" "$HIGH_PRECISION_COMMANDS"
assert_contains "high_precision enables RTCMDECAUTO" "CONFIG RTCMDECAUTO ENABLE" "$HIGH_PRECISION_COMMANDS"

reset_profile_env
UNICORE_PROFILE="high_precision"
UNICORE_OUTPUT_FORMAT="binary"
UNICORE_ENABLE_RAW_OBSERVATIONS="true"
unicore_apply_profile_defaults
HIGH_PRECISION_BINARY_LOGS="$(build_log_commands)"
assert_contains "high_precision binary includes BESTNAVB" "BESTNAVB" "$HIGH_PRECISION_BINARY_LOGS"
assert_contains "high_precision binary includes OBSVMCMPB" "OBSVMCMPB" "$HIGH_PRECISION_BINARY_LOGS"
assert_not_contains "high_precision binary excludes BESTNAVA" "BESTNAVA" "$HIGH_PRECISION_BINARY_LOGS"

reset_profile_env
UNICORE_OUTPUT_FORMAT="hybrid"
unicore_apply_profile_defaults
assert_eq "hybrid output format is preserved" "hybrid" "$UNICORE_OUTPUT_FORMAT"
assert_eq "hybrid enables binary transport" "true" "$(unicore_binary_enabled_from_output_format "$UNICORE_OUTPUT_FORMAT")"

reset_profile_env
UNICORE_OUTPUT_FORMAT="broken"
unicore_apply_profile_defaults
assert_eq "invalid output format falls back to ascii" "ascii" "$UNICORE_OUTPUT_FORMAT"
assert_eq "ascii keeps binary transport disabled" "false" "$(unicore_binary_enabled_from_output_format "$UNICORE_OUTPUT_FORMAT")"

assert_eq "BESTNAVA formats as direct period" "BESTNAVA 0.2" "$(unicore_format_log_command "BESTNAVA" "0.2")"
assert_eq "RTKSTATUSA formats as direct period" "RTKSTATUSA 1" "$(unicore_format_log_command "RTKSTATUSA" "1")"
assert_eq "RTCMSTATUSA formats as onchanged" "RTCMSTATUSA ONCHANGED" "$(unicore_format_log_command "RTCMSTATUSA" "1")"
assert_eq "GPHPR formats as direct period" "GPHPR 1" "$(unicore_format_log_command "GPHPR" "1")"
assert_eq "GPHPR2 formats as onchanged" "GPHPR2 ONCHANGED" "$(unicore_format_log_command "GPHPR2" "1")"
assert_eq "GPGGA keeps log ontime" "LOG GPGGA ONTIME 0.2" "$(unicore_format_log_command "GPGGA" "0.2")"
assert_eq "PVTSLNA keeps log ontime" "LOG PVTSLNA ONTIME 0.2" "$(unicore_format_log_command "PVTSLNA" "0.2")"

reset_profile_env
UNICORE_PROFILE="debug"
UNICORE_ENABLE_SATELLITES="true"
UNICORE_OUTPUT_FORMAT="ascii"
unicore_apply_profile_defaults
gsv_schedule="$(build_log_commands)"
assert_contains "satellite schedule includes GPGSV once" "GPGSV 1" "$gsv_schedule"
assert_not_contains "satellite schedule drops GLGSV command" "GLGSV" "$gsv_schedule"
assert_not_contains "satellite schedule drops GAGSV command" "GAGSV" "$gsv_schedule"
assert_not_contains "satellite schedule drops GBGSV command" "GBGSV" "$gsv_schedule"

reset_profile_env
UNICORE_PROFILE="runtime"
UNICORE_OUTPUT_FORMAT="hybrid"
unicore_apply_profile_defaults
RUNTIME_LOGS="$(build_log_commands)"
assert_eq "runtime main log period default" "0.1" "$UNICORE_MAIN_LOG_PERIOD"
assert_eq "runtime bestnav log period default" "0.1" "$UNICORE_BESTNAV_LOG_PERIOD"
assert_eq "runtime satellite log period default" "2" "$UNICORE_SATELLITE_LOG_PERIOD"
assert_eq "runtime rf log period default" "2" "$UNICORE_RF_LOG_PERIOD"
assert_contains "runtime hybrid includes PVTSLNA" "PVTSLNA" "$RUNTIME_LOGS"
assert_contains "runtime hybrid includes PVTSLNB" "PVTSLNB" "$RUNTIME_LOGS"
assert_contains "runtime hybrid includes BESTNAVA 0.1" $'BESTNAVA 0.1' "$RUNTIME_LOGS"
assert_contains "runtime hybrid includes BESTNAVB 0.1" $'BESTNAVB 0.1' "$RUNTIME_LOGS"
assert_contains "runtime includes BESTSATA" "BESTSATA" "$RUNTIME_LOGS"
assert_contains "runtime includes BESTSATB" "BESTSATB" "$RUNTIME_LOGS"
assert_contains "runtime includes AGCA" "AGCA" "$RUNTIME_LOGS"
assert_contains "runtime includes AGCB" "AGCB" "$RUNTIME_LOGS"
assert_not_contains "runtime excludes raw by default" "OBSVMCMP" "$RUNTIME_LOGS"
assert_not_contains "runtime excludes GPGGAH by default" "GPGGAH" "$RUNTIME_LOGS"

reset_profile_env
UNICORE_PROFILE="runtime"
UNICORE_OUTPUT_FORMAT="hybrid"
UNICORE_ENABLE_GGAH="true"
unicore_apply_profile_defaults
RUNTIME_GGAH_LOGS="$(build_log_commands)"
assert_contains "runtime can enable GPGGAH" "GPGGAH 0.1" "$RUNTIME_GGAH_LOGS"

reset_mocks
MOCK_COMMAND_RESPONSES["LOG GPGGA ONTIME 0.2"]="<OK"
if send_log_command_with_fallback "LOG GPGGA ONTIME 0.2"; then
  rc=0
else
  rc=$?
fi
assert_eq "first syntax accepted rc" "0" "$rc"
assert_eq "first syntax accepted command" "LOG GPGGA ONTIME 0.2" "${UNICORE_ACCEPTED_LOG_COMMANDS[GPGGA]:-}"
assert_eq "first syntax accepted label" "nmea_log_ontime" "${UNICORE_LOG_COMMAND_SYNTAX_BY_MESSAGE[GPGGA]:-}"
assert_eq "first syntax rejected list empty" "" "${UNICORE_REJECTED_LOG_COMMANDS[GPGGA]:-}"
assert_eq "first syntax sends one command" "1" "$(printf '%s' "$COMMAND_LOG" | grep -cve '^$')"

reset_mocks
MOCK_COMMAND_RESPONSES["BESTNAVA 0.2"]="<OK"
if send_log_command_with_fallback "BESTNAVA 0.2"; then
  rc=0
else
  rc=$?
fi
assert_eq "fallback syntax accepted rc" "0" "$rc"
assert_eq "fallback syntax accepted command" "BESTNAVA 0.2" "${UNICORE_ACCEPTED_LOG_COMMANDS[BESTNAVA]:-}"
assert_eq "fallback syntax accepted label" "unicore_direct_period" "${UNICORE_LOG_COMMAND_SYNTAX_BY_MESSAGE[BESTNAVA]:-}"
assert_eq "fallback syntax rejected remembers none" "" "${UNICORE_REJECTED_LOG_COMMANDS[BESTNAVA]:-}"
assert_contains "fallback syntax command log includes accepted variant" "BESTNAVA 0.2" "$COMMAND_LOG"

reset_mocks
MOCK_COMMAND_RESPONSES["RTCMSTATUSA ONCHANGED"]="unsupported command"
MOCK_COMMAND_RESPONSES["RTCMSTATUSA COM1 ONCHANGED"]="unsupported command"
if send_log_command_with_fallback "RTCMSTATUSA ONCHANGED"; then
  rc=0
else
  rc=$?
fi
assert_eq "all syntax rejected rc" "1" "$rc"
assert_eq "all syntax rejected keeps no accepted command" "" "${UNICORE_ACCEPTED_LOG_COMMANDS[RTCMSTATUSA]:-}"
assert_contains "all syntax rejected remembers onchanged" "RTCMSTATUSA ONCHANGED" "${UNICORE_REJECTED_LOG_COMMANDS[RTCMSTATUSA]:-}"
assert_contains "all syntax rejected remembers com_onchanged" "RTCMSTATUSA COM1 ONCHANGED" "${UNICORE_REJECTED_LOG_COMMANDS[RTCMSTATUSA]:-}"

polluted_stream_response=$'\001\002#BESTNAVA,COM1,0,0,FINESTEERING,0,0;SOL_COMPUTED\n$GPGGA,123519,4807.038,N,01131.000,E,4,08,0.9,545.4,M,46.9,M,,*47'
assert_eq "stream-only data is ignored" "ignored_stream_data" "$(unicore_classify_command_response "$polluted_stream_response" "BESTNAVA 0.2")"
assert_eq "can't found device is unsupported" "unsupported" "$(unicore_classify_command_response "response can't found device" "BESTSATB 2")"
assert_eq "command ack exact target is ok" "ok" "$(unicore_classify_command_response '$command,BESTNAVA 0.2,response: OK*' "BESTNAVA 0.2")"
assert_eq "command ack prefix target is ok" "ok" "$(unicore_classify_command_response '$command,LOG GPGGA,response: OK*' "LOG GPGGA ONTIME 0.2")"
assert_eq "wrong command ack is invalid" "invalid_response" "$(unicore_classify_command_response '$command,GPGSV 1,response: OK*' "BESTSATB 2")"

reset_mocks
MOCK_COMMAND_RESPONSES["GPGSV 1"]="$polluted_stream_response"
MOCK_COMMAND_RESPONSES["GPGSV ONTIME 1"]="$polluted_stream_response"
MOCK_COMMAND_RESPONSES["GPGSV COM1 1"]="$polluted_stream_response"
MOCK_COMMAND_RESPONSES["GPGSV COM1 1"]="$polluted_stream_response"
MOCK_COMMAND_RESPONSES["LOG GPGSV ONTIME 1"]="$polluted_stream_response"
if send_log_command_with_fallback "GPGSV 1"; then
  rc=0
else
  rc=$?
fi
assert_eq "polluted stream is not accepted rc" "1" "$rc"
assert_eq "polluted stream does not cache accepted syntax" "" "${UNICORE_ACCEPTED_LOG_COMMANDS[GPGSV]:-}"
assert_contains "polluted stream rejected command recorded" "GPGSV 1" "${UNICORE_REJECTED_LOG_COMMANDS[GPGSV]:-}"

reset_mocks
MOCK_COMMAND_RESPONSES["BESTSATB 2"]=$'\001\002#BESTNAVA,COM1,0,0,FINESTEERING,0,0;SOL_COMPUTED\n$command,BESTSATB 2,response: OK*'
if send_log_command_with_fallback "BESTSATB 2"; then
  rc=0
else
  rc=$?
fi
assert_eq "polluted stream before ack accepted rc" "0" "$rc"
assert_eq "polluted stream before ack caches true ack" "BESTSATB 2" "${UNICORE_ACCEPTED_LOG_COMMANDS[BESTSATB]:-}"

if [ "$failures" -ne 0 ]; then
  echo ""
  echo "$failures test(s) failed."
  exit 1
fi

echo ""
echo "All tests passed."
