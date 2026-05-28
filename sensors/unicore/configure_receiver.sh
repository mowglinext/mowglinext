#!/bin/bash
# =============================================================================
# Send a one-time UM98x rover configuration over the serial port.
#
# Symptoms this fixes:
#   * accuracy stuck at 10 m, RTK never validates, even with RTCM flowing
#   * /diagnostics carr_soln stays "none"
#
# Root cause: out-of-the-box / FRESET-state UM98x receivers default to
# something close to "rover, no NMEA enabled, no PVTSLNA" — and the
# default RTK timeout is short. Without an explicit MODE ROVER + LOG
# directives, the receiver outputs only minimal NMEA and the operator
# sees a single-point fix even though RTCM is being injected.
#
# The N4 manual mixes several output syntaxes:
# - `LOG <msg> ONTIME <period>` for messages like `GPGGA` / `PVTSLNA`
# - `<msg> <period>` for messages like `BESTNAVA` / `RTKSTATUSA` / `GPHPR`
# - `<msg> ONCHANGED` for messages like `RTCMSTATUSA` / `GPHPR2`
# This script therefore uses a per-message syntax table first, then retries a
# small compatibility matrix only if the receiver answers `PARSING FAILED` or
# `GRAMMAR ERROR`.
#
# Reference: https://github.com/CentipedeRTK/docs-centipedeRTK
#            (assets/param_files/UM98x/UM980_aog_rover_last_CONFIG.txt)
#
# Usage: configure_receiver.sh [device] [baudrate]
#
# Persisted via SAVECONFIG in receiver NVRAM, so this only meaningfully
# changes anything on the first run; subsequent calls are idempotent.
# =============================================================================
set -euo pipefail

# NOTE:
# The serial device path (usually /dev/gps) is managed by Linux udev rules
# and must remain stable across all GNSS backends.
#
# DEVICE is the Linux serial path, normally /dev/gps.
# UNICORE_COM_PORT is ONLY the internal receiver-side logical COM port
# used in commands such as:
#   CONFIG COM1 921600
#
# Do NOT derive the Linux serial device path from UNICORE_COM_PORT.
TARGET_BAUD="${UNICORE_TARGET_BAUD:-921600}"
UNICORE_COM_PORT="${UNICORE_COM_PORT:-COM1}"
UNICORE_PROFILE="${UNICORE_PROFILE:-normal}"
UNICORE_OUTPUT_FORMAT="${UNICORE_OUTPUT_FORMAT:-ascii}"
UNICORE_SIGNALGROUP_OVERRIDE="${UNICORE_SIGNALGROUP_OVERRIDE:-}"
UNICORE_MAIN_LOG_PERIOD="${UNICORE_MAIN_LOG_PERIOD:-}"
UNICORE_BESTNAV_LOG_PERIOD="${UNICORE_BESTNAV_LOG_PERIOD:-}"
UNICORE_DIAGNOSTIC_LOG_PERIOD="${UNICORE_DIAGNOSTIC_LOG_PERIOD:-}"
UNICORE_SATELLITE_LOG_PERIOD="${UNICORE_SATELLITE_LOG_PERIOD:-}"
UNICORE_RF_LOG_PERIOD="${UNICORE_RF_LOG_PERIOD:-}"
UNICORE_RAW_LOG_PERIOD="${UNICORE_RAW_LOG_PERIOD:-}"
UNICORE_ENABLE_SATELLITES="${UNICORE_ENABLE_SATELLITES:-}"
UNICORE_ENABLE_RF="${UNICORE_ENABLE_RF:-}"
UNICORE_ENABLE_JAMMING="${UNICORE_ENABLE_JAMMING:-}"
UNICORE_ENABLE_HARDWARE="${UNICORE_ENABLE_HARDWARE:-}"
UNICORE_ENABLE_GGAH="${UNICORE_ENABLE_GGAH:-}"
UNICORE_ENABLE_RAW_OBSERVATIONS="${UNICORE_ENABLE_RAW_OBSERVATIONS:-}"
UNICORE_LAST_SERIAL_COMMAND="${UNICORE_LAST_SERIAL_COMMAND:-}"
UNICORE_LAST_COMMAND_RESPONSE="${UNICORE_LAST_COMMAND_RESPONSE:-}"

declare -A UNICORE_ACCEPTED_LOG_COMMANDS=()
declare -A UNICORE_REJECTED_LOG_COMMANDS=()
declare -A UNICORE_LOG_COMMAND_SYNTAX_BY_MESSAGE=()

unicore_warn() {
  log "WARN: $*"
}

log() {
  echo "[configure_receiver.sh] $*" >&2
}

unicore_is_truthy() {
  case "${1,,}" in
    true|1|yes|y|on)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

unicore_bool_string() {
  if unicore_is_truthy "${1:-false}"; then
    printf '%s\n' "true"
  else
    printf '%s\n' "false"
  fi
}

unicore_normalize_profile() {
  case "${1:-normal}" in
    normal|runtime|debug|survey|high_precision)
      printf '%s\n' "${1:-normal}"
      ;;
    *)
      log "WARN: unknown UNICORE_PROFILE='${1:-}'; falling back to normal."
      printf '%s\n' "normal"
      ;;
  esac
}

unicore_normalize_output_format() {
  case "${1:-ascii}" in
    ascii|binary|hybrid)
      printf '%s\n' "${1:-ascii}"
      ;;
    *)
      log "WARN: unknown UNICORE_OUTPUT_FORMAT='${1:-}'; falling back to ascii."
      printf '%s\n' "ascii"
      ;;
  esac
}

unicore_binary_enabled_from_output_format() {
  case "$(unicore_normalize_output_format "${1:-ascii}")" in
    binary|hybrid)
      printf '%s\n' "true"
      ;;
    *)
      printf '%s\n' "false"
      ;;
  esac
}

unicore_output_has_ascii() {
  case "$(unicore_normalize_output_format "${1:-ascii}")" in
    ascii|hybrid)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

unicore_output_has_binary() {
  case "$(unicore_normalize_output_format "${1:-ascii}")" in
    binary|hybrid)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

unicore_profile_supports_raw() {
  case "$(unicore_normalize_profile "${1:-normal}")" in
    survey|high_precision)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

unicore_clamp_min_period() {
  local value="${1:?unicore_clamp_min_period: missing value}"
  local minimum="${2:?unicore_clamp_min_period: missing minimum}"

  awk -v value="$value" -v minimum="$minimum" 'BEGIN {
    numeric = value + 0.0
    if (numeric < minimum) {
      if (minimum == int(minimum)) {
        printf("%d\n", int(minimum))
      } else {
        printf("%.3f\n", minimum)
      }
    } else {
      print value
    }
  }'
}

unicore_period_to_rate() {
  local period="${1:?unicore_period_to_rate: missing period}"

  awk -v period="$period" 'BEGIN {
    numeric = period + 0.0
    if (numeric <= 0.0) {
      print "0"
    } else {
      printf("%g\n", 1.0 / numeric)
    }
  }'
}

unicore_parse_log_command() {
  local command="${1:?unicore_parse_log_command: missing command}"
  local -a parts=()

  read -r -a parts <<<"$command"
  if [ "${#parts[@]}" -ge 4 ] && [ "${parts[0]}" = "LOG" ] && [ "${parts[2]}" = "ONTIME" ]; then
    unicore_is_known_log_message "${parts[1]}" || return 1
    printf '%s|%s|%s\n' "${parts[1]}" "nmea_log_ontime" "${parts[3]}"
    return 0
  fi
  if [ "${#parts[@]}" -eq 2 ] && [ "${parts[1]}" = "ONCHANGED" ]; then
    unicore_is_known_log_message "${parts[0]}" || return 1
    printf '%s|%s|\n' "${parts[0]}" "unicore_onchanged"
    return 0
  fi
  if [ "${#parts[@]}" -eq 2 ]; then
    unicore_is_known_log_message "${parts[0]}" || return 1
    printf '%s|%s|%s\n' "${parts[0]}" "unicore_direct_period" "${parts[1]}"
    return 0
  fi
  return 1
}

unicore_log_command_variants() {
  local message="${1:?unicore_log_command_variants: missing message}"
  local syntax="${2:?unicore_log_command_variants: missing syntax}"
  local period="${3-}"
  local rate
  local command
  local label
  local -A seen=()

  if [ -n "${UNICORE_ACCEPTED_LOG_COMMANDS[$message]:-}" ]; then
    command="${UNICORE_ACCEPTED_LOG_COMMANDS[$message]}"
    label="${UNICORE_LOG_COMMAND_SYNTAX_BY_MESSAGE[$message]:-cached}"
    printf '%s\t%s\n' "$label" "$command"
    seen["$command"]=1
  fi

  case "$syntax" in
    unicore_onchanged)
      command="${message} ONCHANGED"
      if [ -z "${seen[$command]:-}" ]; then
        printf '%s\t%s\n' "unicore_onchanged" "$command"
        seen["$command"]=1
      fi
      command="${message} ${UNICORE_COM_PORT} ONCHANGED"
      if [ -z "${seen[$command]:-}" ]; then
        printf '%s\t%s\n' "com_onchanged" "$command"
        seen["$command"]=1
      fi
      ;;
    unicore_direct_period)
      command="${message} ${period}"
      if [ -z "${seen[$command]:-}" ]; then
        printf '%s\t%s\n' "unicore_direct_period" "$command"
        seen["$command"]=1
      fi
      command="${message} ONTIME ${period}"
      if [ -z "${seen[$command]:-}" ]; then
        printf '%s\t%s\n' "bare_ontime" "$command"
        seen["$command"]=1
      fi
      command="${message} ${UNICORE_COM_PORT} ${period}"
      if [ -z "${seen[$command]:-}" ]; then
        printf '%s\t%s\n' "com_period" "$command"
        seen["$command"]=1
      fi
      rate="$(unicore_period_to_rate "$period")"
      if [ "$rate" != "0" ]; then
        command="${message} ${UNICORE_COM_PORT} ${rate}"
        if [ -z "${seen[$command]:-}" ]; then
          printf '%s\t%s\n' "com_rate" "$command"
          seen["$command"]=1
        fi
      fi
      command="LOG ${message} ONTIME ${period}"
      if [ -z "${seen[$command]:-}" ]; then
        printf '%s\t%s\n' "nmea_log_ontime" "$command"
        seen["$command"]=1
      fi
      ;;
    *)
      command="LOG ${message} ONTIME ${period}"
      if [ -z "${seen[$command]:-}" ]; then
        printf '%s\t%s\n' "nmea_log_ontime" "$command"
        seen["$command"]=1
      fi
      command="${message} ONTIME ${period}"
      if [ -z "${seen[$command]:-}" ]; then
        printf '%s\t%s\n' "bare_ontime" "$command"
        seen["$command"]=1
      fi
      command="${message} ${period}"
      if [ -z "${seen[$command]:-}" ]; then
        printf '%s\t%s\n' "bare_period" "$command"
        seen["$command"]=1
      fi
      command="${message} ${UNICORE_COM_PORT} ${period}"
      if [ -z "${seen[$command]:-}" ]; then
        printf '%s\t%s\n' "com_period" "$command"
        seen["$command"]=1
      fi
      rate="$(unicore_period_to_rate "$period")"
      if [ "$rate" != "0" ]; then
        command="${message} ${UNICORE_COM_PORT} ${rate}"
        if [ -z "${seen[$command]:-}" ]; then
          printf '%s\t%s\n' "com_rate" "$command"
          seen["$command"]=1
        fi
      fi
      ;;
  esac
}

unicore_classify_command_response() {
  local response="${1-}"
  local command="${2:-$UNICORE_LAST_SERIAL_COMMAND}"
  local line status saw_ignored=0 saw_invalid=0
  local collapsed="${response,,}"
  local compact="${collapsed//[$' \t\r\n']/}"

  if [ -z "$compact" ]; then
    printf '%s\n' "timeout"
    return 0
  fi

  while IFS= read -r line || [ -n "$line" ]; do
    [ -n "$line" ] || continue
    status="$(unicore_classify_command_response_line "$command" "$line")"
    case "$status" in
      ok|unsupported)
        printf '%s\n' "$status"
        return 0
        ;;
      ignored_stream_data)
        saw_ignored=1
        ;;
      invalid_response)
        saw_invalid=1
        ;;
    esac
  done <<<"$response"

  if [ "$saw_invalid" -eq 1 ]; then
    printf '%s\n' "invalid_response"
  elif [ "$saw_ignored" -eq 1 ]; then
    printf '%s\n' "ignored_stream_data"
  else
    printf '%s\n' "timeout"
  fi
}

unicore_first_response_line() {
  local response="${1-}"

  printf '%s\n' "${response%%$'\n'*}"
}

unicore_log_preview() {
  local response="${1-}"
  local preview

  preview="$(unicore_first_response_line "$response")"
  printf '%s' "$preview" | LC_ALL=C tr -cd '\11\12\15\40-\176' | cut -c1-180
}

unicore_log_preview_for_status() {
  local response="${1-}"
  local command="${2:-$UNICORE_LAST_SERIAL_COMMAND}"
  local wanted_status="${3:-}"
  local line status preview=""

  if [ -n "$wanted_status" ]; then
    while IFS= read -r line || [ -n "$line" ]; do
      [ -n "$line" ] || continue
      status="$(unicore_classify_command_response_line "$command" "$line")"
      if [ "$status" = "$wanted_status" ]; then
        preview="$line"
        break
      fi
    done <<<"$response"
  fi

  if [ -z "$preview" ]; then
    preview="$(unicore_first_response_line "$response")"
  fi
  printf '%s' "$preview" | LC_ALL=C tr -cd '\11\12\15\40-\176' | cut -c1-180
}

unicore_normalize_command_for_match() {
  local command="${1-}"

  command="${command//$'\r'/ }"
  command="${command//$'\n'/ }"
  command="${command//,/ }"
  command="$(printf '%s\n' "$command" | awk '{$1=$1; print toupper($0)}')"
  printf '%s\n' "$command"
}

unicore_command_ack_target_matches() {
  local command="${1-}"
  local ack_target="${2-}"
  local normalized_command normalized_ack

  normalized_command="$(unicore_normalize_command_for_match "$command")"
  normalized_ack="$(unicore_normalize_command_for_match "$ack_target")"
  [ -n "$normalized_command" ] || return 1
  [ -n "$normalized_ack" ] || return 1

  if [ "$normalized_ack" = "$normalized_command" ]; then
    return 0
  fi
  case "$normalized_command" in
    "$normalized_ack "*) return 0 ;;
  esac

  return 1
}

unicore_classify_command_response_line() {
  local command="${1-}"
  local line="${2-}"
  local lower="${line,,}"
  local ack_target rest

  if [[ "$lower" == *"response can't found device"* ||
        "$lower" == *"response cant found device"* ||
        "$lower" == *"unsupported"* ||
        "$lower" == *"parsing failed"* ||
        "$lower" == *"grammar error"* ]]; then
    printf '%s\n' "unsupported"
    return 0
  fi

  if [[ "$line" == \$command,* ]]; then
    rest="${line#\$command,}"
    ack_target="${rest%%,response:*}"
    if [[ "$rest" == *",response: OK"* ]] &&
       unicore_command_ack_target_matches "$command" "$ack_target"; then
      printf '%s\n' "ok"
      return 0
    fi
    printf '%s\n' "invalid_response"
    return 0
  fi

  case "$line" in
    "<OK"|"<OK "*|"<OK,"*)
      printf '%s\n' "ok"
      return 0
      ;;
  esac

  if unicore_is_stream_data_line "$line"; then
    printf '%s\n' "ignored_stream_data"
  else
    printf '%s\n' "invalid_response"
  fi
}

unicore_is_stream_data_line() {
  local line="${1-}"
  local printable

  printable="$(printf '%s' "$line" | LC_ALL=C tr -cd '\40-\176')"
  case "$printable" in
    \#BESTNAVA*|\#BESTNAVB*|\#PVTSLNA*|\#PVTSLNB*|\#RTKSTATUSA*|\#RTKSTATUSB*|\#RTCMSTATUSA*|\#RTCMSTATUSB*|\#BESTSATA*|\#BESTSATB*|\#SATSINFOA*|\#SATSINFOB*|\#AGCA*|\#AGCB*|\#HWSTATUSA*|\#HWSTATUSB*|\#JAMSTATUSA*|\#JAMSTATUSB*|\#FREQJAMSTATUSA*|\#FREQJAMSTATUSB*|\#OBSVMCMPA*|\#OBSVMCMPB*)
      return 0
      ;;
    \$GPGGA*|\$GNGGA*|\$GPGSV*|\$GLGSV*|\$GAGSV*|\$GBGSV*|\$GNHPR*|\$GNHPR2*)
      return 0
      ;;
  esac

  if [ "$printable" != "$line" ]; then
    return 0
  fi

  return 1
}

unicore_log_syntax_for_message() {
  local message="${1:?unicore_log_syntax_for_message: missing message}"

  case "$message" in
    GPGGA|PVTSLNA|PVTSLNB|BESTSATA|BESTSATB|SATSINFOA|SATSINFOB|AGCA|AGCB|HWSTATUSA|HWSTATUSB|JAMSTATUSA|JAMSTATUSB|FREQJAMSTATUSA|FREQJAMSTATUSB|OBSVMCMPA|OBSVMCMPB)
      printf '%s\n' "nmea_log_ontime"
      ;;
    BESTNAVA|BESTNAVB|RTKSTATUSA|RTKSTATUSB|GPHPR|GPGSV)
      printf '%s\n' "unicore_direct_period"
      ;;
    RTCMSTATUSA|RTCMSTATUSB|GPHPR2)
      printf '%s\n' "unicore_onchanged"
      ;;
    GPGGAH)
      printf '%s\n' "unicore_direct_period"
      ;;
    *)
      printf '%s\n' "nmea_log_ontime"
      ;;
  esac
}

unicore_is_known_log_message() {
  local message="${1:?unicore_is_known_log_message: missing message}"

  case "$message" in
    GPGGA|GPGGAH|PVTSLNA|PVTSLNB|BESTNAVA|BESTNAVB|GPHPR|GPHPR2|RTKSTATUSA|RTKSTATUSB|RTCMSTATUSA|RTCMSTATUSB|BESTSATA|BESTSATB|SATSINFOA|SATSINFOB|GPGSV|AGCA|AGCB|HWSTATUSA|HWSTATUSB|JAMSTATUSA|JAMSTATUSB|FREQJAMSTATUSA|FREQJAMSTATUSB|OBSVMCMPA|OBSVMCMPB)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

unicore_log_special_output_name() {
  local message="${1:?unicore_log_special_output_name: missing message}"

  case "$message" in
    GPHPR) printf '%s\n' "GNHPR" ;;
    GPHPR2) printf '%s\n' "GNHPR2" ;;
    *) printf '%s\n' "$message" ;;
  esac
}

unicore_format_log_command() {
  local message="${1:?unicore_format_log_command: missing message}"
  local period="${2-}"
  local syntax

  syntax="$(unicore_log_syntax_for_message "$message")"
  case "$syntax" in
    nmea_log_ontime)
      printf '%s\n' "LOG ${message} ONTIME ${period}"
      ;;
    unicore_direct_period)
      printf '%s\n' "${message} ${period}"
      ;;
    unicore_onchanged)
      printf '%s\n' "${message} ONCHANGED"
      ;;
    *)
      printf '%s\n' "LOG ${message} ONTIME ${period}"
      ;;
  esac
}

unicore_emit_ascii_message_log() {
  local message="${1:?unicore_emit_ascii_message_log: missing message}"
  local period="${2-}"

  if unicore_output_has_ascii "$UNICORE_OUTPUT_FORMAT"; then
    unicore_format_log_command "$message" "$period"
  fi
}

unicore_emit_paired_message_log() {
  local ascii_message="${1:?unicore_emit_paired_message_log: missing ascii message}"
  local binary_message="${2:?unicore_emit_paired_message_log: missing binary message}"
  local period="${3-}"

  if unicore_output_has_ascii "$UNICORE_OUTPUT_FORMAT"; then
    unicore_format_log_command "$ascii_message" "$period"
  fi
  if unicore_output_has_binary "$UNICORE_OUTPUT_FORMAT"; then
    unicore_format_log_command "$binary_message" "$period"
  fi
}

unicore_profile_default_binary_consumer() {
  local profile="${1:?unicore_profile_default_binary_consumer: missing profile}"
  local consumer="${2:?unicore_profile_default_binary_consumer: missing consumer}"
  local output_format="${3:-$UNICORE_OUTPUT_FORMAT}"

  if ! unicore_output_has_binary "$output_format"; then
    printf '%s\n' "false"
    return 0
  fi

  case "$(unicore_normalize_profile "$profile"):$consumer" in
    runtime:nav)
      printf '%s\n' "false"
      ;;
    runtime:rtk|runtime:satellite|runtime:rtcm|runtime:rf|runtime:hw|runtime:jamming)
      printf '%s\n' "true"
      ;;
    survey:nav)
      printf '%s\n' "false"
      ;;
    survey:rtk|survey:satellite|survey:rtcm|survey:rf|survey:hw|survey:jamming)
      printf '%s\n' "true"
      ;;
    high_precision:nav|high_precision:rtk|high_precision:satellite|high_precision:rtcm|high_precision:rf|high_precision:hw|high_precision:jamming)
      printf '%s\n' "true"
      ;;
    *)
      printf '%s\n' "false"
      ;;
  esac
}

unicore_profile_default_period() {
  local profile="${1:?unicore_profile_default_period: missing profile}"
  local kind="${2:?unicore_profile_default_period: missing kind}"

  case "$profile:$kind" in
    normal:main) printf '%s\n' "0.2" ;;
    normal:bestnav) printf '%s\n' "0.2" ;;
    normal:diagnostic) printf '%s\n' "1" ;;
    normal:satellite) printf '%s\n' "1" ;;
    normal:rf) printf '%s\n' "1" ;;
    normal:raw) printf '%s\n' "5" ;;
    runtime:main) printf '%s\n' "0.1" ;;
    runtime:bestnav) printf '%s\n' "0.1" ;;
    runtime:diagnostic) printf '%s\n' "1" ;;
    runtime:satellite) printf '%s\n' "2" ;;
    runtime:rf) printf '%s\n' "2" ;;
    runtime:raw) printf '%s\n' "5" ;;
    debug:main) printf '%s\n' "0.2" ;;
    debug:bestnav) printf '%s\n' "0.2" ;;
    debug:diagnostic) printf '%s\n' "1" ;;
    debug:satellite) printf '%s\n' "1" ;;
    debug:rf) printf '%s\n' "1" ;;
    debug:raw) printf '%s\n' "5" ;;
    survey:main) printf '%s\n' "1" ;;
    survey:bestnav) printf '%s\n' "1" ;;
    survey:diagnostic) printf '%s\n' "1" ;;
    survey:satellite) printf '%s\n' "2" ;;
    survey:rf) printf '%s\n' "2" ;;
    survey:raw) printf '%s\n' "5" ;;
    high_precision:main) printf '%s\n' "0.1" ;;
    high_precision:bestnav) printf '%s\n' "0.1" ;;
    high_precision:diagnostic) printf '%s\n' "1" ;;
    high_precision:satellite) printf '%s\n' "1" ;;
    high_precision:rf) printf '%s\n' "1" ;;
    high_precision:raw) printf '%s\n' "5" ;;
    *)
      return 1
      ;;
  esac
}

unicore_profile_default_flag() {
  local profile="${1:?unicore_profile_default_flag: missing profile}"
  local feature="${2:?unicore_profile_default_flag: missing feature}"

  case "$profile:$feature" in
    normal:satellites|normal:rf|normal:jamming|normal:hardware)
      printf '%s\n' "false"
      ;;
    debug:satellites|debug:rf|debug:jamming|debug:hardware)
      printf '%s\n' "true"
      ;;
    survey:satellites|survey:rf|survey:jamming|survey:hardware)
      printf '%s\n' "true"
      ;;
    high_precision:satellites|high_precision:rf|high_precision:jamming|high_precision:hardware)
      printf '%s\n' "true"
      ;;
    runtime:satellites|runtime:rf|runtime:jamming|runtime:hardware)
      printf '%s\n' "true"
      ;;
    *)
      return 1
      ;;
  esac
}

unicore_apply_profile_defaults() {
  UNICORE_PROFILE="$(unicore_normalize_profile "$UNICORE_PROFILE")"
  UNICORE_OUTPUT_FORMAT="$(unicore_normalize_output_format "$UNICORE_OUTPUT_FORMAT")"
  UNICORE_MAIN_LOG_PERIOD="${UNICORE_MAIN_LOG_PERIOD:-$(unicore_profile_default_period "$UNICORE_PROFILE" main)}"
  UNICORE_BESTNAV_LOG_PERIOD="${UNICORE_BESTNAV_LOG_PERIOD:-$(unicore_profile_default_period "$UNICORE_PROFILE" bestnav)}"
  UNICORE_DIAGNOSTIC_LOG_PERIOD="${UNICORE_DIAGNOSTIC_LOG_PERIOD:-$(unicore_profile_default_period "$UNICORE_PROFILE" diagnostic)}"
  UNICORE_SATELLITE_LOG_PERIOD="${UNICORE_SATELLITE_LOG_PERIOD:-$(unicore_profile_default_period "$UNICORE_PROFILE" satellite)}"
  UNICORE_RF_LOG_PERIOD="${UNICORE_RF_LOG_PERIOD:-$(unicore_profile_default_period "$UNICORE_PROFILE" rf)}"
  UNICORE_RAW_LOG_PERIOD="${UNICORE_RAW_LOG_PERIOD:-$(unicore_profile_default_period "$UNICORE_PROFILE" raw)}"
  UNICORE_ENABLE_SATELLITES="${UNICORE_ENABLE_SATELLITES:-$(unicore_profile_default_flag "$UNICORE_PROFILE" satellites)}"
  UNICORE_ENABLE_RF="${UNICORE_ENABLE_RF:-$(unicore_profile_default_flag "$UNICORE_PROFILE" rf)}"
  UNICORE_ENABLE_JAMMING="${UNICORE_ENABLE_JAMMING:-$(unicore_profile_default_flag "$UNICORE_PROFILE" jamming)}"
  UNICORE_ENABLE_HARDWARE="${UNICORE_ENABLE_HARDWARE:-$(unicore_profile_default_flag "$UNICORE_PROFILE" hardware)}"
  UNICORE_ENABLE_RAW_OBSERVATIONS="${UNICORE_ENABLE_RAW_OBSERVATIONS:-false}"
  UNICORE_ENABLE_GGAH="${UNICORE_ENABLE_GGAH:-false}"

  if unicore_is_truthy "$UNICORE_ENABLE_RAW_OBSERVATIONS" &&
     ! unicore_profile_supports_raw "$UNICORE_PROFILE"; then
    unicore_warn "raw observations are only supported in survey/high_precision; disabling for profile=${UNICORE_PROFILE}."
    UNICORE_ENABLE_RAW_OBSERVATIONS="false"
  fi

  if unicore_is_truthy "$UNICORE_ENABLE_RAW_OBSERVATIONS"; then
    local clamped_raw_period
    clamped_raw_period="$(unicore_clamp_min_period "$UNICORE_RAW_LOG_PERIOD" "1")"
    if [ "$clamped_raw_period" != "$UNICORE_RAW_LOG_PERIOD" ]; then
      unicore_warn "raw log period ${UNICORE_RAW_LOG_PERIOD}s is too fast; clamping to ${clamped_raw_period}s."
      UNICORE_RAW_LOG_PERIOD="$clamped_raw_period"
    fi
  fi
}

require_serial_port() {
  local port="${1:?require_serial_port: missing port}"

  if [ ! -c "$port" ]; then
    log "ERROR: $port is not a character device" >&2
    return 1
  fi
}

wait_for_serial_port() {
  local port="${1:?wait_for_serial_port: missing port}"
  local attempt

  for attempt in $(seq 1 20); do
    [ -w "$port" ] && return 0
    sleep 0.1
  done

  log "ERROR: $port is not writable after waiting" >&2
  return 1
}

serial_set_baud() {
  local port="${1:?serial_set_baud: missing port}"
  local baud="${2:?serial_set_baud: missing baud}"

  stty -F "$port" "$baud" raw -echo -echoe -echok -ixon -ixoff || {
    log "ERROR: stty failed on $port @ $baud" >&2
    return 1
  }
}

open_serial() {
  local port="${1:?open_serial: missing port}"

  exec 3<>"$port"
}

close_serial() {
  exec 3>&- || true
}

drain_serial() {
  local line

  while IFS= read -r -t 0.05 -u 3 line; do
    :
  done
}

send_serial_command() {
  local command="${1:?send_serial_command: missing command}"

  UNICORE_LAST_SERIAL_COMMAND="$command"
  printf '%s\r\n' "$command" >&3
}

collect_serial_output() {
  local rounds="${1:?collect_serial_output: missing rounds}"
  local command="${2:-$UNICORE_LAST_SERIAL_COMMAND}"
  local line output=""
  local i status

  for i in $(seq 1 "$rounds"); do
    if IFS= read -r -t 0.15 -u 3 line; then
      output+="${line}"$'\n'
      status="$(unicore_classify_command_response_line "$command" "$line")"
      case "$status" in
        ok|unsupported)
          break
          ;;
      esac
    fi
  done

  printf '%s' "$output"
}

send_serial_command_with_response() {
  local command="${1:?send_serial_command_with_response: missing command}"

  drain_serial
  send_serial_command "$command"
  log "  -> $command"
  sleep 0.1
  UNICORE_LAST_COMMAND_RESPONSE="$(collect_serial_output 12 "$command")"
}

send_log_command_with_fallback() {
  local canonical_command="${1:?send_log_command_with_fallback: missing command}"
  local parsed message syntax period
  local -a variants=()
  local variant label candidate response status
  local rejected=""
  local preview=""

  parsed="$(unicore_parse_log_command "$canonical_command" || true)"
  if [ -z "$parsed" ]; then
    send_serial_command_with_response "$canonical_command"
    response="$UNICORE_LAST_COMMAND_RESPONSE"
    status="$(unicore_classify_command_response "$response" "$canonical_command")"
    preview="$(unicore_log_preview_for_status "$response" "$canonical_command" "$status")"
    [ -n "$preview" ] && log "  <- ${status}: ${preview}"
    return 0
  fi

  message="${parsed%%|*}"
  parsed="${parsed#*|}"
  syntax="${parsed%%|*}"
  period="${parsed#*|}"
  mapfile -t variants < <(unicore_log_command_variants "$message" "$syntax" "$period")

  for variant in "${variants[@]}"; do
    label="${variant%%$'\t'*}"
    candidate="${variant#*$'\t'}"
    send_serial_command_with_response "$candidate"
    response="$UNICORE_LAST_COMMAND_RESPONSE"
    status="$(unicore_classify_command_response "$response" "$candidate")"
    preview="$(unicore_log_preview_for_status "$response" "$candidate" "$status")"

    if [ "$status" != "ok" ]; then
      if [ -n "$rejected" ]; then
        rejected+=" | "
      fi
      rejected+="$candidate"
      log "  <- ${status} via ${label}${preview:+: ${preview}}"
      continue
    fi

    UNICORE_ACCEPTED_LOG_COMMANDS["$message"]="$candidate"
    UNICORE_LOG_COMMAND_SYNTAX_BY_MESSAGE["$message"]="$label"
    UNICORE_REJECTED_LOG_COMMANDS["$message"]="$rejected"
    if [ "$candidate" != "$canonical_command" ]; then
      log "  <- ok accepted ${message} via ${label}: ${candidate}${preview:+ | ${preview}}"
    else
      log "  <- ok accepted ${message} via ${label}${preview:+: ${preview}}"
    fi
    return 0
  done

  UNICORE_REJECTED_LOG_COMMANDS["$message"]="$rejected"
  log "WARN: no accepted LOG syntax found for ${message}; attempted: ${rejected:-$canonical_command}"
  return 1
}

query_receiver_identification() {
  local response=""

  drain_serial

  # `VERSION` is the documented query command; `VERSIONA` is a compatible
  # fallback on receivers that accept the log/message name directly.
  #
  # Use a generous line budget: a receiver that is already streaming
  # (binary/NMEA) at the probe baud can flood the buffer with many lines
  # before the #VERSIONA reply arrives, and a 10-line window was small
  # enough to miss it — which surfaced as "model could not be identified"
  # and skipped the entire config on otherwise-healthy UM98x units.
  send_serial_command "VERSION"
  sleep 0.1
  response="$(collect_serial_output 30)"

  if [[ "$response" != *"UM980"* && "$response" != *"UM981"* &&
        "$response" != *"UM982"* && "$response" != *"#VERSIONA"* ]]; then
    send_serial_command "VERSIONA"
    sleep 0.1
    response+=$(collect_serial_output 30)
  fi

  printf '%s' "$response"
}

model_from_response() {
  local response="${1:-}"

  if [[ "$response" == *"UM980"* ]]; then
    printf '%s\n' "UM980"
  elif [[ "$response" == *"UM981"* ]]; then
    printf '%s\n' "UM981"
  elif [[ "$response" == *"UM982"* ]]; then
    printf '%s\n' "UM982"
  else
    printf '%s\n' "unknown"
  fi
}

detect_receiver_at_baud() {
  local port="${1:?detect_receiver_at_baud: missing port}"
  local baud="${2:?detect_receiver_at_baud: missing baud}"
  local response model

  serial_set_baud "$port" "$baud" || return 1
  open_serial "$port"
  response="$(query_receiver_identification)"
  close_serial

  if [ -z "$response" ]; then
    return 1
  fi

  model="$(model_from_response "$response")"
  printf '%s|%s\n' "$baud" "$model"
}

detect_baud_candidates() {
  local preferred="${1:-}"
  local seen=" "
  local candidate
  local ordered=()

  for candidate in "$preferred" "$TARGET_BAUD" 460800 115200 230400 57600 38400 9600; do
    [ -n "$candidate" ] || continue
    case "$seen" in
      *" $candidate "*) continue ;;
    esac
    ordered+=("$candidate")
    seen+="$(printf '%s ' "$candidate")"
  done

  printf '%s\n' "${ordered[@]}"
}

detect_receiver_baud_and_model() {
  local port="${1:?detect_receiver_baud_and_model: missing port}"
  local preferred_baud="${2:-}"
  local candidate result baud model

  while IFS= read -r candidate; do
    [ -n "$candidate" ] || continue
    log "Probing receiver on ${port} @ ${candidate}..."

    if result="$(detect_receiver_at_baud "$port" "$candidate")"; then
      baud="${result%%|*}"
      model="${result#*|}"
      log "Receiver responded on ${baud}."
      printf '%s|%s\n' "$baud" "$model"
      return 0
    fi
  done < <(detect_baud_candidates "$preferred_baud")

  return 1
}

signalgroup_for_model() {
  local model="${1:?signalgroup_for_model: missing model}"

  if [ -n "$UNICORE_SIGNALGROUP_OVERRIDE" ]; then
    printf '%s\n' "$UNICORE_SIGNALGROUP_OVERRIDE"
    return 0
  fi

  case "$model" in
    UM980) printf '%s\n' "CONFIG SIGNALGROUP 2" ;;
    UM981|UM982) printf '%s\n' "CONFIG SIGNALGROUP 3 6" ;;
    *) return 1 ;;
  esac
}

build_base_config_commands() {
  local model="${1:?build_base_config_commands: missing model}"
  local signalgroup=""

  # SIGNALGROUP is the ONLY model-specific command. When the model can't be
  # identified, fall through with an empty signalgroup so the rest of the
  # (model-independent) rover config still applies — matching the Python
  # reference (tools/unicore_live_validate.py). Gating the whole batch on
  # model detection was the regression that left unidentified UM98x
  # receivers in their default, non-RTK state with every diagnostic stale.
  if ! signalgroup="$(signalgroup_for_model "$model")"; then
    signalgroup=""
    unicore_warn "receiver model '${model}' is unknown or unmapped; skipping SIGNALGROUP (set UNICORE_SIGNALGROUP_OVERRIDE to force one, e.g. 'CONFIG SIGNALGROUP 2' for UM980)."
  fi

  printf '%s\n' \
    "MODE ROVER SURVEY MOW" \
    "CONFIG NMEAVERSION V411" \
    "CONFIG RTK TIMEOUT 10" \
    "CONFIG RTK RELIABILITY 3 1" \
    "CONFIG DGPS TIMEOUT 600" \
    "CONFIG UNDULATION AUTO"
  if [ -n "$signalgroup" ]; then
    printf '%s\n' "${signalgroup}"
  fi
  printf '%s\n' \
    "CONFIG SBAS DISABLE" \
    "CONFIG AGNSS DISABLE" \
    "CONFIG PPS DISABLE" \
    "MASK 10" \
    "UNMASK GPS" \
    "UNMASK GLO" \
    "UNMASK GAL" \
    "UNMASK BDS" \
    "MASK QZSS" \
    "MASK IRNSS"
}

build_log_commands() {
  unicore_emit_ascii_message_log "GPGGA" "${UNICORE_MAIN_LOG_PERIOD}"
  if unicore_is_truthy "${UNICORE_ENABLE_GGAH:-false}"; then
    unicore_emit_ascii_message_log "GPGGAH" "${UNICORE_MAIN_LOG_PERIOD}"
  fi
  unicore_emit_paired_message_log "PVTSLNA" "PVTSLNB" "${UNICORE_MAIN_LOG_PERIOD}"
  unicore_emit_paired_message_log "BESTNAVA" "BESTNAVB" "${UNICORE_BESTNAV_LOG_PERIOD}"
  unicore_emit_ascii_message_log "GPHPR" "${UNICORE_MAIN_LOG_PERIOD}"
  unicore_emit_ascii_message_log "GPHPR2"
  unicore_emit_paired_message_log "RTKSTATUSA" "RTKSTATUSB" "${UNICORE_DIAGNOSTIC_LOG_PERIOD}"
  unicore_emit_paired_message_log "RTCMSTATUSA" "RTCMSTATUSB" "${UNICORE_DIAGNOSTIC_LOG_PERIOD}"

  if unicore_is_truthy "$UNICORE_ENABLE_SATELLITES"; then
    unicore_emit_paired_message_log "BESTSATA" "BESTSATB" "${UNICORE_SATELLITE_LOG_PERIOD}"
    unicore_emit_paired_message_log "SATSINFOA" "SATSINFOB" "${UNICORE_SATELLITE_LOG_PERIOD}"
    unicore_emit_ascii_message_log "GPGSV" "${UNICORE_SATELLITE_LOG_PERIOD}"
  fi

  if unicore_is_truthy "$UNICORE_ENABLE_RF"; then
    unicore_emit_paired_message_log "AGCA" "AGCB" "${UNICORE_RF_LOG_PERIOD}"
  fi

  if unicore_is_truthy "$UNICORE_ENABLE_HARDWARE"; then
    unicore_emit_paired_message_log "HWSTATUSA" "HWSTATUSB" "${UNICORE_RF_LOG_PERIOD}"
  fi

  if unicore_is_truthy "$UNICORE_ENABLE_JAMMING"; then
    unicore_emit_paired_message_log "JAMSTATUSA" "JAMSTATUSB" "${UNICORE_RF_LOG_PERIOD}"
    unicore_emit_paired_message_log "FREQJAMSTATUSA" "FREQJAMSTATUSB" "${UNICORE_RF_LOG_PERIOD}"
  fi

  if unicore_is_truthy "$UNICORE_ENABLE_RAW_OBSERVATIONS" &&
     { [ "$UNICORE_PROFILE" = "survey" ] || [ "$UNICORE_PROFILE" = "high_precision" ]; }; then
    if unicore_output_has_ascii "$UNICORE_OUTPUT_FORMAT"; then
      # ASCII OBSVMCMP remains useful for offline captures even when the ROS
      # raw summary is driven by the binary backend in hybrid mode.
      printf '%s\n' "LOG OBSVMCMPA ONTIME ${UNICORE_RAW_LOG_PERIOD}"
    fi
    if unicore_output_has_binary "$UNICORE_OUTPUT_FORMAT"; then
      printf '%s\n' "LOG OBSVMCMPB ONTIME ${UNICORE_RAW_LOG_PERIOD}"
    fi
  fi
}

build_profile_commands() {
  case "$UNICORE_PROFILE" in
    high_precision)
      # These commands are documented in N4 R1.4 but may still depend on
      # firmware generation. Keep them profile-scoped rather than default.
      printf '%s\n' "CONFIG PVTALG MULTI"
      printf '%s\n' "CONFIG RTCMDECAUTO ENABLE"
      printf '%s\n' "CONFIG RTCMPHASERATE POSITIVE"
      printf '%s\n' "CONFIG RTCMCLOCKOFFSET ENABLE"
      ;;
    *)
      ;;
  esac
}

verify_receiver_at_baud() {
  local port="${1:?verify_receiver_at_baud: missing port}"
  local baud="${2:?verify_receiver_at_baud: missing baud}"
  local response

  serial_set_baud "$port" "$baud" || return 1
  open_serial "$port"
  response="$(query_receiver_identification)"
  close_serial

  [ -n "$response" ]
}

send_config_batch() {
  local port="${1:?send_config_batch: missing port}"
  shift
  local commands=("$@")
  local command
  local response
  local status
  local preview
  local batch_rc=0

  serial_set_baud "$port" "$TARGET_BAUD"
  open_serial "$port"
  drain_serial

  for command in "${commands[@]}"; do
    if unicore_parse_log_command "$command" >/dev/null 2>&1; then
      if ! send_log_command_with_fallback "$command"; then
        batch_rc=1
      fi
      continue
    fi

    send_serial_command_with_response "$command"
    response="$UNICORE_LAST_COMMAND_RESPONSE"
    status="$(unicore_classify_command_response "$response" "$command")"
    preview="$(unicore_log_preview_for_status "$response" "$command" "$status")"
    [ -n "$preview" ] && log "  <- ${status}: ${preview}"
  done

  close_serial
  return "$batch_rc"
}

apply_receiver_configuration() {
  local port="${1:?apply_receiver_configuration: missing port}"
  local detected_baud="${2:?apply_receiver_configuration: missing detected baud}"
  local model="${3:?apply_receiver_configuration: missing model}"
  local cmds=()
  local base_cmds=()
  local log_cmds=()
  local profile_cmds=()

  unicore_apply_profile_defaults
  UNICORE_ACCEPTED_LOG_COMMANDS=()
  UNICORE_REJECTED_LOG_COMMANDS=()
  UNICORE_LOG_COMMAND_SYNTAX_BY_MESSAGE=()

  if [ "$detected_baud" != "$TARGET_BAUD" ]; then
    log "Switching ${UNICORE_COM_PORT} from ${detected_baud} to ${TARGET_BAUD}..."
    serial_set_baud "$port" "$detected_baud"
    open_serial "$port"
    send_serial_command "CONFIG ${UNICORE_COM_PORT} ${TARGET_BAUD}"
    close_serial
    sleep 0.5
  else
    log "Receiver already running at ${TARGET_BAUD}."
  fi

  if ! verify_receiver_at_baud "$port" "$TARGET_BAUD"; then
    log "ERROR: receiver did not answer after switch to ${TARGET_BAUD}." >&2
    return 1
  fi

  log "Receiver verified at ${TARGET_BAUD}; detected model ${model}."

  mapfile -t base_cmds < <(build_base_config_commands "$model")
  mapfile -t log_cmds < <(build_log_commands)
  mapfile -t profile_cmds < <(build_profile_commands)

  cmds=( "${base_cmds[@]}" )
  # Drop previously persisted output schedule on the current port so a
  # transition from debug/survey back to normal actually becomes lighter.
  cmds+=( "UNLOG" )
  cmds+=( "${profile_cmds[@]}" )
  cmds+=( "${log_cmds[@]}" )

  log "Applying UM98x rover config to ${port} @ ${TARGET_BAUD}..."
  log "Profile=${UNICORE_PROFILE} output=${UNICORE_OUTPUT_FORMAT} main=${UNICORE_MAIN_LOG_PERIOD}s bestnav=${UNICORE_BESTNAV_LOG_PERIOD}s diag=${UNICORE_DIAGNOSTIC_LOG_PERIOD}s sat=${UNICORE_SATELLITE_LOG_PERIOD}s rf=${UNICORE_RF_LOG_PERIOD}s raw=${UNICORE_RAW_LOG_PERIOD}s sat_enabled=$(unicore_bool_string "$UNICORE_ENABLE_SATELLITES") rf_enabled=$(unicore_bool_string "$UNICORE_ENABLE_RF") jam_enabled=$(unicore_bool_string "$UNICORE_ENABLE_JAMMING") raw_enabled=$(unicore_bool_string "$UNICORE_ENABLE_RAW_OBSERVATIONS")"
  if ! send_config_batch "$port" "${cmds[@]}"; then
    log "WARN: one or more LOG commands could not find an accepted syntax on this firmware."
  fi

  if [ "${#UNICORE_ACCEPTED_LOG_COMMANDS[@]}" -gt 0 ]; then
    log "Accepted LOG syntax by message:"
    for command in "${!UNICORE_ACCEPTED_LOG_COMMANDS[@]}"; do
      log "  ${command}: ${UNICORE_ACCEPTED_LOG_COMMANDS[$command]} (via ${UNICORE_LOG_COMMAND_SYNTAX_BY_MESSAGE[$command]:-unknown})"
    done
  fi

  if ! verify_receiver_at_baud "$port" "$TARGET_BAUD"; then
    log "ERROR: receiver stopped responding at ${TARGET_BAUD} before SAVECONFIG." >&2
    return 1
  fi

  send_config_batch "$port" "SAVECONFIG"
  log "Done. Receiver config persisted via SAVECONFIG."
}

main() {
  local port="${1:-/dev/gps}"
  local preferred_baud="${2:-$TARGET_BAUD}"
  local detection detected_baud detected_model

  require_serial_port "$port"
  wait_for_serial_port "$port"

  detection="$(detect_receiver_baud_and_model "$port" "$preferred_baud")" || {
    log "ERROR: unable to detect a responding Unicore receiver on ${port}." >&2
    return 1
  }

  detected_baud="${detection%%|*}"
  detected_model="${detection#*|}"

  if [ "$detected_model" = "unknown" ]; then
    log "WARN: receiver model could not be identified from VERSION response."
    if [ -n "$UNICORE_SIGNALGROUP_OVERRIDE" ]; then
      log "WARN: proceeding with config; SIGNALGROUP forced via UNICORE_SIGNALGROUP_OVERRIDE='${UNICORE_SIGNALGROUP_OVERRIDE}'."
    else
      log "WARN: proceeding with model-independent rover config; SIGNALGROUP skipped (set UNICORE_SIGNALGROUP_OVERRIDE, e.g. 'CONFIG SIGNALGROUP 2' for UM980, to force one)."
    fi
  fi

  apply_receiver_configuration "$port" "$detected_baud" "$detected_model"
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
