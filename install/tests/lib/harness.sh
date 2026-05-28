#!/usr/bin/env bash
# =============================================================================
# Non-interactive driver for install/mowglinext.sh
#
# The installer's main() runs interactive prompts via /dev/tty (prompt(),
# confirm(), select_language, select_hardware_backend). We can't easily
# pipe answers in, so we bypass main() and replay the same data-flow
# functions in order with environment variables pre-populated as if the
# user had picked them. This mirrors the contract that the bootstrap
# (docs/install.sh) relies on: write a .preset file, set PRESET_LOADED=true,
# and let the lib/*.sh helpers self-skip their prompts.
#
# Usage:
#   source install/tests/lib/harness.sh
#   harness_init "$SANDBOX_REPO"   # absolute path to a sandboxed checkout
#   harness_set_preset gps=ubx-uart lidar=ldlidar-uart tfluna=none
#   harness_run                    # generate .env + docker-compose.yaml +
#                                  # mowgli_robot.yaml; no docker pull
# =============================================================================

# shellcheck disable=SC2034  # variables consumed by sourced installer libs

harness_init() {
  local repo_dir="${1:?harness_init: pass the absolute sandbox repo dir}"

  # config.sh hard-assigns REPO_DIR="${MOWGLI_HOME:-$HOME/mowglinext}",
  # so set MOWGLI_HOME (the documented override) instead of REPO_DIR
  # before sourcing it.
  export MOWGLI_HOME="$repo_dir"

  # Clear any state leaked from a previous harness_init in the same
  # shell so the matrix tests in test_gps_matrix.sh / test_lidar_matrix.sh
  # don't carry GPS_UART_DEVICE etc. from one preset into the next.
  unset GNSS_BACKEND GPS_CONNECTION GPS_PROTOCOL GPS_PORT GPS_BAUD \
        GPS_UART_DEVICE GPS_BY_ID GPS_DEBUG_ENABLED GPS_DEBUG_PORT \
        GPS_DEBUG_UART_DEVICE GPS_DEBUG_BAUD \
        UNICORE_COM_PORT UNICORE_TARGET_BAUD \
        UBLOX_DEVICE_FAMILY UBLOX_DEVICE_SERIAL_STRING \
        LIDAR_ENABLED LIDAR_TYPE LIDAR_MODEL LIDAR_CONNECTION \
        LIDAR_PORT LIDAR_UART_DEVICE LIDAR_BAUD LIDAR_IMAGE \
        MOWGLI_ROS2_IMAGE GPS_IMAGE UNICORE_IMAGE MAVROS_IMAGE GUI_IMAGE \
        TFLUNA_FRONT_ENABLED TFLUNA_FRONT_PORT TFLUNA_FRONT_UART_DEVICE \
        TFLUNA_FRONT_BAUD TFLUNA_EDGE_ENABLED TFLUNA_EDGE_PORT \
        TFLUNA_EDGE_UART_DEVICE TFLUNA_EDGE_BAUD \
        HARDWARE_BACKEND MAVROS_BY_ID MAVROS_PORT MAVROS_BAUD \
        MAVROS_GCS_URL MAVROS_TGT_SYSTEM MAVROS_TGT_COMPONENT \
        MAVROS_AUTOPILOT MAVROS_ENABLED \
        GPS_UART_RULE GPS_DEBUG_UART_RULE LIDAR_UART_RULE \
        TFLUNA_FRONT_UART_RULE TFLUNA_EDGE_UART_RULE \
        PRESET_LOADED CLI_PRESET STATE_ACTIVE_PRESET_FILE \
        STATE_ACTIVE_PRESET_COUNT 2>/dev/null || true

  # Force English to keep assertions deterministic across hosts.
  export MOWGLI_LANG=en

  # Bootstrap path. config.sh will (re)set INSTALL_DIR from MOWGLI_HOME,
  # but we need to source common.sh first, before config.sh exists in
  # scope, so use a literal path derived from MOWGLI_HOME.
  local lib_dir="$MOWGLI_HOME/install/lib"

  # Source all installer libs, just like the main script does.
  # shellcheck source=/dev/null
  source "$lib_dir/common.sh"
  # shellcheck source=/dev/null
  source "$lib_dir/i18n.sh"
  # shellcheck source=/dev/null
  source "$lib_dir/config.sh"
  # shellcheck source=/dev/null
  source "$lib_dir/state.sh"
  # shellcheck source=/dev/null
  source "$lib_dir/banner.sh"
  # shellcheck source=/dev/null
  source "$lib_dir/progress.sh"
  # shellcheck source=/dev/null
  source "$lib_dir/system.sh"
  # shellcheck source=/dev/null
  source "$lib_dir/docker.sh"
  # shellcheck source=/dev/null
  source "$lib_dir/backend_choice.sh"
  # shellcheck source=/dev/null
  source "$lib_dir/udev.sh"
  # shellcheck source=/dev/null
  source "$lib_dir/deploy.sh"
  # shellcheck source=/dev/null
  source "$lib_dir/env.sh"
  # shellcheck source=/dev/null
  source "$lib_dir/serial_probe.sh"
  # shellcheck source=/dev/null
  source "$lib_dir/unicore_config.sh"
  # shellcheck source=/dev/null
  source "$lib_dir/ublox_config.sh"
  # shellcheck source=/dev/null
  source "$lib_dir/gps.sh"
  # shellcheck source=/dev/null
  source "$lib_dir/lidar.sh"
  # shellcheck source=/dev/null
  source "$lib_dir/range.sh"
  # shellcheck source=/dev/null
  source "$lib_dir/uart.sh"
  # shellcheck source=/dev/null
  source "$lib_dir/rc_local.sh"
  # shellcheck source=/dev/null
  source "$lib_dir/compose.sh"

  # config.sh sets REPO_DIR/INSTALL_DIR/DOCKER_DIR/etc from MOWGLI_HOME.
  # Re-export them so child processes (the docker mock) see the right paths.
  export REPO_DIR INSTALL_DIR DOCKER_DIR COMPOSE_SRC_DIR
  export FINAL_COMPOSE_FILE FINAL_ENV_FILE

  load_locale

  # The installer's lib/common.sh redefined pass() and fail() with output-
  # only versions that don't increment TESTS_PASSED/TESTS_FAILED. Reapply
  # the test framework's counter-aware versions on top.
  if declare -f reapply_test_assertions >/dev/null 2>&1; then
    reapply_test_assertions
  fi

  # Keep prompts from being triggered if a function path slips through.
  prompt() { REPLY="${2:-}"; }
  confirm() { return 1; }   # default: answer "no"
  pick_uart_port() { REPLY="${1:-/dev/ttyAMA4}"; }
  pick_serial_by_id() { REPLY="${1:-${GPS_BY_ID:-/dev/serial/by-id/usb-stub}}"; }
  step() { :; }
  export -f prompt confirm pick_uart_port pick_serial_by_id step

  # Defaults that interactive_config would otherwise prompt for.
  CONFIG_DATUM_LAT="0.0"
  CONFIG_DATUM_LON="0.0"
  CONFIG_NTRIP_ENABLED="false"
  CONFIG_NTRIP_HOST="crtk.net"
  CONFIG_NTRIP_PORT="2101"
  CONFIG_NTRIP_USER="centipede"
  CONFIG_NTRIP_PASSWORD="centipede"
  CONFIG_NTRIP_MOUNTPOINT="NEAR"
  CONFIG_LIDAR_X="0.20"
  CONFIG_LIDAR_Y="0.0"
  CONFIG_LIDAR_Z="0.22"
  CONFIG_LIDAR_YAW="0.0"
  CONFIG_DOCK_X="0.0"
  CONFIG_DOCK_Y="0.0"
  CONFIG_DOCK_YAW="0.0"
  SKIP_WRITE_CONFIG=false

  # Defaults for backend selection — overridden by harness_set_preset().
  HARDWARE_BACKEND="${HARDWARE_BACKEND:-mowgli}"
  GNSS_BACKEND="${GNSS_BACKEND:-gps}"
  GPS_CONNECTION="${GPS_CONNECTION:-uart}"
  GPS_PROTOCOL="${GPS_PROTOCOL:-UBX}"
  GPS_BAUD="${GPS_BAUD:-921600}"
  GPS_UART_DEVICE="${GPS_UART_DEVICE:-/dev/ttyAMA4}"
  UBLOX_DEVICE_FAMILY="${UBLOX_DEVICE_FAMILY:-F9P}"
  GPS_DEBUG_ENABLED="${GPS_DEBUG_ENABLED:-false}"
  LIDAR_ENABLED="${LIDAR_ENABLED:-true}"
  LIDAR_TYPE="${LIDAR_TYPE:-ldlidar}"
  LIDAR_MODEL="${LIDAR_MODEL:-LDLiDAR_LD19}"
  LIDAR_CONNECTION="${LIDAR_CONNECTION:-uart}"
  LIDAR_UART_DEVICE="${LIDAR_UART_DEVICE:-/dev/ttyAMA5}"
  LIDAR_BAUD="${LIDAR_BAUD:-230400}"
  TFLUNA_FRONT_ENABLED="${TFLUNA_FRONT_ENABLED:-false}"
  TFLUNA_EDGE_ENABLED="${TFLUNA_EDGE_ENABLED:-false}"

  PRESET_LOADED=true
}

# Apply a key=value list of presets, mimicking what install/lib/config.sh
# parse_args does for --gps / --lidar / --tfluna flags.
harness_set_preset() {
  local kv proto conn
  for kv in "$@"; do
    local key="${kv%%=*}" val="${kv#*=}"
    case "$key" in
      backend)
        HARDWARE_BACKEND="$val"
        if [ "$val" = "mavros" ]; then
          # NOTE: env.sh::setup_env() flips GNSS_BACKEND to "disabled" when
          # HARDWARE_BACKEND=mavros — do NOT pre-set it here, otherwise
          # configure_gps's preset validator rejects "disabled".
          export MAVROS_BY_ID="${MAVROS_BY_ID:-/dev/serial/by-id/usb-Pixhawk-stub}"
          export MAVROS_PORT="${MAVROS_PORT:-/dev/mavros}"
          export MAVROS_BAUD="${MAVROS_BAUD:-921600}"
        fi
        ;;
      gnss)
        GNSS_BACKEND="$val"
        if [ "$val" = "ublox" ]; then
          GPS_CONNECTION="usb"
          GPS_PROTOCOL="UBX"
          GPS_UART_DEVICE=""
          GPS_BY_ID="${GPS_BY_ID:-/dev/serial/by-id/ublox-test-serial}"
          GPS_PORT="${GPS_BY_ID}"
          UBLOX_DEVICE_SERIAL_STRING="${UBLOX_DEVICE_SERIAL_STRING:-}"
        fi
        ;;
      gps)
        proto="${val%%-*}"; conn="${val##*-}"
        case "$proto" in
          ubx)  GPS_PROTOCOL="UBX"; unset GPS_BAUD ;;
          nmea) GPS_PROTOCOL="NMEA"; unset GPS_BAUD ;;
        esac
        case "$conn" in
          usb)  GPS_CONNECTION="usb";  GPS_UART_DEVICE="" ;;
          uart) GPS_CONNECTION="uart"; GPS_UART_DEVICE="${GPS_UART_DEVICE:-/dev/ttyAMA4}" ;;
        esac
        ;;
      lidar)
        case "$val" in
          none)
            LIDAR_ENABLED=false; LIDAR_TYPE=none; LIDAR_MODEL=""
            LIDAR_CONNECTION=""; LIDAR_UART_DEVICE=""
            ;;
          rplidar-usb)
            LIDAR_ENABLED=true; LIDAR_TYPE=rplidar; LIDAR_MODEL=RPLIDAR_A1
            LIDAR_CONNECTION=usb; LIDAR_UART_DEVICE=""; LIDAR_BAUD=115200
            ;;
          rplidar-uart)
            LIDAR_ENABLED=true; LIDAR_TYPE=rplidar; LIDAR_MODEL=RPLIDAR_A1
            LIDAR_CONNECTION=uart; LIDAR_UART_DEVICE=/dev/ttyAMA5; LIDAR_BAUD=115200
            ;;
          ldlidar-usb)
            LIDAR_ENABLED=true; LIDAR_TYPE=ldlidar; LIDAR_MODEL=LDLiDAR_LD19
            LIDAR_CONNECTION=usb; LIDAR_UART_DEVICE=""; LIDAR_BAUD=230400
            ;;
          ldlidar-uart)
            LIDAR_ENABLED=true; LIDAR_TYPE=ldlidar; LIDAR_MODEL=LDLiDAR_LD19
            LIDAR_CONNECTION=uart; LIDAR_UART_DEVICE=/dev/ttyAMA5; LIDAR_BAUD=230400
            ;;
          stl27l-usb)
            LIDAR_ENABLED=true; LIDAR_TYPE=stl27l; LIDAR_MODEL=STL27L
            LIDAR_CONNECTION=usb; LIDAR_UART_DEVICE=""; LIDAR_BAUD=230400
            ;;
          stl27l-uart)
            LIDAR_ENABLED=true; LIDAR_TYPE=stl27l; LIDAR_MODEL=STL27L
            LIDAR_CONNECTION=uart; LIDAR_UART_DEVICE=/dev/ttyAMA5; LIDAR_BAUD=230400
            ;;
        esac
        ;;
      tfluna)
        case "$val" in
          none)  TFLUNA_FRONT_ENABLED=false; TFLUNA_EDGE_ENABLED=false ;;
          front) TFLUNA_FRONT_ENABLED=true;  TFLUNA_EDGE_ENABLED=false ;;
          edge)  TFLUNA_FRONT_ENABLED=false; TFLUNA_EDGE_ENABLED=true  ;;
          both)  TFLUNA_FRONT_ENABLED=true;  TFLUNA_EDGE_ENABLED=true  ;;
        esac
        ;;
      datum_lat) CONFIG_DATUM_LAT="$val" ;;
      datum_lon) CONFIG_DATUM_LON="$val" ;;
      ntrip)     CONFIG_NTRIP_ENABLED="$val" ;;
    esac
  done
}

# Run the same data-flow steps that mowglinext.sh main() runs, minus the
# steps that touch the host (system update, docker install, UART overlays,
# udev install, systemd) and minus the live `docker compose pull/up`.
harness_run() {
  configure_gps   >/dev/null 2>&1 || return 1
  configure_lidar >/dev/null 2>&1 || return 1
  configure_rangefinders >/dev/null 2>&1 || return 1

  migrate_runtime_paths >/dev/null 2>&1 || true
  setup_env >/dev/null 2>&1 || return 1

  # Materialise the merged compose file (uses real `docker compose config`
  # via the mock that forwards `config` calls to the host docker).
  ensure_default_configs >/dev/null 2>&1 || true
  build_compose_stack    >/dev/null 2>&1 || return 1
  write_compose_merged   >/dev/null 2>&1 || return 1

  # Always write the robot YAML (interactive_config is skipped).
  write_config >/dev/null 2>&1 || return 1
}
