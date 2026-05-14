#!/usr/bin/env bash
# =============================================================================
# A.2 Output sanity — .env file shape
#
# After a clean run, the generated docker/.env must contain the union of
# keys that env.sh::setup_env writes (plus any image overrides). If any
# of these keys go missing, the docker-compose ${VAR} expansions silently
# fall back to nothing and the stack starts with broken parameters.
# =============================================================================

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=lib/framework.sh
source "$SCRIPT_DIR/lib/framework.sh"
# shellcheck source=lib/mocks.sh
source "$SCRIPT_DIR/lib/mocks.sh"
# shellcheck source=lib/harness.sh
source "$SCRIPT_DIR/lib/harness.sh"

setup_sandbox
install_all_mocks

SANDBOX_REPO="$SANDBOX/repo"
sandbox_repo "$SANDBOX_REPO"
harness_init "$SANDBOX_REPO"
harness_set_preset gps=ubx-uart lidar=ldlidar-uart tfluna=none

if ! harness_run; then
  fail "harness_run for default preset" "non-zero exit"
  test_summary
  exit 1
fi

ENV_FILE="$SANDBOX_REPO/docker/.env"

section ".env contains all required keys"

# Keys env.sh::setup_env() unconditionally writes — see install/lib/env.sh
REQUIRED_KEYS=(
  ROS_DOMAIN_ID
  MOWER_IP
  DISABLE_BLUETOOTH
  ENABLE_FOXGLOVE
  GNSS_BACKEND
  GPS_CONNECTION
  GPS_PROTOCOL
  GPS_PORT
  GPS_BAUD
  GPS_BY_ID
  GPS_UART_DEVICE
  UBLOX_DEVICE_FAMILY
  UBLOX_DEVICE_SERIAL_STRING
  GNSS_ENABLE_FIX_DIAGNOSTICS
  GPS_DEBUG_ENABLED
  GPS_DEBUG_PORT
  GPS_DEBUG_UART_DEVICE
  GPS_DEBUG_BAUD
  UNICORE_COM_PORT
  UNICORE_PROFILE
  UNICORE_OUTPUT_FORMAT
  UNICORE_TARGET_BAUD
  UNICORE_SIGNALGROUP_OVERRIDE
  UNICORE_MAIN_LOG_PERIOD
  UNICORE_BESTNAV_LOG_PERIOD
  UNICORE_DIAGNOSTIC_LOG_PERIOD
  UNICORE_SATELLITE_LOG_PERIOD
  UNICORE_RF_LOG_PERIOD
  UNICORE_RAW_LOG_PERIOD
  UNICORE_ENABLE_SATELLITES
  UNICORE_ENABLE_RF
  UNICORE_ENABLE_JAMMING
  UNICORE_ENABLE_HARDWARE
  UNICORE_ENABLE_GGAH
  UNICORE_ENABLE_RAW_OBSERVATIONS
  UNICORE_ENABLE_UNICORE_BINARY
  UNICORE_USE_BINARY_NAV
  UNICORE_USE_BINARY_SATELLITE_DIAG
  UNICORE_USE_BINARY_RTCM_DIAG
  UNICORE_USE_BINARY_RTK_DIAG
  UNICORE_USE_BINARY_RF_DIAG
  UNICORE_USE_BINARY_HW_DIAG
  UNICORE_USE_BINARY_JAMMING_DIAG
  UNICORE_ENABLE_RAW_OBSERVATION_DIAG
  UNICORE_USE_BINARY_RAW_OBSERVATIONS
  UNICORE_ROS_PACKAGE
  UNICORE_ROS_EXECUTABLE
  LIDAR_ENABLED
  LIDAR_TYPE
  LIDAR_MODEL
  LIDAR_CONNECTION
  LIDAR_PORT
  LIDAR_UART_DEVICE
  LIDAR_BAUD
  TFLUNA_FRONT_ENABLED
  TFLUNA_FRONT_PORT
  TFLUNA_FRONT_UART_DEVICE
  TFLUNA_FRONT_BAUD
  TFLUNA_EDGE_ENABLED
  TFLUNA_EDGE_PORT
  TFLUNA_EDGE_UART_DEVICE
  TFLUNA_EDGE_BAUD
  MOWGLI_ROS2_IMAGE
  GPS_IMAGE
  UNICORE_IMAGE
  LIDAR_IMAGE
  MAVROS_IMAGE
  NMEA_IMAGE
  GUI_IMAGE
  HARDWARE_BACKEND
  MAVROS_ENABLED
  MAVROS_BAUD
  MAVROS_TGT_SYSTEM
  MAVROS_TGT_COMPONENT
  MAVROS_AUTOPILOT
)

for key in "${REQUIRED_KEYS[@]}"; do
  if grep -qE "^${key}=" "$ENV_FILE"; then
    pass ".env has $key"
  else
    fail ".env has $key" "missing in $ENV_FILE"
  fi
done

section ".env values match the requested preset"

ENV_CONTENT="$(cat "$ENV_FILE")"
assert_contains "GPS_PROTOCOL=UBX (preset)" "GPS_PROTOCOL=UBX" "$ENV_CONTENT"
assert_contains "GPS_BAUD=921600 (runtime target)" "GPS_BAUD=921600" "$ENV_CONTENT"
assert_contains "GPS_CONNECTION=uart (preset)" "GPS_CONNECTION=uart" "$ENV_CONTENT"
assert_contains "LIDAR_TYPE=ldlidar (preset)" "LIDAR_TYPE=ldlidar" "$ENV_CONTENT"
assert_contains "LIDAR_BAUD=230400 (preset)" "LIDAR_BAUD=230400" "$ENV_CONTENT"
assert_contains "HARDWARE_BACKEND=mowgli (default)" "HARDWARE_BACKEND=mowgli" "$ENV_CONTENT"
assert_contains "GNSS_BACKEND=gps (default)" "GNSS_BACKEND=gps" "$ENV_CONTENT"
assert_contains "UNICORE_ROS_PACKAGE=unicore_gnss (default)" "UNICORE_ROS_PACKAGE=unicore_gnss" "$ENV_CONTENT"
assert_contains "UNICORE_ROS_EXECUTABLE=unicore_node (default)" "UNICORE_ROS_EXECUTABLE=unicore_node" "$ENV_CONTENT"
assert_contains "TFLUNA_FRONT_ENABLED=false (default)" "TFLUNA_FRONT_ENABLED=false" "$ENV_CONTENT"
assert_contains "TFLUNA_EDGE_ENABLED=false (default)" "TFLUNA_EDGE_ENABLED=false" "$ENV_CONTENT"

section ".env image references point at ghcr.io"

# Every image var should be a ghcr.io path — guards against accidental
# Docker Hub or local paths leaking into production .env.
for img_var in MOWGLI_ROS2_IMAGE GPS_IMAGE UNICORE_IMAGE LIDAR_IMAGE MAVROS_IMAGE NMEA_IMAGE GUI_IMAGE; do
  val="$(grep -E "^${img_var}=" "$ENV_FILE" | head -1 | cut -d= -f2-)"
  case "$val" in
    ghcr.io/*) pass "${img_var} is ghcr.io ($val)" ;;
    *)         fail "${img_var} is ghcr.io" "got '$val'" ;;
  esac
done

section ".env permissions are reasonable"

mode=$(stat -c '%a' "$ENV_FILE" 2>/dev/null || stat -f '%A' "$ENV_FILE" 2>/dev/null)
case "$mode" in
  6??|644|640|600) pass ".env permissions ($mode)" ;;
  *)               fail ".env permissions ($mode)" "expected 6xx, got $mode" ;;
esac

test_summary
