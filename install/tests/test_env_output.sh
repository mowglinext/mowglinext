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
harness_set_preset gnss=auto gnss_connection=uart lidar=ldlidar-uart tfluna=none

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
  IMAGE_TAG
  GNSS_BACKEND
  GNSS_STATUS_SOURCE
  GNSS_STACK
  GNSS_RECEIVER_FAMILY
  GNSS_TRANSPORT
  GNSS_SERIAL_DEVICE
  GNSS_SERIAL_BAUD
  GNSS_FRAME_ID
  GNSS_NTRIP_ENABLED
  GNSS_NTRIP_HOST
  GNSS_NTRIP_PORT
  GNSS_NTRIP_MOUNTPOINT
  GNSS_NTRIP_USERNAME
  GNSS_NTRIP_PASSWORD
  GNSS_RTCM_FORWARDING
  GNSS_NTRIP_GGA_ENABLED
  GNSS_NTRIP_GGA_INTERVAL_S
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
  LIDAR_IMAGE
  MAVROS_IMAGE
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
assert_contains "IMAGE_TAG=main (default)" "IMAGE_TAG=main" "$ENV_CONTENT"
assert_contains "GNSS_STACK=universal (default)" "GNSS_STACK=universal" "$ENV_CONTENT"
assert_contains "GNSS_RECEIVER_FAMILY=auto (default)" "GNSS_RECEIVER_FAMILY=auto" "$ENV_CONTENT"
assert_contains "GNSS_TRANSPORT=serial (default)" "GNSS_TRANSPORT=serial" "$ENV_CONTENT"
assert_contains "GNSS_SERIAL_DEVICE=/dev/ttyAMA4 (derived)" "GNSS_SERIAL_DEVICE=/dev/ttyAMA4" "$ENV_CONTENT"
assert_contains "GNSS_SERIAL_BAUD=921600 (derived)" "GNSS_SERIAL_BAUD=921600" "$ENV_CONTENT"
assert_contains "GNSS_FRAME_ID=gps_link (default)" "GNSS_FRAME_ID=gps_link" "$ENV_CONTENT"
assert_contains "GNSS_NTRIP_ENABLED=true (rover default)" "GNSS_NTRIP_ENABLED=true" "$ENV_CONTENT"
assert_contains "GNSS_NTRIP_HOST=crtk.net (default)" "GNSS_NTRIP_HOST=crtk.net" "$ENV_CONTENT"
assert_contains "GNSS_RTCM_FORWARDING=true (rover default)" "GNSS_RTCM_FORWARDING=true" "$ENV_CONTENT"
assert_contains "GNSS_NTRIP_GGA_ENABLED=true (NEAR mountpoint)" "GNSS_NTRIP_GGA_ENABLED=true" "$ENV_CONTENT"
assert_contains "LIDAR_TYPE=ldlidar (preset)" "LIDAR_TYPE=ldlidar" "$ENV_CONTENT"
assert_contains "LIDAR_BAUD=230400 (preset)" "LIDAR_BAUD=230400" "$ENV_CONTENT"
assert_contains "HARDWARE_BACKEND=mowgli (default)" "HARDWARE_BACKEND=mowgli" "$ENV_CONTENT"
assert_contains "GNSS_BACKEND=universal (public runtime)" "GNSS_BACKEND=universal" "$ENV_CONTENT"
assert_contains "GNSS_STATUS_SOURCE=universal (default)" "GNSS_STATUS_SOURCE=universal" "$ENV_CONTENT"
assert_contains "TFLUNA_FRONT_ENABLED=false (default)" "TFLUNA_FRONT_ENABLED=false" "$ENV_CONTENT"
assert_contains "TFLUNA_EDGE_ENABLED=false (default)" "TFLUNA_EDGE_ENABLED=false" "$ENV_CONTENT"
legacy_protocol_key="GPS_""PROTOCOL="
legacy_runtime_mode_key="GPS_""RUNTIME_MODE="
legacy_port_key="GPS_""PORT="
legacy_by_id_key="GPS_""BY_ID="
legacy_ublox_key="UBLOX_""DEVICE_SERIAL_STRING="
legacy_unicore_key="UNICORE_""ROS_EXECUTABLE="
assert_not_contains "legacy GPS protocol omitted" "$legacy_protocol_key" "$ENV_CONTENT"
assert_not_contains "legacy GPS runtime mode omitted" "$legacy_runtime_mode_key" "$ENV_CONTENT"
assert_not_contains "legacy GPS port omitted" "$legacy_port_key" "$ENV_CONTENT"
assert_not_contains "legacy GPS by-id omitted" "$legacy_by_id_key" "$ENV_CONTENT"
assert_not_contains "legacy UBLOX serial omitted" "$legacy_ublox_key" "$ENV_CONTENT"
assert_not_contains "legacy UNICORE runtime omitted" "$legacy_unicore_key" "$ENV_CONTENT"

section "Universal USB presets keep GNSS_SERIAL_DEVICE on a by-id path"

repo_usb="$SANDBOX/repo_usb"
sandbox_repo "$repo_usb"
harness_init "$repo_usb"
harness_set_preset gnss=ublox gnss_connection=usb lidar=none tfluna=none

if ! harness_run; then
  fail "harness_run for USB GNSS preset" "non-zero exit"
else
  usb_env="$(cat "$repo_usb/docker/.env")"
  assert_contains "USB preset writes GNSS_SERIAL_DEVICE by-id" "GNSS_SERIAL_DEVICE=/dev/serial/by-id/usb-stub" "$usb_env"
  assert_not_contains "USB preset does not leak GPS_BY_ID" "$legacy_by_id_key" "$usb_env"
fi

section "install/.preset keeps GNSS_RECEIVER_FAMILY explicit through docker/.env"

repo_unicore="$SANDBOX/repo_unicore"
sandbox_repo "$repo_unicore"
harness_init "$repo_unicore"

cat > "$repo_unicore/install/.preset" <<'EOF'
GNSS_RECEIVER_FAMILY=unicore
GNSS_SERIAL_DEVICE=/dev/ttyAMA4
GNSS_SERIAL_BAUD=921600
LIDAR_ENABLED=false
LIDAR_TYPE=none
TFLUNA_FRONT_ENABLED=false
TFLUNA_EDGE_ENABLED=false
EOF

load_preset_file "$repo_unicore/install/.preset"
PRESET_LOADED=true

if ! harness_run; then
  fail "harness_run for install/.preset Unicore GNSS preset" "non-zero exit"
else
  unicore_env="$(cat "$repo_unicore/docker/.env")"
  assert_contains "install/.preset writes GNSS_RECEIVER_FAMILY=unicore" "GNSS_RECEIVER_FAMILY=unicore" "$unicore_env"
  assert_contains "install/.preset keeps GNSS_BACKEND=universal" "GNSS_BACKEND=universal" "$unicore_env"
fi

section "Custom feature image tags persist into docker/.env"

repo_feature="$SANDBOX/repo_feature"
sandbox_repo "$repo_feature"
harness_init "$repo_feature"
IMAGE_TAG="feat-universal-gnss-integration"
harness_set_preset gnss=auto gnss_connection=uart lidar=none tfluna=none

if ! harness_run; then
  fail "harness_run for custom feature image tag" "non-zero exit"
else
  feature_env="$(cat "$repo_feature/docker/.env")"
  assert_contains "custom IMAGE_TAG written" "IMAGE_TAG=feat-universal-gnss-integration" "$feature_env"
  assert_contains "custom GPS image tag written" "GPS_IMAGE=ghcr.io/cedbossneo/mowglinext/gps:feat-universal-gnss-integration" "$feature_env"
  assert_contains "custom mowgli-ros2 image tag written" "MOWGLI_ROS2_IMAGE=ghcr.io/cedbossneo/mowglinext/mowgli-ros2:feat-universal-gnss-integration" "$feature_env"
fi

section "NTRIP env is written without leaking secrets to logs"

repo_ntrip="$SANDBOX/repo_ntrip"
sandbox_repo "$repo_ntrip"
harness_init "$repo_ntrip"
harness_set_preset gnss=ublox gnss_connection=usb lidar=none tfluna=none \
  ntrip=true ntrip_host=rtk.local ntrip_port=2102 \
  ntrip_user=operator ntrip_password=super-secret ntrip_mountpoint=FIELD1

ntrip_setup_output="$(setup_env 2>&1)"
ntrip_env="$(cat "$repo_ntrip/docker/.env")"
assert_contains "NTRIP host written to env" "GNSS_NTRIP_HOST=rtk.local" "$ntrip_env"
assert_contains "NTRIP password written to env" "GNSS_NTRIP_PASSWORD=super-secret" "$ntrip_env"
assert_not_contains "NTRIP password not echoed in setup_env logs" "super-secret" "$ntrip_setup_output"

section ".env image references point at ghcr.io"

# Every image var should be a ghcr.io path — guards against accidental
# Docker Hub or local paths leaking into production .env.
for img_var in MOWGLI_ROS2_IMAGE GPS_IMAGE LIDAR_IMAGE MAVROS_IMAGE GUI_IMAGE; do
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
