#!/usr/bin/env bash
# =============================================================================
# Secure parser coverage for install/lib/state.sh
# =============================================================================

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=lib/framework.sh
source "$SCRIPT_DIR/lib/framework.sh"

setup_sandbox

SANDBOX_REPO="$SANDBOX/repo"
sandbox_repo "$SANDBOX_REPO"

# shellcheck source=/dev/null
source "$SANDBOX_REPO/install/lib/common.sh"
# shellcheck source=/dev/null
source "$SANDBOX_REPO/install/lib/i18n.sh"
# shellcheck source=/dev/null
source "$SANDBOX_REPO/install/lib/config.sh"
# shellcheck source=/dev/null
source "$SANDBOX_REPO/install/lib/state.sh"

load_locale
reapply_test_assertions

export REPO_DIR="$SANDBOX_REPO"
export DOCKER_DIR="$SANDBOX_REPO/docker"
export SCRIPT_DIR="$SANDBOX_REPO/install"

section "valid preset parsing"

cat > "$SANDBOX_REPO/install/.preset" <<'EOF'
# Composer preset
HARDWARE_BACKEND=mavros
GPS_CONNECTION=usb
GPS_PROTOCOL="UBX"
GPS_BAUD=460800
LIDAR_MODEL=
EOF

unset HARDWARE_BACKEND GPS_CONNECTION GPS_PROTOCOL GPS_BAUD LIDAR_MODEL 2>/dev/null || true
STATE_ACTIVE_PRESET_FILE=""
STATE_ACTIVE_PRESET_COUNT=0
load_preset_file "$SANDBOX_REPO/install/.preset"
assert_eq "preset loads HARDWARE_BACKEND" "mavros" "${HARDWARE_BACKEND:-}"
assert_eq "preset loads GPS_CONNECTION" "usb" "${GPS_CONNECTION:-}"
assert_eq "preset strips matching quotes" "UBX" "${GPS_PROTOCOL:-}"
assert_eq "preset keeps numeric text" "460800" "${GPS_BAUD:-}"
assert_eq "preset preserves empty values" "" "${LIDAR_MODEL-__unset__}"
assert_eq "preset count tracks loaded keys" "5" "${STATE_ACTIVE_PRESET_COUNT:-0}"

section "invalid preset lines are ignored"

cat > "$SANDBOX_REPO/install/.preset" <<'EOF'
GPS_CONNECTION=uart
NOT A VALID LINE
EOF

parse_output_file="$SANDBOX/preset-invalid.out"
load_preset_file "$SANDBOX_REPO/install/.preset" >"$parse_output_file" 2>&1
parse_output="$(cat "$parse_output_file")"
assert_contains "invalid preset line warns" "Ignoring invalid KEY=VALUE line" "$parse_output"
assert_eq "valid key still loaded" "uart" "${GPS_CONNECTION:-}"

section "shell-like payload is not executed"

touch_marker="$SANDBOX/marker"
cat > "$SANDBOX_REPO/install/.preset" <<EOF
\$(touch "$touch_marker")
GPS_CONNECTION=usb
EOF

rm -f "$touch_marker"
payload_output_file="$SANDBOX/preset-payload.out"
load_preset_file "$SANDBOX_REPO/install/.preset" >"$payload_output_file" 2>&1
payload_output="$(cat "$payload_output_file")"
assert_contains "shell payload rejected as invalid line" "Ignoring invalid KEY=VALUE line" "$payload_output"
assert_file_not_exists "shell payload not executed" "$touch_marker"
assert_eq "valid preset key still loads after payload" "usb" "${GPS_CONNECTION:-}"

section ".env parsing with comments and unknown keys"

cat > "$SANDBOX_REPO/docker/.env" <<'EOF'
# Existing runtime config
export GPS_PROTOCOL=NMEA

GPS_BAUD=115200
LIDAR_MODEL=""
UNKNOWN_TEST_KEY=surprise
EOF

unset GPS_PROTOCOL GPS_BAUD LIDAR_MODEL UNKNOWN_TEST_KEY 2>/dev/null || true
env_output_file="$SANDBOX/env-load.out"
load_env_defaults_file "$SANDBOX_REPO/docker/.env" >"$env_output_file" 2>&1
env_output="$(cat "$env_output_file")"
assert_contains ".env comments ignored and file loads" "Loaded previous configuration" "$env_output"
assert_contains "unknown key warning preserved" "Ignoring unknown installer key 'UNKNOWN_TEST_KEY'" "$env_output"
assert_eq ".env export compatibility preserved" "NMEA" "${GPS_PROTOCOL:-}"
assert_eq ".env numeric text preserved" "115200" "${GPS_BAUD:-}"
assert_eq ".env quoted empty value preserved" "" "${LIDAR_MODEL-__unset__}"

section ".env backup for web preset installs"

cat > "$SANDBOX_REPO/docker/.env" <<'EOF'
GNSS_BACKEND=gps
GPS_CONNECTION=usb
GPS_PROTOCOL=UBX
EOF

backup_env_defaults_file "$SANDBOX_REPO/docker/.env"
assert_file_not_exists ".env moved aside" "$SANDBOX_REPO/docker/.env"
backup_file="$(find "$SANDBOX_REPO/docker" -maxdepth 1 -name '.env.old.*' | sort | tail -n1)"
assert_file_exists ".env backup created" "$backup_file"
backup_content="$(cat "$backup_file")"
assert_contains ".env backup keeps old GPS_CONNECTION" "GPS_CONNECTION=usb" "$backup_content"

section "preset is consumed only on explicit success path"

cat > "$SANDBOX_REPO/install/.preset" <<'EOF'
GPS_CONNECTION=usb
EOF

load_preset_file "$SANDBOX_REPO/install/.preset"
assert_file_exists "preset exists before consumption" "$SANDBOX_REPO/install/.preset"
mark_preset_consumed
assert_file_not_exists "preset removed after consumption" "$SANDBOX_REPO/install/.preset"
assert_file_exists "preset renamed to consumed" "$SANDBOX_REPO/install/.preset.consumed"

section "preset remains after load without consumption"

cat > "$SANDBOX_REPO/install/.preset" <<'EOF'
GPS_CONNECTION=uart
EOF

load_preset_file "$SANDBOX_REPO/install/.preset"
assert_file_exists "preset still present before success marker" "$SANDBOX_REPO/install/.preset"
assert_file_not_exists "consumed preset not created early" "$SANDBOX_REPO/install/.preset.consumed.tmp"

section "historical .env compatibility"

cat > "$SANDBOX_REPO/docker/.env" <<EOF
ROS_DOMAIN_ID=0
MOWER_IP=10.0.0.161
DISABLE_BLUETOOTH=true
ENABLE_FOXGLOVE=true
GNSS_BACKEND=gps
GPS_CONNECTION=uart
GPS_PROTOCOL=UBX
GPS_PORT=/dev/gps
GPS_BY_ID=
GPS_UART_DEVICE=/dev/ttyAMA4
GPS_BAUD=460800
GPS_DEBUG_ENABLED=false
GPS_DEBUG_PORT=/dev/gps_debug
GPS_DEBUG_UART_DEVICE=/dev/ttyS0
GPS_DEBUG_BAUD=115200
LIDAR_ENABLED=true
LIDAR_TYPE=ldlidar
LIDAR_MODEL=LDLiDAR_LD19
LIDAR_CONNECTION=uart
LIDAR_PORT=/dev/lidar
LIDAR_UART_DEVICE=/dev/ttyAMA5
LIDAR_BAUD=230400
TFLUNA_FRONT_ENABLED=false
TFLUNA_FRONT_PORT=/dev/tfluna_front
TFLUNA_FRONT_UART_DEVICE=/dev/ttyAMA3
TFLUNA_FRONT_BAUD=115200
TFLUNA_EDGE_ENABLED=false
TFLUNA_EDGE_PORT=/dev/tfluna_edge
TFLUNA_EDGE_UART_DEVICE=/dev/ttyAMA2
TFLUNA_EDGE_BAUD=115200
MOWGLI_ROS2_IMAGE=${MOWGLI_ROS2_IMAGE_DEFAULT}
GPS_IMAGE=${GPS_IMAGE_DEFAULT}
UNICORE_IMAGE=${UNICORE_IMAGE_DEFAULT}
LIDAR_IMAGE=${LIDAR_LDLIDAR_IMAGE_DEFAULT}
MAVROS_IMAGE=${MAVROS_IMAGE_DEFAULT}
NMEA_IMAGE=${NMEA_IMAGE_DEFAULT}
GUI_IMAGE=${GUI_IMAGE_DEFAULT}
HARDWARE_BACKEND=mowgli
MAVROS_ENABLED=false
MAVROS_BY_ID=
MAVROS_PORT=/dev/mavros
MAVROS_BAUD=921600
MAVROS_GCS_URL=udp-b://@255.255.255.255:14550
MAVROS_TGT_SYSTEM=1
MAVROS_TGT_COMPONENT=1
MAVROS_AUTOPILOT=ardupilot
EOF

unset MAVROS_GCS_URL GUI_IMAGE HARDWARE_BACKEND GPS_CONNECTION 2>/dev/null || true
assert_exit_zero "historical .env loads cleanly" load_env_defaults_file "$SANDBOX_REPO/docker/.env"
assert_eq "historical .env keeps MAVROS_GCS_URL" "udp-b://@255.255.255.255:14550" "${MAVROS_GCS_URL:-}"
assert_eq "historical .env keeps GUI_IMAGE" "${GUI_IMAGE_DEFAULT}" "${GUI_IMAGE:-}"
assert_eq "historical .env keeps HARDWARE_BACKEND" "mowgli" "${HARDWARE_BACKEND:-}"
assert_eq "historical .env keeps GPS_CONNECTION" "uart" "${GPS_CONNECTION:-}"

test_summary
