#!/usr/bin/env bash
# =============================================================================
# A.5 LiDAR backend matrix — none / ldlidar / rplidar / stl27l × usb / uart
#
# Verifies each LIDAR_TYPE selects the right compose fragment, propagates
# the right baud rate, and produces the right LIDAR_IMAGE in .env.
# (The "mavros" entry mentioned in the prompt is a hardware backend, not
# a lidar; it's covered by test_hardware_presets.sh.)
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

services_in() {
  grep -E '^\s+container_name:' "$1/docker/docker-compose.yaml" | awk '{print $2}' | sort
}

# Run a preset and assert on LIDAR_TYPE / LIDAR_MODEL / LIDAR_BAUD / LIDAR_IMAGE
# and the resulting compose. Args:
#   $1 label    — short tag for the section
#   $2 preset   — the lidar=... value
#   $3 type_exp — expected LIDAR_TYPE
#   $4 baud_exp — expected LIDAR_BAUD (or "" to skip)
#   $5 image_substr — expected substring in LIDAR_IMAGE (or "" to skip)
#   $6 want_service — "1" if compose should have mowgli-lidar, "0" if not
check_lidar_preset() {
  local label="$1" preset="$2" type_exp="$3" baud_exp="$4" image_substr="$5" want_service="$6"
  section "lidar=$preset"
  local repo="$SANDBOX/repo_lidar_${label}"
  sandbox_repo "$repo"
  harness_init "$repo"
  harness_set_preset gnss=auto gnss_connection=uart "lidar=$preset" tfluna=none
  if harness_run >/dev/null 2>&1; then
    pass "$label: harness_run"
  else
    fail "$label: harness_run"
    return
  fi
  assert_eq "$label: LIDAR_TYPE" "$type_exp" "$(env_value "$repo" LIDAR_TYPE)"
  case "$type_exp" in
    # setup_env historically keeps the generic LD19 fallback in .env even
    # when the lidar service is disabled; keep that contract pinned here.
    none)    model_exp="LDLiDAR_LD19" ;;
    rplidar) model_exp="RPLIDAR_A1" ;;
    ldlidar) model_exp="LDLiDAR_LD19" ;;
    stl27l)  model_exp="LDLiDAR_STL27L" ;;
    *)       model_exp="" ;;
  esac
  assert_eq "$label: LIDAR_MODEL" "$model_exp" "$(env_value "$repo" LIDAR_MODEL)"
  if [ -n "$baud_exp" ]; then
    assert_eq "$label: LIDAR_BAUD" "$baud_exp" "$(env_value "$repo" LIDAR_BAUD)"
  fi
  if [ -n "$image_substr" ]; then
    img="$(env_value "$repo" LIDAR_IMAGE)"
    case "$img" in
      *"$image_substr"*) pass "$label: LIDAR_IMAGE contains '$image_substr'" ;;
      *)                 fail "$label: LIDAR_IMAGE contains '$image_substr'" "got '$img'" ;;
    esac
  fi

  local compose_content
  compose_content="$(cat "$repo/docker/docker-compose.yaml")"
  case "$type_exp" in
    stl27l)
      # The generated compose intentionally keeps env references so later
      # edits to docker/.env take effect.  Assert both the canonical runtime
      # fallback and the generated env value that supplies it on install.
      assert_contains "$label: STL27L compose product_name fallback" \
        'product_name:=${LIDAR_MODEL:-LDLiDAR_STL27L}' "$compose_content"
      assert_contains "$label: STL27L compose baud fallback" \
        'port_baudrate:=${LIDAR_BAUD:-921600}' "$compose_content"
      assert_not_contains "$label: STL27L compose rejects legacy product name" \
        'product_name:=STL27L' "$compose_content"
      ;;
    rplidar)
      assert_contains "$label: RPLidar compose baud default unchanged" \
        'serial_baudrate:=${LIDAR_BAUD:-115200}' "$compose_content"
      ;;
  esac
  case "$(services_in "$repo")" in
    *mowgli-lidar*)
      if [ "$want_service" = "1" ]; then
        pass "$label: mowgli-lidar service present"
      else
        fail "$label: mowgli-lidar service absent" "service leaked when LiDAR disabled"
      fi
      ;;
    *)
      if [ "$want_service" = "0" ]; then
        pass "$label: mowgli-lidar service absent"
      else
        fail "$label: mowgli-lidar service present"
      fi
      ;;
  esac
}

setup_sandbox
install_all_mocks

check_lidar_preset "none"          "none"          "none"      ""        ""                       "0"
check_lidar_preset "ldlidar_uart"  "ldlidar-uart"  "ldlidar"   "230400"  "lidar-ldlidar"          "1"
check_lidar_preset "ldlidar_usb"   "ldlidar-usb"   "ldlidar"   "230400"  "lidar-ldlidar"          "1"
check_lidar_preset "rplidar_uart"  "rplidar-uart"  "rplidar"   "115200"  "lidar-rplidar"          "1"
check_lidar_preset "rplidar_usb"   "rplidar-usb"   "rplidar"   "115200"  "lidar-rplidar"          "1"
check_lidar_preset "stl27l_uart"   "stl27l-uart"   "stl27l"    "921600"  "lidar-stl27l"           "1"
check_lidar_preset "stl27l_usb"    "stl27l-usb"    "stl27l"    "921600"  "lidar-stl27l"           "1"

section "STL27L type-specific setup_env defaults"
repo_defaults="$SANDBOX/repo_lidar_defaults"
sandbox_repo "$repo_defaults"
harness_init "$repo_defaults"
LIDAR_TYPE="stl27l"
unset LIDAR_MODEL LIDAR_BAUD
if setup_env >/dev/null 2>&1; then
  pass "defaults: setup_env"
  assert_eq "defaults: LIDAR_MODEL" "LDLiDAR_STL27L" "$(env_value "$repo_defaults" LIDAR_MODEL)"
  assert_eq "defaults: LIDAR_BAUD" "921600" "$(env_value "$repo_defaults" LIDAR_BAUD)"
else
  fail "defaults: setup_env"
fi

section "STL27L explicit overrides remain effective"
repo_override="$SANDBOX/repo_lidar_override"
sandbox_repo "$repo_override"
harness_init "$repo_override"
harness_set_preset gnss=auto gnss_connection=uart lidar=stl27l-uart tfluna=none
LIDAR_MODEL="custom-stl27l-driver"
LIDAR_BAUD="123456"
if harness_run >/dev/null 2>&1; then
  pass "override: harness_run"
  assert_eq "override: LIDAR_MODEL" "custom-stl27l-driver" "$(env_value "$repo_override" LIDAR_MODEL)"
  assert_eq "override: LIDAR_BAUD" "123456" "$(env_value "$repo_override" LIDAR_BAUD)"
else
  fail "override: harness_run"
fi

test_summary
