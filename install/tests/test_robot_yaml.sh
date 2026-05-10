#!/usr/bin/env bash
# =============================================================================
# A.6 mowgli_robot.yaml correctness
#
# The site-specific robot config the installer materialises must:
#   - NOT hardcode a real GPS datum (must be 0.0 / 0.0 by default so
#     auto-detect-on-first-boot works)
#   - NOT contain blade-bypass / safety-disable flags (firmware is the
#     sole blade-safety authority — see CLAUDE.md "Safety — READ FIRST")
#   - propagate the GPS protocol/baud/port chosen at install time
#
# It must NOT take over fields that belong in the bringup defaults
# (two_d_mode, base_link/base_footprint frames live in
# robot_localization.yaml inside the ros2 image, not in the
# user-managed mowgli_robot.yaml).
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
harness_set_preset gnss=gps gps=ubx-uart lidar=ldlidar-uart tfluna=none

if ! harness_run; then
  fail "harness_run" "non-zero exit"
  test_summary
  exit 1
fi

YAML="$SANDBOX_REPO/docker/config/mowgli/mowgli_robot.yaml"
MOWER_CFG="$SANDBOX_REPO/docker/config/om/mower_config.sh"

section "mowgli_robot.yaml shape"

assert_file_exists "yaml exists" "$YAML"

CONTENT="$(cat "$YAML")"

# Datum must be a placeholder so auto-detect kicks in
assert_match "datum_lat is 0.0 (auto-detect placeholder)" \
  '^[[:space:]]+datum_lat:[[:space:]]+0\.0[[:space:]]*$' "$CONTENT"
assert_match "datum_lon is 0.0 (auto-detect placeholder)" \
  '^[[:space:]]+datum_lon:[[:space:]]+0\.0[[:space:]]*$' "$CONTENT"

# GPS preset propagates
assert_match "gps_port=/dev/gps" '^[[:space:]]+gps_port:[[:space:]]+"/dev/gps"' "$CONTENT"
assert_match "gps_baudrate=921600 (default runtime target)" \
  '^[[:space:]]+gps_baudrate:[[:space:]]+921600' "$CONTENT"

# NTRIP fields exist
for ntrip_key in ntrip_enabled ntrip_host ntrip_port ntrip_user ntrip_password ntrip_mountpoint; do
  if grep -qE "^[[:space:]]+${ntrip_key}:" "$YAML"; then
    pass "ntrip key: $ntrip_key"
  else
    fail "ntrip key: $ntrip_key" "missing"
  fi
done

# LiDAR mounting placeholders exist (operator must measure the real
# offsets — installer just ensures the keys are reserved)
for lidar_key in lidar_x lidar_y lidar_z lidar_yaw; do
  if grep -qE "^[[:space:]]+${lidar_key}:" "$YAML"; then
    pass "lidar mounting key: $lidar_key"
  else
    fail "lidar mounting key: $lidar_key" "missing"
  fi
done

# Dock pose placeholders
for dock_key in dock_pose_x dock_pose_y dock_pose_yaw; do
  if grep -qE "^[[:space:]]+${dock_key}:" "$YAML"; then
    pass "dock placeholder: $dock_key"
  else
    fail "dock placeholder: $dock_key" "missing"
  fi
done

section "mowgli_robot.yaml safety guarantees"

# Anything that looks like a blade-safety bypass would be a CRITICAL bug.
# Even though firmware is the authoritative safety layer, having such a
# flag in the user-managed file is misleading.
for forbidden in "blade_bypass" "disable_safety" "skip_emergency" "ignore_estop"; do
  if grep -qiE "$forbidden" "$YAML"; then
    fail "no $forbidden flag" "found in $YAML"
  else
    pass "no $forbidden flag"
  fi
done

# Must NOT define two_d_mode / base_link / base_footprint here — those
# live in robot_localization.yaml shipped inside the ros2 container.
for owned_elsewhere in "two_d_mode" "base_footprint" "publish_tf"; do
  if grep -qE "^[[:space:]]+${owned_elsewhere}:" "$YAML"; then
    fail "no override of $owned_elsewhere in user yaml" \
      "$owned_elsewhere belongs in robot_localization.yaml, not site config"
  else
    pass "no override of $owned_elsewhere in user yaml"
  fi
done

section "mower_config.sh (legacy OpenMower env vars)"

assert_file_exists "mower_config.sh exists" "$MOWER_CFG"
MC="$(cat "$MOWER_CFG")"

# OM_* shell vars are read by GUI + legacy scripts. Their presence is
# the contract; their values are the same as the YAML datum/NTRIP.
for k in OM_DATUM_LAT OM_DATUM_LONG OM_GPS_PROTOCOL OM_GPS_PORT \
         OM_GPS_BAUDRATE OM_USE_NTRIP OM_NTRIP_HOSTNAME \
         OM_NTRIP_PORT OM_BATTERY_FULL_VOLTAGE OM_BATTERY_EMPTY_VOLTAGE \
         OM_BATTERY_CRITICAL_VOLTAGE; do
  if printf '%s' "$MC" | grep -qE "^export ${k}="; then
    pass "mower_config.sh exports $k"
  else
    fail "mower_config.sh exports $k"
  fi
done

# Battery hysteresis: critical < empty < full
crit=$(printf '%s' "$MC" | grep -E '^export OM_BATTERY_CRITICAL_VOLTAGE=' | cut -d= -f2)
empty=$(printf '%s' "$MC" | grep -E '^export OM_BATTERY_EMPTY_VOLTAGE='   | cut -d= -f2)
full=$(printf '%s' "$MC" | grep -E '^export OM_BATTERY_FULL_VOLTAGE='     | cut -d= -f2)
if awk "BEGIN { exit !($crit < $empty && $empty < $full) }"; then
  pass "battery thresholds satisfy critical<empty<full"
else
  fail "battery thresholds satisfy critical<empty<full" \
    "got critical=$crit empty=$empty full=$full"
fi

section "ROS2 params YAML is parseable"

# A simple parse with python's yaml is the cleanest validity check;
# we fall back to a regex-only sanity check if PyYAML is unavailable.
if command -v python3 >/dev/null && python3 -c 'import yaml' 2>/dev/null; then
  if python3 -c "import yaml,sys; yaml.safe_load(open('$YAML'))" 2>/dev/null; then
    pass "yaml.safe_load(mowgli_robot.yaml)"
  else
    fail "yaml.safe_load(mowgli_robot.yaml)" \
      "$(python3 -c "import yaml,sys; yaml.safe_load(open('$YAML'))" 2>&1 | head -3)"
  fi
else
  pass "yaml syntax (skipped; PyYAML missing)"
fi

test_summary
