#!/usr/bin/env bash
# =============================================================================
# A.6 mowgli_robot.yaml correctness
#
# The site-specific robot config materialised by the installer must:
#   - keep datum placeholders at zero until site calibration
#   - store the Universal GNSS contract in YAML
#   - omit retired legacy GPS compatibility keys
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
  fail "harness_run" "non-zero exit"
  test_summary
  exit 1
fi

YAML="$SANDBOX_REPO/docker/config/mowgli/mowgli_robot.yaml"
OM_CFG="$SANDBOX_REPO/docker/config/om/mower_config.sh"

section "mowgli_robot.yaml shape"

assert_file_exists "yaml exists" "$YAML"

CONTENT="$(cat "$YAML")"

assert_match "datum_lat is zero placeholder" \
  '^[[:space:]]+datum_lat:[[:space:]]+0(\.0)?[[:space:]]*$' "$CONTENT"
assert_match "datum_lon is zero placeholder" \
  '^[[:space:]]+datum_lon:[[:space:]]+0(\.0)?[[:space:]]*$' "$CONTENT"
assert_match "gnss_receiver_family=auto" \
  '^[[:space:]]+gnss_receiver_family:[[:space:]]+"?auto"?[[:space:]]*$' "$CONTENT"
assert_match "gnss_serial_device=/dev/ttyAMA4" \
  '^[[:space:]]+gnss_serial_device:[[:space:]]+"?/dev/ttyAMA4"?[[:space:]]*$' "$CONTENT"
assert_match "gnss_serial_baud=921600" \
  '^[[:space:]]+gnss_serial_baud:[[:space:]]+921600[[:space:]]*$' "$CONTENT"
assert_match "ntrip_enabled=true" \
  '^[[:space:]]+ntrip_enabled:[[:space:]]+true[[:space:]]*$' "$CONTENT"

legacy_yaml_keys=(
  "gps_""protocol:"
  "gps_""port:"
  "gps_""baudrate:"
)
for forbidden in "${legacy_yaml_keys[@]}"; do
  if grep -qi "$forbidden" "$YAML"; then
    fail "legacy GNSS key absent: $forbidden" "found in $YAML"
  else
    pass "legacy GNSS key absent: $forbidden"
  fi
done

section "mowgli_robot.yaml safety guarantees"

for forbidden in "blade_bypass" "disable_safety" "skip_emergency" "ignore_estop"; do
  if grep -qiE "$forbidden" "$YAML"; then
    fail "no $forbidden flag" "found in $YAML"
  else
    pass "no $forbidden flag"
  fi
done

for owned_elsewhere in "two_d_mode" "base_footprint" "publish_tf"; do
  if grep -qE "^[[:space:]]+${owned_elsewhere}:" "$YAML"; then
    fail "no override of $owned_elsewhere in user yaml" \
      "$owned_elsewhere belongs in robot_localization.yaml, not site config"
  else
    pass "no override of $owned_elsewhere in user yaml"
  fi
done

section "legacy OpenMower shell dump is no longer generated"

assert_file_not_exists "mower_config.sh omitted" "$OM_CFG"

section "ROS2 params YAML is parseable"

if command -v python3 >/dev/null && python3 -c 'import yaml' 2>/dev/null; then
  if python3 -c "import yaml; yaml.safe_load(open('$YAML'))" 2>/dev/null; then
    pass "yaml.safe_load(mowgli_robot.yaml)"
  else
    fail "yaml.safe_load(mowgli_robot.yaml)" \
      "$(python3 -c "import yaml; yaml.safe_load(open('$YAML'))" 2>&1 | head -3)"
  fi
else
  pass "yaml syntax (skipped; PyYAML missing)"
fi

test_summary
