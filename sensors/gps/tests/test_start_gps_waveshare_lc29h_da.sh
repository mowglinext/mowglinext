#!/usr/bin/env bash
# Static dry-run coverage for the LC29H(DA) GNSS hardware preset.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
START_GPS="$SCRIPT_DIR/../start_gps.sh"
CONFIG="$SCRIPT_DIR/fixtures/waveshare_lc29h_da.yaml"
TEST_SETUP_DIR="$(mktemp -d)"
trap 'rm -rf "$TEST_SETUP_DIR"' EXIT
ROS_SETUP_BASH="$TEST_SETUP_DIR/ros_setup.bash"
GNSS_SIDECAR_SETUP_BASH="$TEST_SETUP_DIR/gnss_sidecar_setup.bash"
touch "$ROS_SETUP_BASH" "$GNSS_SIDECAR_SETUP_BASH"

output="$(GNSS_CONFIG_PATH="$CONFIG" \
  ROS_SETUP_BASH="$ROS_SETUP_BASH" \
  GNSS_SIDECAR_SETUP_BASH="$GNSS_SIDECAR_SETUP_BASH" \
  GNSS_DRY_RUN=true \
  bash "$START_GPS")"

grep -Fq 'preset=waveshare_lc29h_da receiver_family=nmea' <<<"$output"
grep -Fq 'baud=115200 rate_hz=1.0' <<<"$output"
grep -Fq 'receiver_node' <<<"$output"
grep -Fq -- '-p receiver_family:=nmea' <<<"$output"
grep -Fq -- '-p serial_baud:=115200' <<<"$output"
grep -Fq -- '-p publish_rate_hz:=1.0' <<<"$output"
if grep -Fq '[start_gps.sh] config_apply' <<<"$output"; then
  echo "LC29H(DA) preset must not invoke receiver configuration" >&2
  exit 1
fi
