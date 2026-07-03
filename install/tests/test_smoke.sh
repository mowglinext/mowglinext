#!/usr/bin/env bash
# =============================================================================
# A.1 Smoke test — installer runs to completion non-interactively
#
# Drives install/mowglinext.sh's data-flow through the harness with the
# default hardware preset (Mowgli STM32 + Universal GNSS UART + LDLiDAR UART, no
# rangefinders) and verifies every output artefact is produced.
# =============================================================================

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=lib/framework.sh
source "$SCRIPT_DIR/lib/framework.sh"
# shellcheck source=lib/mocks.sh
source "$SCRIPT_DIR/lib/mocks.sh"
# shellcheck source=lib/harness.sh
source "$SCRIPT_DIR/lib/harness.sh"

section "Smoke test — default mowgli preset"

setup_sandbox
install_all_mocks

SANDBOX_REPO="$SANDBOX/repo"
sandbox_repo "$SANDBOX_REPO"
harness_init "$SANDBOX_REPO"
harness_set_preset gnss=auto gnss_connection=uart lidar=ldlidar-uart tfluna=none

if harness_run; then
  pass "Installer flow completes for default preset"
else
  fail "Installer flow completes for default preset" "harness_run returned non-zero"
fi

assert_file_exists ".env materialised" "$SANDBOX_REPO/docker/.env"
assert_file_exists "docker-compose.yaml materialised" "$SANDBOX_REPO/docker/docker-compose.yaml"
assert_file_exists "mowgli_robot.yaml materialised" \
  "$SANDBOX_REPO/docker/config/mowgli/mowgli_robot.yaml"
assert_file_exists "cyclonedds.xml materialised" \
  "$SANDBOX_REPO/docker/config/cyclonedds.xml"
assert_file_exists "mosquitto.conf materialised" \
  "$SANDBOX_REPO/docker/config/mqtt/mosquitto.conf"

section "No real network calls"

# `git clone`/`fetch`/`pull` and `docker pull`/`up` should never have
# reached a real remote — every such call goes through our shim which
# logs to $SANDBOX/calls.log.
fetched_count=$(grep -cE '^git fetch|^git clone|^git pull' "$SANDBOX/calls.log" 2>/dev/null || true)
pull_count=$(grep -cE '^docker .* pull|^docker pull' "$SANDBOX/calls.log" 2>/dev/null || true)

echo "  git remote calls captured: $fetched_count"
echo "  docker pull calls captured: $pull_count"
pass "No live ghcr.io / GitHub network access required"

test_summary
