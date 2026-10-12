#!/usr/bin/env bash
# =============================================================================
# Retired optional hardware (TF-Luna rangefinders, VESC) can never come back
# through a stale docker/.env: the keys are dropped on reload AND scrubbed on
# write, and no compose fragment exists for them any more.
# =============================================================================
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/framework.sh
source "$SCRIPT_DIR/lib/framework.sh"
# shellcheck source=lib/mocks.sh
source "$SCRIPT_DIR/lib/mocks.sh"
# shellcheck source=lib/harness.sh
source "$SCRIPT_DIR/lib/harness.sh"

setup_sandbox
install_all_mocks

section "Retired fragments are gone from the installer sources"
for fragment in tfluna-front tfluna-edge vesc foxglove; do
  assert_file_not_exists "no docker-compose.${fragment}.yml fragment" "$REPO_ROOT/install/compose/docker-compose.${fragment}.yml"
done

section "Legacy docker/.env with TF-Luna and VESC enabled is neutralized"
repo_legacy="$SANDBOX/repo_legacy"
sandbox_repo "$repo_legacy"
harness_init "$repo_legacy"
mkdir -p "$repo_legacy/docker"
cat > "$repo_legacy/docker/.env" <<'EOF'
TFLUNA_FRONT_ENABLED=true
TFLUNA_FRONT_UART_DEVICE=/dev/ttyAMA3
TFLUNA_EDGE_ENABLED=true
ENABLE_VESC=true
VESC_CAN_INTERFACE=can0
RANGE_IMAGE=ghcr.io/...
EOF
reload_out="$(load_env_defaults_file "$repo_legacy/docker/.env" 2>&1)"
load_env_defaults_file "$repo_legacy/docker/.env" >/dev/null 2>&1
assert_not_contains "retired keys are dropped silently, not warned about" "Ignoring unknown installer key 'TFLUNA" "$reload_out"
assert_eq "TFLUNA_FRONT_ENABLED dropped on reload" "" "${TFLUNA_FRONT_ENABLED:-}"
assert_eq "ENABLE_VESC dropped on reload" "" "${ENABLE_VESC:-}"
harness_set_preset gnss=auto gnss_connection=uart lidar=ldlidar-uart
if harness_run; then
  pass "legacy harness_run"
else
  fail "legacy harness_run" "non-zero exit"
fi
env_legacy="$(cat "$repo_legacy/docker/.env")"
compose_legacy="$(cat "$repo_legacy/docker/docker-compose.yaml")"
assert_not_contains "legacy TF-Luna keys scrubbed from .env" "TFLUNA_" "$env_legacy"
assert_not_contains "legacy VESC keys scrubbed from .env" "VESC" "$env_legacy"
assert_not_contains "legacy RANGE_IMAGE scrubbed from .env" "RANGE_IMAGE" "$env_legacy"
assert_not_contains "compose omits mowgli-tfluna-front" "mowgli-tfluna-front" "$compose_legacy"
assert_not_contains "compose omits mowgli-tfluna-edge" "mowgli-tfluna-edge" "$compose_legacy"
assert_not_contains "compose omits mowgli-vesc" "mowgli-vesc" "$compose_legacy"
assert_not_contains "compose omits the placeholder image" "ghcr.io/..." "$compose_legacy"

test_summary
