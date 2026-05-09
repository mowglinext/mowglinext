#!/usr/bin/env bash
# =============================================================================
# A.9 Optional feature guardrails — unsupported TF-Luna / VESC must not leak
# into the generated runtime compose, even when old docker/.env files still
# carry those flags.
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

section "Standard install keeps unsupported optional services out of compose"

repo_standard="$SANDBOX/repo_standard"
sandbox_repo "$repo_standard"
harness_init "$repo_standard"
harness_set_preset gps=ubx-uart lidar=ldlidar-uart tfluna=none

if harness_run; then
  pass "standard harness_run"
else
  fail "standard harness_run" "non-zero exit"
  test_summary
  exit 1
fi

compose_standard="$(cat "$repo_standard/docker/docker-compose.yaml")"
assert_not_contains "standard compose omits TF-Luna placeholder image" "ghcr.io/..." "$compose_standard"
assert_not_contains "standard compose omits mowgli-tfluna-front" "mowgli-tfluna-front" "$compose_standard"
assert_not_contains "standard compose omits mowgli-tfluna-edge" "mowgli-tfluna-edge" "$compose_standard"
assert_not_contains "standard compose omits mowgli-vesc" "mowgli-vesc" "$compose_standard"

section "Legacy docker/.env with TF-Luna enabled is neutralized safely"

repo_tfluna="$SANDBOX/repo_tfluna"
sandbox_repo "$repo_tfluna"
harness_init "$repo_tfluna"
mkdir -p "$repo_tfluna/docker"
cat > "$repo_tfluna/docker/.env" <<'EOF'
TFLUNA_FRONT_ENABLED=true
TFLUNA_FRONT_PORT=/dev/tfluna_front
TFLUNA_FRONT_UART_DEVICE=/dev/ttyAMA3
TFLUNA_FRONT_BAUD=115200
TFLUNA_EDGE_ENABLED=false
EOF

load_env_defaults_file "$repo_tfluna/docker/.env"
if harness_run; then
  pass "legacy TF-Luna env rerun"
else
  fail "legacy TF-Luna env rerun" "non-zero exit"
  test_summary
  exit 1
fi

env_tfluna="$(cat "$repo_tfluna/docker/.env")"
compose_tfluna="$(cat "$repo_tfluna/docker/docker-compose.yaml")"
assert_contains "legacy TF-Luna env rewritten with front disabled" "TFLUNA_FRONT_ENABLED=false" "$env_tfluna"
assert_not_contains "legacy TF-Luna compose omits front service" "mowgli-tfluna-front" "$compose_tfluna"
assert_not_contains "legacy TF-Luna compose omits placeholder image" "ghcr.io/..." "$compose_tfluna"

section "Legacy docker/.env with VESC enabled is skipped safely"

repo_vesc="$SANDBOX/repo_vesc"
sandbox_repo "$repo_vesc"
harness_init "$repo_vesc"
mkdir -p "$repo_vesc/docker"
cat > "$repo_vesc/docker/.env" <<'EOF'
ENABLE_VESC=true
VESC_IMAGE=ghcr.io/example/vesc:test
VESC_CAN_INTERFACE=can0
EOF

load_env_defaults_file "$repo_vesc/docker/.env"
if harness_run; then
  pass "legacy VESC env rerun"
else
  fail "legacy VESC env rerun" "non-zero exit"
  test_summary
  exit 1
fi

compose_vesc="$(cat "$repo_vesc/docker/docker-compose.yaml")"
assert_not_contains "legacy VESC compose omits mowgli-vesc" "mowgli-vesc" "$compose_vesc"
assert_not_contains "legacy VESC compose omits TF-Luna placeholder image" "ghcr.io/..." "$compose_vesc"

test_summary
