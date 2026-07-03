#!/usr/bin/env bash
# =============================================================================
# A.2 Output sanity — docker-compose.yaml is valid + has required services
#
# Validates the merged compose file via `docker compose config -q` and
# spot-checks that the service blocks the user's preset implies are
# actually present (mowgli, gui, lidar, mavros, ntrip) and that
# Universal GNSS does not leak the legacy direct GNSS containers.
# =============================================================================

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=lib/framework.sh
source "$SCRIPT_DIR/lib/framework.sh"
# shellcheck source=lib/mocks.sh
source "$SCRIPT_DIR/lib/mocks.sh"
# shellcheck source=lib/harness.sh
source "$SCRIPT_DIR/lib/harness.sh"

real_docker_compose_available() {
  PATH="$ORIG_PATH" command -v docker >/dev/null 2>&1 \
    && HOME="$ORIG_HOME" PATH="$ORIG_PATH" docker compose version >/dev/null 2>&1
}

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

COMPOSE_FILE="$SANDBOX_REPO/docker/docker-compose.yaml"
ENV_FILE="$SANDBOX_REPO/docker/.env"

section "Compose file validates"

# `docker compose config` is the canonical YAML validator — if this
# fails the user's stack will refuse to start.
if real_docker_compose_available; then
  if HOME="$ORIG_HOME" docker compose -f "$COMPOSE_FILE" --env-file "$ENV_FILE" config -q 2>/dev/null; then
    pass "docker compose config -q passes"
  else
    fail "docker compose config -q passes" \
      "$(HOME="$ORIG_HOME" docker compose -f "$COMPOSE_FILE" --env-file "$ENV_FILE" config -q 2>&1 | head -3)"
  fi
else
  if [ -s "$COMPOSE_FILE" ]; then
    pass "compose fallback file generated (docker unavailable)"
  else
    fail "compose fallback file generated (docker unavailable)" "generated compose is empty"
  fi
fi

section "Required services present (default mowgli + ldlidar preset)"

CONTAINERS=$(grep -E '^\s+container_name:' "$COMPOSE_FILE" | awk '{print $2}' | sort)

for required in mowgli-ros2 mowgli-gps mowgli-gui mowgli-lidar mowgli-mqtt mowgli-watchtower; do
  if printf '%s\n' "$CONTAINERS" | grep -qx "$required"; then
    pass "service: $required"
  else
    fail "service: $required" "missing from compose"
  fi
done

# Negative: with HARDWARE_BACKEND=mowgli + GNSS_STACK=universal, mavros and
# the MAVROS-only NTRIP sidecar must NOT be present.
for forbidden in mowgli-mavros mowgli-ntrip; do
  if printf '%s\n' "$CONTAINERS" | grep -qx "$forbidden"; then
    fail "service NOT present: $forbidden" "should not be in mowgli backend compose"
  else
    pass "service NOT present: $forbidden"
  fi
done

section "Universal GNSS compose uses the canonical mowgli-gps sidecar"

for required in "GNSS_STACK:" "GNSS_RECEIVER_FAMILY:" "GNSS_SERIAL_DEVICE:" "GNSS_FRAME_ID:" "GNSS_NTRIP_GGA_ENABLED:"; do
  if grep -q "$required" "$COMPOSE_FILE"; then
    pass "compose contains sidecar env: $required"
  else
    fail "compose contains sidecar env: $required" "missing from generated compose"
  fi
done

for forbidden in "gnss_unicore:" "UNICORE_IMAGE" "GPS_""RUNTIME_MODE:" "GPS_""PROTOCOL:" "GPS_""PORT:" "GPS_""BAUD:"; do
  if grep -q "$forbidden" "$COMPOSE_FILE"; then
    fail "legacy standalone GNSS absent: $forbidden" "found in generated universal compose"
  else
    pass "legacy standalone GNSS absent: $forbidden"
  fi
done

# Negative: unsupported optional services must not be emitted
for forbidden in mowgli-tfluna-front mowgli-tfluna-edge mowgli-vesc; do
  if printf '%s\n' "$CONTAINERS" | grep -qx "$forbidden"; then
    fail "service NOT present: $forbidden" "unsupported optional service leaked into compose"
  else
    pass "service NOT present: $forbidden"
  fi
done

section "Compose env-var expansion does not have unresolved placeholders"

if real_docker_compose_available; then
  # After `docker compose config` fully expands ${VAR} references, no `${`
  # placeholder should remain. `image:` is the most common breakage point.
  EXPANDED=$(HOME="$ORIG_HOME" docker compose -f "$COMPOSE_FILE" --env-file "$ENV_FILE" config 2>/dev/null)
  if printf '%s' "$EXPANDED" | grep -qE 'image:.*\${' ; then
    fail "no unresolved \${VAR} in image:" \
      "$(printf '%s' "$EXPANDED" | grep -E 'image:.*\${' | head -1)"
  else
    pass "no unresolved \${VAR} in image:"
  fi

  if printf '%s' "$EXPANDED" | grep -qE 'GNSS_STACK: universal$'; then
    pass "Universal GNSS sidecar expands to GNSS_STACK=universal"
  else
    fail "Universal GNSS sidecar expands to GNSS_STACK=universal" \
      "$(printf '%s' "$EXPANDED" | grep -n 'GNSS_STACK:' | head -1)"
  fi

  # Privileged volumes contain /dev mount — required for sensor passthrough.
  # `docker compose config` rewrites `- /dev:/dev` into the long-form
  # `source: /dev / target: /dev` block, so we look for the long form.
  if printf '%s' "$EXPANDED" | grep -qE 'source: /dev$' \
    && printf '%s' "$EXPANDED" | grep -qE 'target: /dev$'; then
    pass "/dev:/dev volume mount present (sensor passthrough)"
  else
    fail "/dev:/dev volume mount present" "/dev passthrough missing — sensors won't work"
  fi

  # Foxglove environment toggle present in expanded mowgli service env
  if printf '%s' "$EXPANDED" | grep -qE 'ENABLE_FOXGLOVE'; then
    pass "ENABLE_FOXGLOVE env var wired into mowgli service"
  else
    fail "ENABLE_FOXGLOVE env var wired into mowgli service" "not found in expanded compose"
  fi

  section "Compose 'volumes:' section declares mowgli_maps"

  # mowgli_maps is the bind-mount that persists garden_map + fusion_graph
  # files across container restarts.
  if printf '%s' "$EXPANDED" | grep -qE '^\s+mowgli_maps:'; then
    pass "named volume mowgli_maps declared"
  else
    fail "named volume mowgli_maps declared" "missing — maps would be lost on restart"
  fi
else
  pass "no unresolved \${VAR} in image: (skipped; docker unavailable)"
  if grep -q '/dev:/dev' "$COMPOSE_FILE"; then
    pass "/dev:/dev volume mount present (fallback compose)"
  else
    fail "/dev:/dev volume mount present (fallback compose)" "/dev passthrough missing"
  fi
  if grep -q 'ENABLE_FOXGLOVE' "$COMPOSE_FILE"; then
    pass "ENABLE_FOXGLOVE env var wired into mowgli service"
  else
    fail "ENABLE_FOXGLOVE env var wired into mowgli service" "not found in fallback compose"
  fi

  section "Compose 'volumes:' section declares mowgli_maps"
  if grep -qE '^\s*mowgli_maps:' "$COMPOSE_FILE"; then
    pass "named volume mowgli_maps declared"
  else
    fail "named volume mowgli_maps declared" "missing — maps would be lost on restart"
  fi
fi

test_summary
