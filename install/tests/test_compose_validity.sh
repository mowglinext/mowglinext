#!/usr/bin/env bash
# =============================================================================
# A.2 Output sanity — docker-compose.yaml is valid + has required services
#
# Validates the merged compose file via `docker compose config -q` and
# spot-checks that the service blocks the user's preset implies are
# actually present (mowgli, gui, lidar, mavros) and that
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

GPS_SERVICE_BLOCK="$(awk '
  /^  gps:$/ { in_service=1 }
  in_service && /^  [[:alnum:]_]+:$/ && $0 != "  gps:" { exit }
  in_service { print }
' "$COMPOSE_FILE")"

for required in \
  "UNIVERSAL_GNSS_CONFIGURATION_SCHEMA_VERSION:" \
  "GNSS_NTRIP_ENABLED:" \
  "/dev/gnss-receiver" \
  "parameters_file:=/etc/universal_gnss/parameters.yaml" \
  "fix_topic:=/gps/fix" \
  "status_topic:=/universal_gnss_receiver/status" \
  "rtcm_topic:=/universal_gnss_receiver/rtcm"; do
  if printf '%s' "$GPS_SERVICE_BLOCK" | grep -q "$required"; then
    pass "compose contains sidecar env: $required"
  else
    fail "compose contains sidecar env: $required" "missing from generated gps service"
  fi
done

assert_not_contains "Universal GNSS command has no legacy ROS CLI remaps" \
  "--ros-args" "$GPS_SERVICE_BLOCK"

assert_contains "Universal GNSS uses the combined native launch" \
  "receiver_and_ntrip.launch.py" "$GPS_SERVICE_BLOCK"

for forbidden in "gnss_unicore:" "UNICORE_IMAGE" "GPS_""RUNTIME_MODE:" "GPS_""PROTOCOL:" "GPS_""PORT:" "GPS_""BAUD:"; do
  if printf '%s' "$GPS_SERVICE_BLOCK" | grep -q "$forbidden"; then
    fail "legacy standalone GNSS absent: $forbidden" "found in generated gps service"
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

section "MAVROS compose has one external sidecar that owns NTRIP"

MAVROS_REPO="$SANDBOX/repo_mavros"
sandbox_repo "$MAVROS_REPO"
harness_init "$MAVROS_REPO"
IMAGE_TAG="must-not-affect-mavros"
harness_set_preset backend=mavros gnss=auto gnss_connection=uart lidar=ldlidar-uart tfluna=none

if ! harness_run; then
  fail "MAVROS harness_run" "non-zero exit"
else
  MAVROS_COMPOSE_FILE="$MAVROS_REPO/docker/docker-compose.yaml"
  MAVROS_CONTAINERS=$(grep -E '^\s+container_name:' "$MAVROS_COMPOSE_FILE" | awk '{print $2}' | sort)
  assert_contains "MAVROS compose has the external sidecar" "mowgli-mavros" "$MAVROS_CONTAINERS"
  assert_not_contains "MAVROS compose has no separate NTRIP container" "mowgli-ntrip" "$MAVROS_CONTAINERS"
  assert_contains "MAVROS compose keeps the independent Universal GNSS sidecar" "mowgli-gps" "$MAVROS_CONTAINERS"

  MAVROS_FRAGMENT_CONTENT="$(cat "$MAVROS_REPO/install/compose/docker-compose.mavros.yml")"
  assert_contains "MAVROS sidecar uses MAVROS_IMAGE" "image: \${MAVROS_IMAGE}" "$MAVROS_FRAGMENT_CONTENT"
  assert_not_contains "MAVROS sidecar never uses MOWGLI_ROS2_IMAGE" "MOWGLI_ROS2_IMAGE" "$MAVROS_FRAGMENT_CONTENT"
  assert_not_contains "MAVROS sidecar has no standalone NTRIP launch" "mowgli_ntrip_client" "$MAVROS_FRAGMENT_CONTENT"
  assert_contains "MAVROS sidecar receives its NTRIP-disabled config copy" \
    "./docker/config/mavros:/ros2_ws/config:ro" "$MAVROS_FRAGMENT_CONTENT"

  MAVROS_ENV_CONTENT="$(cat "$MAVROS_REPO/docker/.env")"
  assert_contains "MAVROS image ignores MowgliNext IMAGE_TAG" \
    "MAVROS_IMAGE=ghcr.io/pepeuch/mowglimavros/mowgli-mavros-sidecar:kilted@sha256:04e4eb17b0f5ce38f882f68346b1694774fa87e1945b38b57c94f90da34dd560" \
    "$MAVROS_ENV_CONTENT"
fi

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

  if printf '%s' "$EXPANDED" | grep -qE 'UNIVERSAL_GNSS_CONFIGURATION_SCHEMA_VERSION: "?1"?$'; then
    pass "Universal GNSS sidecar uses schema version 1"
  else
    fail "Universal GNSS sidecar uses schema version 1" \
      "$(printf '%s' "$EXPANDED" | grep -n 'UNIVERSAL_GNSS_CONFIGURATION_SCHEMA_VERSION:' | head -1)"
  fi

  if printf '%s' "$EXPANDED" | grep -qE 'target: /dev/gnss-receiver$'; then
    pass "stable GNSS device mapping present"
  else
    fail "stable GNSS device mapping present" "/dev/gnss-receiver mapping missing"
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
  if grep -q '/dev/gnss-receiver' "$COMPOSE_FILE"; then
    pass "stable GNSS device mapping present (fallback compose)"
  else
    fail "stable GNSS device mapping present (fallback compose)" "GNSS mapping missing"
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
