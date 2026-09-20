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

GPS_SERVICE_BLOCK="$(awk '
  /^  gps:$/ { in_service=1 }
  in_service && /^  [[:alnum:]_]+:$/ && $0 != "  gps:" { exit }
  in_service { print }
' "$COMPOSE_FILE")"

for required in   "UNIVERSAL_GNSS_CONFIGURATION_SCHEMA_VERSION:"   "device_cgroup_rules:"   "universal_gnss_launcher"   "fix_topic:=/gps/fix"   "status_topic:=/universal_gnss_receiver/status"   "rtcm_topic:=/universal_gnss_receiver/rtcm"   "ntrip_enabled:"; do
  # Here-string, not a pipe: under `pipefail`, grep -q exits on its first match
  # and a still-writing printf dies of SIGPIPE, failing the check at random
  # once the service block is long.
  if grep -qF -- "$required" <<<"$GPS_SERVICE_BLOCK"; then
    pass "compose contains sidecar env: $required"
  else
    fail "compose contains sidecar env: $required" "missing from generated gps service"
  fi
done

# The sidecar mounts mowgli_robot.yaml's directory read-only. The installer
# writes the short form; a stack rendered by the updater binary
# (MOWGLI_UPDATER_STACK_BINARY, as in CI) normalises it to the long form.
if grep -qE '(docker/config/mowgli:/config:ro|target: /config$)' <<<"$GPS_SERVICE_BLOCK"; then
  pass "gps sidecar mounts the mowgli_robot.yaml directory at /config"
else
  fail "gps sidecar mounts the mowgli_robot.yaml directory at /config" "mount missing from generated gps service"
fi
if grep -qE 'parameters\.yaml:/etc/universal_gnss|target: /etc/universal_gnss/parameters\.yaml' <<<"$GPS_SERVICE_BLOCK"; then
  fail "no derived GNSS parameter file is mounted from the host" "parameters.yaml bind still present"
else
  pass "no derived GNSS parameter file is mounted from the host"
fi

assert_not_contains "Universal GNSS command has no legacy ROS CLI remaps"   "--ros-args" "$GPS_SERVICE_BLOCK"
assert_contains "Universal GNSS uses combined native launch"   "receiver_and_ntrip.launch.py" "$GPS_SERVICE_BLOCK"

for forbidden in "gnss_unicore:" "UNICORE_IMAGE" "GPS_""RUNTIME_MODE:" "GPS_""PROTOCOL:" "GPS_""PORT:" "GPS_""BAUD:"; do
  if printf '%s' "$GPS_SERVICE_BLOCK" | grep -q "$forbidden"; then
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

section "MAVROS and Universal GNSS are independent sidecars"

MAVROS_REPO="$SANDBOX/repo_mavros"
sandbox_repo "$MAVROS_REPO"
harness_init "$MAVROS_REPO"
harness_set_preset backend=mavros gnss=auto gnss_connection=uart lidar=none tfluna=none

if ! harness_run; then
  fail "MAVROS harness_run" "non-zero exit"
else
  MAVROS_COMPOSE_FILE="$MAVROS_REPO/docker/docker-compose.yaml"
  MAVROS_CONTAINERS=$(grep -E '^\s+container_name:' "$MAVROS_COMPOSE_FILE" | awk '{print $2}' | sort)
  assert_contains "MAVROS sidecar is present" "mowgli-mavros" "$MAVROS_CONTAINERS"
  assert_contains "Universal GNSS sidecar remains present" "mowgli-gps" "$MAVROS_CONTAINERS"
  assert_not_contains "no standalone NTRIP service remains" "mowgli-ntrip" "$MAVROS_CONTAINERS"

  MAVROS_FRAGMENT_CONTENT="$(cat "$MAVROS_REPO/install/compose/docker-compose.mavros.yml")"
  assert_not_contains "MAVROS fragment has no standalone NTRIP launch" "mowgli_ntrip_client" "$MAVROS_FRAGMENT_CONTENT"
  assert_contains "MAVROS consumes its NTRIP-disabled config copy" "./docker/config/mavros:/ros2_ws/config:ro" "$MAVROS_FRAGMENT_CONTENT"

  MAVROS_CONFIG="$(cat "$MAVROS_REPO/docker/config/mavros/mowgli_robot.yaml")"
  assert_match "MAVROS runtime config disables NTRIP" '^[[:space:]]+ntrip_enabled:[[:space:]]+false[[:space:]]*$' "$MAVROS_CONFIG"

  MAVROS_ENV="$(cat "$MAVROS_REPO/docker/.env")"
  assert_contains "MAVROS mode preserves GNSS_BACKEND=universal" "GNSS_BACKEND=universal" "$MAVROS_ENV"
  assert_contains "MAVROS mode preserves GNSS_STACK=universal" "GNSS_STACK=universal" "$MAVROS_ENV"
  assert_contains "MAVROS mode enables MAVROS" "MAVROS_ENABLED=true" "$MAVROS_ENV"
fi

section "Compose env-var expansion does not have unresolved placeholders"

if real_docker_compose_available; then
  # After `docker compose config` fully expands ${VAR} references, no `${`
  # placeholder should remain. `image:` is the most common breakage point.
  EXPANDED=$(HOME="$ORIG_HOME" docker compose -f "$COMPOSE_FILE" --env-file "$ENV_FILE" config 2>/dev/null)
  if grep -qE 'image:.*\$\{' <<<"$EXPANDED"; then
    fail "no unresolved \${VAR} in image:" \
      "$(printf '%s' "$EXPANDED" | grep -E 'image:.*\$\{'  | head -1)"
  else
    pass "no unresolved \${VAR} in image:"
  fi

  if grep -qE 'UNIVERSAL_GNSS_CONFIGURATION_SCHEMA_VERSION: "?1"?$' <<<"$EXPANDED"; then
    pass "Universal GNSS sidecar uses schema version 1"
  else
    fail "Universal GNSS sidecar uses schema version 1" \
      "$(printf '%s' "$EXPANDED" | grep -n 'UNIVERSAL_GNSS_CONFIGURATION_SCHEMA_VERSION:' | head -1)"
  fi

  if grep -qE 'c 166:\* rw' <<<"$EXPANDED"; then
    pass "GNSS sidecar may open tty devices (cgroup rules, not privileged)"
  else
    fail "GNSS sidecar may open tty devices (cgroup rules, not privileged)" "device_cgroup_rules missing"
  fi

  # Foxglove environment toggle present in expanded mowgli service env
  if grep -qE 'ENABLE_FOXGLOVE' <<<"$EXPANDED"; then
    pass "ENABLE_FOXGLOVE env var wired into mowgli service"
  else
    fail "ENABLE_FOXGLOVE env var wired into mowgli service" "not found in expanded compose"
  fi

  section "Compose 'volumes:' section declares mowgli_maps"

  # mowgli_maps is the bind-mount that persists garden_map + fusion_graph
  # files across container restarts.
  if grep -qE '^\s+mowgli_maps:' <<<"$EXPANDED"; then
    pass "named volume mowgli_maps declared"
  else
    fail "named volume mowgli_maps declared" "missing — maps would be lost on restart"
  fi
else
  pass "no unresolved \${VAR} in image: (skipped; docker unavailable)"
  if grep -q 'c 166:\* rw' "$COMPOSE_FILE"; then
    pass "GNSS tty cgroup rules present (fallback compose)"
  else
    fail "GNSS tty cgroup rules present (fallback compose)" "device_cgroup_rules missing"
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

# The MAVROS coexistence scenario above reinitializes the shared installer
# harness against MAVROS_REPO. Restore the original Mowgli preset before the
# managed-updater checks below; managed release updates intentionally support
# the Mowgli hardware backend only.
harness_init "$SANDBOX_REPO"
harness_set_preset gnss=auto gnss_connection=uart lidar=ldlidar-uart tfluna=none
if ! harness_run; then
  fail "restore default Mowgli harness" "non-zero exit"
fi
COMPOSE_FILE="$SANDBOX_REPO/docker/docker-compose.yaml"
ENV_FILE="$SANDBOX_REPO/docker/.env"

section "Managed updater Compose layout"
if real_docker_compose_available && [[ -x "${MOWGLI_UPDATER_STACK_BINARY:-/usr/local/bin/mowgli-updater}" ]]; then
  touch "$DOCKER_DIR/.updater-managed"
  if build_compose_stack && write_compose_merged; then
    pass "managed Compose generated"
    if grep -q 'mowgli-watchtower' "$COMPOSE_FILE"; then fail "managed stack excludes Watchtower"; else pass "managed stack excludes Watchtower"; fi
    MANAGED=$(HOME="$ORIG_HOME" PATH="$ORIG_PATH" docker compose -f "$COMPOSE_FILE" --env-file "$ENV_FILE" config --format json)
    if printf '%s' "$MANAGED" | python3 -c 'import json,sys; s=json.load(sys.stdin)["services"]; assert s["gui"]["labels"]["com.centurylinklabs.watchtower.enable"]=="false"; assert all(s[k]["environment"]["MOWGLI_UPDATE_MAINTENANCE"]=="/var/lib/mowgli-updater/maintenance" for k in ("gui","mowgli")); assert "GPS_PROTOCOL" in s["mowgli"]["environment"]; assert "GPS_PROTOCOL" not in s["gps"]["environment"]'; then
      pass "maintenance gates, Watchtower opt-out and GNSS environment scope"
    else fail "managed Compose contract"; fi
    cp "$COMPOSE_FILE" "$SANDBOX/applied-compose.json"
    printf '{"id":"fixture-release"}\n' > "$DOCKER_DIR/stack-release.json"
    LIDAR_ENABLED=false
    if write_compose_merged && cmp -s "$COMPOSE_FILE" "$SANDBOX/applied-compose.json"; then
      pass "published stack survives installer reconfiguration"
    else fail "published stack survives installer reconfiguration"; fi
    if python3 - "$DOCKER_DIR" <<'PY'
import json, pathlib, sys
root = pathlib.Path(sys.argv[1])
wanted = json.loads((root / 'stack-selection.json').read_text())
applied = json.loads((root / 'stack-applied-selection.json').read_text())
assert wanted['options']['lidar'] == 'none'
assert applied['options']['lidar'] == 'ldlidar'
PY
    then pass "new installer selection remains pending until reviewed"; else fail "pending installer selection"; fi
  else fail "managed Compose generated"; fi
fi

test_summary
