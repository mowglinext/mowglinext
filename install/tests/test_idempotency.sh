#!/usr/bin/env bash
# =============================================================================
# A.7 Idempotency — running the installer twice doesn't corrupt state
#
# After the first run, `migrate_runtime_paths` will move existing .env /
# docker-compose.yaml to .old.<timestamp> backups. The second run must
# regenerate the same content (apart from the backup), and the resulting
# files must still be valid YAML / dotenv with the same key set.
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

# ── First run ────────────────────────────────────────────────────────────
section "First installer run"

harness_init "$SANDBOX_REPO"
harness_set_preset gnss=gps gps=ubx-uart lidar=ldlidar-uart tfluna=none
if harness_run; then
  pass "first run: harness_run"
else
  fail "first run: harness_run"
  test_summary
  exit 1
fi

ENV1="$(cat "$SANDBOX_REPO/docker/.env")"
COMPOSE1_HASH=$(shasum -a 256 "$SANDBOX_REPO/docker/docker-compose.yaml" | cut -d' ' -f1)
ROBOT_YAML1="$(cat "$SANDBOX_REPO/docker/config/mowgli/mowgli_robot.yaml")"

# ── Second run, same preset ───────────────────────────────────────────────
section "Second installer run, same preset"

# Sleep 1s so any backup with timestamp-based name doesn't collide
sleep 1

harness_init "$SANDBOX_REPO"
harness_set_preset gnss=gps gps=ubx-uart lidar=ldlidar-uart tfluna=none
if harness_run; then
  pass "second run: harness_run"
else
  fail "second run: harness_run"
  test_summary
  exit 1
fi

ENV2="$(cat "$SANDBOX_REPO/docker/.env")"
COMPOSE2_HASH=$(shasum -a 256 "$SANDBOX_REPO/docker/docker-compose.yaml" | cut -d' ' -f1)
ROBOT_YAML2="$(cat "$SANDBOX_REPO/docker/config/mowgli/mowgli_robot.yaml")"

assert_eq ".env stable across runs" "$ENV1" "$ENV2"
assert_eq "docker-compose.yaml stable across runs" "$COMPOSE1_HASH" "$COMPOSE2_HASH"
assert_eq "mowgli_robot.yaml stable across runs" "$ROBOT_YAML1" "$ROBOT_YAML2"

section "Backup files were created during second run"

# migrate_runtime_paths backs up .env and docker-compose.yaml on each run.
backup_count=$(find "$SANDBOX_REPO/docker" -maxdepth 1 -name '.env.old.*' | wc -l | tr -d ' ')
[ "$backup_count" -ge 1 ] && pass "at least one .env.old.* backup created" \
  || fail "at least one .env.old.* backup created" "found $backup_count"

compose_backup_count=$(find "$SANDBOX_REPO/docker" -maxdepth 1 -name 'docker-compose.yaml.old.*' | wc -l | tr -d ' ')
[ "$compose_backup_count" -ge 1 ] && pass "at least one docker-compose.yaml.old.* backup created" \
  || fail "at least one docker-compose.yaml.old.* backup created" "found $compose_backup_count"

# Idempotency negative check: live config files still parse after a re-run
section "Generated files still valid after re-run"

if real_docker_compose_available; then
  if HOME="$ORIG_HOME" docker compose \
      -f "$SANDBOX_REPO/docker/docker-compose.yaml" \
      --env-file "$SANDBOX_REPO/docker/.env" \
      config -q 2>/dev/null; then
    pass "compose config -q after re-run"
  else
    fail "compose config -q after re-run" "compose became invalid"
  fi
else
  if [ -s "$SANDBOX_REPO/docker/docker-compose.yaml" ]; then
    pass "compose fallback file present after re-run"
  else
    fail "compose fallback file present after re-run" "generated compose is empty"
  fi
fi

# ── Third run, different preset → outputs must reflect new preset ─────────
section "Third run with different preset propagates change"

sleep 1
harness_init "$SANDBOX_REPO"
harness_set_preset gnss=gps gps=nmea-uart lidar=rplidar-usb tfluna=front
harness_run >/dev/null 2>&1

new_proto=$(grep -E "^GPS_PROTOCOL=" "$SANDBOX_REPO/docker/.env" | cut -d= -f2)
new_baud=$(grep -E "^GPS_BAUD=" "$SANDBOX_REPO/docker/.env" | cut -d= -f2)
new_family=$(grep -E "^GNSS_RECEIVER_FAMILY=" "$SANDBOX_REPO/docker/.env" | cut -d= -f2)
new_lidar=$(grep -E "^LIDAR_TYPE=" "$SANDBOX_REPO/docker/.env" | cut -d= -f2)

assert_eq "third run: GPS_PROTOCOL switched to NMEA" "NMEA" "$new_proto"
assert_eq "third run: GNSS_RECEIVER_FAMILY switched to nmea" "nmea" "$new_family"
assert_eq "third run: GPS_BAUD stays at the validated runtime target" "921600" "$new_baud"
assert_eq "third run: LIDAR_TYPE switched to rplidar" "rplidar" "$new_lidar"

test_summary
