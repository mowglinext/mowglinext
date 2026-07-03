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
harness_set_preset gnss=auto gnss_connection=uart lidar=ldlidar-uart tfluna=none
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
harness_set_preset gnss=auto gnss_connection=uart lidar=ldlidar-uart tfluna=none
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

# ── Third run, preserve existing YAML when no new GNSS flags are supplied ──
section "Third run preserves existing YAML GNSS config by default"

python3 - <<'PY' "$SANDBOX_REPO/docker/config/mowgli/mowgli_robot.yaml"
from pathlib import Path
import sys
path = Path(sys.argv[1])
text = path.read_text()
replacements = {
    'gnss_receiver_family: "auto"': 'gnss_receiver_family: "unicore"',
    'gnss_serial_device: "/dev/ttyAMA4"': 'gnss_serial_device: "/dev/serial/by-id/existing-gnss"',
    'gnss_serial_baud: 921600': 'gnss_serial_baud: 460800',
    'gnss_frame_id: "gps_link"': 'gnss_frame_id: "gps_custom"',
    'ntrip_enabled: true': 'ntrip_enabled: false',
}
for old, new in replacements.items():
    text = text.replace(old, new)
path.write_text(text)
PY

sed -i 's/^GNSS_RECEIVER_FAMILY=.*/GNSS_RECEIVER_FAMILY=auto/' "$SANDBOX_REPO/docker/.env"
sed -i 's|^GNSS_SERIAL_DEVICE=.*|GNSS_SERIAL_DEVICE=/dev/ttyAMA4|' "$SANDBOX_REPO/docker/.env"
sed -i 's/^GNSS_SERIAL_BAUD=.*/GNSS_SERIAL_BAUD=921600/' "$SANDBOX_REPO/docker/.env"
sed -i 's/^GNSS_FRAME_ID=.*/GNSS_FRAME_ID=gps_link/' "$SANDBOX_REPO/docker/.env"
sed -i 's/^GNSS_NTRIP_ENABLED=.*/GNSS_NTRIP_ENABLED=true/' "$SANDBOX_REPO/docker/.env"

sleep 1
harness_init "$SANDBOX_REPO"
if harness_run >/dev/null 2>&1; then
  pass "third run: harness_run without new GNSS flags"
else
  fail "third run: harness_run without new GNSS flags"
fi

ROBOT_YAML3="$(cat "$SANDBOX_REPO/docker/config/mowgli/mowgli_robot.yaml")"
ENV3="$(cat "$SANDBOX_REPO/docker/.env")"

assert_match "third run keeps gnss_receiver_family from YAML" \
  '^[[:space:]]+gnss_receiver_family:[[:space:]]+"?unicore"?[[:space:]]*$' "$ROBOT_YAML3"
assert_match "third run keeps gnss_serial_device from YAML" \
  '^[[:space:]]+gnss_serial_device:[[:space:]]+"?/dev/serial/by-id/existing-gnss"?[[:space:]]*$' "$ROBOT_YAML3"
assert_match "third run keeps gnss_serial_baud from YAML" \
  '^[[:space:]]+gnss_serial_baud:[[:space:]]+460800[[:space:]]*$' "$ROBOT_YAML3"
assert_match "third run keeps gnss_frame_id from YAML" \
  '^[[:space:]]+gnss_frame_id:[[:space:]]+"?gps_custom"?[[:space:]]*$' "$ROBOT_YAML3"
assert_match "third run keeps ntrip_enabled=false from YAML" \
  '^[[:space:]]+ntrip_enabled:[[:space:]]+false[[:space:]]*$' "$ROBOT_YAML3"
assert_contains "third run updates fallback env from preserved YAML receiver family" "GNSS_RECEIVER_FAMILY=unicore" "$ENV3"
assert_contains "third run updates fallback env from preserved YAML serial device" "GNSS_SERIAL_DEVICE=/dev/serial/by-id/existing-gnss" "$ENV3"
assert_contains "third run updates fallback env from preserved YAML serial baud" "GNSS_SERIAL_BAUD=460800" "$ENV3"
assert_contains "third run updates fallback env from preserved YAML ntrip false" "GNSS_NTRIP_ENABLED=false" "$ENV3"

source "$SANDBOX_REPO/install/lib/checks.sh"
check_output="$(check_generated_gps_yaml_alignment 2>&1 || true)"
assert_not_contains "check_generated_gps_yaml_alignment no longer errors on YAML/env divergence" "diverges between docker/.env and mowgli_robot.yaml" "$check_output"
assert_contains "check_generated_gps_yaml_alignment reports YAML-first resolution" "resolves from mowgli_robot.yaml" "$check_output"

# ── Fourth run, different preset → outputs must reflect the explicit change ─
section "Fourth run with different preset propagates explicit change"

sleep 1
harness_init "$SANDBOX_REPO"
harness_set_preset gnss=nmea gnss_connection=uart lidar=rplidar-usb tfluna=front
harness_run >/dev/null 2>&1

new_family=$(grep -E "^GNSS_RECEIVER_FAMILY=" "$SANDBOX_REPO/docker/.env" | cut -d= -f2)
new_device=$(grep -E "^GNSS_SERIAL_DEVICE=" "$SANDBOX_REPO/docker/.env" | cut -d= -f2)
new_baud=$(grep -E "^GNSS_SERIAL_BAUD=" "$SANDBOX_REPO/docker/.env" | cut -d= -f2)
new_lidar=$(grep -E "^LIDAR_TYPE=" "$SANDBOX_REPO/docker/.env" | cut -d= -f2)

assert_eq "fourth run: GNSS_RECEIVER_FAMILY switched to nmea" "nmea" "$new_family"
assert_eq "fourth run: GNSS_SERIAL_DEVICE switches to the explicit UART fallback" "/dev/ttyAMA4" "$new_device"
assert_eq "fourth run: GNSS_SERIAL_BAUD keeps preserved YAML baud without explicit baud flag" "460800" "$new_baud"
assert_eq "fourth run: LIDAR_TYPE switched to rplidar" "rplidar" "$new_lidar"

test_summary
