#!/usr/bin/env bash
# =============================================================================
# Regression — stack.sh regen recovers from a directory squatting at a
# bind-mounted *file* path and recreates derived runtime configuration.
#
# When `docker compose up` runs while a bind-mount source file is missing,
# Docker creates an empty *directory* at that host path. This test reproduces
# that state for the static files and the two generated GNSS/MAVROS files.
# =============================================================================

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=lib/framework.sh
source "$SCRIPT_DIR/lib/framework.sh"
# shellcheck source=lib/mocks.sh
source "$SCRIPT_DIR/lib/mocks.sh"
# shellcheck source=lib/harness.sh
source "$SCRIPT_DIR/lib/harness.sh"

section "stack.sh regen heals dir-squat at file-mount paths"

setup_sandbox
install_all_mocks

SANDBOX_REPO="$SANDBOX/repo"
sandbox_repo "$SANDBOX_REPO"
harness_init "$SANDBOX_REPO"
harness_set_preset gnss=auto gnss_connection=uart lidar=none tfluna=none
harness_run

# Simulate Docker's empty-dir squat at every relevant bind-mounted file path.
# Remove tracked/default files first: mkdir -p cannot turn an existing regular
# file into the Docker-created directory this regression covers.
rm -f "$DOCKER_DIR/config/cyclonedds.xml"
rm -f "$DOCKER_DIR/config/mqtt/mosquitto.conf"
rm -f "$DOCKER_DIR/config/universal_gnss/parameters.yaml"
rm -f "$DOCKER_DIR/config/mavros/mowgli_robot.yaml"
mkdir -p "$DOCKER_DIR/config/cyclonedds.xml"
mkdir -p "$DOCKER_DIR/config/mqtt/mosquitto.conf"
mkdir -p "$DOCKER_DIR/config/universal_gnss/parameters.yaml"
mkdir -p "$DOCKER_DIR/config/mavros/mowgli_robot.yaml"

assert_dir_exists "cyclonedds.xml starts as a squatting directory" \
  "$DOCKER_DIR/config/cyclonedds.xml"
assert_dir_exists "mosquitto.conf starts as a squatting directory" \
  "$DOCKER_DIR/config/mqtt/mosquitto.conf"
assert_dir_exists "Universal GNSS parameters.yaml starts as a squatting directory" \
  "$DOCKER_DIR/config/universal_gnss/parameters.yaml"
assert_dir_exists "MAVROS mowgli_robot.yaml starts as a squatting directory" \
  "$DOCKER_DIR/config/mavros/mowgli_robot.yaml"

if "$SANDBOX_REPO/docker/stack.sh" regen; then
  pass "stack.sh regen returns success"
else
  fail "stack.sh regen returns success" "non-zero exit"
fi

# assert_file_exists uses `[ -f ]`, which is false for a directory.
assert_file_exists "cyclonedds.xml healed to a regular file" \
  "$DOCKER_DIR/config/cyclonedds.xml"
assert_file_exists "mosquitto.conf healed to a regular file" \
  "$DOCKER_DIR/config/mqtt/mosquitto.conf"
assert_file_exists "Universal GNSS parameters.yaml healed to a regular file" \
  "$DOCKER_DIR/config/universal_gnss/parameters.yaml"
assert_file_exists "MAVROS mowgli_robot.yaml healed to a regular file" \
  "$DOCKER_DIR/config/mavros/mowgli_robot.yaml"

# The squatted directories must have been moved aside, not deleted (the
# installer's conservative backup-not-destroy policy via backup_path_if_exists).
if compgen -G "$DOCKER_DIR/config/cyclonedds.xml.old.*" >/dev/null; then
  pass "squatted cyclonedds.xml directory backed up, not destroyed"
else
  fail "squatted cyclonedds.xml directory backed up, not destroyed" \
    "no cyclonedds.xml.old.* backup found"
fi

if compgen -G "$DOCKER_DIR/config/universal_gnss/parameters.yaml.old.*" >/dev/null; then
  pass "squatted Universal GNSS parameters.yaml directory backed up, not destroyed"
else
  fail "squatted Universal GNSS parameters.yaml directory backed up, not destroyed" \
    "no parameters.yaml.old.* backup found"
fi

if compgen -G "$DOCKER_DIR/config/mavros/mowgli_robot.yaml.old.*" >/dev/null; then
  pass "squatted MAVROS mowgli_robot.yaml directory backed up, not destroyed"
else
  fail "squatted MAVROS mowgli_robot.yaml directory backed up, not destroyed" \
    "no mowgli_robot.yaml.old.* backup found"
fi

# Healed file must carry the real default contents, not be an empty placeholder.
if grep -q "MaxAutoParticipantIndex" "$DOCKER_DIR/config/cyclonedds.xml" 2>/dev/null; then
  pass "healed cyclonedds.xml has the bundled default contents"
else
  fail "healed cyclonedds.xml has the bundled default contents" \
    "MaxAutoParticipantIndex not found in materialised file"
fi

if grep -q '^    serial_device: /dev/gnss-receiver$' \
  "$DOCKER_DIR/config/universal_gnss/parameters.yaml" 2>/dev/null; then
  pass "regenerated Universal GNSS config keeps the stable receiver path"
else
  fail "regenerated Universal GNSS config keeps the stable receiver path" \
    "serial_device was not regenerated as /dev/gnss-receiver"
fi

if grep -q '^[[:space:]]*ntrip_enabled: false$' \
  "$DOCKER_DIR/config/mavros/mowgli_robot.yaml" 2>/dev/null; then
  pass "regenerated MAVROS config keeps NTRIP disabled"
else
  fail "regenerated MAVROS config keeps NTRIP disabled" \
    "ntrip_enabled was not forced to false"
fi

# A second regen must not alter either derived output.
first_ug_checksum="$(sha256sum "$DOCKER_DIR/config/universal_gnss/parameters.yaml" | awk '{print $1}')"
first_mavros_checksum="$(sha256sum "$DOCKER_DIR/config/mavros/mowgli_robot.yaml" | awk '{print $1}')"
if "$SANDBOX_REPO/docker/stack.sh" regen; then
  pass "second stack.sh regen returns success"
else
  fail "second stack.sh regen returns success" "non-zero exit"
fi
assert_eq "Universal GNSS regen is idempotent" "$first_ug_checksum" \
  "$(sha256sum "$DOCKER_DIR/config/universal_gnss/parameters.yaml" | awk '{print $1}')"
assert_eq "MAVROS regen is idempotent" "$first_mavros_checksum" \
  "$(sha256sum "$DOCKER_DIR/config/mavros/mowgli_robot.yaml" | awk '{print $1}')"

# Missing derived files are also recreated as regular files on the next regen.
rm -f "$DOCKER_DIR/config/universal_gnss/parameters.yaml"
rm -f "$DOCKER_DIR/config/mavros/mowgli_robot.yaml"
if "$SANDBOX_REPO/docker/stack.sh" regen; then
  pass "stack.sh regen recreates deleted derived files"
else
  fail "stack.sh regen recreates deleted derived files" "non-zero exit"
fi
assert_file_exists "deleted Universal GNSS parameters.yaml is recreated as a regular file" \
  "$DOCKER_DIR/config/universal_gnss/parameters.yaml"
assert_file_exists "deleted MAVROS mowgli_robot.yaml is recreated as a regular file" \
  "$DOCKER_DIR/config/mavros/mowgli_robot.yaml"

test_summary
