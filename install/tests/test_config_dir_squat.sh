#!/usr/bin/env bash
# =============================================================================
# Regression — ensure_default_configs recovers from a directory squatting at
# a bind-mounted *file* path.
#
# When `docker compose up` runs while a bind-mount source file is missing,
# Docker creates an empty *directory* at that host path. On the next install
# the `[ ! -f ]` guard in ensure_default_configs stays true (a dir is not a
# file) and `cp src dir/` would copy *into* the directory, leaving the broken
# empty-dir mount in place — the container then fails with "not a directory",
# or (for cyclonedds.xml) Cyclone DDS silently ignores the unreadable URI and
# falls back to defaults (MaxAutoParticipantIndex=9), starving discovery.
#
# This test reproduces the squat and asserts ensure_default_configs heals it.
# =============================================================================

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=lib/framework.sh
source "$SCRIPT_DIR/lib/framework.sh"
# shellcheck source=lib/mocks.sh
source "$SCRIPT_DIR/lib/mocks.sh"
# shellcheck source=lib/harness.sh
source "$SCRIPT_DIR/lib/harness.sh"

section "ensure_default_configs heals dir-squat at file-mount paths"

setup_sandbox
install_all_mocks

SANDBOX_REPO="$SANDBOX/repo"
sandbox_repo "$SANDBOX_REPO"
harness_init "$SANDBOX_REPO"   # sources libs; sets DOCKER_DIR/INSTALL_DIR

# Simulate Docker's empty-dir squat at both bind-mounted file paths.
mkdir -p "$DOCKER_DIR/config"
mkdir -p "$DOCKER_DIR/config/cyclonedds.xml"
mkdir -p "$DOCKER_DIR/config/mqtt/mosquitto.conf"

assert_dir_exists "cyclonedds.xml starts as a squatting directory" \
  "$DOCKER_DIR/config/cyclonedds.xml"
assert_dir_exists "mosquitto.conf starts as a squatting directory" \
  "$DOCKER_DIR/config/mqtt/mosquitto.conf"

if ensure_default_configs; then
  pass "ensure_default_configs returns success"
else
  fail "ensure_default_configs returns success" "non-zero exit"
fi

# After healing, both paths must be regular files (assert_file_exists uses
# `[ -f ]`, which is false for a directory — so this is the real check).
assert_file_exists "cyclonedds.xml healed to a regular file" \
  "$DOCKER_DIR/config/cyclonedds.xml"
assert_file_exists "mosquitto.conf healed to a regular file" \
  "$DOCKER_DIR/config/mqtt/mosquitto.conf"

# The squatted directories must have been moved aside, not deleted (the
# installer's conservative backup-not-destroy policy via backup_path_if_exists).
if compgen -G "$DOCKER_DIR/config/cyclonedds.xml.old.*" >/dev/null; then
  pass "squatted cyclonedds.xml directory backed up, not destroyed"
else
  fail "squatted cyclonedds.xml directory backed up, not destroyed" \
    "no cyclonedds.xml.old.* backup found"
fi

# Healed file must carry the real default contents, not be an empty placeholder.
if grep -q "MaxAutoParticipantIndex" "$DOCKER_DIR/config/cyclonedds.xml" 2>/dev/null; then
  pass "healed cyclonedds.xml has the bundled default contents"
else
  fail "healed cyclonedds.xml has the bundled default contents" \
    "MaxAutoParticipantIndex not found in materialised file"
fi

test_summary
