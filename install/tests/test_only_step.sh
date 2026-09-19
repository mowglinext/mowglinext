#!/usr/bin/env bash
# =============================================================================
# --only=<step> (issue #632, option 1) — run exactly one named step instead
# of the full install flow. Invokes the REAL install/mowglinext.sh as a
# subprocess (like test_check_mode.sh) rather than the simplified in-process
# harness_run(), because the dispatcher under test (run_only_step(), and the
# ONLY_STEP branch in main()) lives in mowglinext.sh's own main(), which
# harness_run() never calls.
#
# --only= deliberately skips select_repo_branch()/select_language() (see the
# comment above the ONLY_STEP branch in mowglinext.sh) precisely so it stays
# usable non-interactively like this — without that, this test would hang on
# select_language()'s unconditional prompt() read from /dev/tty.
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

section "--only=bogus rejects with a clear error and lists valid steps"

repo_bad="$SANDBOX/repo_bad"
sandbox_repo "$repo_bad"
harness_init "$repo_bad"

output_bad="$(bash "$repo_bad/install/mowglinext.sh" --only=bogus 2>&1)"
ec_bad=$?

assert_neq "--only=bogus exits non-zero" "0" "$ec_bad"
assert_contains "--only=bogus names the bad step" "Unknown --only= step: 'bogus'" "$output_bad"
assert_contains "--only=bogus lists a real step (updater)" "updater" "$output_bad"
assert_contains "--only=bogus lists a real step (env)" "env" "$output_bad"

section "--only=env runs just that step, non-interactively, from a blank slate"

repo_env="$SANDBOX/repo_env"
sandbox_repo "$repo_env"
harness_init "$repo_env"

assert_file_not_exists "docker/.env absent before the run" "$repo_env/docker/.env"
assert_file_not_exists "mowgli_robot.yaml absent before the run" \
  "$repo_env/docker/config/mowgli/mowgli_robot.yaml"

output_env="$(timeout 30 bash "$repo_env/install/mowglinext.sh" --only=env 2>&1)"
ec_env=$?

assert_eq "--only=env exits 0" "0" "$ec_env"
assert_file_exists "docker/.env created by --only=env" "$repo_env/docker/.env"
assert_contains "docker/.env has a real default key" "ROS_DOMAIN_ID=0" \
  "$(cat "$repo_env/docker/.env")"

# The env step alone must not reach into steps it wasn't asked for: no
# mowgli_robot.yaml (that's write_config, via the "mower" step), and no
# interactive hardware-selection banners (those belong to "backend"/"gps"/
# "lidar"/"rangefinders", never invoked here).
assert_file_not_exists "mowgli_robot.yaml still absent (mower step never ran)" \
  "$repo_env/docker/config/mowgli/mowgli_robot.yaml"
assert_not_contains "no hardware-backend prompt banner leaked in" "Select hardware backend" "$output_env"
# Not a bare "LiDAR" substring check — the System Health Check section at the
# end of main() (which --only= runs too, same as the full flow) legitimately
# prints LiDAR device/container status. Check for the interactive TYPE-SELECT
# menu specifically (configure_lidar(), never printed outside it).
assert_not_contains "no LiDAR configuration menu leaked in" "RPLidar Slamtec" "$output_env"

section "--only= skips the interactive branch/language prompts"

# A real select_language() call reads from /dev/tty unconditionally (no
# MOWGLI_LANG guard) and would hang or error out with no controlling
# terminal in this harness — the fact that the run above completed inside
# the 30s timeout and printed no language/branch prompt text is itself the
# proof --only= took the non-interactive path.
assert_not_contains "no language prompt leaked in" "Language / Langue" "$output_env"
assert_not_contains "no branch-switch banner leaked in" "Switching repository to" "$output_env"

test_summary
