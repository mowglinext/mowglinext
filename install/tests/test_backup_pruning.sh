#!/usr/bin/env bash
# =============================================================================
# TODO-runtime-backups.md #1/#6 — "avoid creating backups when files are
# unchanged" / "avoid backup spam during repeated reruns/tests".
#
# migrate_runtime_paths() still backs up docker/.env and docker-compose.yaml
# unconditionally up front (it has to: the regenerated content isn't known
# until later steps run) — prune_backup_if_unchanged() is what removes that
# backup retroactively once it turns out to have been redundant. This tests
# the pure backup + prune logic in isolation, without a full harness_run
# (test_idempotency.sh already covers the full end-to-end flow, but a
# pre-existing, unrelated NTRIP flakiness there makes the two installer runs
# it compares never actually produce byte-identical output, so it can't
# exercise the "nothing changed, prune the backup" path at all).
# =============================================================================

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=lib/framework.sh
source "$SCRIPT_DIR/lib/framework.sh"

# backup_path_if_exists()/prune_backup_if_unchanged() call info/require_root_for
# (common.sh) but never with root-requiring effect on a plain file move in a
# sandbox — source common.sh for real so those calls behave normally instead
# of being stubbed out.
# shellcheck source=../lib/common.sh
source "$SCRIPT_DIR/../lib/common.sh"
# shellcheck source=../lib/deploy.sh
source "$SCRIPT_DIR/../lib/deploy.sh"

setup_sandbox

section "backup_path_if_exists() records the backup path it made"

work="$SANDBOX/work1"
mkdir -p "$work"
printf 'hello\n' > "$work/f"
backup_path_if_exists "$work/f"
assert_file_not_exists "original path moved away" "$work/f"
backup_count=$(find "$work" -maxdepth 1 -name 'f.old.*' | wc -l | tr -d ' ')
assert_eq "exactly one backup created" "1" "$backup_count"
assert_eq "LAST_BACKUP_PATH points at it" "1" \
  "$( [[ -n "$LAST_BACKUP_PATH" && -e "$LAST_BACKUP_PATH" ]] && echo 1 || echo 0 )"

section "backup_path_if_exists() on a missing path is a no-op"

LAST_BACKUP_PATH="unset-me"
backup_path_if_exists "$work/does-not-exist"
assert_eq "LAST_BACKUP_PATH cleared for a no-op backup" "" "$LAST_BACKUP_PATH"

section "prune_backup_if_unchanged() removes a backup that matches the regenerated file"

work="$SANDBOX/work2"
mkdir -p "$work"
printf 'same content\n' > "$work/f"
backup_path_if_exists "$work/f"
backup="$LAST_BACKUP_PATH"
# Simulate the later step regenerating the file with THE SAME content.
printf 'same content\n' > "$work/f"
prune_backup_if_unchanged "$work/f" "$backup"
assert_file_not_exists "unchanged-content backup was pruned" "$backup"
assert_file_exists "live (regenerated) file is untouched" "$work/f"

section "prune_backup_if_unchanged() keeps a backup that differs from the regenerated file"

work="$SANDBOX/work3"
mkdir -p "$work"
printf 'old content\n' > "$work/f"
backup_path_if_exists "$work/f"
backup="$LAST_BACKUP_PATH"
# Simulate the later step regenerating the file with DIFFERENT content.
printf 'new content\n' > "$work/f"
prune_backup_if_unchanged "$work/f" "$backup"
assert_file_exists "changed-content backup is kept" "$backup"
assert_eq "kept backup still holds the OLD content" "old content" "$(cat "$backup")"
assert_eq "live file holds the NEW content" "new content" "$(cat "$work/f")"

section "prune_backup_if_unchanged() is a safe no-op with no backup to prune"

work="$SANDBOX/work4"
mkdir -p "$work"
printf 'content\n' > "$work/f"
assert_exit_zero "empty backup path does not error" prune_backup_if_unchanged "$work/f" ""
assert_file_exists "live file survives a no-op prune call" "$work/f"

test_summary
