#!/usr/bin/env bash
# =============================================================================
# Repository/deploy flow — setup_directory must stay non-destructive
# =============================================================================

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=lib/framework.sh
source "$SCRIPT_DIR/lib/framework.sh"

setup_sandbox

source_test_libs() {
  local repo_dir="${1:?source_test_libs: missing repo dir}"

  export MOWGLI_HOME="$repo_dir"

  # shellcheck source=/dev/null
  source "$repo_dir/install/lib/common.sh"
  # shellcheck source=/dev/null
  source "$repo_dir/install/lib/i18n.sh"
  # shellcheck source=/dev/null
  source "$repo_dir/install/lib/config.sh"
  # shellcheck source=/dev/null
  source "$repo_dir/install/lib/deploy.sh"

  load_locale
  reapply_test_assertions
}

run_setup_directory_capture() {
  local repo_dir="${1:?run_setup_directory_capture: missing repo dir}"
  local output_file="${2:?run_setup_directory_capture: missing output file}"

  (
    source_test_libs "$repo_dir"
    setup_directory
  ) >"$output_file" 2>&1
}

create_work_repo_with_remote() {
  local source_repo="${1:?create_work_repo_with_remote: missing source repo}"
  local bare_repo="${2:?create_work_repo_with_remote: missing bare repo}"
  local work_repo="${3:?create_work_repo_with_remote: missing work repo}"

  git clone --bare "$source_repo" "$bare_repo" >/dev/null 2>&1
  git clone "$bare_repo" "$work_repo" >/dev/null 2>&1
}

make_remote_commit() {
  local bare_repo="${1:?make_remote_commit: missing bare repo}"
  local upstream_repo="$SANDBOX/upstream_$RANDOM"

  git clone "$bare_repo" "$upstream_repo" >/dev/null 2>&1
  printf 'upstream %s\n' "$(date +%s)" >> "$upstream_repo/REMOTE_STATUS.txt"
  git -C "$upstream_repo" add REMOTE_STATUS.txt >/dev/null 2>&1
  git -C "$upstream_repo" -c user.email=test@example.com -c user.name=test \
    commit -m "upstream change" >/dev/null 2>&1
  git -C "$upstream_repo" push origin main >/dev/null 2>&1
}

write_runtime_files() {
  local repo_dir="${1:?write_runtime_files: missing repo dir}"

  mkdir -p "$repo_dir/docker/config/mowgli" \
           "$repo_dir/docker/config/mqtt" \
           "$repo_dir/docker/config/om"

  cat > "$repo_dir/docker/.env" <<'EOF'
ROS_DOMAIN_ID=42
GPS_PROTOCOL=UBX
EOF

  cat > "$repo_dir/docker/config/mowgli/mowgli_robot.yaml" <<'EOF'
datum_lat: 48.0
EOF

  cat > "$repo_dir/docker/config/mqtt/mosquitto.conf" <<'EOF'
listener 1883
EOF

  cat > "$repo_dir/docker/config/om/mower_config.sh" <<'EOF'
#!/usr/bin/env bash
export TEST_MOWER_CONFIG=1
EOF
}

section "clean repository stays untouched"

SOURCE_REPO="$SANDBOX/source"
REMOTE_REPO="$SANDBOX/remote.git"
WORK_REPO="$SANDBOX/work"

sandbox_repo "$SOURCE_REPO"
create_work_repo_with_remote "$SOURCE_REPO" "$REMOTE_REPO" "$WORK_REPO"

head_before="$(git -C "$WORK_REPO" rev-parse HEAD)"
clean_output="$SANDBOX/clean.out"
if run_setup_directory_capture "$WORK_REPO" "$clean_output"; then
  pass "setup_directory on clean repo"
else
  fail "setup_directory on clean repo" "non-zero exit"
fi
head_after="$(git -C "$WORK_REPO" rev-parse HEAD)"

assert_eq "clean repo HEAD unchanged" "$head_before" "$head_after"
assert_contains "clean repo reports current checkout" "Using existing repository at $WORK_REPO" "$(cat "$clean_output")"
assert_contains "clean repo reports up-to-date" "Repository is up to date with origin/main" "$(cat "$clean_output")"

section "behind repository only warns"

make_remote_commit "$REMOTE_REPO"
behind_before="$(git -C "$WORK_REPO" rev-parse HEAD)"
behind_output="$SANDBOX/behind.out"
if run_setup_directory_capture "$WORK_REPO" "$behind_output"; then
  pass "setup_directory on behind repo"
else
  fail "setup_directory on behind repo" "non-zero exit"
fi
behind_after="$(git -C "$WORK_REPO" rev-parse HEAD)"

assert_eq "behind repo HEAD unchanged" "$behind_before" "$behind_after"
assert_contains "behind repo warns instead of updating" "behind origin/main" "$(cat "$behind_output")"

section "local changes and runtime files are preserved"

printf 'local note\n' > "$WORK_REPO/LOCAL_NOTES.txt"
write_runtime_files "$WORK_REPO"

env_before="$(cat "$WORK_REPO/docker/.env")"
yaml_before="$(cat "$WORK_REPO/docker/config/mowgli/mowgli_robot.yaml")"
mqtt_before="$(cat "$WORK_REPO/docker/config/mqtt/mosquitto.conf")"
mower_before="$(cat "$WORK_REPO/docker/config/om/mower_config.sh")"

dirty_output="$SANDBOX/dirty.out"
if run_setup_directory_capture "$WORK_REPO" "$dirty_output"; then
  pass "setup_directory on dirty repo"
else
  fail "setup_directory on dirty repo" "non-zero exit"
fi

assert_contains "dirty repo warning emitted" "Local repository changes detected" "$(cat "$dirty_output")"
assert_eq "docker/.env preserved" "$env_before" "$(cat "$WORK_REPO/docker/.env")"
assert_eq "mowgli_robot.yaml preserved" "$yaml_before" "$(cat "$WORK_REPO/docker/config/mowgli/mowgli_robot.yaml")"
assert_eq "mosquitto.conf preserved" "$mqtt_before" "$(cat "$WORK_REPO/docker/config/mqtt/mosquitto.conf")"
assert_eq "mower_config.sh preserved" "$mower_before" "$(cat "$WORK_REPO/docker/config/om/mower_config.sh")"
assert_contains "local changes still present after setup_directory" "LOCAL_NOTES.txt" "$(git -C "$WORK_REPO" status --short)"

second_dirty_output="$SANDBOX/dirty-second.out"
if run_setup_directory_capture "$WORK_REPO" "$second_dirty_output"; then
  pass "setup_directory rerun after prior dirty run"
else
  fail "setup_directory rerun after prior dirty run" "non-zero exit"
fi
assert_eq "runtime files remain stable on rerun" "$env_before" "$(cat "$WORK_REPO/docker/.env")"

section "non-git checkout continues with warning"

NON_GIT_REPO="$SANDBOX/non_git"
sandbox_repo "$NON_GIT_REPO"
rm -rf "$NON_GIT_REPO/.git"

non_git_output="$SANDBOX/non-git.out"
if run_setup_directory_capture "$NON_GIT_REPO" "$non_git_output"; then
  pass "setup_directory on non-git checkout"
else
  fail "setup_directory on non-git checkout" "non-zero exit"
fi

assert_contains "non-git warning emitted" "is not a git repository" "$(cat "$non_git_output")"
assert_contains "non-git checkout continues" "Continuing with current files" "$(cat "$non_git_output")"

test_summary
