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

run_parse_and_sync_capture() {
  local repo_dir="${1:?run_parse_and_sync_capture: missing repo dir}"
  local output_file="${2:?run_parse_and_sync_capture: missing output file}"
  shift 2

  (
    source_test_libs "$repo_dir"
    MOWGLI_INSTALLER_ARGV=("$@")
    parse_args "$@"
    sync_repo_branch_to_selected_branch
  ) >"$output_file" 2>&1
}

run_select_image_channel_capture() {
  local repo_dir="${1:?run_select_image_channel_capture: missing repo dir}"
  local output_file="${2:?run_select_image_channel_capture: missing output file}"
  local choice="${3:?run_select_image_channel_capture: missing choice}"
  local custom_tag="${4:-}"

  (
    source_test_libs "$repo_dir"
    __prompt_calls=0
    prompt() {
      __prompt_calls=$((__prompt_calls + 1))
      case "$__prompt_calls" in
        1) REPLY="$choice" ;;
        2) REPLY="${custom_tag:-${2:-}}" ;;
        *) REPLY="${2:-}" ;;
      esac
    }
    select_image_channel
    printf 'CURRENT_BRANCH=%s\n' "$(git -C "$repo_dir" symbolic-ref --quiet --short HEAD 2>/dev/null || echo detached)"
    printf 'FINAL_IMAGE_TAG=%s\n' "$IMAGE_TAG"
  ) >"$output_file" 2>&1
}

run_select_repo_branch_and_sync_capture() {
  local repo_dir="${1:?run_select_repo_branch_and_sync_capture: missing repo dir}"
  local output_file="${2:?run_select_repo_branch_and_sync_capture: missing output file}"
  local choice="${3:?run_select_repo_branch_and_sync_capture: missing choice}"
  local custom_branch="${4:-}"

  (
    source_test_libs "$repo_dir"
    MOWGLI_INSTALLER_ARGV=()
    __prompt_calls=0
    prompt() {
      __prompt_calls=$((__prompt_calls + 1))
      case "$__prompt_calls" in
        1) REPLY="$choice" ;;
        2) REPLY="${custom_branch:-${2:-}}" ;;
        *) REPLY="${2:-}" ;;
      esac
    }
    select_repo_branch
    sync_repo_branch_to_selected_branch
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

publish_remote_branch_with_installer_stub() {
  local bare_repo="${1:?publish_remote_branch_with_installer_stub: missing bare repo}"
  local branch_name="${2:?publish_remote_branch_with_installer_stub: missing branch name}"
  local upstream_repo="$SANDBOX/branch_$(printf '%s' "$branch_name" | tr '/.' '__')_$RANDOM"

  git clone "$bare_repo" "$upstream_repo" >/dev/null 2>&1
  git -C "$upstream_repo" checkout -q -B "$branch_name" origin/main

  cat > "$upstream_repo/install/mowglinext.sh" <<'EOF'
#!/usr/bin/env bash
repo_dir="$(cd "$(dirname "$0")/.." && pwd)"
branch="$(git -C "$repo_dir" symbolic-ref --quiet --short HEAD 2>/dev/null || git -C "$repo_dir" rev-parse --short HEAD 2>/dev/null || echo unknown)"
echo "REEXEC_INSTALLER branch=${branch} args:$*"
EOF
  chmod +x "$upstream_repo/install/mowglinext.sh"

  git -C "$upstream_repo" add install/mowglinext.sh >/dev/null 2>&1
  git -C "$upstream_repo" -c user.email=test@example.com -c user.name=test \
    commit -m "stub installer for ${branch_name}" >/dev/null 2>&1
  git -C "$upstream_repo" push -u origin "$branch_name" >/dev/null 2>&1
}

write_runtime_files() {
  local repo_dir="${1:?write_runtime_files: missing repo dir}"

  mkdir -p "$repo_dir/docker/config/mowgli" \
           "$repo_dir/docker/config/mqtt" \
           "$repo_dir/docker/config/om"

  cat > "$repo_dir/docker/.env" <<'EOF'
ROS_DOMAIN_ID=42
GNSS_BACKEND=universal
GNSS_STACK=universal
GNSS_RECEIVER_FAMILY=auto
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

section "repository branch and image tag stay decoupled"

BRANCH_SOURCE="$SANDBOX/branch_source"
BRANCH_REMOTE="$SANDBOX/branch_remote.git"
BRANCH_WORK="$SANDBOX/branch_work"

sandbox_repo "$BRANCH_SOURCE"
create_work_repo_with_remote "$BRANCH_SOURCE" "$BRANCH_REMOTE" "$BRANCH_WORK"
publish_remote_branch_with_installer_stub "$BRANCH_REMOTE" "dev"
publish_remote_branch_with_installer_stub "$BRANCH_REMOTE" "feat/universal-gnss-integration"

branch_switch_output="$SANDBOX/branch-switch.out"
if run_parse_and_sync_capture "$BRANCH_WORK" "$branch_switch_output" --branch=feat/universal-gnss-integration; then
  pass "--branch switches to feat/universal-gnss-integration"
else
  fail "--branch switches to feat/universal-gnss-integration" "non-zero exit"
fi

assert_eq "--branch leaves checkout on feature branch" \
  "feat/universal-gnss-integration" \
  "$(git -C "$BRANCH_WORK" symbolic-ref --quiet --short HEAD)"
assert_contains "--branch re-execs installer from feature branch" \
  "REEXEC_INSTALLER branch=feat/universal-gnss-integration" \
  "$(cat "$branch_switch_output")"

image_tag_only_output="$SANDBOX/image-tag-only.out"
if run_parse_and_sync_capture "$BRANCH_WORK" "$image_tag_only_output" --image-tag=feat-universal-gnss-integration; then
  pass "--image-tag does not switch repository branch"
else
  fail "--image-tag does not switch repository branch" "non-zero exit"
fi

assert_eq "--image-tag keeps feature branch checked out" \
  "feat/universal-gnss-integration" \
  "$(git -C "$BRANCH_WORK" symbolic-ref --quiet --short HEAD)"
assert_not_contains "--image-tag does not re-exec installer" \
  "REEXEC_INSTALLER" \
  "$(cat "$image_tag_only_output")"

dev_image_tag_output="$SANDBOX/dev-image-tag.out"
if run_select_image_channel_capture "$BRANCH_WORK" "$dev_image_tag_output" "2"; then
  pass "choosing dev image tag keeps current feature branch"
else
  fail "choosing dev image tag keeps current feature branch" "non-zero exit"
fi

assert_contains "feature branch advertises sanitized default image tag" \
  "default tag: feat-universal-gnss-integration" \
  "$(cat "$dev_image_tag_output")"
assert_contains "dev image tag selection writes IMAGE_TAG=dev" "FINAL_IMAGE_TAG=dev" "$(cat "$dev_image_tag_output")"
assert_contains "dev image tag selection keeps feature branch" \
  "CURRENT_BRANCH=feat/universal-gnss-integration" \
  "$(cat "$dev_image_tag_output")"

dev_branch_switch_output="$SANDBOX/dev-branch-switch.out"
if run_select_repo_branch_and_sync_capture "$BRANCH_WORK" "$dev_branch_switch_output" "3"; then
  pass "choosing dev repo branch switches checkout"
else
  fail "choosing dev repo branch switches checkout" "non-zero exit"
fi

assert_eq "interactive repo branch switch ends on dev" \
  "dev" \
  "$(git -C "$BRANCH_WORK" symbolic-ref --quiet --short HEAD)"
assert_contains "interactive repo branch switch re-execs from dev" \
  "REEXEC_INSTALLER branch=dev" \
  "$(cat "$dev_branch_switch_output")"

test_summary
