#!/usr/bin/env bash
# =============================================================================
# Bootstrap repo preparation — docs/install.sh must stay conservative
# =============================================================================

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=lib/framework.sh
source "$SCRIPT_DIR/lib/framework.sh"

setup_sandbox

create_bootstrap_source_repo() {
  local repo_dir="${1:?create_bootstrap_source_repo: missing repo dir}"

  mkdir -p "$repo_dir/docs" "$repo_dir/install"
  cp "$REPO_ROOT/docs/install.sh" "$repo_dir/docs/install.sh"

  cat > "$repo_dir/install/mowglinext.sh" <<'EOF'
#!/usr/bin/env bash
echo "BOOTSTRAP_INSTALLER_ARGS:$*"
EOF
  chmod +x "$repo_dir/install/mowglinext.sh"

  (
    cd "$repo_dir"
    git init -q -b main >/dev/null 2>&1 || git init -q >/dev/null 2>&1
    git -c user.email=test@example.com -c user.name=test add -A >/dev/null 2>&1
    git -c user.email=test@example.com -c user.name=test \
      commit -q -m "bootstrap source" >/dev/null 2>&1
  )
}

clone_bootstrap_repo() {
  local source_repo="${1:?clone_bootstrap_repo: missing source repo}"
  local bare_repo="${2:?clone_bootstrap_repo: missing bare repo}"
  local work_repo="${3:?clone_bootstrap_repo: missing work repo}"

  git clone --bare "$source_repo" "$bare_repo" >/dev/null 2>&1
  git clone "$bare_repo" "$work_repo" >/dev/null 2>&1
}

push_bootstrap_remote_commit() {
  local bare_repo="${1:?push_bootstrap_remote_commit: missing bare repo}"
  local upstream_repo="$SANDBOX/bootstrap_upstream_$RANDOM"

  git clone "$bare_repo" "$upstream_repo" >/dev/null 2>&1
  printf 'upstream %s\n' "$(date +%s)" >> "$upstream_repo/UPSTREAM.txt"
  git -C "$upstream_repo" add UPSTREAM.txt >/dev/null 2>&1
  git -C "$upstream_repo" -c user.email=test@example.com -c user.name=test \
    commit -m "upstream bootstrap change" >/dev/null 2>&1
  git -C "$upstream_repo" push origin main >/dev/null 2>&1
}

run_bootstrap_capture() {
  local repo_dir="${1:?run_bootstrap_capture: missing repo dir}"
  local output_file="${2:?run_bootstrap_capture: missing output file}"
  shift 2

  MOWGLI_HOME="$repo_dir" bash "$repo_dir/docs/install.sh" "$@" >"$output_file" 2>&1 || true
}

section "bootstrap fast-forwards a clean existing repo"

BOOTSTRAP_SOURCE="$SANDBOX/bootstrap_source"
BOOTSTRAP_REMOTE="$SANDBOX/bootstrap_remote.git"
BOOTSTRAP_WORK="$SANDBOX/bootstrap_work"

create_bootstrap_source_repo "$BOOTSTRAP_SOURCE"
clone_bootstrap_repo "$BOOTSTRAP_SOURCE" "$BOOTSTRAP_REMOTE" "$BOOTSTRAP_WORK"
push_bootstrap_remote_commit "$BOOTSTRAP_REMOTE"

clean_before="$(git -C "$BOOTSTRAP_WORK" rev-parse HEAD)"
clean_output="$SANDBOX/bootstrap-clean.out"
run_bootstrap_capture "$BOOTSTRAP_WORK" "$clean_output"
pass "bootstrap on clean repo"
clean_after="$(git -C "$BOOTSTRAP_WORK" rev-parse HEAD)"

assert_neq "bootstrap fast-forwarded HEAD" "$clean_before" "$clean_after"
assert_contains "bootstrap reports fast-forward" "Fast-forwarded existing installation to origin/main" "$(cat "$clean_output")"
assert_contains "bootstrap reaches installer handoff" "── Launching installer ──" "$(cat "$clean_output")"
assert_contains "bootstrap runs installer stub" "BOOTSTRAP_INSTALLER_ARGS:" "$(cat "$clean_output")"
assert_contains "bootstrap forwards --branch as repository branch" "BOOTSTRAP_INSTALLER_ARGS:--branch=main" "$(cat "$clean_output")"

section "bootstrap preserves dirty existing repo"

printf 'local change\n' > "$BOOTSTRAP_WORK/LOCAL_BOOTSTRAP_NOTES.txt"
push_bootstrap_remote_commit "$BOOTSTRAP_REMOTE"

dirty_before="$(git -C "$BOOTSTRAP_WORK" rev-parse HEAD)"
dirty_output="$SANDBOX/bootstrap-dirty.out"
run_bootstrap_capture "$BOOTSTRAP_WORK" "$dirty_output"
pass "bootstrap on dirty repo"
dirty_after="$(git -C "$BOOTSTRAP_WORK" rev-parse HEAD)"

assert_eq "bootstrap keeps dirty repo HEAD" "$dirty_before" "$dirty_after"
assert_contains "bootstrap warns about local changes" "Local changes detected" "$(cat "$dirty_output")"
assert_contains "bootstrap reaches installer handoff on dirty repo" "── Launching installer ──" "$(cat "$dirty_output")"
assert_file_exists "bootstrap keeps local dirty file" "$BOOTSTRAP_WORK/LOCAL_BOOTSTRAP_NOTES.txt"

section "bootstrap can pass an explicit image tag without changing checkout"

explicit_output="$SANDBOX/bootstrap-explicit-image-tag.out"
run_bootstrap_capture "$BOOTSTRAP_WORK" "$explicit_output" --image-tag=feat-universal-gnss-integration
pass "bootstrap with explicit image tag"
assert_contains "bootstrap still forwards --branch to installer" "--branch=main" "$(cat "$explicit_output")"
assert_contains "bootstrap forwards --image-tag to installer" "--image-tag=feat-universal-gnss-integration" "$(cat "$explicit_output")"

test_summary
