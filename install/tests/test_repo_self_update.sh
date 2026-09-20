#!/usr/bin/env bash
# =============================================================================
# Repository self-update — field report 2026-09-20: "git submodules block
# `git pull`, and locally changed files block it too".
#
# The documented update was `git pull && ./install/mowglinext.sh`. On a robot
# that dies on a raw git error because:
#   - an older installer initialised the ROS2 submodules (the robot never
#     builds them), and `git pull` fetches initialised submodules on demand:
#     once a submodule's URL/pin moves, the pull fails before merging anything;
#   - tracked files get modified on the robot (docker/config/cyclonedds.xml,
#     the drive-tuning backup the GUI rewrites);
#   - the installer treated ANY untracked file as a local change, and the host
#     updater writes several into docker/, so a managed checkout was never
#     updated again.
#
# Real git against throwaway repositories; no mocks (they no-op `git fetch`).
# =============================================================================

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=lib/framework.sh
source "$SCRIPT_DIR/lib/framework.sh"

setup_sandbox

# Local-path submodules need the file transport; identity for test commits.
export GIT_CONFIG_COUNT=3
export GIT_CONFIG_KEY_0=protocol.file.allow GIT_CONFIG_VALUE_0=always
export GIT_CONFIG_KEY_1=user.name GIT_CONFIG_VALUE_1=test
export GIT_CONFIG_KEY_2=user.email GIT_CONFIG_VALUE_2=test@example.com

make_library_repo() {
  local dir="$1" content="$2"
  git init -q -b main "$dir"
  printf '%s\n' "$content" > "$dir/lib.txt"
  git -C "$dir" add lib.txt
  git -C "$dir" commit -q -m "library $content"
}

# $1 = work repo, $2 = confirm answer (yes|no), $3 = local-changes answer
run_update_capture() {
  local repo_dir="$1" confirm_answer="$2" choice="$3" output_file="$4"
  (
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
    confirm() { [[ "$confirm_answer" == "yes" ]]; }
    prompt() { REPLY="$choice"; }
    MOWGLI_INSTALLER_ARGV=(--branch=main)
    parse_args --branch=main
    sync_repo_branch_to_selected_branch
  ) >"$output_file" 2>&1
}

# --- fixture ----------------------------------------------------------------
LIB_OLD="$SANDBOX/lib_old"
LIB_NEW="$SANDBOX/lib_new"
SOURCE_REPO="$SANDBOX/source"
REMOTE_REPO="$SANDBOX/remote.git"
UPSTREAM="$SANDBOX/upstream"

make_library_repo "$LIB_OLD" "old upstream"
make_library_repo "$LIB_NEW" "fork"   # unrelated history, like a re-pointed fork

sandbox_repo "$SOURCE_REPO"
# sandbox_repo copies install/, docker/ and docs/ only; the ignore rules under
# test live in the repository root.
cp "$REPO_ROOT/.gitignore" "$SOURCE_REPO/.gitignore"
git -C "$SOURCE_REPO" add -f .gitignore
git -C "$SOURCE_REPO" submodule add -q "$LIB_OLD" ros2/src/external/lib >/dev/null 2>&1
git -C "$SOURCE_REPO" commit -q -m "add submodule"
git clone -q --bare "$SOURCE_REPO" "$REMOTE_REPO"

clone_robot() {
  local dir="$1" init_submodules="$2"
  git clone -q "$REMOTE_REPO" "$dir"
  if [[ "$init_submodules" == "yes" ]]; then
    git -C "$dir" submodule update -q --init >/dev/null 2>&1
  fi
  # What a real robot has on disk: ignored config + updater state.
  mkdir -p "$dir/docker/config/mowgli"
  printf 'ROS_DOMAIN_ID=42\n' > "$dir/docker/.env"
  printf 'datum_lat: 48.0\n' > "$dir/docker/config/mowgli/mowgli_robot.yaml"
  printf 'services: {}\n' > "$dir/docker/stack-overrides.yaml"
  printf '{}\n' > "$dir/docker/stack-selection.json"
  printf 'sha256:0\n' > "$dir/docker/stack-definition.sha256"
  printf 'services: {}\n' > "$dir/docker/docker-compose.yaml"
}

ROBOT="$SANDBOX/robot"
DEV_FREE="$SANDBOX/robot_no_submodule"
clone_robot "$ROBOT" yes
clone_robot "$DEV_FREE" no

# Upstream moves: the submodule goes to another URL + pin, a tracked file the
# robot also edited changes, and the installer becomes a stub that reports the
# re-exec.
git clone -q "$REMOTE_REPO" "$UPSTREAM"
git -C "$UPSTREAM" submodule update -q --init >/dev/null 2>&1
git -C "$UPSTREAM" config -f .gitmodules submodule.ros2/src/external/lib.url "$LIB_NEW"
git -C "$UPSTREAM" submodule sync -q >/dev/null 2>&1
git -C "$UPSTREAM/ros2/src/external/lib" fetch -q "$LIB_NEW" main
git -C "$UPSTREAM/ros2/src/external/lib" checkout -q FETCH_HEAD
printf '<!-- upstream edit -->\n' >> "$UPSTREAM/docker/config/cyclonedds.xml"
cat > "$UPSTREAM/install/mowglinext.sh" <<'EOF'
#!/usr/bin/env bash
echo "REEXEC_INSTALLER updated=${MOWGLI_REPO_UPDATED:-} args:$*"
EOF
git -C "$UPSTREAM" add -A
git -C "$UPSTREAM" commit -q -m "re-pin submodule, edit cyclonedds, stub installer"
git -C "$UPSTREAM" push -q origin main
REMOTE_HEAD="$(git -C "$UPSTREAM" rev-parse HEAD)"

# The robot edited the same tracked file.
printf '<!-- robot edit -->\n' >> "$ROBOT/docker/config/cyclonedds.xml"

section "updater and installer runtime files are not local changes"

assert_eq "runtime files under docker/ are ignored" "" \
  "$(git -C "$DEV_FREE" status --porcelain --untracked-files=all)"

section "root cause: plain git pull dies on the moved submodule"

pull_output="$(git -C "$ROBOT" pull 2>&1)"
pull_status=$?
assert_neq "git pull fails" "0" "$pull_status"
assert_contains "git pull fails in the submodule fetch" "submodule" "$pull_output"
robot_before="$(git -C "$ROBOT" rev-parse HEAD)"

section "declining the update leaves everything alone"

out="$SANDBOX/decline.out"
run_update_capture "$ROBOT" no k "$out"
assert_eq "declined: exit 0" "0" "$?"
assert_eq "declined: HEAD unchanged" "$robot_before" "$(git -C "$ROBOT" rev-parse HEAD)"
assert_not_contains "declined: no re-exec" "REEXEC_INSTALLER" "$(cat "$out")"

section "keep: local modifications are reported, not a raw git error"

out="$SANDBOX/keep.out"
run_update_capture "$ROBOT" yes k "$out"
assert_eq "keep: exit 0, the installer continues" "0" "$?"
assert_contains "keep: names the modified file" "docker/config/cyclonedds.xml" "$(cat "$out")"
assert_contains "keep: says configuration is safe" "never touched" "$(cat "$out")"
assert_eq "keep: HEAD unchanged" "$robot_before" "$(git -C "$ROBOT" rev-parse HEAD)"
assert_contains "keep: robot edit still there" "robot edit" "$(cat "$ROBOT/docker/config/cyclonedds.xml")"
assert_not_contains "keep: no re-exec" "REEXEC_INSTALLER" "$(cat "$out")"

section "abort: stops the installer without touching anything"

out="$SANDBOX/abort.out"
run_update_capture "$ROBOT" yes a "$out"
assert_neq "abort: non-zero exit" "0" "$?"
assert_eq "abort: HEAD unchanged" "$robot_before" "$(git -C "$ROBOT" rev-parse HEAD)"
assert_contains "abort: robot edit still there" "robot edit" "$(cat "$ROBOT/docker/config/cyclonedds.xml")"

section "stash: named backup, fast-forward, re-exec, config untouched"

out="$SANDBOX/stash.out"
run_update_capture "$ROBOT" yes s "$out"
assert_eq "stash: exit 0" "0" "$?"
assert_eq "stash: HEAD is origin/main" "$REMOTE_HEAD" "$(git -C "$ROBOT" rev-parse HEAD)"
assert_contains "stash: named stash entry exists" "mowglinext-installer-" "$(git -C "$ROBOT" stash list)"
assert_contains "stash: the entry holds the robot edit" "robot edit" "$(git -C "$ROBOT" stash show -p 'stash@{0}')"
assert_contains "stash: tells how to restore" "stash apply" "$(cat "$out")"
assert_contains "stash: installer re-executed from the new tree" "REEXEC_INSTALLER updated=true args:--branch=main" "$(cat "$out")"
assert_eq "stash: docker/.env untouched" "ROS_DOMAIN_ID=42" "$(cat "$ROBOT/docker/.env")"
assert_eq "stash: mowgli_robot.yaml untouched" "datum_lat: 48.0" "$(cat "$ROBOT/docker/config/mowgli/mowgli_robot.yaml")"
assert_eq "stash: stack-overrides.yaml untouched" "services: {}" "$(cat "$ROBOT/docker/stack-overrides.yaml")"
assert_file_exists "stash: generated compose untouched" "$ROBOT/docker/docker-compose.yaml"
assert_not_contains "stash: initialised submodule followed its new URL and pin" "+" \
  "$(git -C "$ROBOT" submodule status | cut -c1)"
assert_eq "stash: submodule content is the fork's" "fork" "$(cat "$ROBOT/ros2/src/external/lib/lib.txt")"

section "a robot never gets a submodule it did not have"

out="$SANDBOX/nosub.out"
run_update_capture "$DEV_FREE" yes k "$out"
assert_eq "no submodule: exit 0" "0" "$?"
assert_eq "no submodule: fast-forwarded" "$REMOTE_HEAD" "$(git -C "$DEV_FREE" rev-parse HEAD)"
assert_eq "no submodule: still not initialised" "-" "$(git -C "$DEV_FREE" submodule status | cut -c1)"
assert_not_contains "no submodule: no local-changes question" "modified locally" "$(cat "$out")"

section "already updated this run: no second update, no re-exec loop"

printf 'next\n' >> "$UPSTREAM/REMOTE_STATUS.txt"
git -C "$UPSTREAM" add -A
git -C "$UPSTREAM" commit -q -m "next"
git -C "$UPSTREAM" push -q origin main
loop_before="$(git -C "$DEV_FREE" rev-parse HEAD)"
out="$SANDBOX/loop.out"
MOWGLI_REPO_UPDATED=true run_update_capture "$DEV_FREE" yes k "$out"
assert_eq "re-exec guard: HEAD unchanged" "$loop_before" "$(git -C "$DEV_FREE" rev-parse HEAD)"
assert_not_contains "re-exec guard: no re-exec" "REEXEC_INSTALLER" "$(cat "$out")"

section "own commits: reported, never rewritten"

printf 'mine\n' > "$DEV_FREE/MINE.txt"
git -C "$DEV_FREE" add MINE.txt
git -C "$DEV_FREE" commit -q -m "local commit"
diverged_before="$(git -C "$DEV_FREE" rev-parse HEAD)"
out="$SANDBOX/diverged.out"
run_update_capture "$DEV_FREE" yes k "$out"
assert_eq "diverged: exit 0" "0" "$?"
assert_eq "diverged: HEAD unchanged" "$diverged_before" "$(git -C "$DEV_FREE" rev-parse HEAD)"
assert_contains "diverged: explained" "cannot be fast-forwarded" "$(cat "$out")"

section "repository entries owned by another user"

if [[ "$(id -u)" -eq 0 ]]; then
  touch "$DEV_FREE/.git/ROOT_LEFTOVER"
  chown 12345 "$DEV_FREE/.git/ROOT_LEFTOVER"
  out="$SANDBOX/owner.out"
  run_update_capture "$DEV_FREE" yes k "$out"
  assert_eq "foreign owner: exit 0" "0" "$?"
  assert_contains "foreign owner: names the fix" "chown -R" "$(cat "$out")"
  assert_eq "foreign owner: HEAD unchanged" "$diverged_before" "$(git -C "$DEV_FREE" rev-parse HEAD)"
else
  echo "  SKIP  needs root to create a file owned by another user"
fi

test_summary
