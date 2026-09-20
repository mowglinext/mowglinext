#!/usr/bin/env bash

# Local changes = modifications to TRACKED files, nothing else.
#   - Untracked files never block a fast-forward (git refuses only on a real
#     path collision, and then leaves everything in place). The installer, the
#     GUI and the host updater write plenty of them under docker/; counting
#     them made every updater-managed checkout permanently "dirty", so it was
#     never updated again.
#   - Submodule state is ignored: a robot runs published images and builds
#     nothing from ros2/src, and a re-pinned gitlink otherwise reads as a local
#     change that blocks the very sync that would clear it.
repo_local_changes() {
  local repo_dir="${1:?repo_local_changes: missing repo dir}"
  git -C "$repo_dir" status --porcelain --untracked-files=no --ignore-submodules=all 2>/dev/null || true
}

repo_has_local_changes() {
  local repo_dir="${1:?repo_has_local_changes: missing repo dir}"
  [ -n "$(repo_local_changes "$repo_dir")" ]
}

# First path under .git that the current user does not own — what a past
# `sudo git pull` leaves behind, and git then fails with a bare "permission
# denied" / "dubious ownership".
repo_foreign_owned_path() {
  local repo_dir="${1:?repo_foreign_owned_path: missing repo dir}"
  find "$repo_dir/.git" ! -user "$(id -u)" -print -quit 2>/dev/null || true
}

# Decide what to do with tracked local modifications before the checkout is
# moved. Returns 0 = clean (or stashed), 1 = keep them and leave the checkout
# alone, 2 = abort the installer. Runtime configuration (docker/.env,
# docker/config/**, docker/stack-overrides.yaml, docker-compose.yaml) is
# untracked or ignored, so a stash WITHOUT --include-untracked cannot touch it.
resolve_repo_local_changes() {
  local repo_dir="${1:?resolve_repo_local_changes: missing repo dir}"
  local changes stash_name

  changes="$(repo_local_changes "$repo_dir")"
  [ -n "$changes" ] || return 0

  warn "$MSG_REPO_LOCAL_CHANGES"
  printf '%s\n' "$changes" | sed 's/^/        /'
  info "$MSG_REPO_LOCAL_CHANGES_SAFE"
  prompt "$MSG_REPO_LOCAL_CHANGES_CHOICE" "k"
  case "${REPLY,,}" in
    s|stash)
      stash_name="mowglinext-installer-$(date +%Y%m%d_%H%M%S)"
      # A robot has no git identity configured; stash needs one to commit.
      if ! git -C "$repo_dir" -c user.name="MowgliNext installer" -c user.email="installer@mowgli.invalid" \
           stash push --quiet -m "$stash_name" >/dev/null 2>&1; then
        error "$MSG_REPO_STASH_FAILED"
        return 1
      fi
      info "$MSG_REPO_STASHED $stash_name"
      info "$MSG_REPO_STASH_RESTORE git -C $repo_dir stash list && git -C $repo_dir stash apply"
      return 0
      ;;
    a|abort)
      error "$MSG_REPO_UPDATE_ABORTED"
      return 2
      ;;
    *)
      warn "$MSG_REPO_LOCAL_CHANGES_KEPT"
      return 1
      ;;
  esac
}

# Bring the current branch up to origin, then re-exec the installer from the
# updated tree. Replaces the documented `git pull && ./install/mowglinext.sh`,
# which died on a raw git error for three unrelated reasons (field report
# 2026-09-20): an initialised submodule whose URL or pin moved ("Errors during
# submodule fetch" — git pull fetches them on demand), tracked files modified
# on the robot, and .git entries owned by root after a sudo run.
# Never fatal except when the operator picks "abort"; every other obstacle
# leaves the checkout exactly as it was and continues with it.
update_repo_checkout_if_behind() {
  local repo_dir="${1:?update_repo_checkout_if_behind: missing repo dir}"
  local branch="${2:?update_repo_checkout_if_behind: missing branch}"
  local remote_ref="origin/$branch" behind foreign rc=0

  [[ "${MOWGLI_REPO_UPDATED:-false}" != "true" ]] || return 0
  repo_has_origin_remote "$repo_dir" || return 0

  foreign="$(repo_foreign_owned_path "$repo_dir")"
  if [ -n "$foreign" ]; then
    warn "$MSG_REPO_FOREIGN_OWNER $foreign"
    warn "$MSG_REPO_FOREIGN_OWNER_FIX sudo chown -R $(id -un) $repo_dir/.git"
    return 0
  fi

  ensure_repo_fetch_refspec "$repo_dir"
  if ! git -C "$repo_dir" fetch --quiet --no-recurse-submodules origin "$branch" >/dev/null 2>&1; then
    warn "$MSG_REPO_FETCH_FAILED $remote_ref"
    return 0
  fi
  behind="$(git -C "$repo_dir" rev-list --count "HEAD..$remote_ref" 2>/dev/null || true)"
  [[ "$behind" =~ ^[0-9]+$ && "$behind" -gt 0 ]] || return 0

  if ! confirm "$remote_ref: $behind $MSG_REPO_UPDATE_CONFIRM"; then
    return 0
  fi

  resolve_repo_local_changes "$repo_dir" || rc=$?
  [ "$rc" -ne 2 ] || return 1
  [ "$rc" -eq 0 ] || return 0

  if ! git -C "$repo_dir" -c submodule.recurse=false merge --quiet --ff-only "$remote_ref" >/dev/null 2>&1; then
    warn "$MSG_REPO_NOT_FAST_FORWARD $remote_ref"
    return 0
  fi
  info "$MSG_REPO_UPDATED $remote_ref ($(git -C "$repo_dir" rev-parse --short HEAD))"
  sync_repo_submodules_for_current_checkout "$repo_dir"
  reexec_installer_from_checkout "$branch"
}

reexec_installer_from_checkout() {
  local branch="${1:?reexec_installer_from_checkout: missing branch}"

  info "Re-executing installer from '$branch' branch..."
  export REPO_BRANCH="$branch"
  export REPO_BRANCH_PRESET=true
  export MOWGLI_REPO_UPDATED=true
  export IMAGE_TAG="${IMAGE_TAG:-}"
  export IMAGE_CHANNEL_PRESET="${IMAGE_CHANNEL_PRESET:-false}"

  if declare -p MOWGLI_INSTALLER_ARGV >/dev/null 2>&1; then
    exec bash "$INSTALL_DIR/mowglinext.sh" "${MOWGLI_INSTALLER_ARGV[@]+"${MOWGLI_INSTALLER_ARGV[@]}"}"
  fi
  exec bash "$INSTALL_DIR/mowglinext.sh"
}

repo_current_ref() {
  local repo_dir="${1:?repo_current_ref: missing repo dir}"
  local ref

  ref="$(git -C "$repo_dir" symbolic-ref --quiet --short HEAD 2>/dev/null || true)"
  if [ -n "$ref" ]; then
    printf '%s\n' "$ref"
    return 0
  fi

  git -C "$repo_dir" rev-parse --short HEAD 2>/dev/null || printf 'unknown\n'
}

repo_has_origin_remote() {
  local repo_dir="${1:?repo_has_origin_remote: missing repo dir}"
  git -C "$repo_dir" remote get-url origin >/dev/null 2>&1
}

fetch_repo_branch_metadata() {
  local repo_dir="${1:?fetch_repo_branch_metadata: missing repo dir}"

  repo_has_origin_remote "$repo_dir" || return 1
  ensure_repo_fetch_refspec "$repo_dir"
  git -C "$repo_dir" fetch --quiet --no-recurse-submodules origin "$REPO_BRANCH" >/dev/null 2>&1
}

ensure_repo_fetch_refspec() {
  local repo_dir="${1:?ensure_repo_fetch_refspec: missing repo dir}"
  local refspec="+refs/heads/*:refs/remotes/origin/*"

  repo_has_origin_remote "$repo_dir" || return 1
  if ! git -C "$repo_dir" config --get-all remote.origin.fetch 2>/dev/null | grep -qxF "$refspec"; then
    git -C "$repo_dir" config --replace-all remote.origin.fetch "$refspec"
  fi
}

report_repository_sync_status() {
  local repo_dir="${1:?report_repository_sync_status: missing repo dir}"
  local current_ref remote_ref counts ahead behind

  current_ref="$(repo_current_ref "$repo_dir")"
  remote_ref="origin/${REPO_BRANCH}"

  if [ "$current_ref" != "$REPO_BRANCH" ]; then
    warn "Repository is currently on '${current_ref}' while the selected branch is '${REPO_BRANCH}'."
  fi

  if repo_has_local_changes "$repo_dir"; then
    warn "Local repository changes detected. The checkout is left as it is."
  fi

  if ! repo_has_origin_remote "$repo_dir"; then
    warn "No origin remote configured for $repo_dir. Continuing with the current checkout."
    return 0
  fi

  if ! fetch_repo_branch_metadata "$repo_dir"; then
    warn "Could not refresh repository metadata from ${remote_ref}. Continuing with the current checkout."
    return 0
  fi

  if ! git -C "$repo_dir" rev-parse --verify "refs/remotes/${remote_ref}" >/dev/null 2>&1; then
    warn "Remote branch ${remote_ref} is not available after fetch. Continuing with the current checkout."
    return 0
  fi

  counts="$(git -C "$repo_dir" rev-list --left-right --count HEAD..."${remote_ref}" 2>/dev/null || true)"
  ahead="$(printf '%s' "$counts" | awk '{print $1}')"
  behind="$(printf '%s' "$counts" | awk '{print $2}')"

  if [ -z "$ahead" ] || [ -z "$behind" ]; then
    warn "Could not compare the current checkout to ${remote_ref}. Continuing with the current checkout."
    return 0
  fi

  if [ "$ahead" -eq 0 ] && [ "$behind" -eq 0 ]; then
    info "Repository is up to date with ${remote_ref}"
    return 0
  fi

  if [ "$ahead" -eq 0 ]; then
    warn "Repository is ${behind} commit(s) behind ${remote_ref}. Continuing with the current checkout to avoid mid-run version skew."
    return 0
  fi

  if [ "$behind" -eq 0 ]; then
    info "Repository is ${ahead} commit(s) ahead of ${remote_ref}"
    return 0
  fi

  warn "Repository has diverged from ${remote_ref} (ahead ${ahead}, behind ${behind}). Continuing with the current checkout."
}

# A robot needs NO submodule: both (universal-gnss, opennav_coverage) are ROS2
# build inputs and the robot runs published images. So this never initialises
# one — an initialised submodule is exactly what makes a later `git pull` fail
# when its URL or pin moves. Submodules a developer DID initialise are
# followed (URL first, then the pin), and a failure is only a warning.
sync_repo_submodules_for_current_checkout() {
  local repo_dir="${1:?sync_repo_submodules_for_current_checkout: missing repo dir}"
  local submodule_status=""

  [ -e "$repo_dir/.git" ] || return 0
  submodule_status="$(git -C "$repo_dir" submodule status --recursive 2>/dev/null || true)"
  # '+' = initialised but not at the recorded commit. '-' = not initialised.
  printf '%s\n' "$submodule_status" | grep -q '^+' || return 0

  info "Synchronizing initialised git submodules for the current checkout"
  git -C "$repo_dir" submodule sync --quiet --recursive >/dev/null 2>&1 || true
  if git -C "$repo_dir" submodule update --recursive >/dev/null 2>&1; then
    info "Git submodules ready for $(repo_current_ref "$repo_dir")"
  else
    warn "$MSG_REPO_SUBMODULE_SKIPPED"
  fi
}

sync_repo_branch_to_selected_branch() {
  local current_branch=""
  local current_ref=""
  local target_branch=""
  local has_remote_branch=false

  if [ ! -d "$REPO_DIR/.git" ]; then
    return 0
  fi

  current_branch="$(git -C "$REPO_DIR" symbolic-ref --quiet --short HEAD 2>/dev/null || true)"
  current_ref="$(repo_current_ref "$REPO_DIR")"
  target_branch="${REPO_BRANCH:-${current_branch:-}}"

  if [ -z "$target_branch" ]; then
    warn "No repository branch selected; keeping the current checkout."
    sync_repo_submodules_for_current_checkout "$REPO_DIR"
    return 0
  fi

  if [[ -n "$current_branch" && "$current_branch" == "$target_branch" ]]; then
    info "Repository checkout: ${current_branch}"
    update_repo_checkout_if_behind "$REPO_DIR" "$target_branch" || return 1
    sync_repo_submodules_for_current_checkout "$REPO_DIR"
    return 0
  fi

  if [[ -z "$current_branch" && "$current_ref" == "$target_branch" ]]; then
    warn "Repository is on detached HEAD (${current_ref}). Keeping the current checkout."
    sync_repo_submodules_for_current_checkout "$REPO_DIR"
    return 0
  fi

  step "Switching repository to '$target_branch' branch"

  if ! resolve_repo_local_changes "$REPO_DIR"; then
    error "Cannot switch repository branches with local changes present in $REPO_DIR"
    return 1
  fi

  if repo_has_origin_remote "$REPO_DIR"; then
    ensure_repo_fetch_refspec "$REPO_DIR"
    if ! git -C "$REPO_DIR" fetch --quiet --no-recurse-submodules --unshallow origin "$target_branch" 2>/dev/null; then
      git -C "$REPO_DIR" fetch --quiet --no-recurse-submodules origin "$target_branch" >/dev/null 2>&1 || true
    fi

    if git -C "$REPO_DIR" rev-parse --verify "refs/remotes/origin/$target_branch" >/dev/null 2>&1; then
      has_remote_branch=true
    fi
  fi

  if git -C "$REPO_DIR" rev-parse --verify "refs/heads/$target_branch" >/dev/null 2>&1; then
    if ! git -C "$REPO_DIR" checkout --quiet "$target_branch"; then
      error "Could not check out local branch '$target_branch'"
      return 1
    fi
    if [[ "$has_remote_branch" == "true" ]]; then
      if git -C "$REPO_DIR" merge --ff-only "origin/$target_branch" >/dev/null 2>&1; then
        info "Fast-forwarded '$target_branch' to origin/$target_branch"
      else
        warn "Local branch '$target_branch' could not be fast-forwarded to origin/$target_branch; keeping the local branch tip."
      fi
    fi
  elif [[ "$has_remote_branch" == "true" ]]; then
    if ! git -C "$REPO_DIR" checkout --quiet -b "$target_branch" "origin/$target_branch"; then
      error "Could not create local branch '$target_branch' from origin/$target_branch"
      return 1
    fi
  else
    error "Branch '$target_branch' was not found locally or on origin"
    return 1
  fi

  sync_repo_submodules_for_current_checkout "$REPO_DIR"
  info "Repository now on '$target_branch' branch"
  reexec_installer_from_checkout "$target_branch"
}

setup_directory() {
  step "Preparing repository"

  if [ -d "$REPO_DIR/.git" ]; then
    if [ ! -d "$INSTALL_DIR" ]; then
      error "Install directory not found in existing repository: $INSTALL_DIR"
      return 1
    fi

    info "Using existing repository at $REPO_DIR"
    report_repository_sync_status "$REPO_DIR"
    return 0
  fi

  if [ -d "$REPO_DIR" ]; then
    warn "Directory $REPO_DIR already exists but is not a git repository"
    if [ ! -d "$INSTALL_DIR" ]; then
      error "Install directory not found in current checkout: $INSTALL_DIR"
      return 1
    fi
    warn "Continuing with current files. Repository updates must be handled manually for this checkout."
    return 0
  fi

  error "Repository directory not found: $REPO_DIR"
  error "Run the bootstrap installer first, or clone $REPO_URL into $REPO_DIR."
  return 1
}

run_startup_step_live() {
  build_compose_stack
  run_compose_stack

  if ! $SKIP_WRITE_CONFIG; then
    auto_detect_position
  fi
}

backup_path_if_exists() {
  local path="$1"
  LAST_BACKUP_PATH=""
  if [ -e "$path" ]; then
    local backup="${path}.old.$(date +%Y%m%d_%H%M%S)"
    mv "$path" "$backup"
    info "Moved old runtime path: $path -> $backup"
    LAST_BACKUP_PATH="$backup"
  fi
}

# Removes a backup backup_path_if_exists()/migrate_runtime_paths() made
# earlier THIS run, if the runtime file it backed up has since been
# regenerated byte-identical to it. The backup still has to be made
# unconditionally up front — migrate_runtime_paths() runs before the step
# that regenerates the file, so the new content isn't known yet — this just
# prunes it retroactively once it turns out to have been unnecessary.
# TODO-runtime-backups.md #1/#6 ("avoid creating backups when files are
# unchanged" / "avoid backup spam during repeated reruns/tests"). Preserves
# the file's "never silently destroy" invariant: nothing is ever pruned
# unless the live file already contains the exact same bytes.
prune_backup_if_unchanged() {
  local live_path="$1"
  local backup_path="$2"
  [[ -n "$backup_path" && -e "$backup_path" && -e "$live_path" ]] || return 0
  if cmp -s "$live_path" "$backup_path"; then
    rm -f "$backup_path"
    info "Regenerated $(basename "$live_path") matches the backup byte-for-byte — removed the redundant copy"
  fi
}

fix_path_type_conflict() {
  local path="$1"
  local expected_type="$2"   # file | dir

  if [ "$expected_type" = "file" ] && [ -d "$path" ]; then
    backup_path_if_exists "$path"
  fi

  if [ "$expected_type" = "dir" ] && [ -f "$path" ]; then
    backup_path_if_exists "$path"
  fi
}

migrate_runtime_paths() {
  step "Preparing runtime directory"

  # Backups are still made unconditionally here (see prune_backup_if_unchanged
  # for why: the regenerated content isn't known until later steps run) but
  # MIGRATED_ENV_BACKUP / MIGRATED_COMPOSE_BACKUP let those later steps prune
  # a backup that turns out to have been redundant, instead of always keeping
  # one per run regardless of whether anything actually changed.
  backup_path_if_exists "$DOCKER_DIR/.env"
  MIGRATED_ENV_BACKUP="$LAST_BACKUP_PATH"
  # Keep the installed definition available for release ownership/baseline
  # validation. Moving it away would let regeneration bypass that check.
  if [[ -f "$DOCKER_DIR/docker-compose.yaml" ]]; then
    MIGRATED_COMPOSE_BACKUP="$DOCKER_DIR/docker-compose.yaml.old.$(date +%Y%m%d_%H%M%S)"
    cp -p "$DOCKER_DIR/docker-compose.yaml" "$MIGRATED_COMPOSE_BACKUP"
  else
    backup_path_if_exists "$DOCKER_DIR/docker-compose.yaml"
    MIGRATED_COMPOSE_BACKUP="$LAST_BACKUP_PATH"
  fi

  # Optional: backup generated runtime config folders only if you want a clean regen
  # backup_path_if_exists "$DOCKER_DIR/config/mqtt"
  # backup_path_if_exists "$DOCKER_DIR/config/mowgli"
  # backup_path_if_exists "$DOCKER_DIR/config/om"
  # backup_path_if_exists "$DOCKER_DIR/config/db"

  mkdir -p "$DOCKER_DIR"
  mkdir -p "$DOCKER_DIR/config/mqtt"
  mkdir -p "$DOCKER_DIR/config/mowgli"
  mkdir -p "$DOCKER_DIR/config/om"
  mkdir -p "$DOCKER_DIR/config/db"

  # Fix bad old mounts that created directories instead of files
  fix_path_type_conflict "$DOCKER_DIR/config/mqtt/mosquitto.conf" "file"
  fix_path_type_conflict "$DOCKER_DIR/config/cyclonedds.xml" "file"
}
