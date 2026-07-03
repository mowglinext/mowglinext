#!/usr/bin/env bash

repo_has_local_changes() {
  local repo_dir="${1:?repo_has_local_changes: missing repo dir}"
  [ -n "$(git -C "$repo_dir" status --porcelain --untracked-files=all 2>/dev/null || true)" ]
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
  git -C "$repo_dir" fetch --quiet origin "$REPO_BRANCH" >/dev/null 2>&1
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
    warn "Local repository changes detected. Skipping in-run repository update to preserve your checkout."
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

sync_repo_submodules_for_current_checkout() {
  local repo_dir="${1:?sync_repo_submodules_for_current_checkout: missing repo dir}"
  local submodule_status=""

  [ -d "$repo_dir/.git" ] || return 0
  submodule_status="$(git -C "$repo_dir" submodule status --recursive 2>/dev/null || true)"
  [ -n "$submodule_status" ] || return 0

  if ! printf '%s\n' "$submodule_status" | grep -qE '^[+-]'; then
    return 0
  fi

  if repo_has_local_changes "$repo_dir"; then
    warn "Local repository changes detected — skipping automatic submodule sync for the current checkout."
    return 0
  fi

  info "Synchronizing git submodules for the current checkout"
  if git -C "$repo_dir" submodule update --init --recursive >/dev/null 2>&1; then
    info "Git submodules ready for $(repo_current_ref "$repo_dir")"
  else
    warn "Could not synchronize git submodules for the current checkout. Continue if the repository is already complete."
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
    sync_repo_submodules_for_current_checkout "$REPO_DIR"
    return 0
  fi

  if [[ -z "$current_branch" && "$current_ref" == "$target_branch" ]]; then
    warn "Repository is on detached HEAD (${current_ref}). Keeping the current checkout."
    sync_repo_submodules_for_current_checkout "$REPO_DIR"
    return 0
  fi

  step "Switching repository to '$target_branch' branch"

  if repo_has_local_changes "$REPO_DIR"; then
    error "Cannot switch repository branches with local changes present in $REPO_DIR"
    return 1
  fi

  if repo_has_origin_remote "$REPO_DIR"; then
    ensure_repo_fetch_refspec "$REPO_DIR"
    if ! git -C "$REPO_DIR" fetch --quiet --unshallow origin "$target_branch" 2>/dev/null; then
      git -C "$REPO_DIR" fetch --quiet origin "$target_branch" >/dev/null 2>&1 || true
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
  info "Re-executing installer from '$target_branch' branch..."

  export REPO_BRANCH="$target_branch"
  export REPO_BRANCH_PRESET=true
  export IMAGE_TAG="${IMAGE_TAG:-}"
  export IMAGE_CHANNEL_PRESET="${IMAGE_CHANNEL_PRESET:-false}"

  if declare -p MOWGLI_INSTALLER_ARGV >/dev/null 2>&1; then
    exec bash "$INSTALL_DIR/mowglinext.sh" "${MOWGLI_INSTALLER_ARGV[@]+"${MOWGLI_INSTALLER_ARGV[@]}"}"
  fi
  exec bash "$INSTALL_DIR/mowglinext.sh"
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
  if [ -e "$path" ]; then
    local backup="${path}.old.$(date +%Y%m%d_%H%M%S)"
    mv "$path" "$backup"
    info "Moved old runtime path: $path -> $backup"
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

  # TODO: backup policy cleanup tracked in /TODO-runtime-backups.md
  # Keep the current conservative behavior during installer hardening.
  # Only runtime files under docker/
  backup_path_if_exists "$DOCKER_DIR/.env"
  backup_path_if_exists "$DOCKER_DIR/docker-compose.yaml"

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
