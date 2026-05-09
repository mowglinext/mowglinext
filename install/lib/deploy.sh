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
  git -C "$repo_dir" fetch --quiet origin "$REPO_BRANCH" >/dev/null 2>&1
}

report_repository_sync_status() {
  local repo_dir="${1:?report_repository_sync_status: missing repo dir}"
  local current_ref remote_ref counts ahead behind

  current_ref="$(repo_current_ref "$repo_dir")"
  remote_ref="origin/${REPO_BRANCH}"

  if [ "$current_ref" != "$REPO_BRANCH" ]; then
    warn "Repository is currently on '${current_ref}' (expected '${REPO_BRANCH}'). The installer will not switch branches during this run."
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
