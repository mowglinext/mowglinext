#!/usr/bin/env bash

setup_directory() {
  step "Preparing repository"

  if [ -d "$REPO_DIR/.git" ]; then
    info "Updating existing git repository in $REPO_DIR"

    if ! git -C "$REPO_DIR" fetch origin "$REPO_BRANCH"; then
      error "Failed to fetch branch '$REPO_BRANCH' from origin"
      return 1
    fi

    if ! git -C "$REPO_DIR" rev-parse --verify FETCH_HEAD >/dev/null 2>&1; then
      error "Fetched branch '$REPO_BRANCH' is not available"
      return 1
    fi

    if ! git -C "$REPO_DIR" checkout -B "$REPO_BRANCH" FETCH_HEAD; then
      error "Failed to check out branch '$REPO_BRANCH'"
      return 1
    fi

    if ! git -C "$REPO_DIR" reset --hard FETCH_HEAD; then
      error "Failed to reset repository to fetched branch '$REPO_BRANCH'"
      return 1
    fi

    if [ ! -d "$INSTALL_DIR" ]; then
      error "Install directory not found in existing repository: $INSTALL_DIR"
      return 1
    fi

    return 0
  fi

  if [ -d "$REPO_DIR" ]; then
    warn "Directory $REPO_DIR already exists but is not a git repository"

    if confirm "Do you want to backup and replace it?"; then
      local backup_dir="${REPO_DIR}_backup_$(date +%Y%m%d_%H%M%S)"
      mv "$REPO_DIR" "$backup_dir"
      info "Backup created at $backup_dir"
    else
      error "Cannot continue without a clean repository directory"
      return 1
    fi
  fi

  info "Cloning repository into $REPO_DIR"
  if ! git clone --branch "$REPO_BRANCH" --depth 1 "$REPO_URL" "$REPO_DIR"; then
    error "Failed to clone branch '$REPO_BRANCH' from $REPO_URL"
    return 1
  fi

  if [ ! -d "$INSTALL_DIR" ]; then
    error "Install directory not found after clone: $INSTALL_DIR"
    return 1
  fi

  return 0
}

run_startup_step_live() {
  build_compose_stack
  run_compose_stack

  if ! $SKIP_WRITE_CONFIG; then
    auto_detect_position
  fi
}

# Operator-edited runtime files that must survive `git reset --hard`.
# Paths are relative to $REPO_DIR. Adding to this list automatically
# protects them across upgrades; the file is git-ignored upstream so
# the reset would otherwise wipe local edits.
_runtime_config_paths() {
  cat <<'EOF'
docker/config/mowgli/mowgli_robot.yaml
docker/config/cyclonedds.xml
docker/config/mqtt/mosquitto.conf
docker/config/om/mower_config.sh
docker/.env
EOF
}

_stash_runtime_configs() {
  local stash_dir="$1"
  local rel
  while IFS= read -r rel; do
    [ -z "$rel" ] && continue
    local src="$REPO_DIR/$rel"
    if [ -f "$src" ]; then
      local dst="$stash_dir/$rel"
      mkdir -p "$(dirname "$dst")"
      cp -p "$src" "$dst"
    fi
  done < <(_runtime_config_paths)
}

_restore_runtime_configs() {
  local stash_dir="$1"
  local rel
  while IFS= read -r rel; do
    [ -z "$rel" ] && continue
    local src="$stash_dir/$rel"
    local dst="$REPO_DIR/$rel"
    if [ -f "$src" ] && [ ! -f "$dst" ]; then
      mkdir -p "$(dirname "$dst")"
      cp -p "$src" "$dst"
      info "Restored operator-edited config: $rel"
    fi
  done < <(_runtime_config_paths)
}

# Operator-edited runtime files that must survive `git reset --hard`.
# Paths are relative to $REPO_DIR. Adding to this list automatically
# protects them across upgrades; the file is git-ignored upstream so
# the reset would otherwise wipe local edits.
_runtime_config_paths() {
  cat <<'EOF'
docker/config/mowgli/mowgli_robot.yaml
docker/config/cyclonedds.xml
docker/config/mqtt/mosquitto.conf
docker/config/om/mower_config.sh
docker/.env
EOF
}

_stash_runtime_configs() {
  local stash_dir="$1"
  local rel
  while IFS= read -r rel; do
    [ -z "$rel" ] && continue
    local src="$REPO_DIR/$rel"
    if [ -f "$src" ]; then
      local dst="$stash_dir/$rel"
      mkdir -p "$(dirname "$dst")"
      cp -p "$src" "$dst"
    fi
  done < <(_runtime_config_paths)
}

_restore_runtime_configs() {
  local stash_dir="$1"
  local rel
  while IFS= read -r rel; do
    [ -z "$rel" ] && continue
    local src="$stash_dir/$rel"
    local dst="$REPO_DIR/$rel"
    if [ -f "$src" ] && [ ! -f "$dst" ]; then
      mkdir -p "$(dirname "$dst")"
      cp -p "$src" "$dst"
      info "Restored operator-edited config: $rel"
    fi
  done < <(_runtime_config_paths)
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