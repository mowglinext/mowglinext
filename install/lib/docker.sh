#!/usr/bin/env bash

docker_group_session_ready() {
  id -nG 2>/dev/null | grep -qw docker 2>/dev/null
}

docker_user_in_group() {
  local user_name
  user_name="${SUDO_USER:-${USER:-$(id -un 2>/dev/null || echo "")}}"

  [ -n "$user_name" ] || return 1

  if command -v getent >/dev/null 2>&1; then
    getent group docker 2>/dev/null | awk -F: '{print $4}' | tr ',' '\n' | grep -Fxq "$user_name"
  else
    grep -E '^docker:' /etc/group 2>/dev/null | awk -F: '{print $4}' | tr ',' '\n' | grep -Fxq "$user_name"
  fi
}

run_with_docker_access() {
  local cmd

  if [ "${EUID:-$(id -u)}" -eq 0 ]; then
    docker "$@"
    return
  fi

  if docker_group_session_ready; then
    docker "$@"
    return
  fi

  if docker_user_in_group && command -v sg >/dev/null 2>&1; then
    printf -v cmd '%q ' docker "$@"
    sg docker -c "$cmd"
    return
  fi

  require_root_for "docker"
  "$SUDO" docker "$@"
}

docker_access_ready() {
  if [ "${EUID:-$(id -u)}" -eq 0 ]; then
    return 0
  fi

  if docker_group_session_ready; then
    return 0
  fi

  if docker_user_in_group && command -v sg >/dev/null 2>&1; then
    return 0
  fi

  command -v sudo >/dev/null 2>&1
}

docker_cmd() {
  run_with_docker_access "$@"
}

docker_compose_cmd() {
  docker_cmd compose "$@"
}

install_docker() {
  step "1/6  Docker"

  if command_exists docker; then
    info "Docker $(docker_cmd --version 2>/dev/null | grep -oP '[\d.]+' | head -1)"
  else
    info "Installing Docker..."
    curl -fsSL https://get.docker.com | sh
    info "Docker installed"
  fi

  require_root_for "docker group"

  if ! docker_user_in_group; then
    $SUDO usermod -aG docker "${SUDO_USER:-$USER}"
    warn "Added $USER to docker group — current shell may not have access yet"
  fi

  if ! docker_access_ready; then
    error "Docker is installed but cannot be accessed from this session."
    error "Log out and back in, then re-run the installer."
    exit 1
  fi

  if docker_compose_cmd version &>/dev/null; then
    info "Docker Compose $(docker_compose_cmd version --short 2>/dev/null)"
  else
    error "Docker Compose v2 not found. Install docker-compose-plugin."
    exit 1
  fi
}
