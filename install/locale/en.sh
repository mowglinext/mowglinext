#!/usr/bin/env bash
MSG_UPDATER_UNSUPPORTED="Automatic updates require Linux amd64/arm64, systemd and Docker Compose. Version viewing remains available."
MSG_UPDATER_RECOVERY="An update is in maintenance/recovery. Resolve it before rerunning the installer."
MSG_UPDATER_SOURCE="Unsupported updater source repository."
MSG_UPDATER_UNPUBLISHED="The updater binary for this checkout is unavailable. Wait for its Host updater workflow to finish, or provide MOWGLI_UPDATER_BINARY from this checkout. Installation stopped; Watchtower was not enabled as a fallback."
MSG_UPDATER_CHECKSUM="Updater checksum verification failed."
MSG_UPDATER_INSTALLED="Host updater installed. Settings > Updates shows its version, checks and deployments."
# English locale (default)

# ── Common ──
MSG_YES_NO="Y/n"
MSG_YOUR_CHOICE="Your choice"
MSG_CHOICE="Choice"

# ── System update (system.sh) ──
MSG_SYSTEM_UPDATE="Do you want to update the system?"
MSG_SYSTEM_UPDATE_SKIPPED="System update skipped"
MSG_APT_PIN_CONFIRM="Do you want to pin APT to the current release"
MSG_APT_PINNED="APT pinned to"
MSG_APT_NO_PIN="No APT release pin applied"
MSG_APT_UPGRADE_CONFIRM="Do you want to run apt upgrade -y now?"
MSG_APT_UPGRADING="Running apt upgrade..."
MSG_APT_UPGRADED="System upgraded"
MSG_APT_UPGRADE_SKIPPED="APT upgrade skipped"

# ── UART detection ──
MSG_UART_DETECTING="Detecting available UART ports..."
MSG_UART_AVAILABLE="Available UART ports:"
MSG_UART_NONE_FOUND="No UART ports detected. Enter the device path manually."
MSG_UART_SELECT="Select UART port"
MSG_UART_MANUAL="Enter manually"
MSG_UART_MANUAL_PROMPT="UART device path?"
MSG_UART_INVALID="Invalid choice"
MSG_UART_AFTER_REBOOT="available after reboot"

# ── GPS (gps.sh) ──
MSG_GNSS_CONNECTION="GNSS connection:"
MSG_GPS_DEBUG_CONFIRM="Enable GPS debug port (miniUART / gps_debug)?"
MSG_GPS_INVALID_CONNECTION="Invalid GPS connection choice"
MSG_GPS_INVALID_PROTOCOL="Invalid protocol choice"
MSG_GPS_MAIN="GPS main"

# ── LiDAR (lidar.sh) ──
MSG_LIDAR_TYPE="LiDAR type:"
MSG_LIDAR_NONE="None"
MSG_LIDAR_CONNECTION="LiDAR connection:"
MSG_LIDAR_INVALID_TYPE="Invalid LiDAR choice"
MSG_LIDAR_INVALID_CONNECTION="Invalid LiDAR connection choice"

# ── Rangefinders (range.sh) ──
MSG_TFLUNA_CONFIG="TF-Luna sensor configuration:"
MSG_TFLUNA_NONE="None"
MSG_TFLUNA_FRONT_ONLY="Front only"
MSG_TFLUNA_EDGE_ONLY="Edge only"
MSG_TFLUNA_FRONT_EDGE="Front + edge"
MSG_TFLUNA_INVALID="Invalid TF-Luna choice"

# ── Tools (tools.sh) ──
MSG_TOOLS_DOCKER_CLI="Optional tools: Docker CLI manager"
MSG_TOOLS_DOCKER_LAZY="Yes, install lazydocker (recommended)"
MSG_TOOLS_DOCKER_CTOP="Yes, install ctop (alternative)"
MSG_TOOLS_NO="No"
MSG_TOOLS_FILE_MANAGER="Optional tools: file manager"
MSG_TOOLS_FILE_MC="Yes, install Midnight Commander (mc)"
MSG_TOOLS_FILE_RANGER="Yes, install ranger"
MSG_TOOLS_DEBUG="Optional tools: development and debug"
MSG_TOOLS_DEBUG_ALL="All tools (recommended)"
MSG_TOOLS_DEBUG_ESSENTIAL="Essential tools only"
MSG_TOOLS_DEBUG_NONE="None"
MSG_TOOLS_HELPERS="Optional tools: Mowgli helpers"
MSG_TOOLS_HELPERS_CONFIRM="Install Mowgli helper commands?"

# ── MOTD (motd.sh) ──
MSG_MOTD_NOT_CONNECTED="not connected"
MSG_MOTD_FREE="free"
MSG_MOTD_PACKAGES="package(s)"
MSG_MOTD_LOCAL_IP="Local IP"
MSG_MOTD_NOT_SET="not set"
MSG_MOTD_RUNNING="running"

MSG_UPDATER_STACK_BACKEND="Managed release updates support the Mowgli hardware backend."
MSG_UPDATER_HARDWARE_LEGACY="These hardware choices require the existing installer path (MAVROS, TF-Luna or VESC). Keeping their selected containers; coordinated release updates are not enabled."
MSG_UPDATER_HARDWARE_MANAGED="This installation already uses managed updates. MAVROS, TF-Luna and VESC selections require an explicit stack migration; runtime files have not been regenerated."
MSG_UPDATER_STACK_REVIEW="Saved hardware choices. Review Software updates to apply container changes; the installed release definition has been retained."

# Compose baseline / legacy adoption (install/lib/compose.sh)
MSG_COMPOSE_BASELINE_UNAVAILABLE="Could not record a checksum of the generated Compose file (sha256sum/shasum missing, or docker/stack-definition.sha256 not writable); no baseline recorded."
MSG_COMPOSE_LEGACY_EXPLAIN="docker/docker-compose.yaml was generated before managed updates existed, so no checksum of it was recorded. It differs from the current definition in the settings listed above. If you never edited that file by hand, this is only the release evolving and it is safe to replace."
MSG_COMPOSE_LEGACY_BACKUP="The current file is kept as docker/docker-compose.yaml.legacy-<date>. Hand-made changes you want to keep belong in docker/stack-overrides.yaml."
MSG_COMPOSE_LEGACY_CONFIRM="Replace docker/docker-compose.yaml with the current definition?"
MSG_COMPOSE_LEGACY_DECLINED="docker/docker-compose.yaml left untouched. Move your changes into docker/stack-overrides.yaml, then rerun the installer (non-interactive: MOWGLI_ADOPT_LEGACY_COMPOSE=true)."

# Repository self-update (install/lib/deploy.sh)
MSG_REPO_LOCAL_CHANGES="Tracked files in this checkout were modified locally:"
MSG_REPO_LOCAL_CHANGES_SAFE="Your robot configuration (docker/.env, docker/config/, docker/stack-overrides.yaml) is not tracked by git and is never touched here."
MSG_REPO_LOCAL_CHANGES_CHOICE="(s)tash them under a named backup and continue, (k)eep them and leave the checkout as it is, (a)bort"
MSG_REPO_LOCAL_CHANGES_KEPT="Local modifications kept; the checkout was not changed."
MSG_REPO_STASHED="Local modifications saved as git stash:"
MSG_REPO_STASH_RESTORE="Restore them later with:"
MSG_REPO_STASH_FAILED="git stash failed; the checkout was not changed."
MSG_REPO_UPDATE_ABORTED="Installer aborted; nothing was changed."
MSG_REPO_UPDATE_CONFIRM="new commit(s) available. Update this checkout before continuing?"
MSG_REPO_UPDATED="Checkout fast-forwarded to"
MSG_REPO_NOT_FAST_FORWARD="This checkout has its own commits and cannot be fast-forwarded; continuing without updating. Remote:"
MSG_REPO_FETCH_FAILED="Could not reach the remote; continuing with the current checkout. Remote:"
MSG_REPO_FOREIGN_OWNER="Part of the repository belongs to another user (usually after 'sudo git ...'), so git cannot update it:"
MSG_REPO_FOREIGN_OWNER_FIX="Continuing with the current checkout. Fix it with:"
MSG_REPO_SUBMODULE_SKIPPED="Could not update the git submodules. They are only needed to BUILD the ROS2 sources; a robot running the published images does not use them."
