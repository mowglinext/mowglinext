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

# ── Mode menu (bare run on an installed robot) ──
MSG_MODE_TITLE="This robot is already installed. What do you want to do?"
MSG_MODE_UPDATE="Update — sync the checkout, pull the images, restart (no host updater involved)"
MSG_MODE_REPAIR="Repair — re-apply udev rules, UARTs, .env, compose and helpers from the saved choices"
MSG_MODE_REINSTALL="Reinstall / reconfigure — go through the hardware questions again"
MSG_MODE_CHECK="Check — diagnostics only"
MSG_MODE_INVALID="Invalid choice, running an update"
MSG_MODE_SELECTED="Mode:"
MSG_MODE_UNINSTALL="Uninstall — remove everything except the maps and mowgli_robot.yaml"

# ── Uninstall (uninstall.sh) ──
MSG_UNINSTALL_TITLE="Uninstall MowgliNext"
MSG_UNINSTALL_REMOVES="This removes:"
MSG_UNINSTALL_CONTAINERS="all mowgli-* containers and their images (maps volume untouched)"
MSG_UNINSTALL_UPDATER="the host updater service, binary, config and state"
MSG_UNINSTALL_HOST="udev rules, DDS sysctl, MOTD, mowgli-* helper commands, our rc.local"
MSG_UNINSTALL_KEEPS="This keeps:"
MSG_UNINSTALL_MAPS="Docker volume: areas.dat and the saved localization graph"
MSG_UNINSTALL_NOT_OURS="Docker itself and the /boot UART overlays (not ours)"
MSG_UNINSTALL_CONFIRM="Remove MowgliNext from this host?"
MSG_UNINSTALL_NEEDS_YES="No terminal to confirm on; pass --yes to uninstall non-interactively."
MSG_UNINSTALL_ABORTED="Uninstall aborted; nothing was changed."
MSG_UNINSTALL_COMPOSE_FAILED="docker compose down failed; removing the containers one by one."
MSG_UNINSTALL_NOT_A_CHECKOUT="Refusing to remove a directory that is not a MowgliNext checkout:"
MSG_UNINSTALL_REMOVED="Removed"
MSG_UNINSTALL_RESTORED="Restored"
MSG_UNINSTALL_KEPT="Kept"
MSG_UNINSTALL_DONE="MowgliNext removed. Reinstall with the command from https://mowgli.garden — it will find mowgli_robot.yaml and the maps volume again."
MSG_UNINSTALL_MAPS_HINT="To delete the maps too: docker volume rm"
MSG_MODE_NO_TTY="already installed, no terminal to ask; pass install|repair|check to choose"

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
MSG_UART_PORT_TAKEN="%s cannot use %s: the %s controller is wired there. Pick another port (or USB)."
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


# ── Tools (tools.sh) ──
MSG_TOOLS_HELPERS="Optional tools: Mowgli helpers"

# ── MOTD (motd.sh) ──
MSG_MOTD_NOT_CONNECTED="not connected"
MSG_MOTD_FREE="free"
MSG_MOTD_PACKAGES="package(s)"
MSG_MOTD_LOCAL_IP="Local IP"
MSG_MOTD_NOT_SET="not set"
MSG_MOTD_RUNNING="running"

MSG_UPDATER_STACK_BACKEND="Managed release updates support the Mowgli and OpenMower v1 hardware backends."
MSG_UPDATE_MANUAL_UPDATER="The host updater is installed. This manual update regenerates docker-compose.yaml from this checkout and lets the images follow the .env tags instead of the updater's pinned release; Settings > Updates will report the installation as drifted until the next managed release, which adopts the result."
MSG_UPDATE_MANUAL_PINS="Updater image pins (docker/update-images.json) set aside as a dated copy; images now follow docker/.env."
MSG_UPDATER_DIRECTORY_MISMATCH="The host updater is configured for another checkout directory than this run. Run the installer from the same path the updater was installed with (MOWGLI_HOME=<that path>), or re-register it with --only=updater."
MSG_UPDATER_HARDWARE_LEGACY="These hardware choices require the existing installer path (MAVROS). Keeping their selected containers; coordinated release updates are not enabled."
MSG_UPDATER_HARDWARE_MANAGED="This installation already uses managed updates. A MAVROS selection requires an explicit stack migration; runtime files have not been regenerated."
MSG_UPDATER_STACK_REVIEW="Saved hardware choices. Review Software updates to apply container changes; the installed release definition has been retained."

# Compose baseline / legacy adoption (install/lib/compose.sh)
MSG_COMPOSE_BASELINE_UNAVAILABLE="Could not record a checksum of the generated Compose file (sha256sum/shasum missing, or docker/stack-definition.sha256 not writable); no baseline recorded."
MSG_COMPOSE_LEGACY_EXPLAIN="docker/docker-compose.yaml cannot be vouched for by the updater: the reason is printed above (either it predates the recorded baseline and differs from the current definition, or it was edited by hand after it was generated). If the edit was not yours, this is only the release evolving and it is safe to regenerate."
MSG_COMPOSE_LEGACY_BACKUP="The current file is kept next to the new one as docker/docker-compose.yaml.legacy-<date> or .edited-<date>. Hand-made changes you want to keep belong in docker/stack-overrides.yaml."
MSG_COMPOSE_LEGACY_CONFIRM="Back up the current docker/docker-compose.yaml and regenerate it?"
MSG_COMPOSE_MISSING="docker/docker-compose.yaml is missing; it will be recreated from the installed definition (plus docker/stack-overrides.yaml if present)."
MSG_COMPOSE_LEGACY_DECLINED="docker/docker-compose.yaml left untouched. Move your changes into docker/stack-overrides.yaml, then rerun (non-interactive: MOWGLI_ADOPT_LEGACY_COMPOSE=true)."

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
MSG_COMPOSE_MISSING_CONFIRM="Recreate docker/docker-compose.yaml?"
MSG_COMPOSE_MISSING_DECLINED="docker/docker-compose.yaml not recreated; the stack cannot start without it."
