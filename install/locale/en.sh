#!/usr/bin/env bash
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
