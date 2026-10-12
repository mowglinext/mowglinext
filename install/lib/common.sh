#colours
#info, warn, error, step
#prompt, confirm
#require_root_for

#!/usr/bin/env bash

# ── Colours & helpers ───────────────────────────────────────────────────────

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
DIM='\033[2m'
NC='\033[0m'

info()  { echo -e "  ${GREEN}OK${NC}  $*"; }
warn()  { echo -e "  ${YELLOW}!!${NC}  $*"; }
fail()  { echo -e "  ${RED}FAIL${NC}  $*"; }
error() { echo -e "${RED}[x]${NC} $*" >&2; }
step()  { echo -e "\n${CYAN}${BOLD}── $* ──${NC}"; }
ask()   { echo -en "${BOLD}$1${NC} "; }

# Prompt with default value. Sets REPLY global.
# Every prompt carries its default as $2. In non-interactive mode (explicit
# --non-interactive, or no controlling terminal) that default IS the answer,
# so a flow never blocks on read and every unset choice is the documented one.
prompt() {
  local answer
  if [[ "${NON_INTERACTIVE:-false}" == "true" ]]; then
    REPLY="${2:-}"
    return 0
  fi
  echo -en "${BOLD}$1 [${2:-}]:${NC} " >/dev/tty
  read -r answer </dev/tty
  echo >/dev/tty
  REPLY="${answer:-$2}"
}

# Yes/no prompt. Usage: if confirm "Continue?"; then ...
confirm() {
  local answer
  if [[ "${NON_INTERACTIVE:-false}" == "true" ]]; then
    return 0
  fi
  echo -en "${BOLD}$1 [Y/n]:${NC} " >/dev/tty
  read -r answer </dev/tty
  echo >/dev/tty
  [[ "${answer,,}" != "n" ]]
}

command_exists() {
  command -v "$1" &>/dev/null
}

# List available UART devices. Returns array in UART_DEVICES.
# Includes both existing ports and common Raspberry Pi UART ports
# (which may not exist yet until dtoverlays are enabled and system reboots).
detect_uart_ports() {
  UART_DEVICES=()
  local dev seen=""

  # First: ports that actually exist right now
  for dev in /dev/ttyAMA* /dev/ttyS* /dev/ttyUSB*; do
    if [ -e "$dev" ]; then
      UART_DEVICES+=("$dev")
      seen+=" $dev "
    fi
  done

  # Second: common Pi UART ports that will appear after dtoverlay + reboot
  local common_ports=(/dev/ttyAMA0 /dev/ttyAMA1 /dev/ttyAMA2 /dev/ttyAMA3 /dev/ttyAMA4 /dev/ttyAMA5 /dev/ttyS0)
  for dev in "${common_ports[@]}"; do
    if [[ "$seen" != *" $dev "* ]]; then
      UART_DEVICES+=("${dev} (*)")
    fi
  done
}

# Interactive UART port picker.
# Usage: pick_uart_port "default_device"
# Sets REPLY to the selected device path.
pick_uart_port() {
  local default_device="${1:-}"

  detect_uart_ports

  echo ""
  info "$MSG_UART_DETECTING"

  if [ ${#UART_DEVICES[@]} -eq 0 ]; then
    warn "$MSG_UART_NONE_FOUND"
    prompt "$MSG_UART_MANUAL_PROMPT" "$default_device"
    return
  fi

  echo "$MSG_UART_AVAILABLE"
  echo -e "  ${DIM}(*) = ${MSG_UART_AFTER_REBOOT:-available after reboot}${NC}"
  local i=1
  local default_idx=""
  for dev in "${UART_DEVICES[@]}"; do
    local clean_dev="${dev% (*)}"
    local marker=""
    if [ "$clean_dev" = "$default_device" ]; then
      marker="  <--"
      default_idx="$i"
    fi
    echo "  ${i}) ${dev}${marker}"
    i=$((i + 1))
  done
  echo "  ${i}) $MSG_UART_MANUAL"

  prompt "$MSG_UART_SELECT" "${default_idx:-1}"
  local choice="$REPLY"

  if [ "$choice" -eq "$i" ] 2>/dev/null; then
    prompt "$MSG_UART_MANUAL_PROMPT" "$default_device"
    return
  fi

  if [ "$choice" -ge 1 ] 2>/dev/null && [ "$choice" -lt "$i" ] 2>/dev/null; then
    # Strip the " (*)" suffix for ports not yet present
    REPLY="${UART_DEVICES[$((choice - 1))]}"
    REPLY="${REPLY% (*)}"
    return
  fi

  warn "$MSG_UART_INVALID"
  REPLY="$default_device"
}

require_root_for() {
  if [ "$(id -u)" -ne 0 ]; then
    SUDO="sudo"
  else
    SUDO=""
  fi
}

require_root() {
  if [[ $EUID -ne 0 ]]; then
    error "This action requires root privileges. Please run as root or with sudo."
    exit 1
  fi
}