#!/usr/bin/env bash

install_motd() {
  step "MOTD"

  require_root_for "motd"

  # Resolve locale strings at install time since they won't be available at login
  local motd_not_connected="$MSG_MOTD_NOT_CONNECTED"
  local motd_free="$MSG_MOTD_FREE"
  local motd_packages="$MSG_MOTD_PACKAGES"
  local motd_local_ip="$MSG_MOTD_LOCAL_IP"
  local motd_not_set="$MSG_MOTD_NOT_SET"
  local motd_running="$MSG_MOTD_RUNNING"

  cat <<EOF | $SUDO tee /etc/profile.d/mowgli-motd.sh > /dev/null
#!/usr/bin/env bash

# Avoid noise in non-interactive shells
case "\$-" in
  *i*) ;;
  *) return ;;
esac

ENV_FILE="\$HOME/mowglinext/docker/.env"

get_env_value() {
  local key="\$1"
  [ -f "\$ENV_FILE" ] || return 0
  grep -m1 "^\${key}=" "\$ENV_FILE" 2>/dev/null | cut -d= -f2-
}

# Colors
ORANGE='\e[38;5;208m'
GREEN='\e[0;32m'
CYAN='\e[0;36m'
BOLD='\e[1m'
DIM='\e[2m'
NC='\e[0m'

# Banner
echo -e "\${ORANGE}"
cat <<'BANNER'
███╗   ███╗ ██████╗ ██╗    ██╗ ██████╗ ██╗     ██╗   ███╗   ██╗███████╗██╗  ██╗████████╗
████╗ ████║██╔═══██╗██║    ██║██╔════╝ ██║     ██║   ████╗  ██║██╔════╝╚██╗██╔╝╚══██╔══╝
██╔████╔██║██║   ██║██║ █╗ ██║██║  ███╗██║     ██║   ██╔██╗ ██║█████╗   ╚███╔╝    ██║
██║╚██╔╝██║██║   ██║██║███╗██║██║   ██║██║     ██║   ██║╚██╗██║██╔══╝   ██╔██╗    ██║
██║ ╚═╝ ██║╚██████╔╝╚███╔███╔╝╚██████╔╝███████╗██║   ██║ ╚████║███████╗██╔╝ ██╗   ██║
╚═╝     ╚═╝ ╚═════╝  ╚══╝╚══╝  ╚═════╝ ╚══════╝╚═╝   ╚═╝  ╚═══╝╚══════╝╚═╝  ╚═╝   ╚═╝
BANNER
echo -e "\${NC}"

echo -e "\e[2mMowgli II — Next Gen Mower Stack\e[0m"
echo ""

# System info
UPDATES=\$(apt list --upgradable 2>/dev/null | awk 'NR>1' | wc -l | tr -d ' ')
if [ -f /etc/apt/apt.conf.d/99defaultrelease ]; then
  PINNED="yes"
else
  PINNED="no"
fi
HOSTNAME=\$(hostname)
IP=\$(hostname -I 2>/dev/null | awk '{print \$1}')
IFACE=\$(ip route 2>/dev/null | awk '/default/ {print \$5; exit}')
MAC=\$(ip link show "\$IFACE" 2>/dev/null | awk '/ether/ {print \$2}' || echo "n/a")
SSID=\$(iwgetid -r 2>/dev/null || echo "${motd_not_connected}")
UPTIME=\$(uptime -p 2>/dev/null || echo "n/a")
TEMP=\$(vcgencmd measure_temp 2>/dev/null | cut -d= -f2 || echo "n/a")
LOAD=\$(awk '{print \$1, \$2, \$3}' /proc/loadavg 2>/dev/null || echo "n/a")
MEM=\$(free -m 2>/dev/null | awk '/Mem/ {printf "%d/%d MiB", \$3, \$2}' || echo "n/a")
DISK=\$(df -h / 2>/dev/null | awk 'END {print \$4 " ${motd_free} / " \$2}' || echo "n/a")

# Mowgli env
ROS_DOMAIN_ID=\$(get_env_value ROS_DOMAIN_ID)
MOWER_IP=\$(get_env_value MOWER_IP)
GNSS_RECEIVER_FAMILY=\$(get_env_value GNSS_RECEIVER_FAMILY)
GNSS_SERIAL_DEVICE=\$(get_env_value GNSS_SERIAL_DEVICE)
GNSS_SERIAL_BAUD=\$(get_env_value GNSS_SERIAL_BAUD)
LIDAR_PORT=\$(get_env_value LIDAR_PORT)
LIDAR_BAUD=\$(get_env_value LIDAR_BAUD)
GNSS_NTRIP_ENABLED=\$(get_env_value GNSS_NTRIP_ENABLED)
GNSS_RTCM_FORWARDING=\$(get_env_value GNSS_RTCM_FORWARDING)

# Docker/container info
if command -v docker >/dev/null 2>&1; then
  DOCKER_RUNNING=\$(docker ps -q 2>/dev/null | wc -l)
else
  DOCKER_RUNNING="n/a"
fi

echo -e "\${BOLD}System\${NC}"
echo "  Updates    : \${UPDATES:-n/a} ${motd_packages}"
echo "  APT pin    : \${PINNED}"
echo "  Hostname   : \$HOSTNAME"
echo "  ${motd_local_ip}   : \${IP:-n/a}"
echo "  MAC        : \$MAC"
echo "  Wi-Fi      : \$SSID"
echo "  Uptime     : \$UPTIME"
echo "  Temp       : \$TEMP"
echo "  CPU load   : \$LOAD"
echo "  RAM        : \$MEM"
echo "  Disk       : \$DISK"
echo ""

echo -e "\${BOLD}Mowgli config\${NC}"
echo "  ROS_DOMAIN : \${ROS_DOMAIN_ID:-${motd_not_set}}"
echo "  MOWER_IP   : \${MOWER_IP:-${motd_not_set}}"
echo "  GNSS       : \${GNSS_SERIAL_DEVICE:-${motd_not_set}} @ \${GNSS_SERIAL_BAUD:-?} (\${GNSS_RECEIVER_FAMILY:-auto})"
echo "  NTRIP      : \${GNSS_NTRIP_ENABLED:-false}"
echo "  RTCM fwd   : \${GNSS_RTCM_FORWARDING:-true}"
echo "  LiDAR      : \${LIDAR_PORT:-${motd_not_set}} @ \${LIDAR_BAUD:-?}"
echo ""

echo -e "\${BOLD}Docker\${NC}"
echo "  Containers : \${DOCKER_RUNNING} ${motd_running}"
echo ""

echo -e "\${BOLD}Helpers\${NC}"
echo "  mowgli-check   mowgli-logs   mowgli-restart   mowgli-ps"
echo "  mowgli-gps-logs   mowgli-lidar-logs   mowgli-shell"
echo ""
EOF

  $SUDO chmod +x /etc/profile.d/mowgli-motd.sh
  info "Installed /etc/profile.d/mowgli-motd.sh"
}
