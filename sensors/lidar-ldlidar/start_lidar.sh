#!/bin/bash
# Launch Myzhar's LDLidar ROS 2 driver with our custom launch file that
# remaps ~/scan → /scan, auto-activates the lifecycle node, and (issue #569)
# starts mowgli_lidar_pwm alongside it.
set -euo pipefail

set +u
source /opt/ros/lyrical/setup.bash
source /opt/ldlidar/setup.bash
set -u

# lidar_pwm_* settings live in mowgli_robot.yaml (the GUI-edited config, same
# source of truth as lidar_enabled — root CLAUDE.md Invariant 15), not in
# docker/.env: config resolution here is always YAML -> built-in default,
# matching sensors/CLAUDE.md's convention. The installed file is SPARSE, so
# every key needs the default below. NOT using a grep/sed scrape (see
# sensors/gps/start_gps.sh's parse_yaml gotcha) — a real YAML parser avoids
# picking up an indented key from the wrong node's block.
CONFIG="${MOWGLI_ROBOT_CONFIG:-/config/mowgli_robot.yaml}"

read -r LIDAR_PWM_ENABLED LIDAR_PWM_GPIO_PIN <<EOF_PY
$(python3 - "$CONFIG" <<'PY'
import sys
import yaml

config_path = sys.argv[1]
try:
    with open(config_path, "r", encoding="utf-8") as handle:
        document = yaml.safe_load(handle) or {}
except FileNotFoundError:
    document = {}
params = (document.get("mowgli") or {}).get("ros__parameters") or {}

def truthy(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("true", "1", "yes", "on")

enabled = truthy(params.get("lidar_pwm_enabled", False))
gpio_pin = int(params.get("lidar_pwm_gpio_pin", 12))
print(f"{'true' if enabled else 'false'} {gpio_pin}")
PY
)
EOF_PY

exec ros2 launch /ldlidar_scan.launch.py \
  "lidar_pwm_enabled:=${LIDAR_PWM_ENABLED}" \
  "lidar_pwm_gpio_pin:=${LIDAR_PWM_GPIO_PIN}"
