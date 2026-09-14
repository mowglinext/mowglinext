#!/bin/bash
# Launch Myzhar's LDLidar ROS 2 driver with our custom launch file that
# remaps ~/scan → /scan and auto-activates the lifecycle node.
set -euo pipefail

set +u
source /opt/ros/lyrical/setup.bash
source /opt/ldlidar/setup.bash
set -u

exec ros2 launch /ldlidar_scan.launch.py
