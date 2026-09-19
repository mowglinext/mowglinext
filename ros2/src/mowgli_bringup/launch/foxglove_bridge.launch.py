# Copyright 2026 Mowgli Project
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


"""
foxglove_bridge.launch.py

Starts the foxglove_bridge WebSocket server for Foxglove Studio.

Foxglove Bridge provides a high-performance binary WebSocket protocol,
replacing the JSON-based rosbridge_server used on Humble.

Connect Foxglove Studio to: ws://<host>:8765
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # ------------------------------------------------------------------
    # Declared arguments
    # ------------------------------------------------------------------
    port_arg = DeclareLaunchArgument(
        "port",
        default_value="8765",
        description="Port number for the Foxglove Bridge WebSocket server.",
    )

    send_buffer_limit_arg = DeclareLaunchArgument(
        "send_buffer_limit",
        # 1 MB (was 10 MB): when a client falls behind on the WebSocket,
        # the bridge buffers up to this many bytes per client before
        # dropping old messages. 10 MB at the typical visualisation
        # bandwidth (~250 KB/s) is ~40 s of backlog, which manifests as
        # multi-second pose/scan lag in the viewer even though the ROS2
        # side is real-time. 1 MB caps backlog at ~4 s; combined with
        # the lower /tf rate after the recent tuning (99 Hz instead of
        # 190 Hz), the buffer should rarely fill at all on a healthy
        # link, and when it does the latest pose/scan is preferred over
        # buffered history.
        default_value="1000000",
        description="Maximum bytes buffered per client before dropping messages.",
    )

    # ------------------------------------------------------------------
    # Resolved substitutions
    # ------------------------------------------------------------------
    port = LaunchConfiguration("port")
    send_buffer_limit = LaunchConfiguration("send_buffer_limit")

    # ------------------------------------------------------------------
    # foxglove_bridge node
    # ------------------------------------------------------------------
    foxglove_bridge_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        # RESPAWN: this bridge is the GUI's ONLY link to ROS (gui/pkg/providers/
        # ros.go connects to ws://localhost:8765 and every topic, service and
        # parameter the operator sees goes through it). When it dies the web UI
        # silently shows no robot on the map and "no GPS" while the robot is
        # perfectly localised and still mowing — field 2026-09-18, where it
        # segfaulted (exit -11) moments after the GUI backend connected and was
        # never restarted, leaving the operator blind for a whole run.
        # It is outside the motion path, so restarting it can only restore
        # observability; a crash loop is bounded by respawn_delay.
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            {
                "port": port,
                "address": "0.0.0.0",
                "send_buffer_limit": send_buffer_limit,
                "num_threads": 0,
            },
        ],
    )

    # ------------------------------------------------------------------
    # LaunchDescription
    # ------------------------------------------------------------------
    return LaunchDescription(
        [
            port_arg,
            send_buffer_limit_arg,
            foxglove_bridge_node,
        ]
    )
