#!/usr/bin/env python3
# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later

"""
sim_cmd_vel_unstamp.py — SIMULATION ONLY

Strip the header off geometry_msgs/TwistStamped on /cmd_vel_stamped (the
twist_mux output) and republish as geometry_msgs/Twist on /cmd_vel,
which is what Gazebo's gz.msgs.Twist bridge expects.

Why a custom relay instead of `topic_tools transform`: the upstream
`topic_tools transform` validates the input topic's type at startup and
exits with `RuntimeError: Wrong input topic` when the publisher hasn't
come up yet. With sim_full_system.launch.py firing twist_mux and the
relay together, the relay frequently races and dies. This relay
subscribes regardless of current publisher state and starts forwarding
as soon as messages arrive.

Wiring
------
  twist_mux  → /cmd_vel_stamped (TwistStamped)
  this node  → /cmd_vel          (Twist)            → gz_ros2_bridge

Safety: read-only consumer of one topic, single Twist publisher; no
TF, no safety topic. Sim-only — real hardware uses /cmd_vel as
TwistStamped end-to-end.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from geometry_msgs.msg import Twist, TwistStamped


class SimCmdVelUnstamp(Node):
    def __init__(self) -> None:
        super().__init__("sim_cmd_vel_unstamp")

        self._input_topic = str(
            self.declare_parameter("input_topic", "/cmd_vel_stamped").value
        )
        self._output_topic = str(
            self.declare_parameter("output_topic", "/cmd_vel").value
        )

        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._pub = self.create_publisher(Twist, self._output_topic, pub_qos)
        self._sub = self.create_subscription(
            TwistStamped, self._input_topic, self._on_msg, sub_qos
        )
        self._count = 0
        self.create_timer(15.0, self._log_stats)

        self.get_logger().info(
            "sim_cmd_vel_unstamp ready: %s (TwistStamped) -> %s (Twist)"
            % (self._input_topic, self._output_topic)
        )

    def _on_msg(self, msg: TwistStamped) -> None:
        self._pub.publish(msg.twist)
        self._count += 1

    def _log_stats(self) -> None:
        self.get_logger().info(
            "sim_cmd_vel_unstamp stats: forwarded %d msgs in last 15s"
            % self._count
        )
        self._count = 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimCmdVelUnstamp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
