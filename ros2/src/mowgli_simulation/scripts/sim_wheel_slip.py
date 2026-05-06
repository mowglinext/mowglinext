#!/usr/bin/env python3
# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later

"""
sim_wheel_slip.py — SIMULATION ONLY

Relays /wheel_odom_raw (Gazebo ground-truth) → /wheel_odom (what the EKF
sees), with periodic short slip events that briefly inflate the reported
longitudinal velocity. Models the encoder-vs-ground-truth divergence a
real diff-drive robot sees on grass: wheel keeps rotating, encoder keeps
ticking, but the chassis under-translates — so wheel_odom over-reports.

Default cycle (tunable via parameters):
  every  slip_period_s  (30 s),
  for    slip_duration_s (1.0 s),
  add    slip_vx_bias  (+0.05 m/s)  to twist.linear.x.

Pose is passed through unchanged. The dual EKF only fuses /wheel_odom
twist (not pose), so the slip surfaces as a transient encoder vs GPS /
gyro divergence.

Wiring
------
  gazebo_bridge.yaml: ros_topic_name=/wheel_odom_raw  (was /wheel_odom)
  this node:          /wheel_odom_raw -> /wheel_odom

Safety: read-only consumer of one Gazebo-bridged topic, publishes a
single sensor topic. No drive commands, no TF, no safety topic.
"""

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from nav_msgs.msg import Odometry


class SimWheelSlip(Node):
    def __init__(self) -> None:
        super().__init__("sim_wheel_slip")

        self._input_topic = str(
            self.declare_parameter("input_topic", "/wheel_odom_raw").value
        )
        self._output_topic = str(
            self.declare_parameter("output_topic", "/wheel_odom").value
        )
        self._slip_period = float(
            self.declare_parameter("slip_period_s", 30.0).value
        )
        self._slip_duration = float(
            self.declare_parameter("slip_duration_s", 1.0).value
        )
        self._slip_vx_bias = float(
            self.declare_parameter("slip_vx_bias", 0.05).value
        )

        if self._slip_duration >= self._slip_period:
            self.get_logger().warn(
                "slip_duration_s (%.2f) >= slip_period_s (%.2f) — slip will be "
                "permanent. Clamping duration to half-period."
                % (self._slip_duration, self._slip_period)
            )
            self._slip_duration = self._slip_period / 2.0

        self._cycle_origin_s: Optional[float] = None
        self._slip_count = 0

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

        self._pub = self.create_publisher(Odometry, self._output_topic, pub_qos)
        self._sub = self.create_subscription(
            Odometry, self._input_topic, self._on_odom, sub_qos
        )

        self.create_timer(15.0, self._log_stats)

        self.get_logger().info(
            "sim_wheel_slip ready: %s -> %s; slip every %.1fs for %.2fs at "
            "+%.3f m/s longitudinal"
            % (
                self._input_topic,
                self._output_topic,
                self._slip_period,
                self._slip_duration,
                self._slip_vx_bias,
            )
        )

    def _slip_active(self, ros_now_s: float) -> bool:
        if self._cycle_origin_s is None:
            self._cycle_origin_s = ros_now_s
        elapsed = (ros_now_s - self._cycle_origin_s) % self._slip_period
        # Slip window starts at the boundary of each period.
        return elapsed < self._slip_duration

    def _on_odom(self, msg: Odometry) -> None:
        ros_now_s = self.get_clock().now().nanoseconds * 1e-9
        out = msg  # nav_msgs/Odometry — we mutate twist.linear.x and covariance
        if self._slip_active(ros_now_s):
            out.twist.twist.linear.x += self._slip_vx_bias
            self._slip_count += 1
        # Override covariance to match the real hardware_bridge values
        # (hardware_bridge_node.cpp ~lines 1196-1212). gz-sim diff-drive
        # publishes default covariance which doesn't enforce the
        # non-holonomic constraint on vy. Without cov[7]=1e-4, GPS lateral
        # noise leaks into apparent sideways drift inside the EKF.
        # Indices: row-major 6×6 — vx=[0], vy=[7], vz=[14], wx=[21], wy=[28], wz=[35].
        cov = list(out.twist.covariance)
        cov[0] = 0.01
        cov[7] = 1e-4
        cov[14] = 1e6
        cov[21] = 1e6
        cov[28] = 1e6
        cov[35] = 9e-4
        out.twist.covariance = cov
        self._pub.publish(out)

    def _log_stats(self) -> None:
        self.get_logger().info(
            "sim_wheel_slip stats: slipped samples in last 15s = %d"
            % self._slip_count
        )
        self._slip_count = 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimWheelSlip()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
