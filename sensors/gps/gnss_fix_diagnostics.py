#!/usr/bin/env python3
"""Small common GNSS diagnostic publisher for backends with only /gps/fix."""

from __future__ import annotations

import os

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus


class GnssFixDiagnostics(Node):
    def __init__(self) -> None:
        super().__init__("gnss_fix_diagnostics")
        self._backend = os.environ.get("GNSS_DIAGNOSTIC_BACKEND", "generic").strip() or "generic"
        self._last_fix: NavSatFix | None = None
        self._last_fix_t = 0.0
        self.create_subscription(NavSatFix, "/gps/fix", self._on_fix, 10)
        self._pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_timer(1.0, self._publish)
        self.get_logger().info(f"GNSS diagnostics running for backend={self._backend}")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_fix(self, msg: NavSatFix) -> None:
        self._last_fix = msg
        self._last_fix_t = self._now()

    def _publish(self) -> None:
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = f"GNSS ({self._backend}): fix"
        status.hardware_id = self._backend

        now = self._now()
        if self._last_fix is None or now - self._last_fix_t > 5.0:
            status.level = DiagnosticStatus.ERROR
            status.message = "no /gps/fix in 5 s"
            status.values = [KeyValue(key="fix_topic", value="/gps/fix")]
            arr.status.append(status)
            self._pub.publish(arr)
            return

        fix = self._last_fix
        status.values = [
            KeyValue(key="fix_topic", value="/gps/fix"),
            KeyValue(key="frame_id", value=fix.header.frame_id),
            KeyValue(key="latitude", value=f"{fix.latitude:.9f}"),
            KeyValue(key="longitude", value=f"{fix.longitude:.9f}"),
            KeyValue(key="altitude", value=f"{fix.altitude:.3f}"),
            KeyValue(key="status", value=str(int(fix.status.status))),
        ]

        if fix.status.status == NavSatStatus.STATUS_NO_FIX:
            status.level = DiagnosticStatus.ERROR
            status.message = "no fix"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "fix received"

        arr.status.append(status)
        self._pub.publish(arr)


def main() -> None:
    rclpy.init()
    node = GnssFixDiagnostics()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
