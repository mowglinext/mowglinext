#!/usr/bin/env python3
"""
precision_monitor.py — Real-time precision metrics publisher for Foxglove.

Subscribes to wheel odometry, GPS, LiDAR, SLAM scan match, SLAM map, and
cmd_vel, then publishes Float64 topics at ~2 Hz that can be plotted directly
in Foxglove Studio Plot panels.

Published topics:
  /precision/gps_error_m          — distance between GPS pose and wheel odom pose
  /precision/wheel_odom_speed     — wheel odom linear velocity magnitude (m/s)
  /precision/lidar_scan_count     — number of valid LiDAR points in current scan
  /precision/lidar_min_range      — closest obstacle distance (m)
  /precision/slam_map_known_pct   — percentage of SLAM map cells that are known
  /precision/localization_quality — composite quality score 0–100

Usage (inside the dev-sim container, after sourcing the workspace):
  python3 /ros2_ws/scripts/precision_monitor.py

Or from host:
  docker compose exec dev-sim bash -c \
    "source /ros2_ws/install/setup.bash && python3 /ros2_ws/scripts/precision_monitor.py"
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Float64


# ---------------------------------------------------------------------------
# QoS helpers — mirrors what the rest of the mowgli stack uses
# ---------------------------------------------------------------------------

#: Best-effort, shallow queue — for high-rate sensor topics.
_SENSOR_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

#: Reliable + transient-local — for latched map / SLAM topics.
_MAP_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

#: Reliable — for odometry and cmd_vel.
_RELIABLE_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

# ---------------------------------------------------------------------------
# Quality scoring constants
# ---------------------------------------------------------------------------

#: GPS error above this threshold (m) starts penalising the score.
_GPS_ERROR_PENALTY_THRESHOLD_M: float = 0.1

#: Maximum GPS penalty points (subtracted when GPS error is very large).
_GPS_MAX_PENALTY: float = 30.0

#: GPS error that gives the full penalty (linear interpolation up to this).
_GPS_ERROR_MAX_M: float = 2.0

#: LiDAR scan count below this starts penalising the score.
_SCAN_COUNT_PENALTY_THRESHOLD: int = 100

#: Maximum LiDAR scan count penalty points.
_SCAN_COUNT_MAX_PENALTY: float = 20.0

#: SLAM map known percentage below this starts penalising the score.
_MAP_KNOWN_PCT_PENALTY_THRESHOLD: float = 10.0

#: Maximum SLAM map known percentage penalty points.
_MAP_KNOWN_MAX_PENALTY: float = 20.0

#: Maximum wheel-speed penalty (robot stationary when cmd_vel says move).
_SPEED_MAX_PENALTY: float = 30.0

#: cmd_vel linear speed below which the robot is considered stationary.
_SPEED_MOVING_THRESHOLD_M_S: float = 0.05


class PrecisionMonitorNode(Node):
    """Publishes precision and localization quality metrics for Foxglove."""

    def __init__(self) -> None:
        super().__init__("precision_monitor")

        # ── Latest message cache (None = not received yet) ───────────────
        self._wheel_odom: Odometry | None = None
        self._gps_pose: PoseWithCovarianceStamped | None = None
        self._scan: LaserScan | None = None
        self._map: OccupancyGrid | None = None
        self._cmd_vel: Twist | None = None

        # ── Subscribers ──────────────────────────────────────────────────
        self.create_subscription(
            Odometry,
            "/wheel_odom",
            self._on_wheel_odom,
            _SENSOR_QOS,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/gps/pose_sim",
            self._on_gps_pose,
            _SENSOR_QOS,
        )
        self.create_subscription(
            LaserScan,
            "/scan",
            self._on_scan,
            _SENSOR_QOS,
        )
        # NOTE: the /slam_toolbox/scan_visualization subscription was removed —
        # slam_toolbox is gone (fusion_graph is the sole localizer) and nothing
        # publishes that PointCloud2 topic anymore.
        self.create_subscription(
            OccupancyGrid,
            "/map",
            self._on_map,
            _MAP_QOS,
        )
        # cmd_vel is used only for the speed penalty in localization_quality.
        self.create_subscription(
            Twist,
            "/cmd_vel",
            self._on_cmd_vel,
            _SENSOR_QOS,
        )

        # ── Publishers ───────────────────────────────────────────────────
        self._pub_gps_error = self._make_pub("/precision/gps_error_m")
        self._pub_odom_speed = self._make_pub("/precision/wheel_odom_speed")
        self._pub_scan_count = self._make_pub("/precision/lidar_scan_count")
        self._pub_min_range = self._make_pub("/precision/lidar_min_range")
        self._pub_map_known = self._make_pub("/precision/slam_map_known_pct")
        self._pub_quality = self._make_pub("/precision/localization_quality")

        # ── Publish timer — 2 Hz ─────────────────────────────────────────
        self.create_timer(0.5, self._publish_metrics)

        self.get_logger().info("precision_monitor started — publishing at 2 Hz")

    # ── Subscriber callbacks ─────────────────────────────────────────────

    def _on_wheel_odom(self, msg: Odometry) -> None:
        self._wheel_odom = msg

    def _on_gps_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._gps_pose = msg

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = msg

    def _on_slam_scan(self, _msg: PointCloud2) -> None:
        # Received but not processed further — presence confirms SLAM is alive.
        # Future: could compare point density to odom data if needed.
        pass

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map = msg

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._cmd_vel = msg

    # ── Metric computations ──────────────────────────────────────────────

    def _compute_gps_error(self) -> float | None:
        """3-D distance between GPS pose and wheel odom pose (m)."""
        if self._gps_pose is None or self._wheel_odom is None:
            return None

        gp = self._gps_pose.pose.pose.position
        op = self._wheel_odom.pose.pose.position

        return math.sqrt(
            (gp.x - op.x) ** 2
            + (gp.y - op.y) ** 2
            + (gp.z - op.z) ** 2
        )

    def _compute_odom_speed(self) -> float | None:
        """Linear velocity magnitude from wheel odometry (m/s)."""
        if self._wheel_odom is None:
            return None
        v = self._wheel_odom.twist.twist.linear
        return math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)

    def _compute_scan_stats(self) -> tuple[int, float] | None:
        """Return (valid_point_count, min_range) from the latest scan, or None."""
        if self._scan is None:
            return None
        s = self._scan
        valid = [
            r for r in s.ranges
            if s.range_min < r < s.range_max and not math.isinf(r) and not math.isnan(r)
        ]
        count = len(valid)
        min_range = min(valid) if valid else float("inf")
        return count, min_range

    def _compute_map_known_pct(self) -> float | None:
        """Percentage of OccupancyGrid cells that are known (value >= 0, != -1)."""
        if self._map is None:
            return None
        total = len(self._map.data)
        if total == 0:
            return 0.0
        known = sum(1 for c in self._map.data if c >= 0)
        return known / total * 100.0

    def _compute_localization_quality(
        self,
        gps_error: float | None,
        scan_count: int | None,
        map_known_pct: float | None,
        odom_speed: float | None,
    ) -> float:
        """
        Composite localization quality score in [0, 100].

        Starts at 100 and subtracts penalties:
          - GPS error penalty:      up to 30 pts when error > 0.1 m
          - Scan count penalty:     up to 20 pts when count < 100 points
          - SLAM map known penalty: up to 20 pts when known < 10 %
          - Speed mismatch penalty: up to 30 pts when robot is stationary
                                    but cmd_vel requests motion
        """
        score: float = 100.0

        # GPS error penalty — linear from 0 at threshold to max at _GPS_ERROR_MAX_M
        if gps_error is not None and gps_error > _GPS_ERROR_PENALTY_THRESHOLD_M:
            excess = gps_error - _GPS_ERROR_PENALTY_THRESHOLD_M
            max_excess = _GPS_ERROR_MAX_M - _GPS_ERROR_PENALTY_THRESHOLD_M
            ratio = min(excess / max_excess, 1.0)
            score -= ratio * _GPS_MAX_PENALTY

        # LiDAR scan count penalty — linear from 0 at threshold to max at 0
        if scan_count is not None and scan_count < _SCAN_COUNT_PENALTY_THRESHOLD:
            ratio = 1.0 - (scan_count / _SCAN_COUNT_PENALTY_THRESHOLD)
            score -= ratio * _SCAN_COUNT_MAX_PENALTY

        # SLAM map known % penalty — linear from 0 at threshold to max at 0 %
        if map_known_pct is not None and map_known_pct < _MAP_KNOWN_PCT_PENALTY_THRESHOLD:
            ratio = 1.0 - (map_known_pct / _MAP_KNOWN_PCT_PENALTY_THRESHOLD)
            score -= ratio * _MAP_KNOWN_MAX_PENALTY

        # Speed mismatch penalty — penalise when cmd_vel says move but odom is still
        if self._cmd_vel is not None and odom_speed is not None:
            cmd_speed = abs(self._cmd_vel.linear.x)
            if cmd_speed > _SPEED_MOVING_THRESHOLD_M_S and odom_speed < _SPEED_MOVING_THRESHOLD_M_S:
                # Scale: the larger the commanded speed the worse the mismatch
                ratio = min(cmd_speed / 1.0, 1.0)  # 1 m/s = full penalty
                score -= ratio * _SPEED_MAX_PENALTY

        return max(0.0, score)

    # ── Publish cycle ────────────────────────────────────────────────────

    def _publish_metrics(self) -> None:
        """Compute and publish all precision metrics."""
        gps_error = self._compute_gps_error()
        odom_speed = self._compute_odom_speed()
        scan_stats = self._compute_scan_stats()
        map_known_pct = self._compute_map_known_pct()

        scan_count: int | None = scan_stats[0] if scan_stats is not None else None
        min_range: float | None = scan_stats[1] if scan_stats is not None else None

        quality = self._compute_localization_quality(
            gps_error, scan_count, map_known_pct, odom_speed
        )

        self._publish_float(self._pub_gps_error, gps_error if gps_error is not None else -1.0)
        self._publish_float(self._pub_odom_speed, odom_speed if odom_speed is not None else -1.0)
        self._publish_float(self._pub_scan_count, float(scan_count) if scan_count is not None else -1.0)
        self._publish_float(self._pub_min_range, min_range if min_range is not None else -1.0)
        self._publish_float(self._pub_map_known, map_known_pct if map_known_pct is not None else -1.0)
        self._publish_float(self._pub_quality, quality)

    # ── Helpers ──────────────────────────────────────────────────────────

    def _make_pub(self, topic: str):
        """Create a Float64 publisher with a shallow, best-effort QoS."""
        return self.create_publisher(Float64, topic, 10)

    @staticmethod
    def _publish_float(publisher, value: float) -> None:
        msg = Float64()
        msg.data = value
        publisher.publish(msg)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main() -> None:
    rclpy.init()
    node = PrecisionMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("precision_monitor shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
