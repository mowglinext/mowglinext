#!/usr/bin/env python3
# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later

"""
sim_navsat_rtk_fix.py — SIMULATION ONLY.

Programmable GPS-quality controller for the simulator. Subscribes to a
raw Gazebo NavSatFix and republishes on the production topic with
status, covariance, and position noise consistent with one of three
quality regimes:

  RTK_FIXED  status=GBAS_FIX (4), sigma_xy ~3 mm, no position noise
  RTK_FLOAT  status=SBAS_FIX (1), sigma_xy ~30 cm, Gaussian position noise
  NO_FIX     status=NO_FIX (-1), sigma_xy ~2 m, larger Gaussian noise

Gazebo's gz-sim-navsat-system plugin always emits status.status==
STATUS_FIX (0). Production code (navsat_to_absolute_pose_node,
slam_pose_anchor_node) gates on STATUS_GBAS_FIX, so without this relay
the sim's GPS never feeds /gps/pose_cov.

Quality cycling
---------------
The `quality_pattern` parameter is a string of `duration_s,REGIME`
segments separated by `;`. Empty (default) means always RTK_FIXED, which
matches the previous behaviour (`sim_navsat_rtk_fix` legacy mode).

Examples:
  ""                                       always RTK-Fixed
  "30,RTK_FIXED;15,RTK_FLOAT;5,NO_FIX"     30 s fixed, 15 s float, 5 s
                                            no-fix, then loops
  "120,RTK_FIXED;30,RTK_FLOAT"             80 % fixed, 20 % float

Wiring
------
  gazebo_bridge.yaml: ros_topic_name=/gps/fix_raw  (was /gps/fix)
  this node:          /gps/fix_raw -> /gps/fix     (with quality regime)

Safety: read-only consumer of one Gazebo-bridged topic, publishes a
single sensor topic. No drive commands, no TF, no safety topic.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import NavSatFix, NavSatStatus


# Quality regime -> (status code, sigma_xy meters, sigma_z meters).
# sigma_xy doubles as the position noise std-dev added to lat/lon.
QUALITY_REGIMES: dict[str, Tuple[int, float, float]] = {
    "RTK_FIXED": (NavSatStatus.STATUS_GBAS_FIX, 0.003, 0.006),
    "RTK_FLOAT": (NavSatStatus.STATUS_SBAS_FIX, 0.30, 0.60),
    "NO_FIX":    (NavSatStatus.STATUS_NO_FIX,   2.0,  4.0),
}


@dataclass(frozen=True)
class _Segment:
    duration_s: float
    regime: str


def _parse_pattern(spec: str) -> List[_Segment]:
    """Parse "30,RTK_FIXED;15,RTK_FLOAT" into a list of segments.

    Whitespace tolerant; raises ValueError on bad regime names.
    Empty input returns [].
    """
    if not spec.strip():
        return []
    out: List[_Segment] = []
    for raw in spec.split(";"):
        part = raw.strip()
        if not part:
            continue
        if "," not in part:
            raise ValueError(f"bad segment '{part}': expected 'duration,REGIME'")
        dur_s, name = part.split(",", 1)
        regime = name.strip().upper()
        if regime not in QUALITY_REGIMES:
            raise ValueError(
                f"unknown regime '{regime}'; valid: {sorted(QUALITY_REGIMES)}"
            )
        out.append(_Segment(float(dur_s.strip()), regime))
    if any(s.duration_s <= 0.0 for s in out):
        raise ValueError("segment durations must be > 0")
    return out


class SimNavSatRtkFix(Node):
    def __init__(self) -> None:
        super().__init__("sim_navsat_rtk_fix")

        self._input_topic = self.declare_parameter(
            "input_topic", "/gps/fix_raw"
        ).value
        self._output_topic = self.declare_parameter(
            "output_topic", "/gps/fix"
        ).value
        # Programmable cycle. Empty means always RTK_FIXED (legacy mode).
        pattern_spec = str(
            self.declare_parameter("quality_pattern", "").value
        )
        # Noise reproducibility — lets us compare two sessions with the
        # same quality cycle.
        self._rng = random.Random(
            int(self.declare_parameter("noise_seed", 42).value)
        )

        try:
            self._segments = _parse_pattern(pattern_spec)
        except ValueError as exc:
            self.get_logger().error(
                "Bad quality_pattern '%s': %s — falling back to always-FIXED"
                % (pattern_spec, exc)
            )
            self._segments = []

        self._cycle_total_s = sum(s.duration_s for s in self._segments) or 0.0
        self._start_t: Optional[float] = None  # ros time at first fix

        # Subscribe with BEST_EFFORT (matches Gazebo bridge sensor QoS).
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        # Publish with RELIABLE so production navsat_to_absolute_pose_node
        # (which subscribes with RELIABLE QoS, matching the real GPS driver
        # behaviour) actually receives our messages. With a BEST_EFFORT
        # publisher the production node ignored every fix.
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._pub = self.create_publisher(
            NavSatFix, self._output_topic, pub_qos
        )
        self._sub = self.create_subscription(
            NavSatFix, self._input_topic, self._on_fix, sub_qos
        )

        # Diagnostics — surface the regime distribution observed.
        self._regime_counts: dict[str, int] = {k: 0 for k in QUALITY_REGIMES}
        self.create_timer(15.0, self._log_stats)

        if self._segments:
            pattern_str = " -> ".join(
                f"{s.duration_s:.0f}s {s.regime}" for s in self._segments
            )
            self.get_logger().info(
                "sim_navsat_rtk_fix ready: %s -> %s; cycle (%.0f s): %s"
                % (
                    self._input_topic,
                    self._output_topic,
                    self._cycle_total_s,
                    pattern_str,
                )
            )
        else:
            self.get_logger().info(
                "sim_navsat_rtk_fix ready: %s -> %s; always RTK_FIXED"
                % (self._input_topic, self._output_topic)
            )

    # ------------------------------------------------------------------
    # Quality selection
    # ------------------------------------------------------------------

    def _current_regime(self, ros_now_s: float) -> str:
        if not self._segments:
            return "RTK_FIXED"
        if self._start_t is None:
            self._start_t = ros_now_s
        elapsed = (ros_now_s - self._start_t) % self._cycle_total_s
        cursor = 0.0
        for seg in self._segments:
            cursor += seg.duration_s
            if elapsed < cursor:
                return seg.regime
        # Floating-point edge: return last segment.
        return self._segments[-1].regime

    @staticmethod
    def _cov(sigma_xy: float, sigma_z: float) -> List[float]:
        var_xy = sigma_xy * sigma_xy
        var_z = sigma_z * sigma_z
        return [
            var_xy, 0.0,    0.0,
            0.0,    var_xy, 0.0,
            0.0,    0.0,    var_z,
        ]

    # ------------------------------------------------------------------
    # Callback
    # ------------------------------------------------------------------

    def _on_fix(self, msg: NavSatFix) -> None:
        ros_now_s = self.get_clock().now().nanoseconds * 1e-9
        regime = self._current_regime(ros_now_s)
        status_code, sigma_xy, sigma_z = QUALITY_REGIMES[regime]
        self._regime_counts[regime] += 1

        lat = msg.latitude
        lon = msg.longitude
        # Add Gaussian position noise above the RTK-Fixed bedrock.
        if sigma_xy > 0.01:
            # 1 deg latitude  ~= 111 320 m  =>  m_per_deg_lat = 111320
            # 1 deg longitude ~= 111 320 * cos(lat)
            m_per_deg_lat = 111320.0
            cos_lat = math.cos(math.radians(lat))
            m_per_deg_lon = m_per_deg_lat * max(cos_lat, 1e-6)
            lat += self._rng.gauss(0.0, sigma_xy) / m_per_deg_lat
            lon += self._rng.gauss(0.0, sigma_xy) / m_per_deg_lon

        out = NavSatFix()
        out.header = msg.header
        out.status.status = status_code
        out.status.service = msg.status.service or NavSatStatus.SERVICE_GPS
        out.latitude = lat
        out.longitude = lon
        out.altitude = msg.altitude
        out.position_covariance = self._cov(sigma_xy, sigma_z)
        out.position_covariance_type = (
            NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        )
        self._pub.publish(out)

    def _log_stats(self) -> None:
        total = sum(self._regime_counts.values())
        if total == 0:
            return
        parts = ", ".join(
            f"{k}={v} ({100*v/total:.0f}%)"
            for k, v in self._regime_counts.items()
        )
        self.get_logger().info("regime distribution: " + parts)
        for k in self._regime_counts:
            self._regime_counts[k] = 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimNavSatRtkFix()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
