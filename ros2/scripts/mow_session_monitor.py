#!/usr/bin/env python3
# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later

"""
mow_session_monitor.py — Record a robot-state timeline during a mowing
session, so different tuning runs can be replayed and compared offline.

Subscribes to every relevant source of pose, orientation, sensor health,
and navigation state. Writes one JSON object per sample (JSONL format)
at a fixed rate (default 10 Hz) to
    docker/logs/mow_sessions/<session_name>.jsonl
plus a metadata header (first line) with branch / commit / image tags
so sessions from different builds are distinguishable, and a summary
record (last line) on shutdown.

Designed to run alongside the live stack — it only subscribes, never
publishes velocity commands or changes BT state.

Usage:
    python3 mow_session_monitor.py --session my-first-run
    python3 mow_session_monitor.py --session tuning-v2 --rate 20 \
        --output-dir /home/ubuntu/mowglinext/docker/logs/mow_sessions

    # via docker exec:
    docker exec mowgli-ros2 bash -c 'source /opt/ros/kilted/setup.bash && \
        source /ros2_ws/install/setup.bash && \
        python3 /host/scripts/mow_session_monitor.py --session NAME'

Exit with Ctrl-C; a summary record is written to the same file.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import math
import os
import signal
import socket
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener, TransformException

from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import BatteryState, Imu, LaserScan, NavSatFix


# -----------------------------------------------------------------------------
# Utility helpers
# -----------------------------------------------------------------------------


def _ros_stamp_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _quat_to_yaw(qz: float, qw: float) -> float:
    """Return yaw in radians from the z/w components of a Z-axis quaternion."""
    return 2.0 * math.atan2(qz, qw)


def _wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _safe_float(v: str) -> Optional[float]:
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def _safe_int(v: str) -> Optional[int]:
    try:
        return int(v)
    except (TypeError, ValueError):
        return None


# -----------------------------------------------------------------------------
# QoS profiles
# -----------------------------------------------------------------------------

QOS_SENSOR = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

QOS_RELIABLE = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


# -----------------------------------------------------------------------------
# RTK-covariance-drop check
#
# With RTK Fixed (σ~3 mm raw GPS), every GPS update accepted by ekf_map_node
# should pull /odometry/filtered_map's xx/yy covariance down to roughly fix
# precision within one publish tick (~33 ms at 30 Hz).
#
# If RTK Fixed is streaming at 5 Hz but /odometry/filtered_map covariance stays
# loose (σ > ~5 cm), something is rejecting or diluting the fix (covariance
# mismatch, navsat_transform datum drift, or the EKF saturating process noise).
#
# Thresholds below are tuned for RTK Fixed specifically. Looser fix types
# (Float / DGPS) are not checked — their larger covariance is expected.
# -----------------------------------------------------------------------------

# Consider a raw GPS fix "RTK Fixed-class" when σ_xy below this.
# RTK Fixed typically reports 3-10 mm → 1e-4 m²; pad generously.
RTK_FIXED_GPS_COV_THRESHOLD = 1.0e-3      # var (σ ≤ ~3.2 cm)

# Expect /odometry/filtered_map σ_xy to sit under this shortly after RTK Fixed.
FUSION_COV_TARGET = 1.0e-3                # var (σ ≤ ~3.2 cm)

# Window after an RTK-Fixed GPS arrival within which fusion cov must drop.
# One GPS tick (200 ms @ 5 Hz) + a few fusion ticks (20 ms each) is plenty.
RTK_COV_WINDOW_SEC = 0.30


# -----------------------------------------------------------------------------
# Sample record — each subscription updates a slot; the periodic timer
# snapshots the whole state into a JSON line.
# -----------------------------------------------------------------------------


@dataclass
class LatestState:
    # --- Fused pose (ekf_map_node output) ---
    fusion_x: Optional[float] = None
    fusion_y: Optional[float] = None
    fusion_z: Optional[float] = None
    fusion_yaw_rad: Optional[float] = None
    fusion_vx: Optional[float] = None
    fusion_vy: Optional[float] = None
    fusion_wz: Optional[float] = None
    fusion_cov_xx: Optional[float] = None
    fusion_cov_yy: Optional[float] = None

    # --- Wheel (hardware_bridge) ---
    wheel_vx: Optional[float] = None
    wheel_wz: Optional[float] = None
    wheel_vx_var: Optional[float] = None
    wheel_wz_var: Optional[float] = None

    # --- IMU ---
    imu_gyro_x: Optional[float] = None
    imu_gyro_y: Optional[float] = None
    imu_gyro_z: Optional[float] = None
    imu_accel_x: Optional[float] = None
    imu_accel_y: Optional[float] = None
    imu_accel_z: Optional[float] = None

    # --- GPS ---
    gps_lat: Optional[float] = None
    gps_lon: Optional[float] = None
    gps_alt: Optional[float] = None
    gps_status: Optional[int] = None
    gps_service: Optional[int] = None
    gps_cov_xx: Optional[float] = None
    gps_cov_yy: Optional[float] = None
    gps_cov_type: Optional[int] = None
    # ENU-projected position (from /gps/absolute_pose if present)
    gps_abs_x: Optional[float] = None
    gps_abs_y: Optional[float] = None

    # --- Dock heading (when charging) ---
    gnss_heading_yaw_rad: Optional[float] = None

    # --- Yaw-source attribution (diagnose 180° flip / lever-arm jumps) ---
    # COG yaw = GPS course-over-ground (cog_to_imu_node) — the physical
    # travel direction. fg_yaw = fusion_graph's own published yaw. The
    # gap between cog_yaw, fg_yaw, and fusion_yaw_rad localises a yaw
    # corruption to the COG unary vs gyro integration vs the publish path.
    cog_yaw_rad: Optional[float] = None
    cog_yaw_var: Optional[float] = None
    fg_yaw_rad: Optional[float] = None
    # fusion_graph diagnostics: yaw marginal cov, online gyro bias, and the
    # cumulative wrong-fix reject counter (diffed across samples for a rate).
    fg_cov_yawyaw: Optional[float] = None
    fg_gyro_bias_z: Optional[float] = None
    fg_gps_rejects_wrongfix: Optional[int] = None
    fg_cog_flip_recoveries: Optional[int] = None

    # --- BT / status ---
    bt_state: Optional[int] = None
    bt_state_name: Optional[str] = None
    bt_area: Optional[int] = None
    bt_strip: Optional[int] = None
    hw_current_mode: Optional[int] = None
    is_charging: Optional[bool] = None
    emergency_active: Optional[bool] = None
    emergency_latched: Optional[bool] = None
    battery_voltage: Optional[float] = None
    battery_percentage: Optional[float] = None

    # --- Commands ---
    cmd_vel_nav_vx: Optional[float] = None
    cmd_vel_nav_wz: Optional[float] = None
    cmd_vel_final_vx: Optional[float] = None  # what actually reaches motors
    cmd_vel_final_wz: Optional[float] = None

    # --- Plan ---
    plan_n_poses: Optional[int] = None
    plan_next_x: Optional[float] = None
    plan_next_y: Optional[float] = None
    plan_goal_x: Optional[float] = None
    plan_goal_y: Optional[float] = None

    # --- LiDAR ---
    scan_n_points: Optional[int] = None
    scan_min_range: Optional[float] = None

    # --- slam_toolbox RTK fallback ---
    # Last /slam/pose_cov message — already in the GPS map frame
    # (slam_pose_anchor_node publishes it composed through the EWMA-anchored
    # map -> map_slam transform).
    slam_pose_x: Optional[float] = None
    slam_pose_y: Optional[float] = None
    slam_pose_yaw_rad: Optional[float] = None
    slam_pose_cov_xx: Optional[float] = None
    slam_pose_cov_yy: Optional[float] = None
    slam_pose_msg_t: Optional[float] = None

    # --- Integrators for session totals / drift ---
    gyro_yaw_integrated_rad: float = 0.0
    wheel_yaw_integrated_rad: float = 0.0
    wheel_distance_integrated_m: float = 0.0
    _last_gyro_time: Optional[float] = None
    _last_wheel_time: Optional[float] = None


# -----------------------------------------------------------------------------
# The node
# -----------------------------------------------------------------------------


class MowSessionMonitor(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("mow_session_monitor")

        self.args = args
        self.state = LatestState()
        self.state_lock = threading.Lock()

        # --- output file ---
        os.makedirs(args.output_dir, exist_ok=True)
        output_path = os.path.join(args.output_dir, f"{args.session}.jsonl")
        self.output_path = output_path
        self.file = open(output_path, "w", buffering=1)  # line-buffered
        self.sample_count = 0
        self.started_at = time.time()
        self.start_fusion_xy: Optional[tuple[float, float]] = None
        self.peak_fusion_gps_err = 0.0
        self.peak_wheel_gyro_yaw_drift = 0.0
        self.peak_cog_fusion_yaw_gap = 0.0

        # --- RTK covariance-drop check state ---
        # See the comment block above RTK_FIXED_GPS_COV_THRESHOLD for why this
        # matters. The check is armed on every RTK-Fixed GPS arrival and fires
        # on the next /odometry/filtered_map sample within RTK_COV_WINDOW_SEC:
        # if fusion cov hasn't dropped to FUSION_COV_TARGET, count it as a
        # suspected rejection/dilution. A non-zero count at end of session is
        # a red flag worth correlating with the EKF tuning.
        self._last_rtk_fixed_at: Optional[float] = None   # wallclock seconds
        self._last_rtk_fixed_cov_xx: Optional[float] = None
        self.rtk_fixed_arrivals = 0
        self.rtk_cov_checks_ok = 0
        self.rtk_cov_checks_violations = 0
        self.rtk_cov_check_pending = False       # true between arrival and first fusion tick
        self.peak_fusion_cov_post_rtk = 0.0      # worst cov seen during a pending check

        # --- write metadata header ---
        self._write_metadata()

        # --- subscriptions ---
        cb = ReentrantCallbackGroup()

        def sub(topic: str, msg_type, callback, qos=QOS_RELIABLE):
            self.create_subscription(msg_type, topic, callback, qos, callback_group=cb)

        # Fused pose output (ekf_map_node / map frame).
        sub("/odometry/filtered_map", Odometry, self._fusion_odom_cb, QOS_RELIABLE)

        # Wheels + IMU (hardware_bridge publishes RELIABLE; IMU is sensor QoS from
        # some sources, so accept both with best-effort as a fallback subscriber)
        sub("/wheel_odom", Odometry, self._wheel_odom_cb, QOS_RELIABLE)
        sub("/imu/data", Imu, self._imu_cb, QOS_SENSOR)

        # GPS + dock heading
        sub("/gps/fix", NavSatFix, self._gps_fix_cb, QOS_SENSOR)
        sub("/gps/absolute_pose", PoseWithCovarianceStamped, self._gps_abs_cb, QOS_RELIABLE)
        sub("/gnss/heading", Imu, self._gnss_heading_cb, QOS_RELIABLE)

        # Yaw-source attribution: COG (GPS travel direction), fusion_graph's
        # own yaw, and its diagnostics. cog_to_imu / fusion publish these
        # BEST_EFFORT, so subscribe with sensor QoS.
        sub("/imu/cog_heading", Imu, self._cog_heading_cb, QOS_SENSOR)
        sub("/imu/fg_yaw", Imu, self._fg_yaw_cb, QOS_SENSOR)
        try:
            from diagnostic_msgs.msg import DiagnosticArray  # type: ignore
            sub("/fusion_graph/diagnostics", DiagnosticArray, self._fg_diag_cb, QOS_RELIABLE)
        except ImportError as exc:
            self.get_logger().warn(f"diagnostic_msgs not available: {exc} — fg diagnostics will be missing.")

        # BT + hardware state — imported lazily so we only require the
        # mowgli_interfaces package when those topics are available
        try:
            from mowgli_interfaces.msg import (  # type: ignore
                Emergency,
                HighLevelStatus,
                Status as HwStatus,
            )
            sub("/behavior_tree_node/high_level_status", HighLevelStatus, self._bt_cb)
            sub("/hardware_bridge/status", HwStatus, self._hw_status_cb)
            sub("/hardware_bridge/emergency", Emergency, self._emergency_cb)
        except ImportError as exc:
            self.get_logger().warn(f"mowgli_interfaces not available: {exc} — BT/status fields will be missing.")

        sub("/battery_state", BatteryState, self._battery_cb, QOS_RELIABLE)

        # Commands — both what Nav2 emits AND what reaches the motors, so we
        # can see the effect of collision_monitor / velocity_smoother.
        sub("/cmd_vel_nav", TwistStamped, self._cmd_vel_nav_cb, QOS_RELIABLE)
        sub("/cmd_vel", TwistStamped, self._cmd_vel_final_cb, QOS_RELIABLE)

        # Plan + LiDAR
        sub("/plan", Path, self._plan_cb, QOS_RELIABLE)
        sub("/scan", LaserScan, self._scan_cb, QOS_SENSOR)

        # NOTE: the legacy /slam/pose_cov subscription was removed — slam_toolbox
        # and slam_pose_anchor_node are gone (fusion_graph is the sole localizer
        # and there is no publisher on that topic). The map-frame estimate is
        # captured from /odometry/filtered_map instead.

        # --- TF buffer for map lookups ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- periodic sample timer ---
        period = 1.0 / float(args.rate)
        self.timer = self.create_timer(period, self._tick, callback_group=cb)

        self.get_logger().info(
            f"Monitor started — writing {output_path} at {args.rate} Hz"
        )

    # ------------------------------------------------------------------
    # Subscription callbacks (each just copies fields under the state lock)
    # ------------------------------------------------------------------

    def _fusion_odom_cb(self, msg: Odometry) -> None:
        with self.state_lock:
            s = self.state
            s.fusion_x = msg.pose.pose.position.x
            s.fusion_y = msg.pose.pose.position.y
            s.fusion_z = msg.pose.pose.position.z
            s.fusion_yaw_rad = _quat_to_yaw(
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
            )
            s.fusion_vx = msg.twist.twist.linear.x
            s.fusion_vy = msg.twist.twist.linear.y
            s.fusion_wz = msg.twist.twist.angular.z
            # Position covariance diagonal: 6×6 row-major, xx=0, yy=7.
            s.fusion_cov_xx = float(msg.pose.covariance[0])
            s.fusion_cov_yy = float(msg.pose.covariance[7])
            if self.start_fusion_xy is None and s.fusion_x is not None:
                self.start_fusion_xy = (s.fusion_x, s.fusion_y)

            # --- RTK covariance-drop check ---
            # If an RTK-Fixed GPS arrived recently, this fusion tick should
            # show σ_xy ≤ FUSION_COV_TARGET; otherwise the UKF rejected it
            # (Mahalanobis outlier gate at outlier_threshold_gnss=16.27).
            if (
                self.rtk_cov_check_pending
                and self._last_rtk_fixed_at is not None
            ):
                elapsed = time.time() - self._last_rtk_fixed_at
                cov_max = max(s.fusion_cov_xx or 0.0, s.fusion_cov_yy or 0.0)
                if elapsed > RTK_COV_WINDOW_SEC:
                    # Window elapsed without cov dropping below target: count
                    # as violation and disarm the check until next RTK fix.
                    self.rtk_cov_checks_violations += 1
                    self.rtk_cov_check_pending = False
                elif cov_max <= FUSION_COV_TARGET:
                    # Cov dropped inside the window — healthy, disarm.
                    self.rtk_cov_checks_ok += 1
                    self.rtk_cov_check_pending = False
                else:
                    # Still pending within window — track worst cov seen.
                    self.peak_fusion_cov_post_rtk = max(
                        self.peak_fusion_cov_post_rtk, cov_max
                    )

    def _wheel_odom_cb(self, msg: Odometry) -> None:
        with self.state_lock:
            s = self.state
            s.wheel_vx = msg.twist.twist.linear.x
            s.wheel_wz = msg.twist.twist.angular.z
            s.wheel_vx_var = float(msg.twist.covariance[0])
            s.wheel_wz_var = float(msg.twist.covariance[35])
            # integrate yaw + distance for session totals
            t = _ros_stamp_sec(msg.header.stamp)
            if s._last_wheel_time is not None:
                dt = t - s._last_wheel_time
                if 0.0 < dt < 1.0:
                    s.wheel_yaw_integrated_rad += (s.wheel_wz or 0.0) * dt
                    s.wheel_distance_integrated_m += abs((s.wheel_vx or 0.0) * dt)
            s._last_wheel_time = t

    def _imu_cb(self, msg: Imu) -> None:
        with self.state_lock:
            s = self.state
            s.imu_gyro_x = msg.angular_velocity.x
            s.imu_gyro_y = msg.angular_velocity.y
            s.imu_gyro_z = msg.angular_velocity.z
            s.imu_accel_x = msg.linear_acceleration.x
            s.imu_accel_y = msg.linear_acceleration.y
            s.imu_accel_z = msg.linear_acceleration.z
            t = _ros_stamp_sec(msg.header.stamp)
            if s._last_gyro_time is not None:
                dt = t - s._last_gyro_time
                if 0.0 < dt < 1.0:
                    s.gyro_yaw_integrated_rad += (s.imu_gyro_z or 0.0) * dt
            s._last_gyro_time = t

    def _gps_fix_cb(self, msg: NavSatFix) -> None:
        with self.state_lock:
            s = self.state
            s.gps_lat = msg.latitude
            s.gps_lon = msg.longitude
            s.gps_alt = msg.altitude
            s.gps_status = msg.status.status
            s.gps_service = msg.status.service
            s.gps_cov_xx = float(msg.position_covariance[0])
            s.gps_cov_yy = float(msg.position_covariance[4])
            s.gps_cov_type = msg.position_covariance_type

            # --- RTK-Fixed detection: arm the cov-drop check ---
            # u-blox drivers usually set status.status=2 (GBAS/RTK) for RTK
            # Fixed, but that isn't guaranteed — use the covariance itself
            # as a fallback: RTK Fixed reports σ ≤ a few mm (var ≤ 1e-3).
            # Both paths arm the check; false positives are harmless because
            # the fusion cov floor for any Float/DGPS fix is looser anyway.
            is_rtk_fixed = (
                msg.status.status == 2
                or (
                    s.gps_cov_xx is not None
                    and s.gps_cov_xx > 0.0
                    and s.gps_cov_xx <= RTK_FIXED_GPS_COV_THRESHOLD
                )
            )
            if is_rtk_fixed:
                # If a previous check is still pending, the next fusion tick
                # will see a newer arrival anyway — just overwrite the arm.
                self._last_rtk_fixed_at = time.time()
                self._last_rtk_fixed_cov_xx = s.gps_cov_xx
                self.rtk_cov_check_pending = True
                self.rtk_fixed_arrivals += 1

    def _gps_abs_cb(self, msg: PoseWithCovarianceStamped) -> None:
        with self.state_lock:
            s = self.state
            s.gps_abs_x = msg.pose.pose.position.x
            s.gps_abs_y = msg.pose.pose.position.y

    def _gnss_heading_cb(self, msg: Imu) -> None:
        with self.state_lock:
            self.state.gnss_heading_yaw_rad = _quat_to_yaw(
                msg.orientation.z, msg.orientation.w
            )

    def _cog_heading_cb(self, msg: Imu) -> None:
        with self.state_lock:
            self.state.cog_yaw_rad = _quat_to_yaw(
                msg.orientation.z, msg.orientation.w
            )
            var = msg.orientation_covariance[8]
            self.state.cog_yaw_var = float(var) if var and var > 0.0 else None

    def _fg_yaw_cb(self, msg: Imu) -> None:
        with self.state_lock:
            self.state.fg_yaw_rad = _quat_to_yaw(
                msg.orientation.z, msg.orientation.w
            )

    def _fg_diag_cb(self, msg) -> None:
        # DiagnosticArray with key/value pairs published by fusion_graph_node
        # (cov_yawyaw, cog_flip_recoveries, gps_rejects_wrongfix, ...).
        with self.state_lock:
            for status in msg.status:
                for kv in status.values:
                    if kv.key == "cov_yawyaw":
                        self.state.fg_cov_yawyaw = _safe_float(kv.value)
                    elif kv.key == "gyro_bias_z_rad_per_s":
                        self.state.fg_gyro_bias_z = _safe_float(kv.value)
                    elif kv.key == "gps_rejects_wrongfix":
                        self.state.fg_gps_rejects_wrongfix = _safe_int(kv.value)
                    elif kv.key == "cog_flip_recoveries":
                        self.state.fg_cog_flip_recoveries = _safe_int(kv.value)

    def _bt_cb(self, msg) -> None:
        with self.state_lock:
            self.state.bt_state = int(msg.state)
            self.state.bt_state_name = str(msg.state_name)
            self.state.bt_area = int(getattr(msg, "current_area", -1))
            self.state.bt_strip = int(getattr(msg, "current_path", -1))

    def _hw_status_cb(self, msg) -> None:
        with self.state_lock:
            self.state.hw_current_mode = int(getattr(msg, "current_mode", 0))
            self.state.is_charging = bool(msg.is_charging)

    def _emergency_cb(self, msg) -> None:
        with self.state_lock:
            self.state.emergency_active = bool(msg.active_emergency)
            self.state.emergency_latched = bool(msg.latched_emergency)

    def _battery_cb(self, msg: BatteryState) -> None:
        with self.state_lock:
            self.state.battery_voltage = float(msg.voltage)
            self.state.battery_percentage = float(msg.percentage)

    def _cmd_vel_nav_cb(self, msg: TwistStamped) -> None:
        with self.state_lock:
            self.state.cmd_vel_nav_vx = msg.twist.linear.x
            self.state.cmd_vel_nav_wz = msg.twist.angular.z

    def _cmd_vel_final_cb(self, msg: TwistStamped) -> None:
        with self.state_lock:
            self.state.cmd_vel_final_vx = msg.twist.linear.x
            self.state.cmd_vel_final_wz = msg.twist.angular.z

    def _plan_cb(self, msg: Path) -> None:
        with self.state_lock:
            s = self.state
            s.plan_n_poses = len(msg.poses)
            if msg.poses:
                s.plan_next_x = msg.poses[0].pose.position.x
                s.plan_next_y = msg.poses[0].pose.position.y
                s.plan_goal_x = msg.poses[-1].pose.position.x
                s.plan_goal_y = msg.poses[-1].pose.position.y

    def _scan_cb(self, msg: LaserScan) -> None:
        with self.state_lock:
            s = self.state
            valid = [r for r in msg.ranges if math.isfinite(r) and r > 0.0]
            s.scan_n_points = len(valid)
            s.scan_min_range = min(valid) if valid else None

    def _slam_pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        with self.state_lock:
            s = self.state
            s.slam_pose_x = msg.pose.pose.position.x
            s.slam_pose_y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            s.slam_pose_yaw_rad = _quat_to_yaw(q.z, q.w)
            cov = msg.pose.covariance
            # 6x6 row-major: [0]=xx, [7]=yy
            s.slam_pose_cov_xx = float(cov[0])
            s.slam_pose_cov_yy = float(cov[7])
            s.slam_pose_msg_t = _ros_stamp_sec(msg.header.stamp)

    # ------------------------------------------------------------------
    # Periodic snapshot
    # ------------------------------------------------------------------

    def _tf_lookup(self, parent: str, child: str) -> Optional[dict]:
        try:
            tf = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
            q = tf.transform.rotation
            yaw = _quat_to_yaw(q.z, q.w)
            return {
                "x": tf.transform.translation.x,
                "y": tf.transform.translation.y,
                "z": tf.transform.translation.z,
                "yaw_deg": math.degrees(yaw),
            }
        except TransformException:
            return None

    def _tick(self) -> None:
        now_unix = time.time()
        now_ros = self.get_clock().now().nanoseconds / 1e9
        with self.state_lock:
            s = self.state

            # --- TF snapshots (cartographer + fusion composition) ---
            carto = self._tf_lookup("map", "base_footprint")
            m2o = self._tf_lookup("map", "odom")
            o2bf = self._tf_lookup("odom", "base_footprint")

            # --- Cross-source consistency ---
            fusion_gps_dist = None
            if s.fusion_x is not None and s.gps_abs_x is not None:
                fusion_gps_dist = math.hypot(
                    s.fusion_x - s.gps_abs_x, s.fusion_y - s.gps_abs_y
                )
                self.peak_fusion_gps_err = max(
                    self.peak_fusion_gps_err, fusion_gps_dist
                )

            fusion_carto_dist = None
            fusion_carto_yaw_diff = None
            if carto is not None and s.fusion_yaw_rad is not None:
                # Cartographer's map→base_footprint vs the EKF's map-frame yaw.
                # We fuse /odometry/filtered_map (already in map frame), so no
                # composition with map→odom is needed here.
                if m2o is not None:
                    f_yaw_in_map_deg = (
                        m2o["yaw_deg"] + math.degrees(s.fusion_yaw_rad)
                    )
                    fusion_carto_yaw_diff = _wrap_pi(
                        math.radians(carto["yaw_deg"] - f_yaw_in_map_deg)
                    )
                    fusion_carto_yaw_diff = math.degrees(fusion_carto_yaw_diff)

            wheel_gyro_drift = None
            if s.wheel_yaw_integrated_rad != 0.0 or s.gyro_yaw_integrated_rad != 0.0:
                wheel_gyro_drift = math.degrees(
                    s.wheel_yaw_integrated_rad - s.gyro_yaw_integrated_rad
                )
                self.peak_wheel_gyro_yaw_drift = max(
                    self.peak_wheel_gyro_yaw_drift, abs(wheel_gyro_drift)
                )

            # --- Yaw-source disagreement (180° flip / lever-arm corruption) ---
            # cog_minus_fusion ≈ ±180° is the signature of the East→turn fault:
            # the fused yaw is flipped vs the GPS travel direction, so the
            # gps_x lever-arm projects position to the wrong side.
            cog_minus_fusion_deg = None
            if s.cog_yaw_rad is not None and s.fusion_yaw_rad is not None:
                cog_minus_fusion_deg = math.degrees(
                    _wrap_pi(s.cog_yaw_rad - s.fusion_yaw_rad)
                )
                self.peak_cog_fusion_yaw_gap = max(
                    self.peak_cog_fusion_yaw_gap, abs(cog_minus_fusion_deg)
                )
            fg_minus_fusion_deg = None
            if s.fg_yaw_rad is not None and s.fusion_yaw_rad is not None:
                fg_minus_fusion_deg = math.degrees(
                    _wrap_pi(s.fg_yaw_rad - s.fusion_yaw_rad)
                )

            # --- Distance to plan goal ---
            dist_to_goal = None
            if (
                s.fusion_x is not None
                and s.plan_goal_x is not None
            ):
                dist_to_goal = math.hypot(
                    s.fusion_x - s.plan_goal_x, s.fusion_y - s.plan_goal_y
                )

            record: dict[str, Any] = {
                "type": "sample",
                "t": now_unix,
                "t_ros": now_ros,
                "session_elapsed_sec": now_unix - self.started_at,
                "fusion": {
                    "x": s.fusion_x, "y": s.fusion_y, "z": s.fusion_z,
                    "yaw_deg": math.degrees(s.fusion_yaw_rad) if s.fusion_yaw_rad is not None else None,
                    "vx": s.fusion_vx, "vy": s.fusion_vy, "wz": s.fusion_wz,
                    "cov_xx": s.fusion_cov_xx,
                    "cov_yy": s.fusion_cov_yy,
                    "sigma_xy_m": (
                        math.sqrt(max((s.fusion_cov_xx or 0.0)
                                      + (s.fusion_cov_yy or 0.0), 0.0) * 0.5)
                        if s.fusion_cov_xx is not None and s.fusion_cov_yy is not None
                        else None
                    ),
                },
                "cartographer_map_base": carto,        # map → base_footprint
                "map_to_odom": m2o,                    # Cartographer correction
                "odom_to_base": o2bf,                  # ekf_odom_node output as TF
                "wheel": {
                    "vx": s.wheel_vx, "wz": s.wheel_wz,
                    "var_vx": s.wheel_vx_var, "var_wz": s.wheel_wz_var,
                    "distance_m_integrated": s.wheel_distance_integrated_m,
                    "yaw_integrated_deg": math.degrees(s.wheel_yaw_integrated_rad),
                },
                "imu": {
                    "gyro": [s.imu_gyro_x, s.imu_gyro_y, s.imu_gyro_z],
                    "accel": [s.imu_accel_x, s.imu_accel_y, s.imu_accel_z],
                    "gyro_yaw_integrated_deg": math.degrees(s.gyro_yaw_integrated_rad),
                },
                "gps": {
                    "lat": s.gps_lat, "lon": s.gps_lon, "alt": s.gps_alt,
                    "status": s.gps_status, "service": s.gps_service,
                    "cov_xx": s.gps_cov_xx, "cov_yy": s.gps_cov_yy,
                    "cov_type": s.gps_cov_type,
                    "sigma_xy_mm": (
                        math.sqrt((s.gps_cov_xx + s.gps_cov_yy) * 0.5) * 1000.0
                        if s.gps_cov_xx is not None and s.gps_cov_yy is not None
                        else None
                    ),
                },
                "gps_absolute_pose": {
                    "x": s.gps_abs_x, "y": s.gps_abs_y,
                },
                "yaw_sources": {
                    # COG = GPS travel direction (cog_to_imu); fg = fusion_graph's
                    # published yaw; fusion = /odometry/filtered_map yaw.
                    "cog_deg": math.degrees(s.cog_yaw_rad) if s.cog_yaw_rad is not None else None,
                    "cog_var": s.cog_yaw_var,
                    "fg_deg": math.degrees(s.fg_yaw_rad) if s.fg_yaw_rad is not None else None,
                    "cog_minus_fusion_deg": cog_minus_fusion_deg,
                    "fg_minus_fusion_deg": fg_minus_fusion_deg,
                    "cov_yawyaw": s.fg_cov_yawyaw,
                    "gyro_bias_z": s.fg_gyro_bias_z,
                    "gps_rejects_wrongfix": s.fg_gps_rejects_wrongfix,
                    "cog_flip_recoveries": s.fg_cog_flip_recoveries,
                },
                "slam": {
                    # /slam/pose_cov, already in GPS map frame
                    "pose": {
                        "x": s.slam_pose_x,
                        "y": s.slam_pose_y,
                        "yaw_deg": (
                            math.degrees(s.slam_pose_yaw_rad)
                            if s.slam_pose_yaw_rad is not None
                            else None
                        ),
                        "cov_xx": s.slam_pose_cov_xx,
                        "cov_yy": s.slam_pose_cov_yy,
                        "sigma_xy_m": (
                            math.sqrt(
                                max(
                                    (s.slam_pose_cov_xx or 0.0)
                                    + (s.slam_pose_cov_yy or 0.0),
                                    0.0,
                                )
                                * 0.5
                            )
                            if s.slam_pose_cov_xx is not None
                            and s.slam_pose_cov_yy is not None
                            else None
                        ),
                        "msg_age_sec": (
                            now_ros - s.slam_pose_msg_t
                            if s.slam_pose_msg_t is not None
                            else None
                        ),
                    },
                    # TF: map -> map_slam (the EWMA anchor). When stale,
                    # /slam/pose_cov.sigma_xy_m grows to reflect drift
                    # since the last RTK-Fixed update.
                    "anchor": self._tf_lookup("map", "map_slam"),
                    # Cross-source sanity check: slam pose vs fusion pose,
                    # in the same gps map frame. Should track tightly when
                    # RTK is healthy; divergence under RTK-Float is the
                    # signal we want to measure.
                    "fusion_slam_dist_m": (
                        math.hypot(
                            s.fusion_x - s.slam_pose_x,
                            s.fusion_y - s.slam_pose_y,
                        )
                        if s.fusion_x is not None and s.slam_pose_x is not None
                        else None
                    ),
                    "fusion_slam_yaw_diff_deg": (
                        math.degrees(
                            _wrap_pi(s.fusion_yaw_rad - s.slam_pose_yaw_rad)
                        )
                        if s.fusion_yaw_rad is not None
                        and s.slam_pose_yaw_rad is not None
                        else None
                    ),
                },
                "gnss_heading_yaw_deg": (
                    math.degrees(s.gnss_heading_yaw_rad)
                    if s.gnss_heading_yaw_rad is not None
                    else None
                ),
                "bt": {
                    "state": s.bt_state,
                    "state_name": s.bt_state_name,
                    "area": s.bt_area,
                    "strip": s.bt_strip,
                },
                "hardware": {
                    "is_charging": s.is_charging,
                    "current_mode": s.hw_current_mode,
                    "emergency_active": s.emergency_active,
                    "emergency_latched": s.emergency_latched,
                    "battery_voltage": s.battery_voltage,
                    "battery_pct": s.battery_percentage,
                },
                "cmd_vel": {
                    "nav": {"vx": s.cmd_vel_nav_vx, "wz": s.cmd_vel_nav_wz},
                    "final": {"vx": s.cmd_vel_final_vx, "wz": s.cmd_vel_final_wz},
                },
                "plan": {
                    "n_poses": s.plan_n_poses,
                    "next": {"x": s.plan_next_x, "y": s.plan_next_y},
                    "goal": {"x": s.plan_goal_x, "y": s.plan_goal_y},
                    "distance_to_goal_m": dist_to_goal,
                },
                "scan": {
                    "n_points": s.scan_n_points,
                    "min_range_m": s.scan_min_range,
                },
                "cross_checks": {
                    "fusion_gps_dist_m": fusion_gps_dist,
                    "fusion_carto_dist_m": (
                        math.hypot(
                            (carto["x"] if carto else 0.0)
                            - (s.gps_abs_x if s.gps_abs_x is not None else 0.0),
                            (carto["y"] if carto else 0.0)
                            - (s.gps_abs_y if s.gps_abs_y is not None else 0.0),
                        )
                        if carto is not None and s.gps_abs_x is not None
                        else None
                    ),
                    "fusion_carto_yaw_diff_deg": fusion_carto_yaw_diff,
                    "wheel_gyro_yaw_drift_deg": wheel_gyro_drift,
                    # RTK covariance-drop health (see file-top constants).
                    # violations > 0 ⇒ outlier gate rejecting RTK fixes.
                    "rtk_cov_check": {
                        "arrivals": self.rtk_fixed_arrivals,
                        "ok": self.rtk_cov_checks_ok,
                        "violations": self.rtk_cov_checks_violations,
                        "pending": self.rtk_cov_check_pending,
                        "last_rtk_fixed_age_sec": (
                            now_unix - self._last_rtk_fixed_at
                            if self._last_rtk_fixed_at is not None
                            else None
                        ),
                    },
                },
            }

        try:
            self.file.write(json.dumps(record, default=_json_default) + "\n")
            self.sample_count += 1
        except Exception as exc:
            self.get_logger().warn(f"Failed to write sample: {exc}")

    # ------------------------------------------------------------------
    # Metadata + summary
    # ------------------------------------------------------------------

    def _write_metadata(self) -> None:
        header = {
            "type": "metadata",
            "session_name": self.args.session,
            "started_at": dt.datetime.now(dt.timezone.utc).isoformat(),
            "hostname": socket.gethostname(),
            "rate_hz": self.args.rate,
            "git_branch": _git_branch(),
            "git_commit": _git_commit(),
            "git_status_dirty": _git_dirty(),
            "image_tags": _image_tags(),
            "config_hashes": _config_hashes(),
            "cmd_args": vars(self.args),
        }
        self.file.write(json.dumps(header, default=_json_default) + "\n")

    def write_summary_and_close(self) -> None:
        duration = time.time() - self.started_at
        with self.state_lock:
            s = self.state
            total_wheel_dist = s.wheel_distance_integrated_m
            fusion_xy_travel = None
            if self.start_fusion_xy and s.fusion_x is not None:
                fusion_xy_travel = math.hypot(
                    s.fusion_x - self.start_fusion_xy[0],
                    s.fusion_y - self.start_fusion_xy[1],
                )
            total_rtk_checks = self.rtk_cov_checks_ok + self.rtk_cov_checks_violations
            rtk_ok_ratio = (
                self.rtk_cov_checks_ok / total_rtk_checks
                if total_rtk_checks > 0
                else None
            )
            summary = {
                "type": "summary",
                "ended_at": dt.datetime.now(dt.timezone.utc).isoformat(),
                "duration_sec": duration,
                "samples_written": self.sample_count,
                "session_path_m_wheel": total_wheel_dist,
                "session_straight_line_displacement_m": fusion_xy_travel,
                "peak_fusion_gps_error_m": self.peak_fusion_gps_err,
                "peak_wheel_gyro_yaw_drift_deg": self.peak_wheel_gyro_yaw_drift,
                "peak_cog_fusion_yaw_gap_deg": self.peak_cog_fusion_yaw_gap,
                "rtk_cov_check": {
                    "rtk_fixed_arrivals": self.rtk_fixed_arrivals,
                    "ok": self.rtk_cov_checks_ok,
                    "violations": self.rtk_cov_checks_violations,
                    "ok_ratio": rtk_ok_ratio,
                    "peak_fusion_cov_xx_post_rtk": self.peak_fusion_cov_post_rtk,
                    "verdict": _rtk_cov_verdict(
                        self.rtk_fixed_arrivals,
                        self.rtk_cov_checks_violations,
                        total_rtk_checks,
                    ),
                },
                "final_battery_voltage": s.battery_voltage,
                "final_bt_state_name": s.bt_state_name,
            }
        try:
            self.file.write(json.dumps(summary, default=_json_default) + "\n")
        finally:
            self.file.close()
        print(f"[mow_session_monitor] wrote {self.sample_count} samples to {self.output_path}", file=sys.stderr)


# -----------------------------------------------------------------------------
# Helpers outside the node
# -----------------------------------------------------------------------------


def _json_default(o: Any) -> Any:
    if isinstance(o, (set, tuple)):
        return list(o)
    if isinstance(o, (bytes, bytearray)):
        return o.hex()
    return str(o)


def _rtk_cov_verdict(arrivals: int, violations: int, total_checks: int) -> str:
    """One-word verdict for the RTK cov-drop health check, used in the
    summary record so `jq '.rtk_cov_check.verdict'` gives a fast readout.

    - "no_rtk": session never saw an RTK Fixed fix (under cover, RTK down)
    - "healthy": ≥95% of arrivals saw fusion cov drop to target in the
      window — GPS updates are flowing through the outlier gate normally
    - "gate_rejecting": violations ≥ 5% of checks — consistent with the
      UKF rejecting fixes. Correlate with localization.yaml's
      outlier_threshold_gnss (16.27 as of this writing).
    """
    if arrivals == 0:
        return "no_rtk"
    if total_checks == 0:
        return "insufficient_data"
    violation_ratio = violations / total_checks
    if violation_ratio <= 0.05:
        return "healthy"
    if violation_ratio <= 0.20:
        return "intermittent"
    return "gate_rejecting"


def _git(cmd: list[str], cwd: Optional[str] = None) -> Optional[str]:
    try:
        target = cwd or os.path.dirname(os.path.abspath(__file__))
        out = subprocess.check_output(
            ["git", "-C", target, *cmd], stderr=subprocess.DEVNULL
        )
        return out.decode().strip()
    except Exception:
        return None


def _git_branch() -> Optional[str]:
    # Try the script's own location first (works on host), then the
    # well-known host checkout path (works when running from a bind-mount).
    return (
        _git(["rev-parse", "--abbrev-ref", "HEAD"])
        or _git(["rev-parse", "--abbrev-ref", "HEAD"], cwd="/home/ubuntu/mowglinext")
    )


def _git_commit() -> Optional[str]:
    # Inside the container there's no git repo; fall back to the
    # OCI image revision label baked in at build time (set by
    # docker/metadata-action@v5 in ros2-docker.yml) via env var or
    # /proc/1/status inspection isn't reliable. Easiest: the image tag
    # itself encodes the branch, and config_hashes still pin tuning —
    # so a missing commit hash isn't catastrophic.
    host = (
        _git(["rev-parse", "--short", "HEAD"])
        or _git(["rev-parse", "--short", "HEAD"], cwd="/home/ubuntu/mowglinext")
    )
    if host is not None:
        return host
    # Container fallback: git env var set by the Dockerfile (if any) or
    # the image revision label read via a well-known env.
    return os.environ.get("GIT_COMMIT") or os.environ.get("OCI_REVISION")


def _git_dirty() -> Optional[bool]:
    out = (
        _git(["status", "--porcelain"])
        or _git(["status", "--porcelain"], cwd="/home/ubuntu/mowglinext")
    )
    if out is None:
        return None
    return len(out) > 0


def _image_tags() -> dict[str, Optional[str]]:
    """Best-effort lookup of the image tags the running containers were
    pulled from. Tries the host .env, then the container's own image
    revision label (readable via the docker runtime OCI spec file)."""
    tags: dict[str, Optional[str]] = {}
    # Host-side .env
    env_path = "/home/ubuntu/mowglinext/install/.env"
    try:
        with open(env_path) as f:
            for line in f:
                if "_IMAGE=" in line and ":" in line:
                    key, value = line.strip().split("=", 1)
                    tag = value.rsplit(":", 1)[-1]
                    tags[key] = tag
    except Exception:
        pass
    # Container-side fallback: OCI image revision is exposed as env var by
    # some CI systems, and the container's own ROS image tag is hinted at
    # via /.dockerenv + image labels (not always available at runtime).
    # The git_commit field below carries the same info from build labels
    # when accessible, so we don't need to break a sweat here.
    return tags


# Config files that influence runtime behavior. First entry is the host
# path (used when the monitor is run from the host), second is the
# container-internal path (used when run via docker exec). The first
# readable file wins.
_CONFIG_LOCATIONS = {
    "mowgli_robot.yaml": [
        "/home/ubuntu/mowglinext/install/config/mowgli/mowgli_robot.yaml",
        "/ros2_ws/config/mowgli_robot.yaml",
    ],
    "localization.yaml": [
        "/home/ubuntu/mowglinext/ros2/src/mowgli_bringup/config/localization.yaml",
        "/ros2_ws/install/mowgli_bringup/share/mowgli_bringup/config/localization.yaml",
    ],
    # nav2_params.yaml was split into a shared base + lidar/no-lidar overlays
    # (deep-merged at launch); hash all three so a session's effective Nav2
    # tuning is captured regardless of which variant was active.
    "nav2_params_base.yaml": [
        "/home/ubuntu/mowglinext/ros2/src/mowgli_bringup/config/nav2_params_base.yaml",
        "/ros2_ws/install/mowgli_bringup/share/mowgli_bringup/config/nav2_params_base.yaml",
    ],
    "nav2_params_lidar.yaml": [
        "/home/ubuntu/mowglinext/ros2/src/mowgli_bringup/config/nav2_params_lidar.yaml",
        "/ros2_ws/install/mowgli_bringup/share/mowgli_bringup/config/nav2_params_lidar.yaml",
    ],
    "nav2_params_no_lidar.yaml": [
        "/home/ubuntu/mowglinext/ros2/src/mowgli_bringup/config/nav2_params_no_lidar.yaml",
        "/ros2_ws/install/mowgli_bringup/share/mowgli_bringup/config/nav2_params_no_lidar.yaml",
    ],
    # robot_localization.yaml and slam_toolbox.yaml were removed (fusion_graph
    # is the sole localizer) — both files no longer exist, so they are not
    # hashed here. fusion_graph.yaml carries the relevant tuning now.
    "fusion_graph.yaml": [
        "/home/ubuntu/mowglinext/ros2/src/fusion_graph/config/fusion_graph.yaml",
        "/ros2_ws/install/fusion_graph/share/fusion_graph/config/fusion_graph.yaml",
    ],
}


def _config_hashes() -> dict[str, Optional[str]]:
    """SHA-256 of the key config files that influence behavior. Lets us
    group sessions that ran with the same tuning."""
    out: dict[str, Optional[str]] = {}
    for name, candidates in _CONFIG_LOCATIONS.items():
        out[name] = None
        for path in candidates:
            try:
                with open(path, "rb") as f:
                    out[name] = hashlib.sha256(f.read()).hexdigest()[:16]
                break
            except Exception:
                continue
    return out


# -----------------------------------------------------------------------------
# main
# -----------------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--session",
        required=True,
        help="Session name (becomes the JSONL filename)",
    )
    parser.add_argument(
        "--output-dir",
        default="/home/ubuntu/mowglinext/docker/logs/mow_sessions",
        help="Directory where the JSONL file is written",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=10.0,
        help="Sampling rate in Hz (default: 10)",
    )
    args = parser.parse_args()

    rclpy.init()
    node = MowSessionMonitor(args)

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    stop = threading.Event()

    def _on_signal(signum, frame):
        stop.set()

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    try:
        while not stop.is_set():
            executor.spin_once(timeout_sec=0.1)
    finally:
        node.write_summary_and_close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
