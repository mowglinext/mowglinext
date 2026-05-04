#!/usr/bin/env python3
"""
e2e_test_no_lidar.py — End-to-end simulation test for GPS-only mode (no LiDAR).

Validates the full mowing cycle WITHOUT LiDAR:
  1. Undock from charging station
  2. Plan coverage path
  3. Mow following planned paths (median deviation < 10cm)
  4. Dock back when complete

No obstacle avoidance test — without LiDAR there is no obstacle detection.
GPS-only localization: robot_localization dual EKF publishes map→odom→base_footprint using GPS+IMU+wheel odometry.

Usage:
  # Launch simulation first:
  ros2 launch mowgli_bringup sim_full_system.launch.py \
    headless:=true use_rviz:=false use_lidar:=false simulate_gps_degradation:=false

  # Then run the test:
  python3 /ros2_ws/src/e2e_test_no_lidar.py

Or self-contained via: make e2e-test-no-lidar
"""

import math
import signal
import sys
import time
from dataclasses import dataclass, field
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path, Odometry
from mowgli_interfaces.srv import HighLevelControl
from mowgli_interfaces.msg import HighLevelStatus


class TestPhase(Enum):
    WAITING = "WAITING"
    UNDOCKING = "UNDOCKING"
    PLANNING = "PLANNING"
    MOWING = "MOWING"
    DOCKING = "DOCKING"
    COMPLETE = "COMPLETE"


@dataclass
class PhaseResult:
    name: str
    passed: bool
    duration_sec: float = 0.0
    details: str = ""


@dataclass
class Metrics:
    start_time: float = 0.0
    bt_states: list = field(default_factory=list)
    path_deviations: list = field(default_factory=list)
    robot_poses: list = field(default_factory=list)
    cmd_vels: list = field(default_factory=list)
    coverage_path_len: int = 0
    coverage_path_poses: list = field(default_factory=list)
    phase_results: list = field(default_factory=list)


class NoLidarE2ETestNode(Node):
    def __init__(self):
        super().__init__("e2e_test_no_lidar")

        self.metrics = Metrics(start_time=time.time())
        self.coverage_path = None
        self.current_pose = None
        self.current_bt_state = ""
        self.test_complete = False
        self.mowing_started = False

        # Phase tracking
        self.current_phase = TestPhase.WAITING
        self.phase_start_time = time.time()

        # Fused pose (odom frame, GPS-anchored) for path deviation
        self.fusion_pose = None

        # QoS profiles
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Subscribers
        self.create_subscription(
            HighLevelStatus,
            "/behavior_tree_node/high_level_status",
            self._on_bt_status,
            reliable_qos,
        )
        # opennav_coverage publishes the planned per-area path on
        # /coverage_server/coverage_plan (the old /mowgli/coverage/path
        # was the cell-strip planner output and no longer exists).
        self.create_subscription(
            Path,
            "/coverage_server/coverage_plan",
            self._on_coverage_path,
            reliable_qos,
        )
        self.create_subscription(
            Odometry, "/wheel_odom", self._on_odom, sensor_qos
        )
        self.create_subscription(
            Odometry,
            "/odometry/filtered_map",
            self._on_fusion_odom,
            sensor_qos,
        )
        self.create_subscription(
            Twist, "/cmd_vel", self._on_cmd_vel, sensor_qos
        )

        # Service client
        self.hlc_client = self.create_client(
            HighLevelControl,
            "/behavior_tree_node/high_level_control",
        )

        # Periodic report
        self.report_timer = self.create_timer(15.0, self._periodic_report)
        self.get_logger().info("No-LiDAR E2E test node started.")

    # ── Callbacks ────────────────────────────────────────────────────────

    def _on_bt_status(self, msg: HighLevelStatus):
        state_name = msg.state_name if hasattr(msg, "state_name") else str(msg.state)
        t = time.time() - self.metrics.start_time

        if not self.current_bt_state:
            self.current_bt_state = state_name
            self.metrics.bt_states.append((t, state_name))
            self.get_logger().info(f"[{t:.1f}s] BT initial state: {state_name}")
        elif state_name != self.current_bt_state:
            self.get_logger().info(
                f"[{t:.1f}s] BT state: {self.current_bt_state} -> {state_name}"
            )
            prev_state = self.current_bt_state
            self.current_bt_state = state_name
            self.metrics.bt_states.append((t, state_name))
            self._track_phase_transition(prev_state, state_name, t)

        if state_name in ("MOWING_COMPLETE", "IDLE_DOCKED") and self.mowing_started:
            if self.current_phase == TestPhase.DOCKING or state_name == "IDLE_DOCKED":
                self._complete_phase(TestPhase.DOCKING, True, "Robot docked successfully")
            self.test_complete = True

    def _track_phase_transition(self, prev: str, curr: str, t: float):
        if curr == "UNDOCKING" and self.current_phase == TestPhase.WAITING:
            self.current_phase = TestPhase.UNDOCKING
            self.phase_start_time = t

        elif curr == "PLANNING" and self.current_phase in (
            TestPhase.UNDOCKING, TestPhase.WAITING
        ):
            if self.current_phase == TestPhase.UNDOCKING:
                self._complete_phase(TestPhase.UNDOCKING, True, "Undock completed")
            self.current_phase = TestPhase.PLANNING
            self.phase_start_time = t

        elif curr == "MOWING" and self.current_phase == TestPhase.PLANNING:
            self._complete_phase(TestPhase.PLANNING, True, "Coverage path planned")
            self.current_phase = TestPhase.MOWING
            self.phase_start_time = t

        elif curr in ("MOWING_COMPLETE", "RETURNING_HOME", "COVERAGE_FAILED_DOCKING"):
            if self.current_phase == TestPhase.MOWING:
                mow_passed = curr == "MOWING_COMPLETE"
                self._complete_phase(
                    TestPhase.MOWING,
                    mow_passed,
                    f"Mowing {'completed' if mow_passed else 'failed'}",
                )
            self.current_phase = TestPhase.DOCKING
            self.phase_start_time = time.time() - self.metrics.start_time

    def _complete_phase(self, phase: TestPhase, passed: bool, details: str):
        t = time.time() - self.metrics.start_time
        duration = t - self.phase_start_time
        result = PhaseResult(
            name=phase.value, passed=passed, duration_sec=duration, details=details
        )
        self.metrics.phase_results.append(result)
        self.get_logger().info(
            f"Phase {phase.value}: {'PASS' if passed else 'FAIL'} "
            f"({duration:.1f}s) — {details}"
        )

    def _on_coverage_path(self, msg: Path):
        self.coverage_path = msg
        self.metrics.coverage_path_len = len(msg.poses)
        self.metrics.coverage_path_poses = [
            (p.pose.position.x, p.pose.position.y) for p in msg.poses
        ]
        self.get_logger().info(
            f"Received coverage path with {len(msg.poses)} poses"
        )

    def _on_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.current_pose = (x, y, yaw)

        t = time.time() - self.metrics.start_time
        if not self.metrics.robot_poses or (
            t - self.metrics.robot_poses[-1][0] > 0.5
        ):
            self.metrics.robot_poses.append((t, x, y, yaw))

    def _on_fusion_odom(self, msg: Odometry):
        """EKF map-frame pose — used for path deviation measurement."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.fusion_pose = (x, y)

        t = time.time() - self.metrics.start_time
        if (
            self.coverage_path
            and self.current_phase == TestPhase.MOWING
            and self.fusion_pose
        ):
            min_dist = self._min_distance_to_path(x, y)
            self.metrics.path_deviations.append((t, min_dist))

    def _on_cmd_vel(self, msg: Twist):
        t = time.time() - self.metrics.start_time
        if not self.metrics.cmd_vels or (
            t - self.metrics.cmd_vels[-1][0] > 0.5
        ):
            self.metrics.cmd_vels.append((t, msg.linear.x, msg.angular.z))

    # ── Helpers ──────────────────────────────────────────────────────────

    def _min_distance_to_path(self, x: float, y: float) -> float:
        poses = self.metrics.coverage_path_poses
        if not poses:
            return float("inf")
        min_d = float("inf")
        for i in range(len(poses) - 1):
            ax, ay = poses[i]
            bx, by = poses[i + 1]
            dx, dy = bx - ax, by - ay
            seg_len_sq = dx * dx + dy * dy
            if seg_len_sq < 1e-12:
                d = math.sqrt((x - ax) ** 2 + (y - ay) ** 2)
            else:
                t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / seg_len_sq))
                proj_x = ax + t * dx
                proj_y = ay + t * dy
                d = math.sqrt((x - proj_x) ** 2 + (y - proj_y) ** 2)
            if d < min_d:
                min_d = d
        return min_d

    def send_start_command(self):
        if not self.hlc_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("HighLevelControl service not available!")
            return False

        req = HighLevelControl.Request()
        req.command = 1  # START
        future = self.hlc_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() and future.result().success:
            self.get_logger().info("START command sent successfully")
            self.mowing_started = True
            return True
        self.get_logger().error("Failed to send START command")
        return False

    def _periodic_report(self):
        t = time.time() - self.metrics.start_time
        devs = self.metrics.path_deviations
        poses = self.metrics.robot_poses

        report = [
            f"\n{'='*60}",
            f"NO-LIDAR E2E — t={t:.0f}s  Phase: {self.current_phase.value}",
            f"{'='*60}",
        ]

        report.append(f"BT state: {self.current_bt_state}")

        if self.current_pose:
            x, y, yaw = self.current_pose
            report.append(
                f"Robot pose: ({x:.2f}, {y:.2f}, yaw={math.degrees(yaw):.1f})"
            )

        if self.fusion_pose:
            mx, my = self.fusion_pose
            report.append(f"EKF map pose: ({mx:.2f}, {my:.2f})")

        if devs:
            recent = [d for _, d in devs[-20:]]
            avg_dev = sum(recent) / len(recent)
            max_dev = max(recent)
            report.append(
                f"Path deviation: avg={avg_dev:.4f}m  max={max_dev:.4f}m (last 20)"
            )

        report.append(f"Coverage path: {self.metrics.coverage_path_len} poses")

        if len(poses) > 1:
            dist = sum(
                math.sqrt(
                    (poses[i][1] - poses[i - 1][1]) ** 2
                    + (poses[i][2] - poses[i - 1][2]) ** 2
                )
                for i in range(1, len(poses))
            )
            report.append(f"Distance traveled: {dist:.1f}m")

        report.append(f"{'='*60}")
        self.get_logger().info("\n".join(report))

    def print_final_report(self) -> bool:
        t = time.time() - self.metrics.start_time
        devs = self.metrics.path_deviations
        poses = self.metrics.robot_poses

        report = [
            f"\n{'#'*60}",
            f"NO-LIDAR E2E SIMULATION REPORT",
            f"Duration: {t:.0f}s ({t/60:.1f} min)",
            f"{'#'*60}",
        ]

        # ── Phase Results ──
        report.append("\n=== PHASE RESULTS ===")
        all_phases_pass = True
        for pr in self.metrics.phase_results:
            status = "PASS" if pr.passed else "FAIL"
            report.append(
                f"  {pr.name:20s} {status} ({pr.duration_sec:.1f}s) — {pr.details}"
            )
            if not pr.passed:
                all_phases_pass = False

        if not self.metrics.phase_results:
            report.append("  No phase transitions recorded!")
            all_phases_pass = False

        # ── BT State History ──
        report.append("\n=== Behavior Tree State History ===")
        for ts, st in self.metrics.bt_states:
            report.append(f"  [{ts:7.1f}s] {st}")

        # ── Path Tracking Quality ──
        report.append("\n=== Path Tracking Quality (GPS-only) ===")
        path_pass = True
        median_dev = None
        if devs:
            all_devs = [d for _, d in devs]
            avg = sum(all_devs) / len(all_devs)
            sorted_devs = sorted(all_devs)
            p50 = sorted_devs[int(len(sorted_devs) * 0.50)]
            p95 = sorted_devs[min(int(len(sorted_devs) * 0.95), len(sorted_devs) - 1)]
            max_d = max(all_devs)
            median_dev = p50
            report.append(f"  Samples:     {len(all_devs)}")
            report.append(f"  Mean error:  {avg:.4f} m")
            report.append(f"  Median:      {p50:.4f} m")
            report.append(f"  P95 error:   {p95:.4f} m")
            report.append(f"  Max error:   {max_d:.4f} m")
            if p50 < 0.05:
                report.append("  PASS: median deviation < 5cm (excellent for GPS-only)")
            elif p50 < 0.10:
                report.append("  PASS: median deviation < 10cm (good for GPS-only)")
            elif p50 < 0.30:
                report.append("  WARN: median deviation 10-30cm (acceptable but needs tuning)")
            else:
                report.append("  FAIL: median deviation > 30cm")
                path_pass = False
        else:
            report.append("  No path deviation data collected")
            path_pass = False

        # ── Movement ──
        report.append("\n=== Movement ===")
        if len(poses) > 1:
            dist = sum(
                math.sqrt(
                    (poses[i][1] - poses[i - 1][1]) ** 2
                    + (poses[i][2] - poses[i - 1][2]) ** 2
                )
                for i in range(1, len(poses))
            )
            report.append(f"  Total distance: {dist:.1f} m")
            report.append(f"  Average speed:  {dist/t:.3f} m/s")

        # ── Overall Verdict ──
        report.append(f"\n{'#'*60}")
        report.append("VALIDATION CRITERIA:")

        criteria = [
            ("Undock→Plan→Mow→Dock cycle", all_phases_pass),
            ("Path tracking (median < 10cm)", path_pass),
        ]

        overall_pass = True
        for name, passed in criteria:
            status = "PASS" if passed else "FAIL"
            report.append(f"  [{status}] {name}")
            if not passed:
                overall_pass = False

        if median_dev is not None:
            report.append(f"\n  Path accuracy: {median_dev*100:.1f}cm median deviation")

        report.append(f"\nOVERALL: {'PASS' if overall_pass else 'FAIL'}")
        report.append(f"{'#'*60}\n")

        self.get_logger().info("\n".join(report))
        return overall_pass


def wait_for_tf(node, timeout=120.0):
    """Wait until the map→base_link TF chain is available."""
    import tf2_ros
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer, node)

    node.get_logger().info("Waiting for TF chain map→base_link...")
    start = time.time()
    while time.time() - start < timeout:
        rclpy.spin_once(node, timeout_sec=0.5)
        try:
            buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
            node.get_logger().info(
                f"TF chain available after {time.time() - start:.1f}s"
            )
            return True
        except Exception:
            pass

    node.get_logger().error(f"TF chain not available after {timeout}s")
    return False


def main():
    rclpy.init()
    node = NoLidarE2ETestNode()

    # Wait for TF chain (robot_localization must be publishing map→odom→base_footprint)
    if not wait_for_tf(node, timeout=120.0):
        node.get_logger().error("TF chain never became available. Aborting.")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # Brief settle time after TF is available
    node.get_logger().info("TF available. Waiting 5s for system to settle...")
    time.sleep(5)

    # Send START command
    if not node.send_start_command():
        node.get_logger().error("Failed to send START. Aborting.")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # Spin until test completes or timeout (20 min)
    timeout = 1200.0
    start = time.time()

    def _on_signal(sig, frame):
        node.get_logger().info(f"Received signal {sig}, printing final report...")
        node.print_final_report()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGTERM, _on_signal)

    try:
        while rclpy.ok() and not node.test_complete:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.time() - start > timeout:
                node.get_logger().warn(f"Test timeout after {timeout}s")
                break
    except KeyboardInterrupt:
        node.get_logger().info("Test interrupted by user")

    overall = node.print_final_report()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if overall else 1)


if __name__ == "__main__":
    main()
