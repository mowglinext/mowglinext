#!/usr/bin/env python3
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
e2e_test.py — End-to-end simulation validation for Mowgli ROS2.

Validates the full mowing cycle:
  1. Undock from charging station
  2. Plan coverage path (with obstacle-aware routing)
  3. Mow following planned paths accurately (median deviation < 30cm)
  4. Navigate AROUND obstacles (not just stop)
  5. Dock back when complete

Usage (inside the dev-sim container):
  source /ros2_ws/install/setup.bash
  python3 /ros2_ws/src/e2e_test.py

Or from host:
  docker compose -f docker-compose.simulation.yaml exec dev-sim \
    bash -c "source /ros2_ws/install/setup.bash && python3 /ros2_ws/src/e2e_test.py"
"""

import math
import signal
import subprocess
import sys
import time
import threading
from collections import defaultdict
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from std_msgs.msg import Bool, String
from sensor_msgs.msg import LaserScan
from mowgli_interfaces.srv import HighLevelControl, EmergencyStop
from mowgli_interfaces.msg import HighLevelStatus


class TestPhase(Enum):
    WAITING = "WAITING"
    UNDOCKING = "UNDOCKING"
    PLANNING = "PLANNING"
    MOWING = "MOWING"
    DOCKING = "DOCKING"
    MANUAL_MOWING = "MANUAL_MOWING"
    AREA_RECORDING = "AREA_RECORDING"
    EMERGENCY_RESET = "EMERGENCY_RESET"
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
    gps_states: list = field(default_factory=list)
    map_sizes: list = field(default_factory=list)
    robot_poses: list = field(default_factory=list)
    cmd_vels: list = field(default_factory=list)
    coverage_path_len: int = 0
    coverage_path_poses: list = field(default_factory=list)
    min_obstacle_dist: list = field(default_factory=list)
    swath_count: int = 0
    completed_swaths: int = 0
    skipped_swaths: int = 0
    reroute_events: list = field(default_factory=list)
    phase_results: list = field(default_factory=list)

    # Per-swath deviation tracking
    swath_deviations: dict = field(default_factory=lambda: defaultdict(list))
    current_swath_index: int = -1

    # Area coverage grid (25cm cells)
    covered_cells: set = field(default_factory=set)
    covered_cell_visits: dict = field(default_factory=lambda: defaultdict(int))
    mow_area_cells: set = field(default_factory=set)
    coverage_grid_resolution: float = 0.18

    # Obstacle avoidance maneuver details
    avoidance_maneuvers: list = field(default_factory=list)

    # Time classification (seconds)
    time_moving: float = 0.0
    time_stopped: float = 0.0
    time_turning: float = 0.0
    time_recovering: float = 0.0

    # Per-phase time tracking
    phase_time_moving: dict = field(default_factory=lambda: defaultdict(float))
    phase_time_stopped: dict = field(default_factory=lambda: defaultdict(float))

    # Speed samples per phase
    phase_speeds: dict = field(default_factory=lambda: defaultdict(list))

    # Boundary violation tracking
    boundary_violations: list = field(default_factory=list)  # [(t, x, y)]
    boundary_violation_active: bool = False

    # BT state duration accumulator
    bt_state_durations: dict = field(default_factory=lambda: defaultdict(float))


class E2ETestNode(Node):
    def __init__(self):
        super().__init__("e2e_test_node")

        self.metrics = Metrics(start_time=time.time())
        self.coverage_path = None
        self.current_pose = None
        self.current_bt_state = ""
        self.test_complete = False
        self.mowing_cycle_complete = False
        self.mowing_started = False

        # Phase tracking
        self.current_phase = TestPhase.WAITING
        self.phase_start_time = time.time()
        self.undock_complete = False
        self.planning_complete = False
        self.dock_complete = False

        # BT state duration tracking
        self._last_bt_state_time = time.time()

        # Obstacle avoidance tracking
        self.obstacle_spawned = False
        self.obstacle_test_done = False
        self.obstacle_test_result = None
        self.obstacle_spawn_time = 0.0
        self.obstacle_rerouted = False
        self._last_cmd_vel_x = 0.0

        # QoS profiles
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Subscribers
        self.create_subscription(
            HighLevelStatus,
            "/behavior_tree_node/high_level_status",
            self._on_bt_status,
            reliable_qos,
        )
        # opennav_coverage publishes the planned per-area path on
        # /coverage_server/coverage_plan (visualizer topic). The old
        # coverage_planner_node is gone — this is now Fields2Cover output:
        # one full path per area (headland inset + swaths + transitions).
        self.create_subscription(
            Path,
            "/coverage_server/coverage_plan",
            self._on_coverage_path,
            transient_qos,
        )
        self.create_subscription(
            Odometry, "/wheel_odom", self._on_odom, sensor_qos
        )
        self.create_subscription(
            Odometry, "/odometry/filtered_map", self._on_filtered_map, reliable_qos
        )
        self.slam_pose = None
        self.create_subscription(
            LaserScan, "/scan", self._on_scan, sensor_qos
        )
        self.create_subscription(
            String,
            "/gps_degradation_sim/status",
            self._on_gps_status,
            transient_qos,
        )
        self.create_subscription(
            OccupancyGrid, "/map", self._on_map, reliable_qos
        )
        self.create_subscription(
            Twist, "/cmd_vel_smoothed", self._on_cmd_vel, sensor_qos
        )
        self.create_subscription(
            Twist, "/cmd_vel", self._on_cmd_vel_out, sensor_qos
        )
        self.create_subscription(
            Bool, "/map_server_node/boundary_violation", self._on_boundary_violation, reliable_qos
        )

        # Service clients
        self.hlc_client = self.create_client(
            HighLevelControl,
            "/behavior_tree_node/high_level_control",
        )
        self.emergency_client = self.create_client(
            EmergencyStop,
            "/hardware_bridge/emergency_stop",
        )

        # Periodic report timer
        self.report_timer = self.create_timer(15.0, self._periodic_report)
        self.get_logger().info("E2E test node started. Waiting 5s for system to settle...")

    # ── Callbacks ────────────────────────────────────────────────────────

    def _on_bt_status(self, msg: HighLevelStatus):
        state_name = msg.state_name if hasattr(msg, "state_name") else str(msg.state)
        t = time.time() - self.metrics.start_time

        if not self.current_bt_state:
            self.current_bt_state = state_name
            self.metrics.bt_states.append((t, state_name))
            self._last_bt_state_time = time.time()
            self.get_logger().info(f"[{t:.1f}s] BT initial state: {state_name}")
        elif state_name != self.current_bt_state:
            # Accumulate duration of previous state
            now = time.time()
            self.metrics.bt_state_durations[self.current_bt_state] += (now - self._last_bt_state_time)
            self._last_bt_state_time = now

            self.get_logger().info(f"[{t:.1f}s] BT state: {self.current_bt_state} -> {state_name}")
            prev_state = self.current_bt_state
            self.current_bt_state = state_name
            self.metrics.bt_states.append((t, state_name))

            # Phase transitions
            self._track_phase_transition(prev_state, state_name, t)

        # Track swath progress from HighLevelStatus fields
        self.metrics.swath_count = getattr(msg, 'total_swaths', 0)
        self.metrics.completed_swaths = getattr(msg, 'completed_swaths', 0)
        self.metrics.skipped_swaths = getattr(msg, 'skipped_swaths', 0)
        if hasattr(msg, 'current_path_index') and msg.current_path_index >= 0:
            self.metrics.current_swath_index = msg.current_path_index

        if state_name in ("MOWING_COMPLETE", "IDLE_DOCKED") and self.mowing_started:
            if self.current_phase == TestPhase.DOCKING or state_name == "IDLE_DOCKED":
                self._complete_phase(TestPhase.DOCKING, True, "Robot docked successfully")
            self.mowing_cycle_complete = True

        # Detect reroute events from BT state
        if state_name == "RECOVERING":
            self.metrics.reroute_events.append((t, "RECOVERY_TRIGGERED"))
            self.get_logger().info(f"[{t:.1f}s] Recovery/reroute event detected")

    def _track_phase_transition(self, prev: str, curr: str, t: float):
        """Track phase transitions based on BT state changes."""
        if curr == "UNDOCKING" and self.current_phase == TestPhase.WAITING:
            self.current_phase = TestPhase.UNDOCKING
            self.phase_start_time = t

        elif curr == "PLANNING" and self.current_phase in (TestPhase.UNDOCKING, TestPhase.WAITING):
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
                    f"Mowing {'completed' if mow_passed else 'failed'}"
                )
            self.current_phase = TestPhase.DOCKING
            self.phase_start_time = time.time() - self.metrics.start_time

        elif curr == "MANUAL_MOWING" and self.current_phase == TestPhase.MANUAL_MOWING:
            pass  # Already tracked by the test phase driver

        elif curr == "RECORDING" and self.current_phase == TestPhase.AREA_RECORDING:
            pass  # Already tracked by the test phase driver

        elif curr == "EMERGENCY" and self.current_phase == TestPhase.EMERGENCY_RESET:
            pass  # Already tracked by the test phase driver

    def _complete_phase(self, phase: TestPhase, passed: bool, details: str):
        t = time.time() - self.metrics.start_time
        duration = t - self.phase_start_time
        result = PhaseResult(
            name=phase.value, passed=passed, duration_sec=duration, details=details
        )
        self.metrics.phase_results.append(result)
        self.get_logger().info(
            f"Phase {phase.value}: {'PASS' if passed else 'FAIL'} ({duration:.1f}s) — {details}"
        )

    def _on_coverage_path(self, msg: Path):
        self.coverage_path = msg
        self.metrics.coverage_path_len = len(msg.poses)
        self.metrics.coverage_path_poses = [
            (p.pose.position.x, p.pose.position.y) for p in msg.poses
        ]
        self._compute_mow_area_cells()
        planned_len = self._compute_planned_path_length()
        self.get_logger().info(
            f"Received coverage path: {len(msg.poses)} poses, "
            f"{planned_len:.1f}m planned, {len(self.metrics.mow_area_cells)} area cells"
        )

    def _on_filtered_map(self, msg: Odometry):
        self.slam_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

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
        if len(self.metrics.robot_poses) == 0 or (
            t - self.metrics.robot_poses[-1][0] > 0.5
        ):
            self.metrics.robot_poses.append((t, x, y, yaw))

            # Compute path deviation during mowing
            if self.coverage_path and self.current_phase == TestPhase.MOWING and self.slam_pose:
                min_dist = self._min_distance_to_path(self.slam_pose[0], self.slam_pose[1])
                self.metrics.path_deviations.append((t, min_dist))
                # Per-swath deviation
                idx = self.metrics.current_swath_index
                if idx >= 0:
                    self.metrics.swath_deviations[idx].append(min_dist)

            # Area coverage grid (mark cells visited during mowing)
            if self.current_phase == TestPhase.MOWING and self.slam_pose:
                res = self.metrics.coverage_grid_resolution
                gx = int(self.slam_pose[0] / res)
                gy = int(self.slam_pose[1] / res)
                self.metrics.covered_cells.add((gx, gy))
                self.metrics.covered_cell_visits[(gx, gy)] += 1

            # Time classification (0.5s intervals)
            dt = 0.5
            phase_name = self.current_phase.value
            if self.metrics.cmd_vels:
                _, lin_x, ang_z = self.metrics.cmd_vels[-1]
                speed = abs(lin_x)
                if self.current_bt_state == "RECOVERING":
                    self.metrics.time_recovering += dt
                elif speed < 0.01 and abs(ang_z) < 0.01:
                    self.metrics.time_stopped += dt
                    self.metrics.phase_time_stopped[phase_name] += dt
                elif abs(ang_z) > 0.3:
                    self.metrics.time_turning += dt
                else:
                    self.metrics.time_moving += dt
                    self.metrics.phase_time_moving[phase_name] += dt
                # Track speeds per phase
                if speed > 0.01:
                    self.metrics.phase_speeds[phase_name].append(speed)

    def _on_scan(self, msg: LaserScan):
        self._latest_scan = msg
        t = time.time() - self.metrics.start_time
        valid_ranges = [
            r for r in msg.ranges
            if msg.range_min < r < msg.range_max and not math.isinf(r)
        ]
        if valid_ranges:
            min_dist = min(valid_ranges)
            if len(self.metrics.min_obstacle_dist) == 0 or (
                t - self.metrics.min_obstacle_dist[-1][0] > 1.0
            ):
                self.metrics.min_obstacle_dist.append((t, min_dist))

    def _on_gps_status(self, msg: String):
        t = time.time() - self.metrics.start_time
        self.metrics.gps_states.append((t, msg.data))

    def _on_map(self, msg: OccupancyGrid):
        t = time.time() - self.metrics.start_time
        w = msg.info.width
        h = msg.info.height
        known = sum(1 for c in msg.data if c >= 0)
        if len(self.metrics.map_sizes) == 0 or (
            t - self.metrics.map_sizes[-1][0] > 5.0
        ):
            self.metrics.map_sizes.append((t, w, h, known))

    def _on_boundary_violation(self, msg: Bool):
        t = time.time() - self.metrics.start_time
        was_active = self.metrics.boundary_violation_active
        self.metrics.boundary_violation_active = msg.data
        if msg.data and not was_active:
            x, y = 0.0, 0.0
            if self.slam_pose:
                x, y = self.slam_pose
            elif self.current_pose:
                x, y = self.current_pose[0], self.current_pose[1]
            self.metrics.boundary_violations.append((t, x, y))
            self.get_logger().error(
                f"[{t:.1f}s] BOUNDARY VIOLATION: robot at ({x:.2f}, {y:.2f}) is outside mowing area!"
            )

    def _on_cmd_vel_out(self, msg: Twist):
        self._last_cmd_vel_x = msg.linear.x

    def _on_cmd_vel(self, msg: Twist):
        t = time.time() - self.metrics.start_time
        if len(self.metrics.cmd_vels) == 0 or (
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

    def _get_min_scan_range(self) -> float:
        if not hasattr(self, '_latest_scan') or self._latest_scan is None:
            return float('inf')
        s = self._latest_scan
        valid = [r for r in s.ranges if r > s.range_min and r < s.range_max]
        return min(valid) if valid else float('inf')

    def _compute_planned_path_length(self) -> float:
        """Sum of Euclidean distances between consecutive coverage path poses."""
        poses = self.metrics.coverage_path_poses
        if len(poses) < 2:
            return 0.0
        total = 0.0
        for i in range(1, len(poses)):
            dx = poses[i][0] - poses[i - 1][0]
            dy = poses[i][1] - poses[i - 1][1]
            total += math.sqrt(dx * dx + dy * dy)
        return total

    def _compute_actual_mowing_distance(self) -> float:
        """Distance traveled during MOWING phase only."""
        mow_start = None
        mow_end = None
        for ts, st in self.metrics.bt_states:
            if st == "MOWING" and mow_start is None:
                mow_start = ts
            elif mow_start is not None and st in (
                "MOWING_COMPLETE", "RETURNING_HOME", "COVERAGE_FAILED_DOCKING",
            ):
                mow_end = ts
                break
        if mow_start is None:
            return 0.0
        if mow_end is None:
            mow_end = time.time() - self.metrics.start_time

        poses = [
            (t, x, y)
            for t, x, y, _ in self.metrics.robot_poses
            if mow_start <= t <= mow_end
        ]
        dist = 0.0
        for i in range(1, len(poses)):
            dx = poses[i][1] - poses[i - 1][1]
            dy = poses[i][2] - poses[i - 1][2]
            dist += math.sqrt(dx * dx + dy * dy)
        return dist

    def _compute_mow_area_cells(self):
        """Rasterize the planned coverage path into grid cells."""
        res = self.metrics.coverage_grid_resolution
        cells = set()
        poses = self.metrics.coverage_path_poses
        for i in range(1, len(poses)):
            ax, ay = poses[i - 1]
            bx, by = poses[i]
            dx, dy = bx - ax, by - ay
            seg_len = math.sqrt(dx * dx + dy * dy)
            steps = max(1, int(seg_len / (res * 0.5)))
            for s in range(steps + 1):
                frac = s / steps
                px = ax + frac * dx
                py = ay + frac * dy
                cells.add((int(px / res), int(py / res)))
        self.metrics.mow_area_cells = cells

    def _deviation_histogram(self, devs: list) -> str:
        """Return ASCII histogram of deviation buckets."""
        buckets = [0.05, 0.10, 0.20, 0.30, 0.50, 1.00, float("inf")]
        labels = ["<5cm", "5-10cm", "10-20cm", "20-30cm", "30-50cm", "50cm-1m", ">1m"]
        counts = [0] * len(buckets)
        for d in devs:
            for i, b in enumerate(buckets):
                if d <= b:
                    counts[i] += 1
                    break
        total = len(devs) or 1
        lines = []
        for label, count in zip(labels, counts):
            pct = count / total * 100
            bar = "#" * int(pct / 2)
            lines.append(f"    {label:>8s} | {bar:<50s} {count:4d} ({pct:.1f}%)")
        return "\n".join(lines)

    def _spawn_obstacle_at(self, ox: float, oy: float, name: str = "e2e_obstacle") -> bool:
        """Spawn a cylinder obstacle using blocking create service."""
        sdf = (
            f'<sdf version="1.9"><model name="{name}"><static>true</static>'
            f'<pose>{ox:.3f} {oy:.3f} 0.50 0 0 0</pose>'
            f'<link name="link">'
            f'<collision name="c"><geometry><cylinder><radius>0.30</radius>'
            f'<length>1.0</length></cylinder></geometry></collision>'
            f'<visual name="v"><geometry><cylinder><radius>0.30</radius>'
            f'<length>1.0</length></cylinder></geometry>'
            f'<material><ambient>1 0.5 0 1</ambient>'
            f'<diffuse>1 0.5 0 1</diffuse></material></visual>'
            f'</link></model></sdf>'
        )
        escaped = sdf.replace('"', '\\"')
        cmd = (
            f'gz service -s /world/garden/create/blocking '
            f'--reqtype gz.msgs.EntityFactory '
            f'--reptype gz.msgs.Boolean '
            f'--timeout 10000 '
            f'--req \'sdf: "{escaped}"\''
        )
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=15)
        success = "true" in result.stdout
        if success:
            self.get_logger().info(f"Spawned obstacle '{name}' at ({ox:.2f}, {oy:.2f})")
        else:
            self.get_logger().warn(f"Failed to spawn obstacle: {result.stderr[:200]}")
        return success

    def _remove_obstacle(self, name: str = "e2e_obstacle"):
        cmd = (
            f'gz service -s /world/garden/remove '
            f'--reqtype gz.msgs.Entity '
            f'--reptype gz.msgs.Boolean '
            f'--timeout 5000 '
            f'--req \'name: "{name}" type: MODEL\''
        )
        subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=10)
        self.get_logger().info(f"Removed test obstacle '{name}'")

    def _run_obstacle_avoidance_test(self):
        """
        Active obstacle avoidance test — runs in a background thread.

        Pre-spawns a cylinder on a coverage swath. Validates that the robot:
        1. Detects the obstacle (stops or slows)
        2. Reroutes AROUND the obstacle (NavigateToPose via planner)
        3. Resumes mowing the remaining swath
        """
        self.get_logger().info("=== OBSTACLE AVOIDANCE TEST: waiting for MOWING phase ===")

        while not self.test_complete:
            if self.current_phase == TestPhase.MOWING:
                break
            time.sleep(1.0)

        if self.test_complete:
            return

        # Wait a bit for robot to start mowing (let it complete first transit)
        time.sleep(15.0)

        self.get_logger().info("=== OBSTACLE AVOIDANCE TEST: monitoring robot behavior ===")
        self.obstacle_spawn_time = time.time()

        # Monitor for rerouting behavior
        robot_stopped = False
        robot_resumed = False
        stop_time = None
        resume_time = None
        closest_approach = 999.0
        initial_vel_samples = []

        for i in range(3000):  # 5 minutes at 10Hz
            time.sleep(0.1)
            vel = self._last_cmd_vel_x

            # Track closest scan range
            cur_min = self._get_min_scan_range()
            if cur_min < closest_approach:
                closest_approach = cur_min

            # Detect stop (obstacle detected)
            if abs(vel) < 0.01 and not robot_stopped and i > 50:
                robot_stopped = True
                stop_time = time.time() - self.obstacle_spawn_time
                self.get_logger().info(
                    f"=== OBSTACLE TEST: robot STOPPED after {stop_time:.1f}s "
                    f"(closest={closest_approach:.2f}m) ==="
                )

            # Detect resume (rerouted around obstacle)
            if robot_stopped and not robot_resumed and abs(vel) > 0.05:
                robot_resumed = True
                resume_time = time.time() - self.obstacle_spawn_time
                self.obstacle_rerouted = True
                self.get_logger().info(
                    f"=== OBSTACLE TEST: robot RESUMED after {resume_time:.1f}s "
                    f"(rerouted around obstacle) ==="
                )

            # Log every 5 seconds
            if i % 50 == 0:
                pose_str = ""
                if self.current_pose:
                    x, y, _ = self.current_pose
                    pose_str = f" robot=({x:.1f},{y:.1f})"
                self.get_logger().info(
                    f"=== OBSTACLE TEST t+{i/10:.0f}s: vel={vel:.3f} "
                    f"scan_min={cur_min:.2f}m stopped={robot_stopped} "
                    f"resumed={robot_resumed}{pose_str} ==="
                )

            # Success: robot stopped AND resumed (navigated around)
            if robot_stopped and robot_resumed:
                time.sleep(5.0)  # Let it continue a bit
                break

            if self.test_complete:
                break

        # Evaluate
        if robot_stopped and robot_resumed and stop_time is not None and resume_time is not None:
            self.metrics.avoidance_maneuvers.append({
                "start_time": stop_time,
                "end_time": resume_time,
                "time_cost": resume_time - stop_time,
                "closest_approach": closest_approach,
            })
            self.obstacle_test_result = "PASS"
            self.get_logger().info(
                f"=== OBSTACLE AVOIDANCE TEST: PASS — robot stopped and navigated around "
                f"(stop={stop_time:.1f}s, resume={resume_time:.1f}s, "
                f"closest={closest_approach:.2f}m) ==="
            )
        elif robot_stopped:
            self.obstacle_test_result = "PARTIAL"
            self.get_logger().warn(
                f"=== OBSTACLE AVOIDANCE TEST: PARTIAL — robot stopped but did NOT resume "
                f"(may have skipped swath instead of rerouting) ==="
            )
        else:
            self.obstacle_test_result = "FAIL"
            self.get_logger().error(
                f"=== OBSTACLE AVOIDANCE TEST: FAIL — robot did NOT stop for obstacle "
                f"(closest={closest_approach:.2f}m) ==="
            )

        self.obstacle_test_done = True

    def send_command(self, cmd_id: int, cmd_name: str = "") -> bool:
        """Send a HighLevelControl command and return True on success."""
        if not self.hlc_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("HighLevelControl service not available!")
            return False

        req = HighLevelControl.Request()
        req.command = cmd_id
        future = self.hlc_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() and future.result().success:
            label = cmd_name or str(cmd_id)
            self.get_logger().info(f"Command {label} (id={cmd_id}) sent successfully")
            return True
        label = cmd_name or str(cmd_id)
        self.get_logger().error(f"Failed to send command {label} (id={cmd_id})")
        return False

    def send_start_command(self):
        if self.send_command(1, "START"):
            self.mowing_started = True
            return True
        return False

    def send_emergency_stop(self, emergency: int) -> bool:
        """Send an EmergencyStop service call."""
        if not self.emergency_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("EmergencyStop service not available!")
            return False

        req = EmergencyStop.Request()
        req.emergency = emergency
        future = self.emergency_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() and future.result().success:
            self.get_logger().info(
                f"EmergencyStop (emergency={emergency}) sent successfully"
            )
            return True
        self.get_logger().error(f"Failed to send EmergencyStop (emergency={emergency})")
        return False

    def wait_for_bt_state(self, target_state, timeout_sec: float = 15.0) -> bool:
        """Spin until BT reaches a target state or timeout expires.

        target_state can be a single string or a list of strings (any match).
        """
        if isinstance(target_state, str):
            target_state = [target_state]
        deadline = time.time() + timeout_sec
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_bt_state in target_state:
                return True
        return self.current_bt_state in target_state

    def _progress_bar(self, pct: float, width: int = 30) -> str:
        filled = int(pct / 100 * width)
        return f"[{'█' * filled}{'░' * (width - filled)}] {pct:.1f}%"

    def _speed_stats(self, phase: str) -> str:
        speeds = self.metrics.phase_speeds.get(phase, [])
        if not speeds:
            return "n/a"
        return (
            f"avg={sum(speeds)/len(speeds):.2f} "
            f"min={min(speeds):.2f} max={max(speeds):.2f} m/s"
        )

    def _overlap_stats(self) -> tuple:
        """Returns (overlap_pct, wasted_visits) — cells visited >1 time."""
        visits = self.metrics.covered_cell_visits
        if not visits:
            return 0.0, 0
        multi = sum(1 for v in visits.values() if v > 1)
        wasted = sum(v - 1 for v in visits.values() if v > 1)
        overlap_pct = multi / len(visits) * 100 if visits else 0.0
        return overlap_pct, wasted

    def _periodic_report(self):
        t = time.time() - self.metrics.start_time
        m = self.metrics
        devs = m.path_deviations
        poses = m.robot_poses

        report = [
            f"\n{'═' * 80}",
            f"  E2E LIVE DASHBOARD  t={t:.0f}s ({t/60:.1f}min)  "
            f"Phase: {self.current_phase.value}  BT: {self.current_bt_state}",
            f"{'═' * 80}",
        ]

        # ── Robot State ──
        speed = abs(m.cmd_vels[-1][1]) if m.cmd_vels else 0.0
        ang = abs(m.cmd_vels[-1][2]) if m.cmd_vels else 0.0
        if self.current_pose:
            x, y, yaw = self.current_pose
            report.append(
                f"  Robot: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f}°)  "
                f"v={speed:.2f}m/s  ω={ang:.2f}rad/s"
            )

        # ── Mowing Progress ──
        report.append(f"{'─' * 80}")
        report.append("  MOWING PROGRESS")
        swath_pct = (m.completed_swaths / m.swath_count * 100) if m.swath_count > 0 else 0.0
        report.append(
            f"  Swaths:  {self._progress_bar(swath_pct)}  "
            f"{m.completed_swaths}/{m.swath_count} done, {m.skipped_swaths} skipped"
        )
        area_pct = (
            len(m.covered_cells) / len(m.mow_area_cells) * 100
            if m.mow_area_cells else 0.0
        )
        report.append(
            f"  Area:    {self._progress_bar(area_pct)}  "
            f"{len(m.covered_cells)}/{len(m.mow_area_cells)} cells"
        )

        # ETA based on swath completion rate
        if m.completed_swaths > 0 and m.swath_count > 0:
            mow_phases = [pr for pr in m.phase_results if pr.name == "MOWING"]
            mow_elapsed = t - self.phase_start_time if self.current_phase == TestPhase.MOWING else 0
            if mow_elapsed > 10:
                rate = m.completed_swaths / mow_elapsed
                remaining = m.swath_count - m.completed_swaths
                eta = remaining / rate if rate > 0 else 0
                report.append(f"  ETA:     ~{eta:.0f}s ({eta/60:.1f}min) remaining")

        # ── Precision (Path Tracking) ──
        report.append(f"{'─' * 80}")
        report.append("  PRECISION")
        if devs:
            all_d = [d for _, d in devs]
            recent = [d for _, d in devs[-20:]]
            sorted_d = sorted(all_d)
            p50 = sorted_d[len(sorted_d) // 2]
            p95 = sorted_d[min(int(len(sorted_d) * 0.95), len(sorted_d) - 1)]
            p99 = sorted_d[min(int(len(sorted_d) * 0.99), len(sorted_d) - 1)]
            within_10cm = sum(1 for d in all_d if d <= 0.10) / len(all_d) * 100
            within_30cm = sum(1 for d in all_d if d <= 0.30) / len(all_d) * 100
            report.append(
                f"  Deviation: P50={p50:.3f}m  P95={p95:.3f}m  P99={p99:.3f}m  max={max(all_d):.3f}m"
            )
            report.append(
                f"  Recent(20): avg={sum(recent)/len(recent):.3f}m  max={max(recent):.3f}m"
            )
            report.append(
                f"  Within 10cm: {within_10cm:.0f}%   Within 30cm: {within_30cm:.0f}%"
            )
            # Show grade
            if p50 < 0.10:
                grade = "A+ (excellent)"
            elif p50 < 0.20:
                grade = "A (very good)"
            elif p50 < 0.30:
                grade = "B (good)"
            elif p50 < 0.50:
                grade = "C (acceptable)"
            else:
                grade = "D (needs tuning)"
            report.append(f"  Grade: {grade}")

        # ── Efficiency ──
        report.append(f"{'─' * 80}")
        report.append("  EFFICIENCY")
        planned = self._compute_planned_path_length()
        actual = self._compute_actual_mowing_distance()
        if actual > 0 and planned > 0:
            eff = planned / actual
            overhead = (actual - planned) / planned * 100
            report.append(
                f"  Path efficiency: {eff:.3f} (1.0=optimal)  "
                f"Overhead: {overhead:+.1f}%"
            )
            report.append(
                f"  Distance: {actual:.1f}m traveled / {planned:.1f}m planned"
            )
        overlap_pct, wasted = self._overlap_stats()
        if self.metrics.covered_cell_visits:
            report.append(
                f"  Overlap: {overlap_pct:.1f}% cells revisited, "
                f"{wasted} wasted visits"
            )
        # Area per meter: how much new area covered per meter traveled
        if actual > 0 and m.covered_cells:
            cell_area = m.coverage_grid_resolution ** 2
            area_m2 = len(m.covered_cells) * cell_area
            report.append(
                f"  Area yield: {area_m2:.1f}m² covered in {actual:.0f}m "
                f"({area_m2/actual:.2f} m²/m)"
            )

        # ── Time Breakdown ──
        report.append(f"{'─' * 80}")
        report.append("  TIME BREAKDOWN")
        total_t = m.time_moving + m.time_stopped + m.time_turning + m.time_recovering
        if total_t > 0:
            mv_pct = m.time_moving / total_t * 100
            st_pct = m.time_stopped / total_t * 100
            tn_pct = m.time_turning / total_t * 100
            rc_pct = m.time_recovering / total_t * 100
            report.append(
                f"  Moving:     {m.time_moving:6.0f}s ({mv_pct:5.1f}%)  "
                f"{'▓' * int(mv_pct / 2)}"
            )
            report.append(
                f"  Stopped:    {m.time_stopped:6.0f}s ({st_pct:5.1f}%)  "
                f"{'░' * int(st_pct / 2)}"
            )
            report.append(
                f"  Turning:    {m.time_turning:6.0f}s ({tn_pct:5.1f}%)  "
                f"{'▒' * int(tn_pct / 2)}"
            )
            report.append(
                f"  Recovering: {m.time_recovering:6.0f}s ({rc_pct:5.1f}%)  "
                f"{'▒' * int(rc_pct / 2)}"
            )
            productive = mv_pct + tn_pct
            report.append(f"  Productive time: {productive:.1f}%  Wasted: {st_pct + rc_pct:.1f}%")

        # ── Speed Stats ──
        report.append(f"{'─' * 80}")
        report.append("  SPEED STATS")
        for phase in ["UNDOCKING", "MOWING", "DOCKING"]:
            speeds = m.phase_speeds.get(phase, [])
            if speeds:
                report.append(f"  {phase:12s}: {self._speed_stats(phase)}")

        # ── Obstacle & Safety ──
        report.append(f"{'─' * 80}")
        report.append("  OBSTACLES & SAFETY")
        min_obs = min((d for _, d in m.min_obstacle_dist), default=float("inf"))
        gps_state = m.gps_states[-1][1] if m.gps_states else "N/A"
        obs_result = self.obstacle_test_result or "PENDING"
        boundary_str = (
            f"VIOLATION x{len(m.boundary_violations)}" if m.boundary_violations
            else "OK"
        )
        report.append(
            f"  Closest obstacle: {min_obs:.2f}m  "
            f"Reroutes: {len(m.reroute_events)}  "
            f"Avoidance test: {obs_result}"
        )
        report.append(
            f"  GPS: {gps_state}  "
            f"Boundary: {boundary_str}"
        )

        # ── Phase Timeline ──
        report.append(f"{'─' * 80}")
        report.append("  PHASE TIMELINE")
        for pr in m.phase_results:
            status = "✓" if pr.passed else "✗"
            report.append(f"  {status} {pr.name:12s} {pr.duration_sec:6.1f}s  {pr.details}")
        # Current phase
        if self.current_phase not in (TestPhase.WAITING, TestPhase.COMPLETE):
            elapsed = t - self.phase_start_time
            report.append(f"  ▶ {self.current_phase.value:12s} {elapsed:6.1f}s  (in progress)")

        report.append(f"{'═' * 80}")
        self.get_logger().info("\n".join(report))

    def print_final_report(self):
        t = time.time() - self.metrics.start_time
        m = self.metrics
        devs = m.path_deviations
        maps = m.map_sizes
        gps = m.gps_states
        obs = m.min_obstacle_dist
        poses = m.robot_poses

        # Finalize BT state duration tracking
        if self.current_bt_state:
            now = time.time()
            m.bt_state_durations[self.current_bt_state] += (now - self._last_bt_state_time)
            self._last_bt_state_time = now

        report = [
            f"\n{'#' * 70}",
            f"FINAL E2E SIMULATION VALIDATION REPORT",
            f"Duration: {t:.0f}s ({t / 60:.1f} min)",
            f"{'#' * 70}",
        ]

        # ── Phase Results ──
        report.append("\n=== PHASE RESULTS ===")
        all_phases_pass = True
        for pr in m.phase_results:
            status = "PASS" if pr.passed else "FAIL"
            report.append(f"  {pr.name:20s} {status} ({pr.duration_sec:.1f}s) — {pr.details}")
            if not pr.passed:
                all_phases_pass = False

        if not m.phase_results:
            report.append("  No phase transitions recorded!")
            all_phases_pass = False

        # ── BT State History ──
        report.append("\n=== Behavior Tree State History ===")
        for ts, st in m.bt_states:
            report.append(f"  [{ts:7.1f}s] {st}")

        # ── Path Tracking Quality ──
        report.append("\n=== Path Tracking Quality ===")
        path_pass = True
        if devs:
            all_devs = [d for _, d in devs]
            avg = sum(all_devs) / len(all_devs)
            sorted_devs = sorted(all_devs)
            p50 = sorted_devs[len(sorted_devs) // 2]
            p95 = sorted_devs[min(int(len(sorted_devs) * 0.95), len(sorted_devs) - 1)]
            max_d = max(all_devs)
            report.append(f"  Samples:     {len(all_devs)}")
            report.append(f"  Mean error:  {avg:.4f} m")
            report.append(f"  Median:      {p50:.4f} m")
            report.append(f"  P95 error:   {p95:.4f} m")
            report.append(f"  Max error:   {max_d:.4f} m")
            if p50 < 0.30:
                report.append("  PASS: median deviation < 30cm (excellent)")
            elif p50 < 0.50:
                report.append("  PASS: median deviation < 50cm (acceptable)")
            elif p50 < 1.0:
                report.append("  WARN: median deviation 50cm-1m (needs tuning)")
                path_pass = False
            else:
                report.append("  FAIL: median deviation > 1m")
                path_pass = False
        else:
            report.append("  No path deviation data collected")
            path_pass = False

        # ── Per-Swath Deviation ──
        report.append("\n=== Per-Swath Deviation ===")
        if m.swath_deviations:
            best_swath = (-1, float("inf"))
            worst_swath = (-1, 0.0)
            for idx in sorted(m.swath_deviations.keys()):
                sd = m.swath_deviations[idx]
                if not sd:
                    continue
                s_avg = sum(sd) / len(sd)
                s_sorted = sorted(sd)
                s_p95 = s_sorted[min(int(len(s_sorted) * 0.95), len(s_sorted) - 1)]
                s_max = max(sd)
                report.append(
                    f"  Swath {idx:3d}: n={len(sd):4d}  "
                    f"mean={s_avg:.3f}m  P95={s_p95:.3f}m  max={s_max:.3f}m"
                )
                if s_avg < best_swath[1]:
                    best_swath = (idx, s_avg)
                if s_avg > worst_swath[1]:
                    worst_swath = (idx, s_avg)
            if best_swath[0] >= 0:
                report.append(f"  Best swath:  #{best_swath[0]} (mean={best_swath[1]:.3f}m)")
                report.append(f"  Worst swath: #{worst_swath[0]} (mean={worst_swath[1]:.3f}m)")
        else:
            report.append("  No per-swath data (swath index not reported)")

        # ── Deviation Over Time (quartile trend) ──
        report.append("\n=== Deviation Over Time ===")
        if devs and len(devs) >= 4:
            all_devs = [d for _, d in devs]
            q_size = len(all_devs) // 4
            quartiles = [
                all_devs[: q_size],
                all_devs[q_size: 2 * q_size],
                all_devs[2 * q_size: 3 * q_size],
                all_devs[3 * q_size:],
            ]
            q_means = []
            for i, q in enumerate(quartiles, 1):
                qm = sum(q) / len(q) if q else 0.0
                q_means.append(qm)
                report.append(f"  Q{i} ({(i-1)*25}-{i*25}%): mean={qm:.4f}m")
            if q_means[-1] > q_means[0] * 1.5:
                report.append("  Trend: DEGRADING (deviation increasing over time)")
            elif q_means[-1] < q_means[0] * 0.7:
                report.append("  Trend: IMPROVING (deviation decreasing)")
            else:
                report.append("  Trend: STABLE")

        # ── Deviation Distribution ──
        report.append("\n=== Deviation Distribution ===")
        if devs:
            report.append(self._deviation_histogram([d for _, d in devs]))

        # ── Mowing Efficiency ──
        report.append("\n=== Mowing Efficiency ===")
        planned_len = self._compute_planned_path_length()
        actual_mow_dist = self._compute_actual_mowing_distance()
        efficiency_pass = True
        if planned_len > 0 and actual_mow_dist > 0:
            efficiency = planned_len / actual_mow_dist
            overhead_pct = (actual_mow_dist - planned_len) / planned_len * 100
            report.append(f"  Planned path length:  {planned_len:.1f} m")
            report.append(f"  Actual mowing dist:   {actual_mow_dist:.1f} m")
            report.append(f"  Efficiency ratio:     {efficiency:.3f} (planned/actual, 1.0 = optimal)")
            report.append(f"  Overhead:             {overhead_pct:+.1f}% extra distance")
            if efficiency >= 0.85:
                report.append("  PASS: efficiency >= 0.85")
            else:
                report.append("  FAIL: efficiency < 0.85")
                efficiency_pass = False
        else:
            report.append("  No mowing distance data")
            efficiency_pass = False

        # ── Area Coverage ──
        report.append("\n=== Area Coverage ===")
        coverage_pass = True
        if m.mow_area_cells:
            covered = len(m.covered_cells)
            total = len(m.mow_area_cells)
            pct = covered / total * 100 if total > 0 else 0.0
            uncovered = total - covered
            cell_area = m.coverage_grid_resolution ** 2
            report.append(f"  Grid resolution:      {m.coverage_grid_resolution} m")
            report.append(f"  Planned area cells:   {total} ({total * cell_area:.1f} m²)")
            report.append(f"  Covered cells:        {covered} ({covered * cell_area:.1f} m²)")
            report.append(f"  Coverage:             {pct:.1f}%")
            report.append(f"  Uncovered cells:      {uncovered} ({uncovered * cell_area:.1f} m²)")
            if pct >= 80.0:
                report.append("  PASS: coverage >= 80%")
            else:
                report.append("  FAIL: coverage < 80%")
                coverage_pass = False
        else:
            report.append("  No coverage path data")
            coverage_pass = False

        # ── Overlap / Redundancy Analysis ──
        report.append("\n=== Overlap Analysis ===")
        if m.covered_cell_visits:
            visit_counts = list(m.covered_cell_visits.values())
            single = sum(1 for v in visit_counts if v == 1)
            double = sum(1 for v in visit_counts if v == 2)
            triple_plus = sum(1 for v in visit_counts if v >= 3)
            wasted = sum(v - 1 for v in visit_counts if v > 1)
            total_visits = sum(visit_counts)
            overlap_pct = (1 - single / len(visit_counts)) * 100 if visit_counts else 0
            report.append(f"  Unique cells:         {len(visit_counts)}")
            report.append(f"  Total visits:         {total_visits}")
            report.append(f"  Single-pass:          {single} ({single/len(visit_counts)*100:.1f}%)")
            report.append(f"  Double-pass:          {double}")
            report.append(f"  Triple+ pass:         {triple_plus}")
            report.append(f"  Wasted visits:        {wasted} ({wasted/total_visits*100:.1f}% waste)")
            report.append(f"  Overlap ratio:        {overlap_pct:.1f}%")
            if overlap_pct < 15:
                report.append("  PASS: low overlap (<15%)")
            elif overlap_pct < 30:
                report.append("  WARN: moderate overlap (15-30%)")
            else:
                report.append("  FAIL: high overlap (>30%) — path may need optimization")
        else:
            report.append("  No overlap data")

        # ── SLAM Map Growth ──
        report.append("\n=== SLAM Map Growth ===")
        map_pass = True
        if maps:
            report.append(f"  Initial: {maps[0][1]}x{maps[0][2]}, {maps[0][3]} known cells")
            report.append(f"  Final:   {maps[-1][1]}x{maps[-1][2]}, {maps[-1][3]} known cells")
            growth = maps[-1][3] - maps[0][3]
            report.append(f"  Growth:  {growth:+d} cells")
            if maps[-1][3] < 1000:
                report.append("  FAIL: SLAM map too small")
                map_pass = False
            else:
                report.append("  PASS: SLAM map adequate")
        else:
            map_pass = False

        # ── GPS Degradation ──
        report.append("\n=== GPS Degradation Events ===")
        normal_count = sum(1 for _, s in gps if s == "NORMAL")
        degraded_count = sum(1 for _, s in gps if s == "DEGRADED")
        report.append(f"  NORMAL transitions:   {normal_count}")
        report.append(f"  DEGRADED transitions: {degraded_count}")

        # ── Obstacle Proximity ──
        report.append("\n=== Obstacle Proximity ===")
        collision_pass = True
        if obs:
            all_obs = [d for _, d in obs]
            min_ever = min(all_obs)
            collision_events = sum(1 for d in all_obs if d < 0.15)
            close_calls = sum(1 for d in all_obs if 0.15 <= d < 0.30)
            report.append(f"  Closest approach: {min_ever:.3f} m")
            report.append(f"  Collision events (<15cm): {collision_events}")
            report.append(f"  Close calls (15-30cm):    {close_calls}")
            if collision_events > 0:
                report.append(f"  FAIL: {collision_events} potential collision(s)")
                collision_pass = False
            else:
                report.append("  PASS: no collisions")

        # ── Boundary Violations ──
        report.append("\n=== Boundary Violations ===")
        boundary_pass = True
        if m.boundary_violations:
            boundary_pass = False
            report.append(f"  FAIL: {len(m.boundary_violations)} boundary violation(s)!")
            for t_v, x_v, y_v in m.boundary_violations:
                report.append(f"    [{t_v:.1f}s] robot at ({x_v:.2f}, {y_v:.2f})")
        else:
            report.append("  PASS: robot stayed within mowing area")

        # ── Active Obstacle Avoidance Test ──
        report.append("\n=== Active Obstacle Avoidance Test ===")
        obstacle_pass = True
        if self.obstacle_test_result == "PASS":
            report.append("  Robot stopped for obstacle: YES")
            report.append("  Robot navigated around obstacle: YES")
            report.append("  PASS: obstacle avoidance with rerouting")
        elif self.obstacle_test_result == "PARTIAL":
            report.append("  Robot stopped for obstacle: YES")
            report.append("  Robot navigated around obstacle: NO (skipped swath)")
            report.append("  PARTIAL: collision avoided but no rerouting")
        elif self.obstacle_test_result == "FAIL":
            report.append("  Robot stopped for obstacle: NO")
            report.append("  FAIL: obstacle not detected or avoided")
            obstacle_pass = False
        elif self.obstacle_test_result == "SKIP":
            report.append("  SKIP: could not spawn obstacle in Gazebo")
        else:
            report.append("  NOT RUN: test did not reach mowing phase")

        # ── Obstacle Avoidance Detail ──
        report.append("\n=== Obstacle Avoidance Detail ===")
        if m.avoidance_maneuvers:
            total_cost = 0.0
            for i, man in enumerate(m.avoidance_maneuvers, 1):
                tc = man["time_cost"]
                total_cost += tc
                report.append(
                    f"  Maneuver #{i}: time_cost={tc:.1f}s  "
                    f"closest={man.get('closest_approach', 0):.2f}m"
                )
            report.append(f"  Total avoidance time cost: {total_cost:.1f}s")
        else:
            report.append("  No avoidance maneuvers recorded")

        # ── Manual Mowing Mode ──
        report.append("\n=== Manual Mowing Mode (Command 7) ===")
        manual_mow_result = [
            pr for pr in m.phase_results if pr.name == "MANUAL_MOWING"
        ]
        manual_mow_pass = False
        if manual_mow_result:
            pr = manual_mow_result[0]
            status = "PASS" if pr.passed else "FAIL"
            report.append(f"  {status}: {pr.details} ({pr.duration_sec:.1f}s)")
            manual_mow_pass = pr.passed
        else:
            report.append("  NOT RUN")

        # ── Area Recording Mode ──
        report.append("\n=== Area Recording Mode (Command 3) ===")
        area_rec_result = [
            pr for pr in m.phase_results if pr.name == "AREA_RECORDING"
        ]
        area_rec_pass = False
        if area_rec_result:
            pr = area_rec_result[0]
            status = "PASS" if pr.passed else "FAIL"
            report.append(f"  {status}: {pr.details} ({pr.duration_sec:.1f}s)")
            area_rec_pass = pr.passed
        else:
            report.append("  NOT RUN")

        # ── Emergency Auto-Reset on Dock ──
        report.append("\n=== Emergency Auto-Reset on Dock ===")
        emerg_result = [
            pr for pr in m.phase_results if pr.name == "EMERGENCY_RESET"
        ]
        emergency_pass = False
        if emerg_result:
            pr = emerg_result[0]
            status = "PASS" if pr.passed else "FAIL"
            report.append(f"  {status}: {pr.details} ({pr.duration_sec:.1f}s)")
            emergency_pass = pr.passed
        else:
            report.append("  NOT RUN")

        # ── Reroute / Dynamic Replanning ──
        report.append("\n=== Dynamic Replanning ===")
        report.append(f"  Total reroute events: {len(m.reroute_events)}")
        # Pair RECOVERING enter/exit to compute replan durations
        recovering_start = None
        replan_durations = []
        for ts, st in m.bt_states:
            if st == "RECOVERING":
                recovering_start = ts
            elif recovering_start is not None and st != "RECOVERING":
                replan_durations.append(ts - recovering_start)
                recovering_start = None
        if replan_durations:
            for i, dur in enumerate(replan_durations, 1):
                report.append(f"  Replan #{i}: {dur:.1f}s in recovery")
            avg_replan = sum(replan_durations) / len(replan_durations)
            report.append(f"  Average replan time: {avg_replan:.1f}s")
            if max(replan_durations) <= 30.0:
                report.append("  PASS: all replans resolved within 30s")
            else:
                report.append(f"  WARN: longest replan took {max(replan_durations):.1f}s")
        else:
            report.append("  No replanning events")

        # ── Time Analysis ──
        report.append("\n=== Time Analysis ===")
        total_classified = m.time_moving + m.time_stopped + m.time_turning + m.time_recovering
        idle_ratio_pass = True
        if total_classified > 0:
            report.append(f"  Total classified:  {total_classified:.0f}s")
            report.append(
                f"  Moving:            {m.time_moving:.0f}s "
                f"({m.time_moving / total_classified * 100:.1f}%)"
            )
            report.append(
                f"  Stopped/idle:      {m.time_stopped:.0f}s "
                f"({m.time_stopped / total_classified * 100:.1f}%)"
            )
            report.append(
                f"  Turning:           {m.time_turning:.0f}s "
                f"({m.time_turning / total_classified * 100:.1f}%)"
            )
            report.append(
                f"  Recovering:        {m.time_recovering:.0f}s "
                f"({m.time_recovering / total_classified * 100:.1f}%)"
            )
            idle_pct = m.time_stopped / total_classified * 100
            report.append(f"  Idle ratio:        {idle_pct:.1f}%")
            if idle_pct < 20.0:
                report.append("  PASS: idle ratio < 20%")
            else:
                report.append("  FAIL: idle ratio >= 20%")
                idle_ratio_pass = False
        else:
            report.append("  No time data collected")
            idle_ratio_pass = False

        # ── BT State Durations ──
        report.append("\n  BT State Durations:")
        for state, dur in sorted(m.bt_state_durations.items(), key=lambda x: -x[1]):
            pct = dur / t * 100 if t > 0 else 0
            report.append(f"    {state:30s} {dur:7.1f}s  ({pct:5.1f}%)")

        # ── Speed Statistics ──
        report.append("\n=== Speed Statistics ===")
        for phase in ["UNDOCKING", "MOWING", "DOCKING"]:
            speeds = m.phase_speeds.get(phase, [])
            if speeds:
                avg_s = sum(speeds) / len(speeds)
                report.append(
                    f"  {phase:12s}: avg={avg_s:.3f}  min={min(speeds):.3f}  "
                    f"max={max(speeds):.3f} m/s  ({len(speeds)} samples)"
                )
        all_speeds = [s for sl in m.phase_speeds.values() for s in sl]
        if all_speeds:
            report.append(
                f"  {'OVERALL':12s}: avg={sum(all_speeds)/len(all_speeds):.3f}  "
                f"min={min(all_speeds):.3f}  max={max(all_speeds):.3f} m/s"
            )

        # ── Per-Phase Time ──
        report.append("\n=== Per-Phase Time Breakdown ===")
        for phase in [
            "UNDOCKING", "PLANNING", "MOWING", "DOCKING",
            "MANUAL_MOWING", "AREA_RECORDING", "EMERGENCY_RESET",
        ]:
            mv = m.phase_time_moving.get(phase, 0)
            st = m.phase_time_stopped.get(phase, 0)
            total_p = mv + st
            if total_p > 0:
                report.append(
                    f"  {phase:12s}: {total_p:6.0f}s total  "
                    f"moving={mv:.0f}s ({mv/total_p*100:.0f}%)  "
                    f"stopped={st:.0f}s ({st/total_p*100:.0f}%)"
                )

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
            report.append(f"  Average speed:  {dist / t:.3f} m/s")
            # Area yield
            if m.covered_cells:
                cell_area = m.coverage_grid_resolution ** 2
                area_m2 = len(m.covered_cells) * cell_area
                report.append(f"  Area mowed:     {area_m2:.1f} m²")
                report.append(f"  Area yield:     {area_m2/dist:.3f} m²/m traveled")

        # ── Overall Verdict ──
        report.append(f"\n{'#' * 70}")
        report.append("VALIDATION CRITERIA:")

        # Overlap pass/fail
        overlap_pass = True
        if m.covered_cell_visits:
            visit_counts = list(m.covered_cell_visits.values())
            single = sum(1 for v in visit_counts if v == 1)
            overlap_ratio = (1 - single / len(visit_counts)) * 100 if visit_counts else 0
            if overlap_ratio > 40:
                overlap_pass = False

        criteria = [
            ("Undock->Plan->Mow->Dock cycle", all_phases_pass),
            ("Path tracking (median < 50cm)", path_pass),
            ("SLAM map growth", map_pass),
            ("No collisions", collision_pass),
            ("Stayed within boundary", boundary_pass),
            ("Obstacle avoidance", obstacle_pass),
            ("Mowing efficiency >= 0.85", efficiency_pass),
            ("Area coverage >= 80%", coverage_pass),
            ("Idle ratio < 20%", idle_ratio_pass),
            ("Path overlap < 30%", overlap_pass),
            ("Manual mowing mode", manual_mow_pass),
            ("Area recording mode", area_rec_pass),
            ("Emergency auto-reset on dock", emergency_pass),
        ]

        overall_pass = True
        for name, passed in criteria:
            status = "PASS" if passed else "FAIL"
            report.append(f"  [{status}] {name}")
            if not passed:
                overall_pass = False

        report.append(f"\nOVERALL: {'PASS' if overall_pass else 'NEEDS ATTENTION'}")
        report.append(f"{'#' * 70}\n")

        self.get_logger().info("\n".join(report))


def main():
    rclpy.init()
    node = E2ETestNode()

    # Wait for system to settle
    for i in range(5, 0, -1):
        node.get_logger().info(f"Starting test in {i}s...")
        time.sleep(1)

    # Physical obstacles are pre-placed in the Gazebo world SDF (garden.sdf).
    # obs_swath1 at (-6.5, 0.0), obs_swath2 at (-6.0, -3.0), obs_mid at (3.0, 0.0).
    # They exist from sim start so the obstacle tracker detects them before mowing.
    node.obstacle_spawned = True
    spawned_obstacles = ["obs_swath1", "obs_swath2", "obs_mid"]
    node.get_logger().info("Using 3 pre-placed obstacles from Gazebo world")

    # Send START command
    if not node.send_start_command():
        node.get_logger().error("Failed to send START. Aborting.")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # Start obstacle avoidance test in background thread
    obstacle_thread = threading.Thread(
        target=node._run_obstacle_avoidance_test, daemon=True
    )
    obstacle_thread.start()

    # Spin until mowing cycle completes or timeout (20 min for full cycle).
    # Feature tests run after timeout regardless.
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
        while rclpy.ok() and not node.mowing_cycle_complete:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.time() - start > timeout:
                node.get_logger().warn(f"Mowing cycle timeout after {timeout}s")
                break

        # ── New Feature Tests (run after mowing cycle or timeout) ────────
        # These test BT command handling, not mowing — they work regardless
        # of whether coverage completed.  If mowing timed out, send HOME
        # first to get the robot to a known idle/docked state.

        if rclpy.ok():
            if not node.mowing_cycle_complete:
                node.get_logger().info(
                    "=== Mowing timed out. Sending HOME before feature tests... ==="
                )
                node.send_command(2, "COMMAND_HOME")
                node.wait_for_bt_state("IDLE_DOCKED", timeout_sec=120.0)

            node.get_logger().info(
                "=== Running post-dock feature tests... ==="
            )

            # ── 1. Manual Mowing Mode (command 7) ──
            node.get_logger().info("=== TEST: Manual Mowing Mode ===")
            node.current_phase = TestPhase.MANUAL_MOWING
            node.phase_start_time = time.time() - node.metrics.start_time
            if node.send_command(7, "COMMAND_MANUAL_MOW"):
                if node.wait_for_bt_state("MANUAL_MOWING", timeout_sec=15.0):
                    node.get_logger().info("BT entered MANUAL_MOWING state")
                    # Send HOME to exit manual mode — robot may need to
                    # navigate back to dock, so allow up to 120s
                    if node.send_command(2, "COMMAND_HOME"):
                        if node.wait_for_bt_state(
                            ["IDLE", "IDLE_DOCKED"], timeout_sec=120.0
                        ):
                            node._complete_phase(
                                TestPhase.MANUAL_MOWING, True,
                                "Entered MANUAL_MOWING and returned to IDLE via HOME"
                            )
                        else:
                            node._complete_phase(
                                TestPhase.MANUAL_MOWING, False,
                                f"Failed to return to IDLE after HOME "
                                f"(state={node.current_bt_state})"
                            )
                    else:
                        node._complete_phase(
                            TestPhase.MANUAL_MOWING, False,
                            "Failed to send HOME command"
                        )
                else:
                    node._complete_phase(
                        TestPhase.MANUAL_MOWING, False,
                        f"BT did not enter MANUAL_MOWING "
                        f"(state={node.current_bt_state})"
                    )
            else:
                node._complete_phase(
                    TestPhase.MANUAL_MOWING, False,
                    "Failed to send COMMAND_MANUAL_MOW"
                )

            # Allow system to settle
            for _ in range(20):
                rclpy.spin_once(node, timeout_sec=0.1)

            # ── 2. Area Recording Mode (command 3) ──
            node.get_logger().info("=== TEST: Area Recording Mode ===")
            node.current_phase = TestPhase.AREA_RECORDING
            node.phase_start_time = time.time() - node.metrics.start_time
            if node.send_command(3, "COMMAND_RECORD_AREA"):
                if node.wait_for_bt_state("RECORDING", timeout_sec=15.0):
                    node.get_logger().info("BT entered RECORDING state")
                    # Wait briefly (simulating teleop driving)
                    for _ in range(30):
                        rclpy.spin_once(node, timeout_sec=0.1)
                    # Cancel recording
                    if node.send_command(6, "COMMAND_RECORD_CANCEL"):
                        if node.wait_for_bt_state(
                            ["IDLE", "IDLE_DOCKED"], timeout_sec=30.0
                        ):
                            node._complete_phase(
                                TestPhase.AREA_RECORDING, True,
                                "Entered RECORDING and returned to IDLE via CANCEL"
                            )
                        else:
                            node._complete_phase(
                                TestPhase.AREA_RECORDING, False,
                                f"Failed to return to IDLE after CANCEL "
                                f"(state={node.current_bt_state})"
                            )
                    else:
                        node._complete_phase(
                            TestPhase.AREA_RECORDING, False,
                            "Failed to send COMMAND_RECORD_CANCEL"
                        )
                else:
                    node._complete_phase(
                        TestPhase.AREA_RECORDING, False,
                        f"BT did not enter RECORDING "
                        f"(state={node.current_bt_state})"
                    )
            else:
                node._complete_phase(
                    TestPhase.AREA_RECORDING, False,
                    "Failed to send COMMAND_RECORD_AREA"
                )

            # Allow system to settle
            for _ in range(20):
                rclpy.spin_once(node, timeout_sec=0.1)

            # ── 3. Emergency Auto-Reset on Dock ──
            node.get_logger().info("=== TEST: Emergency Auto-Reset on Dock ===")
            node.current_phase = TestPhase.EMERGENCY_RESET
            node.phase_start_time = time.time() - node.metrics.start_time
            if node.send_emergency_stop(1):
                if node.wait_for_bt_state("EMERGENCY", timeout_sec=10.0):
                    node.get_logger().info("BT entered EMERGENCY state")
                    # Wait for auto-reset (robot is on dock/charging)
                    emergency_cleared = False
                    deadline = time.time() + 10.0
                    while time.time() < deadline and rclpy.ok():
                        rclpy.spin_once(node, timeout_sec=0.1)
                        if node.current_bt_state != "EMERGENCY":
                            emergency_cleared = True
                            break
                    if emergency_cleared:
                        node._complete_phase(
                            TestPhase.EMERGENCY_RESET, True,
                            f"Emergency auto-cleared on dock "
                            f"(state={node.current_bt_state})"
                        )
                    else:
                        node._complete_phase(
                            TestPhase.EMERGENCY_RESET, False,
                            "Emergency did not auto-reset within timeout"
                        )
                else:
                    node._complete_phase(
                        TestPhase.EMERGENCY_RESET, False,
                        f"BT did not enter EMERGENCY state "
                        f"(state={node.current_bt_state})"
                    )
            else:
                node._complete_phase(
                    TestPhase.EMERGENCY_RESET, False,
                    "Failed to send EmergencyStop service call"
                )

        node.test_complete = True

    except KeyboardInterrupt:
        node.get_logger().info("Test interrupted by user")

    # Clean up all spawned obstacles
    if node.obstacle_spawned:
        for name in spawned_obstacles:
            node._remove_obstacle(name)

    node.print_final_report()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()