# Copyright 2026 Mowgli Project
#
# SPDX-License-Identifier: GPL-3.0

from __future__ import annotations

import argparse
from dataclasses import dataclass, field, replace
from datetime import datetime, timezone
import math
from pathlib import Path
import sys
import time
from typing import Any

from geometry_msgs.msg import TwistStamped
from mowgli_interfaces.msg import (
    AbsolutePose,
    Emergency,
    GnssStatus,
    HighLevelStatus,
    Status,
    WheelTick,
)
from mowgli_interfaces.srv import HighLevelControl
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
import yaml

from .drive_pid_math import (
    DrivePidParams,
    SpeedSample,
    TrialMetrics,
    clamp,
    compute_trial_metrics,
    integrate_distance,
    recommend_drive_pid_params,
    recommend_pid_only_params,
)


# The 8 W profile starts from the theoretical 1964 ticks/m value reduced to
# one hall channel out of three. The motor's 4-pole construction is already
# baked into the theoretical figure, so the live bridge still starts at 1964/3.
PROFILE_PRESETS: dict[str, DrivePidParams] = {
    "yardforce_8w_1964": DrivePidParams(
        ticks_per_meter=655.0,
        wheel_pid_kp=20.0,
        wheel_pid_ki=1000.0,
        wheel_pid_kd=0.0,
        wheel_pid_integral_limit=40.0,
        wheel_pid_pwm_per_mps=450.0,
    ),
    "yardforce_12w_1600": DrivePidParams(
        ticks_per_meter=533.0,
        wheel_pid_kp=18.0,
        wheel_pid_ki=700.0,
        wheel_pid_kd=0.0,
        wheel_pid_integral_limit=30.0,
        wheel_pid_pwm_per_mps=550.0,
    ),
}

PARAMETER_NAMES = (
    "ticks_per_meter",
    "wheel_pid_kp",
    "wheel_pid_ki",
    "wheel_pid_kd",
    "wheel_pid_integral_limit",
    "wheel_pid_pwm_per_mps",
)

HL_CMD_RECORD_AREA = 3
HL_CMD_RECORD_CANCEL = 6
HL_STATE_RECORDING = 3


@dataclass(frozen=True)
class RtkPoseSample:
    time_s: float
    x_m: float
    y_m: float
    horizontal_accuracy_m: float
    fix_valid: bool
    corrections_active: bool
    rtk_mode: int
    mode_label: str


@dataclass(frozen=True)
class TickSample:
    time_s: float
    left_ticks: int
    right_ticks: int


@dataclass
class TrialRecorder:
    name: str
    phase: str
    initial_speed: float
    target_speed: float
    params: DrivePidParams
    command_start_s: float
    ramp_time_s: float
    command_duration_s: float
    ramp_down: bool = True
    speed_samples: list[SpeedSample] = field(default_factory=list)
    rtk_samples: list[RtkPoseSample] = field(default_factory=list)
    tick_samples: list[TickSample] = field(default_factory=list)

    @property
    def command_end_s(self) -> float:
        return self.command_start_s + self.command_duration_s


def _default_backup_path() -> Path:
    return Path.home() / ".ros" / "mowgli_tools" / "drive_pid_last_backup.yaml"


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Controlled drive PID tuning for Mowgli wheel motors.")
    parser.add_argument("--dry-run", action="store_true", help="Print the planned actions without moving the robot.")
    parser.add_argument("--mode", choices=("combined", "ff", "pid"), default="combined",
                        help="Run the legacy combined flow, feed-forward/odometry only, or PID-only auto-tune.")
    parser.add_argument("--profile", choices=("yardforce_8w_1964", "yardforce_12w_1600", "custom"),
                        default="yardforce_8w_1964", help="Starting profile applied before the tests.")
    parser.add_argument("--max-speed", type=float, default=0.3,
                        help="Maximum forward test speed in m/s.")
    parser.add_argument("--test-speed", type=float, default=None,
                        help="Single straight-line test speed for feed-forward mode. Defaults to --max-speed.")
    parser.add_argument("--distance", type=float, default=3.0,
                        help="Straight-line target distance in metres for feed-forward mode.")
    parser.add_argument("--passes", type=int, default=3,
                        help="Number of feed-forward or PID passes to run.")
    parser.add_argument("--duration", type=float, default=5.0,
                        help="Duration of each speed segment in seconds.")
    parser.add_argument("--apply", action="store_true",
                        help="Keep the final recommended parameters live in hardware_bridge.")
    parser.add_argument("--rollback", action="store_true",
                        help="Restore the last saved backup instead of running new trials.")
    parser.add_argument("--output", type=str, default="",
                        help="Optional YAML output path for results and metrics.")
    parser.add_argument("--backup-file", type=str, default=str(_default_backup_path()),
                        help="Path used to save and restore the live parameter backup.")
    parser.add_argument("--cmd-topic", type=str, default="/cmd_vel_teleop",
                        help="TwistStamped topic used for the test commands.")
    parser.add_argument("--hardware-node", type=str, default="hardware_bridge",
                        help="Node name used by the hardware parameter client.")
    parser.add_argument("--rtk-accuracy-threshold", type=float, default=0.05,
                        help="Maximum horizontal accuracy (m) for RTK validation.")
    parser.add_argument("--allow-rtk-float", action="store_true",
                        help="Accept RTK float instead of fixed-only for RTK validation.")
    parser.add_argument("--undock-distance", type=float, default=0.0,
                        help="Reverse distance in metres if the robot starts on the dock.")
    parser.add_argument("--undock-speed", type=float, default=0.16,
                        help="Reverse speed in m/s for undocking.")
    parser.add_argument("--ramp-time", type=float, default=1.0,
                        help="Ramp time in seconds applied at the start and end of each segment.")
    parser.add_argument("--stop-between-tests", type=float, default=1.5,
                        help="Hold zero cmd_vel this many seconds between trials.")
    parser.add_argument("--auto-turn", action="store_true",
                        help="Rotate ~180 degrees between feed-forward passes.")
    parser.add_argument("--turn-direction", choices=("left", "right"), default="right",
                        help="Rotation direction for --auto-turn.")
    parser.add_argument("--turn-rate", type=float, default=0.30,
                        help="Angular speed in rad/s used by the automatic turnaround helper.")
    parser.add_argument("--custom-ticks-per-meter", type=float, default=None)
    parser.add_argument("--custom-kp", type=float, default=None)
    parser.add_argument("--custom-ki", type=float, default=None)
    parser.add_argument("--custom-kd", type=float, default=None)
    parser.add_argument("--custom-integral-limit", type=float, default=None)
    parser.add_argument("--custom-pwm-per-mps", type=float, default=None)
    return parser


class DrivePidTuner(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("tune_drive_pid")
        self._args = args
        self._backup_path = Path(args.backup_file).expanduser()
        self._active_trial: TrialRecorder | None = None
        self._entered_recording = False

        self._latest_status: Status | None = None
        self._latest_emergency: Emergency | None = None
        self._latest_high_level_status: HighLevelStatus | None = None
        self._latest_gnss_status: GnssStatus | None = None
        self._latest_odom_time: float | None = None

        self._parameter_client = AsyncParameterClient(self, args.hardware_node)
        self._high_level_client = self.create_client(
            HighLevelControl,
            "/behavior_tree_node/high_level_control",
        )

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._cmd_pub = self.create_publisher(TwistStamped, args.cmd_topic, reliable_qos)
        self.create_subscription(Status, "/hardware_bridge/status", self._on_status, reliable_qos)
        self.create_subscription(Emergency, "/hardware_bridge/emergency", self._on_emergency, reliable_qos)
        self.create_subscription(
            HighLevelStatus,
            "/behavior_tree_node/high_level_status",
            self._on_high_level_status,
            reliable_qos,
        )
        self.create_subscription(Odometry, "/wheel_odom", self._on_wheel_odom, reliable_qos)
        self.create_subscription(WheelTick, "/wheel_ticks", self._on_wheel_ticks, reliable_qos)
        self.create_subscription(AbsolutePose, "/gps/absolute_pose", self._on_absolute_pose, reliable_qos)
        self.create_subscription(GnssStatus, "/gps/status", self._on_gnss_status, reliable_qos)
        self.create_subscription(Odometry, "/odometry/filtered_map", lambda _: None, sensor_qos)

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _on_status(self, msg: Status) -> None:
        self._latest_status = msg

    def _on_emergency(self, msg: Emergency) -> None:
        self._latest_emergency = msg

    def _on_high_level_status(self, msg: HighLevelStatus) -> None:
        self._latest_high_level_status = msg

    def _on_gnss_status(self, msg: GnssStatus) -> None:
        self._latest_gnss_status = msg

    def _on_wheel_odom(self, msg: Odometry) -> None:
        now_s = time.monotonic()
        self._latest_odom_time = now_s
        if self._active_trial is None:
            return
        self._active_trial.speed_samples.append(
            SpeedSample(time_s=now_s, speed_mps=float(msg.twist.twist.linear.x))
        )

    def _on_wheel_ticks(self, msg: WheelTick) -> None:
        if self._active_trial is None:
            return
        self._active_trial.tick_samples.append(
            TickSample(
                time_s=time.monotonic(),
                left_ticks=int(msg.wheel_ticks_rl),
                right_ticks=int(msg.wheel_ticks_rr),
            )
        )

    def _on_absolute_pose(self, msg: AbsolutePose) -> None:
        if self._active_trial is None:
            return
        status = self._latest_gnss_status
        rtk_mode = GnssStatus.RTK_MODE_UNKNOWN if status is None else int(status.rtk_mode)
        fix_valid = False if status is None else bool(status.fix_valid)
        corrections_active = False if status is None else bool(status.corrections_active)
        mode_label = "UNKNOWN"
        if rtk_mode == GnssStatus.RTK_MODE_FIXED:
            mode_label = "RTK_FIXED"
        elif rtk_mode == GnssStatus.RTK_MODE_FLOAT:
            mode_label = "RTK_FLOAT"
        self._active_trial.rtk_samples.append(
            RtkPoseSample(
                time_s=time.monotonic(),
                x_m=float(msg.pose.pose.position.x),
                y_m=float(msg.pose.pose.position.y),
                horizontal_accuracy_m=(
                    float("inf")
                    if status is None
                    else float(status.horizontal_accuracy_m)
                ),
                fix_valid=fix_valid,
                corrections_active=corrections_active,
                rtk_mode=rtk_mode,
                mode_label=mode_label,
            )
        )

    # ------------------------------------------------------------------
    # Public entrypoint
    # ------------------------------------------------------------------

    def run(self) -> int:
        if self._args.rollback:
            return self._run_rollback()

        self._wait_for_initial_state()
        current_params = self._get_drive_pid_params()
        self._write_backup(current_params)
        working_params = self._resolve_starting_params(current_params)
        if self._args.dry_run:
            self._print_dry_run(current_params, working_params)
            self._write_result_file(
                mode=self._args.mode,
                current_params=current_params,
                starting_params=working_params,
                proposed_params=working_params,
                applied_live=False,
                trials=[],
                reasons=["Dry-run only."],
            )
            return 0

        feedforward_trials: list[TrialMetrics] = []
        response_trials: list[TrialMetrics] = []
        proposed_params = working_params
        reasons: list[str] = []
        keep_final_live = False
        session_started = False

        try:
            self._ensure_safe_to_start()
            session_started = True
            self._enter_recording_if_needed()
            if self._latest_status is not None and self._latest_status.is_charging:
                self._undock_if_requested()

            self._apply_drive_pid_params(working_params)
            if self._args.mode == "ff":
                proposed_params, feedforward_trials, reasons = self._run_feedforward_session(working_params)
            elif self._args.mode == "pid":
                proposed_params, response_trials, reasons = self._run_pid_session(working_params)
            else:
                (
                    proposed_params,
                    feedforward_trials,
                    response_trials,
                    reasons,
                ) = self._run_combined_session(working_params)
            if self._args.apply:
                self._apply_drive_pid_params(proposed_params)
                keep_final_live = True
            self._print_summary(current_params, working_params, proposed_params, feedforward_trials, response_trials,
                                keep_final_live, reasons)
            return 0
        finally:
            try:
                self._stop_robot()
            finally:
                if session_started:
                    if not keep_final_live:
                        try:
                            self._apply_drive_pid_params(current_params)
                        except Exception as exc:  # pragma: no cover - best effort cleanup
                            self.get_logger().error(f"Failed to restore original parameters: {exc}")
                    try:
                        self._cancel_recording_if_needed()
                    except Exception as exc:  # pragma: no cover - best effort cleanup
                        self.get_logger().error(f"Failed to cancel recording state: {exc}")
                self._write_result_file(
                    mode=self._args.mode,
                    current_params=current_params,
                    starting_params=working_params,
                    proposed_params=proposed_params,
                    applied_live=keep_final_live,
                    trials=[*feedforward_trials, *response_trials],
                    reasons=reasons,
                )

    # ------------------------------------------------------------------
    # High-level flow helpers
    # ------------------------------------------------------------------

    def _run_rollback(self) -> int:
        self._wait_for_initial_state()
        backup = self._load_backup()
        self.get_logger().info(f"Restoring backup from {self._backup_path}")
        if self._args.dry_run:
            print(f"Rollback dry-run: would restore {backup.to_dict()} from {self._backup_path}")
            return 0
        self._apply_drive_pid_params(backup)
        self._stop_robot()
        print(f"Rollback applied from {self._backup_path}")
        return 0

    def _wait_for_initial_state(self) -> None:
        if not self._parameter_client.wait_for_services(timeout_sec=10.0):
            raise RuntimeError(f"Parameter services for {self._args.hardware_node} are not ready.")
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._latest_status is not None and self._latest_emergency is not None and self._latest_odom_time is not None:
                return
        raise RuntimeError("Timed out waiting for hardware_bridge status, emergency, and wheel odom.")

    def _ensure_safe_to_start(self) -> None:
        if self._latest_emergency is None or self._latest_status is None:
            raise RuntimeError("Robot status is unavailable.")
        if self._latest_emergency.active_emergency or self._latest_emergency.latched_emergency:
            raise RuntimeError(f"Emergency active: {self._latest_emergency.reason}")
        if self._latest_status.is_charging and self._args.undock_distance <= 0.0:
            raise RuntimeError(
                "Robot is on the dock. Re-run with --undock-distance 2.0 (or another safe value)."
            )

    def _resolve_starting_params(self, current_params: DrivePidParams) -> DrivePidParams:
        if self._args.profile == "custom":
            params = current_params
        else:
            params = PROFILE_PRESETS[self._args.profile]
        updates: dict[str, float] = {}
        custom_fields = {
            "ticks_per_meter": self._args.custom_ticks_per_meter,
            "wheel_pid_kp": self._args.custom_kp,
            "wheel_pid_ki": self._args.custom_ki,
            "wheel_pid_kd": self._args.custom_kd,
            "wheel_pid_integral_limit": self._args.custom_integral_limit,
            "wheel_pid_pwm_per_mps": self._args.custom_pwm_per_mps,
        }
        for key, value in custom_fields.items():
            if value is not None:
                updates[key] = float(value)
        if updates:
            params = replace(params, **updates)
        return params

    def _phase_speeds(self) -> list[float]:
        candidates = [0.10, 0.20, 0.30, 0.50]
        return [speed for speed in candidates if speed <= self._args.max_speed + 1e-6]

    def _response_speeds(self, feedforward_speeds: list[float]) -> list[float]:
        response = [speed for speed in feedforward_speeds if speed >= 0.20]
        if not response:
            response = [feedforward_speeds[-1]]
        return response

    def _feedforward_test_speed(self) -> float:
        if self._args.test_speed is not None:
            return float(self._args.test_speed)
        return float(self._args.max_speed)

    def _feedforward_duration(self, target_speed: float) -> float:
        return max(float(self._args.duration), float(self._args.distance) / max(abs(target_speed), 1e-6) + float(self._args.ramp_time))

    def _pid_step_sequence(self) -> list[tuple[float, float]]:
        step_up = min(0.20, float(self._args.max_speed))
        cruise = min(max(step_up, 0.30), float(self._args.max_speed))
        settle = min(0.10, cruise)
        return [
            (0.0, step_up),
            (step_up, cruise),
            (cruise, settle),
            (settle, 0.0),
        ]

    def _run_combined_session(
        self,
        working_params: DrivePidParams,
    ) -> tuple[DrivePidParams, list[TrialMetrics], list[TrialMetrics], list[str]]:
        feedforward_trials: list[TrialMetrics] = []
        response_trials: list[TrialMetrics] = []
        reasons: list[str] = []

        speeds = self._phase_speeds()
        self.get_logger().info(
            f"Feed-forward phase at speeds: {', '.join(f'{s:.2f}' for s in speeds)}"
        )
        for index, speed in enumerate(speeds, start=1):
            feedforward_trials.append(
                self._run_speed_trial(
                    name=f"ff_{index}_{speed:.2f}mps",
                    phase="feedforward",
                    initial_speed=0.0,
                    target_speed=speed,
                    params=working_params,
                )
            )

        provisional_params, provisional_reasons = recommend_drive_pid_params(
            working_params,
            feedforward_trials,
            [],
        )
        reasons.extend(provisional_reasons)
        self._apply_drive_pid_params(provisional_params)

        response_speeds = self._response_speeds(speeds)
        self.get_logger().info(
            f"Response phase at speeds: {', '.join(f'{s:.2f}' for s in response_speeds)}"
        )
        for index, speed in enumerate(response_speeds, start=1):
            response_trials.append(
                self._run_speed_trial(
                    name=f"resp_{index}_{speed:.2f}mps",
                    phase="response",
                    initial_speed=0.0,
                    target_speed=speed,
                    params=provisional_params,
                )
            )

        proposed_params, final_reasons = recommend_drive_pid_params(
            working_params,
            feedforward_trials,
            response_trials,
        )
        reasons.extend(final_reasons)
        return proposed_params, feedforward_trials, response_trials, reasons

    def _run_feedforward_session(
        self,
        working_params: DrivePidParams,
    ) -> tuple[DrivePidParams, list[TrialMetrics], list[str]]:
        trials: list[TrialMetrics] = []
        reasons: list[str] = []
        params = working_params
        target_speed = self._feedforward_test_speed()
        duration_s = self._feedforward_duration(target_speed)

        self.get_logger().info(
            f"Feed-forward calibration: {self._args.passes} pass(es), "
            f"{self._args.distance:.2f} m target distance, {target_speed:.2f} m/s target speed."
        )
        for pass_index in range(self._args.passes):
            trial = self._run_speed_trial(
                name=f"ff_pass_{pass_index + 1}_{target_speed:.2f}mps",
                phase="feedforward",
                initial_speed=0.0,
                target_speed=target_speed,
                params=params,
                duration_s=duration_s,
            )
            trials.append(trial)

            measured_speed = (
                trial.ground_speed_mean
                if trial.rtk_accepted and trial.ground_speed_mean is not None
                else trial.measured_speed_mean
            )
            next_ticks = params.ticks_per_meter
            if trial.rtk_accepted and trial.rtk_distance_m is not None and trial.odom_distance_m is not None:
                next_ticks = clamp(
                    params.ticks_per_meter * (trial.odom_distance_m / max(trial.rtk_distance_m, 1e-6)),
                    100.0,
                    2500.0,
                )
                reasons.append(
                    f"Pass {pass_index + 1}: ticks_per_meter -> {next_ticks:.2f} from odom/reference "
                    f"{trial.odom_distance_m:.3f}/{trial.rtk_distance_m:.3f} m."
                )
            elif self._latest_gnss_status is not None:
                raise RuntimeError(
                    "RTK/GPS was present but no trustworthy ground-distance estimate was accepted during the feed-forward pass."
                )
            else:
                reasons.append(
                    f"Pass {pass_index + 1}: RTK unavailable, leaving ticks_per_meter unchanged."
                )

            if measured_speed <= 0.02 or trial.stall_detected:
                raise RuntimeError(f"Feed-forward pass {pass_index + 1} stalled or produced near-zero speed.")
            next_pwm = clamp(
                params.wheel_pid_pwm_per_mps * (target_speed / max(measured_speed, 1e-6)),
                50.0,
                600.0,
            )
            reasons.append(
                f"Pass {pass_index + 1}: wheel_pid_pwm_per_mps -> {next_pwm:.2f} from target/measured "
                f"{target_speed:.3f}/{measured_speed:.3f} m/s."
            )
            if trial.oscillation_detected:
                raise RuntimeError(f"Feed-forward pass {pass_index + 1} showed strong oscillation.")

            params = replace(
                params,
                ticks_per_meter=next_ticks,
                wheel_pid_pwm_per_mps=next_pwm,
            )
            if pass_index < self._args.passes - 1:
                self._apply_drive_pid_params(params)
                if self._args.auto_turn:
                    self._turn_around()
                else:
                    self._hold_zero(self._args.stop_between_tests)
        return params, trials, reasons

    def _run_pid_session(
        self,
        working_params: DrivePidParams,
    ) -> tuple[DrivePidParams, list[TrialMetrics], list[str]]:
        trials: list[TrialMetrics] = []
        reasons: list[str] = []
        params = working_params
        sequence = self._pid_step_sequence()
        self.get_logger().info(
            "PID auto-tune step sequence: "
            + ", ".join(f"{start:.2f}->{target:.2f} m/s" for start, target in sequence)
        )

        for pass_index in range(self._args.passes):
            pass_trials: list[TrialMetrics] = []
            for step_index, (initial_speed, target_speed) in enumerate(sequence):
                is_last_step = step_index == len(sequence) - 1
                trial = self._run_speed_trial(
                    name=f"pid_pass_{pass_index + 1}_step_{step_index + 1}",
                    phase="pid",
                    initial_speed=initial_speed,
                    target_speed=target_speed,
                    params=params,
                    ramp_down=is_last_step,
                    hold_zero_after=is_last_step,
                    stop_after=is_last_step,
                )
                pass_trials.append(trial)
            trials.extend(pass_trials)

            params, pass_reasons = recommend_pid_only_params(params, pass_trials)
            reasons.extend(f"Pass {pass_index + 1}: {reason}" for reason in pass_reasons)
            if pass_index < self._args.passes - 1:
                self._apply_drive_pid_params(params)
                self._hold_zero(self._args.stop_between_tests)
        return params, trials, reasons

    def _undock_if_requested(self) -> None:
        distance = float(self._args.undock_distance)
        speed = abs(float(self._args.undock_speed))
        if distance <= 0.0:
            raise RuntimeError("Robot is charging and no undock distance was provided.")
        duration = distance / max(speed, 1e-6)
        self.get_logger().warn(
            f"Robot is charging, reversing {distance:.2f} m at {speed:.2f} m/s to leave the dock."
        )
        self._drive_for_duration(vx=-speed, duration_s=duration)
        self._hold_zero(self._args.stop_between_tests + 1.0)
        if self._latest_status is not None and self._latest_status.is_charging:
            raise RuntimeError("Robot still reports charging after the undock reverse.")

    # ------------------------------------------------------------------
    # Parameter handling
    # ------------------------------------------------------------------

    def _get_drive_pid_params(self) -> DrivePidParams:
        future = self._parameter_client.get_parameters(list(PARAMETER_NAMES))
        response = self._wait_for_future(future, timeout_s=10.0, description="get_parameters")
        values = {
            name: response.values[index].double_value
            for index, name in enumerate(PARAMETER_NAMES)
        }
        return DrivePidParams.from_mapping(values)

    def _apply_drive_pid_params(self, params: DrivePidParams) -> None:
        ros_params = [
            Parameter(name, value=value)
            for name, value in params.to_dict().items()
        ]
        future = self._parameter_client.set_parameters(ros_params)
        response = self._wait_for_future(future, timeout_s=10.0, description="set_parameters")
        failures = [
            result.reason or "unknown reason"
            for result in response.results
            if not result.successful
        ]
        if failures:
            raise RuntimeError(f"Failed to apply drive PID parameters: {failures}")

    def _write_backup(self, params: DrivePidParams) -> None:
        self._backup_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "saved_at": _now_iso(),
            "node": self._args.hardware_node,
            "parameters": params.to_dict(),
        }
        self._backup_path.write_text(yaml.safe_dump(payload, sort_keys=True), encoding="utf-8")
        self.get_logger().info(f"Saved parameter backup to {self._backup_path}")

    def _load_backup(self) -> DrivePidParams:
        if not self._backup_path.exists():
            raise RuntimeError(f"Backup file not found: {self._backup_path}")
        payload = yaml.safe_load(self._backup_path.read_text(encoding="utf-8")) or {}
        parameters = payload.get("parameters")
        if not isinstance(parameters, dict):
            raise RuntimeError(f"Invalid backup file: {self._backup_path}")
        return DrivePidParams.from_mapping(parameters)

    # ------------------------------------------------------------------
    # Motion helpers
    # ------------------------------------------------------------------

    def _enter_recording_if_needed(self) -> None:
        if self._latest_high_level_status is not None and self._latest_high_level_status.state == HL_STATE_RECORDING:
            return
        if not self._high_level_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("/behavior_tree_node/high_level_control is unavailable.")
        request = HighLevelControl.Request()
        request.command = HL_CMD_RECORD_AREA
        future = self._high_level_client.call_async(request)
        response = self._wait_for_future(future, timeout_s=10.0, description="enter_recording")
        if not response.success:
            raise RuntimeError("behavior_tree_node rejected the RECORDING command.")
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._latest_high_level_status is not None and self._latest_high_level_status.state == HL_STATE_RECORDING:
                self._entered_recording = True
                return
        raise RuntimeError("Timed out waiting for behavior_tree_node to enter RECORDING.")

    def _cancel_recording_if_needed(self) -> None:
        if not self._entered_recording:
            return
        if not self._high_level_client.wait_for_service(timeout_sec=2.0):
            return
        request = HighLevelControl.Request()
        request.command = HL_CMD_RECORD_CANCEL
        future = self._high_level_client.call_async(request)
        self._wait_for_future(future, timeout_s=5.0, description="cancel_recording")
        self._entered_recording = False

    def _run_speed_trial(
        self,
        *,
        name: str,
        phase: str,
        initial_speed: float,
        target_speed: float,
        params: DrivePidParams,
        duration_s: float | None = None,
        ramp_down: bool = True,
        hold_zero_after: bool = True,
        stop_after: bool = True,
    ) -> TrialMetrics:
        command_duration_s = float(duration_s if duration_s is not None else self._args.duration)
        self.get_logger().info(
            f"Trial {name}: {initial_speed:.2f} -> {target_speed:.2f} m/s for {command_duration_s:.1f} s"
        )
        recorder = TrialRecorder(
            name=name,
            phase=phase,
            initial_speed=initial_speed,
            target_speed=target_speed,
            params=params,
            command_start_s=time.monotonic(),
            ramp_time_s=min(self._args.ramp_time, max(0.35, command_duration_s / 3.0)),
            command_duration_s=command_duration_s,
            ramp_down=ramp_down,
        )
        self._active_trial = recorder
        try:
            self._drive_segment(recorder)
            if hold_zero_after:
                self._hold_zero(self._args.stop_between_tests)
        finally:
            self._active_trial = None
            if stop_after:
                self._stop_robot()
        metrics = self._finalize_trial(recorder)
        ground_summary = "n/a" if metrics.ground_speed_mean is None else f"{metrics.ground_speed_mean:.3f} m/s"
        self.get_logger().info(
            f"Trial {metrics.name} summary: wheel={metrics.measured_speed_mean:.3f} m/s "
            f"ground={ground_summary} overshoot={metrics.overshoot:.3f} "
            f"stall={metrics.stall_detected} osc={metrics.oscillation_detected}"
        )
        return metrics

    def _drive_segment(self, recorder: TrialRecorder) -> None:
        rate_s = 0.05
        deadline = recorder.command_start_s + recorder.command_duration_s
        while time.monotonic() < deadline:
            now_s = time.monotonic()
            elapsed = now_s - recorder.command_start_s
            self._check_live_trial_safety(recorder, now_s, elapsed)
            vx = self._segment_speed_target(recorder, elapsed)
            self._publish_cmd(vx=vx, wz=0.0)
            rclpy.spin_once(self, timeout_sec=rate_s)
        self._stop_robot()

    def _segment_speed_target(self, recorder: TrialRecorder, elapsed_s: float) -> float:
        ramp = max(0.0, recorder.ramp_time_s)
        initial = recorder.initial_speed
        target = recorder.target_speed
        if ramp > 0.0 and elapsed_s < ramp:
            alpha = clamp(elapsed_s / ramp, 0.0, 1.0)
            return initial + (target - initial) * alpha
        if recorder.ramp_down:
            end_ramp_start = max(ramp, recorder.command_duration_s - ramp)
            if ramp > 0.0 and elapsed_s > end_ramp_start:
                down_elapsed = clamp((elapsed_s - end_ramp_start) / ramp, 0.0, 1.0)
                return target * (1.0 - down_elapsed)
        return target

    def _check_live_trial_safety(self, recorder: TrialRecorder, now_s: float, elapsed_s: float) -> None:
        if self._latest_emergency is not None and (
            self._latest_emergency.active_emergency or self._latest_emergency.latched_emergency
        ):
            raise RuntimeError(f"Emergency asserted during {recorder.name}: {self._latest_emergency.reason}")
        if self._latest_odom_time is None or now_s - self._latest_odom_time > 0.6:
            raise RuntimeError(f"Lost /wheel_odom updates during {recorder.name}.")
        if recorder.phase == "feedforward" and self._latest_gnss_status is not None:
            gnss = self._latest_gnss_status
            require_fixed = not self._args.allow_rtk_float
            mode_ok = gnss.rtk_mode == GnssStatus.RTK_MODE_FIXED
            if not require_fixed:
                mode_ok = gnss.rtk_mode in (GnssStatus.RTK_MODE_FIXED, GnssStatus.RTK_MODE_FLOAT)
            if not bool(gnss.fix_valid) or not bool(gnss.corrections_active) or not mode_ok:
                raise RuntimeError(f"Lost RTK/GPS validity during {recorder.name}.")
        recent = [sample for sample in recorder.speed_samples if now_s - sample.time_s <= 0.8]
        if elapsed_s >= recorder.ramp_time_s + 0.8 and len(recent) >= 5:
            recent_mean = sum(sample.speed_mps for sample in recent) / len(recent)
            expected_speed = max(abs(recorder.initial_speed), abs(recorder.target_speed))
            if expected_speed >= 0.10 and recent_mean < max(0.02, 0.20 * expected_speed):
                raise RuntimeError(f"Live stall detected during {recorder.name}.")
            errors = [recorder.target_speed - sample.speed_mps for sample in recent]
            zero_crossings = 0
            previous_sign = 0
            for error in errors:
                if abs(error) < 0.01:
                    continue
                sign = 1 if error > 0.0 else -1
                if previous_sign and sign != previous_sign:
                    zero_crossings += 1
                previous_sign = sign
            if len(recent) >= 8 and zero_crossings >= 5:
                raise RuntimeError(f"Live oscillation detected during {recorder.name}.")

    def _turn_around(self) -> None:
        turn_rate = clamp(abs(float(self._args.turn_rate)), 0.10, 0.35)
        wz = turn_rate if self._args.turn_direction == "left" else -turn_rate
        duration = math.pi / max(turn_rate, 1e-6)
        self.get_logger().info(
            f"Automatic turnaround: rotating {self._args.turn_direction} at {turn_rate:.2f} rad/s for {duration:.1f} s."
        )
        self._command_for_duration(vx=0.0, wz=wz, duration_s=duration, ramp_s=min(0.8, duration / 4.0))
        self._hold_zero(self._args.stop_between_tests)

    def _hold_zero(self, duration_s: float) -> None:
        deadline = time.monotonic() + max(0.0, duration_s)
        while time.monotonic() < deadline:
            self._publish_cmd(vx=0.0, wz=0.0)
            rclpy.spin_once(self, timeout_sec=0.05)

    def _drive_for_duration(self, *, vx: float, duration_s: float) -> None:
        self._command_for_duration(vx=vx, wz=0.0, duration_s=duration_s, ramp_s=min(0.6, duration_s / 4.0))

    def _command_for_duration(self, *, vx: float, wz: float, duration_s: float, ramp_s: float = 0.0) -> None:
        deadline = time.monotonic() + duration_s
        start_s = time.monotonic()
        while time.monotonic() < deadline:
            if self._latest_emergency is not None and (
                self._latest_emergency.active_emergency or self._latest_emergency.latched_emergency
            ):
                raise RuntimeError(f"Emergency asserted during helper motion: {self._latest_emergency.reason}")
            now_s = time.monotonic()
            elapsed_s = now_s - start_s
            remaining_s = max(0.0, deadline - now_s)
            scale = 1.0
            if ramp_s > 0.0:
                scale = min(scale, clamp(elapsed_s / ramp_s, 0.0, 1.0))
                scale = min(scale, clamp(remaining_s / ramp_s, 0.0, 1.0))
            self._publish_cmd(vx=vx * scale, wz=wz * scale)
            rclpy.spin_once(self, timeout_sec=0.05)
        self._stop_robot()

    def _publish_cmd(self, *, vx: float, wz: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_footprint"
        msg.twist.linear.x = float(vx)
        msg.twist.angular.z = float(wz)
        self._cmd_pub.publish(msg)

    def _stop_robot(self) -> None:
        for _ in range(5):
            self._publish_cmd(vx=0.0, wz=0.0)
            rclpy.spin_once(self, timeout_sec=0.02)

    # ------------------------------------------------------------------
    # Trial post-processing
    # ------------------------------------------------------------------

    def _finalize_trial(self, recorder: TrialRecorder) -> TrialMetrics:
        steady_start = recorder.command_start_s + recorder.ramp_time_s
        steady_end = recorder.command_end_s
        if recorder.ramp_down:
            steady_end = recorder.command_end_s - max(0.5, recorder.ramp_time_s * 0.5)
        speed_samples = [
            sample for sample in recorder.speed_samples
            if steady_start <= sample.time_s <= steady_end
        ]
        response_samples = [
            sample for sample in recorder.speed_samples
            if recorder.command_start_s <= sample.time_s <= steady_end
        ]
        if len(speed_samples) < 3:
            speed_samples = recorder.speed_samples
        if len(response_samples) < 3:
            response_samples = recorder.speed_samples
        if not speed_samples:
            raise RuntimeError(f"No /wheel_odom samples captured during {recorder.name}.")

        odom_distance_m = integrate_distance(speed_samples)
        ticks_seen, left_ticks_seen, right_ticks_seen = self._ticks_seen(recorder.tick_samples)
        ground_speed_mean, rtk_distance_m, notes = self._compute_rtk_metrics(
            recorder=recorder,
            steady_start=steady_start,
            steady_end=steady_end,
        )
        return compute_trial_metrics(
            name=recorder.name,
            phase=recorder.phase,
            target_speed=recorder.target_speed,
            speed_samples=speed_samples,
            response_samples=response_samples,
            ticks_seen=ticks_seen,
            left_ticks_seen=left_ticks_seen,
            right_ticks_seen=right_ticks_seen,
            params_used=recorder.params,
            ground_speed_mean=ground_speed_mean,
            odom_distance_m=odom_distance_m,
            rtk_distance_m=rtk_distance_m,
            notes=notes,
        )

    def _ticks_seen(self, samples: list[TickSample]) -> tuple[int, int, int]:
        if len(samples) < 2:
            return 0, 0, 0
        first = samples[0]
        last = samples[-1]
        left_delta = abs(last.left_ticks - first.left_ticks)
        right_delta = abs(last.right_ticks - first.right_ticks)
        average_delta = int(round((left_delta + right_delta) * 0.5))
        return average_delta, int(left_delta), int(right_delta)

    def _compute_rtk_metrics(
        self,
        *,
        recorder: TrialRecorder,
        steady_start: float,
        steady_end: float,
    ) -> tuple[float | None, float | None, list[str]]:
        notes: list[str] = []
        if self._latest_gnss_status is None:
            notes.append("/gps/status absent, RTK validation disabled.")
            return None, None, notes

        accepted: list[RtkPoseSample] = []
        require_fixed = not self._args.allow_rtk_float
        for sample in recorder.rtk_samples:
            if not (steady_start <= sample.time_s <= steady_end):
                continue
            if not sample.fix_valid:
                continue
            if not sample.corrections_active:
                continue
            if (
                not math.isfinite(sample.horizontal_accuracy_m)
                or sample.horizontal_accuracy_m > self._args.rtk_accuracy_threshold
            ):
                continue
            if require_fixed and sample.rtk_mode != GnssStatus.RTK_MODE_FIXED:
                continue
            if not require_fixed and sample.rtk_mode not in (GnssStatus.RTK_MODE_FIXED, GnssStatus.RTK_MODE_FLOAT):
                continue
            accepted.append(sample)

        if len(accepted) < 2:
            notes.append("Not enough RTK-accepted pose samples in the steady-state window.")
            return None, None, notes

        if recorder.target_speed < 0.12:
            notes.append("Target speed too low for reliable RTK distance validation.")
            return None, None, notes

        jump_limit_m = 0.50
        jump_speed_limit_mps = 1.2
        for previous, current in zip(accepted[:-1], accepted[1:]):
            dt = current.time_s - previous.time_s
            if dt <= 0.0:
                continue
            dx = current.x_m - previous.x_m
            dy = current.y_m - previous.y_m
            distance = (dx * dx + dy * dy) ** 0.5
            if distance > jump_limit_m or distance / dt > jump_speed_limit_mps:
                notes.append("RTK jump detected, rejecting RTK metrics for this trial.")
                return None, None, notes

        start = accepted[0]
        end = accepted[-1]
        dt = end.time_s - start.time_s
        if dt <= 0.5:
            notes.append("RTK steady-state time span too short.")
            return None, None, notes
        dx = end.x_m - start.x_m
        dy = end.y_m - start.y_m
        distance = (dx * dx + dy * dy) ** 0.5
        if distance < 1.0:
            notes.append("RTK segment too short for a stable distance estimate.")
            return None, None, notes
        return distance / dt, distance, notes

    # ------------------------------------------------------------------
    # Reporting
    # ------------------------------------------------------------------

    def _print_dry_run(self, current_params: DrivePidParams, working_params: DrivePidParams) -> None:
        print("=== DRIVE PID TUNER DRY RUN ===")
        print(f"mode: {self._args.mode}")
        print(f"profile: {self._args.profile}")
        print(f"cmd topic: {self._args.cmd_topic}")
        print(f"backup file: {self._backup_path}")
        print(f"feedforward speeds: {[f'{s:.2f}' for s in self._phase_speeds()]}")
        if self._latest_status is not None and self._latest_status.is_charging:
            print(f"robot is charging: yes, requested undock_distance={self._args.undock_distance:.2f} m")
        print(f"current params: {current_params.to_dict()}")
        print(f"starting params: {working_params.to_dict()}")

    def _print_summary(
        self,
        current_params: DrivePidParams,
        starting_params: DrivePidParams,
        proposed_params: DrivePidParams,
        feedforward_trials: list[TrialMetrics],
        response_trials: list[TrialMetrics],
        applied_live: bool,
        reasons: list[str],
    ) -> None:
        print("\n=== DRIVE PID TUNER SUMMARY ===")
        print(f"mode: {self._args.mode}")
        print(f"profile: {self._args.profile}")
        print(f"backup file: {self._backup_path}")
        print(f"current params:  {current_params.to_dict()}")
        print(f"starting params: {starting_params.to_dict()}")
        print(f"proposed params: {proposed_params.to_dict()}")
        print(f"applied live: {'yes' if applied_live else 'no, original params restored'}")
        print("\nTrials:")
        for trial in [*feedforward_trials, *response_trials]:
            ground = "n/a" if trial.ground_speed_mean is None else f"{trial.ground_speed_mean:.3f}"
            print(
                f"  {trial.name}: target={trial.target_speed:.2f} "
                f"wheel_mean={trial.measured_speed_mean:.3f} "
                f"ground_mean={ground} "
                f"overshoot={trial.overshoot:.3f} "
                f"settle={trial.settling_time if trial.settling_time is not None else 'n/a'} "
                f"stall={trial.stall_detected} osc={trial.oscillation_detected}"
            )
        print("\nReasons:")
        for reason in reasons:
            print(f"  - {reason}")
        print("===============================\n")

    def _write_result_file(
        self,
        *,
        mode: str,
        current_params: DrivePidParams,
        starting_params: DrivePidParams,
        proposed_params: DrivePidParams,
        applied_live: bool,
        trials: list[TrialMetrics],
        reasons: list[str],
    ) -> None:
        if not self._args.output:
            return
        output_path = Path(self._args.output).expanduser()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "generated_at": _now_iso(),
            "mode": mode,
            "profile": self._args.profile,
            "hardware_node": self._args.hardware_node,
            "backup_file": str(self._backup_path),
            "cmd_topic": self._args.cmd_topic,
            "applied_live": applied_live,
            "requested_apply": bool(self._args.apply),
            "distance_m": float(self._args.distance),
            "max_speed_mps": float(self._args.max_speed),
            "test_speed_mps": self._args.test_speed,
            "segment_duration_s": float(self._args.duration),
            "passes": int(self._args.passes),
            "auto_turn": bool(self._args.auto_turn),
            "turn_direction": self._args.turn_direction,
            "current_params": current_params.to_dict(),
            "starting_params": starting_params.to_dict(),
            "proposed_params": proposed_params.to_dict(),
            "reasons": reasons,
            "trials": [trial.to_dict() for trial in trials],
        }
        output_path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
        self.get_logger().info(f"Saved tuning result to {output_path}")

    # ------------------------------------------------------------------
    # Utilities
    # ------------------------------------------------------------------

    def _wait_for_future(self, future: Any, *, timeout_s: float, description: str) -> Any:
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            raise RuntimeError(f"Timed out waiting for {description}.")
        return future.result()


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args, ros_args = parser.parse_known_args(argv)
    if args.max_speed <= 0.0:
        parser.error("--max-speed must be positive.")
    if args.max_speed > 0.5:
        parser.error("--max-speed must be <= 0.5 m/s.")
    if args.test_speed is not None and args.test_speed <= 0.0:
        parser.error("--test-speed must be positive.")
    if args.test_speed is not None and args.test_speed > 0.5:
        parser.error("--test-speed must be <= 0.5 m/s.")
    if args.distance < 2.0 or args.distance > 10.0:
        parser.error("--distance must be between 2 and 10 metres.")
    if args.passes < 1:
        parser.error("--passes must be >= 1.")
    if args.duration < 2.0:
        parser.error("--duration must be at least 2 seconds.")
    if args.undock_distance < 0.0:
        parser.error("--undock-distance must be >= 0.")
    if args.undock_speed <= 0.0:
        parser.error("--undock-speed must be positive.")
    if args.turn_rate <= 0.0:
        parser.error("--turn-rate must be positive.")
    if args.rollback and args.apply:
        parser.error("--rollback and --apply are mutually exclusive.")
    if args.mode == "pid" and args.cmd_topic != "/cmd_vel_teleop":
        parser.error("--mode pid requires --cmd-topic /cmd_vel_teleop.")

    rclpy.init(args=ros_args)
    node = DrivePidTuner(args)
    try:
        return node.run()
    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted by operator, stopping and restoring state.")
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
