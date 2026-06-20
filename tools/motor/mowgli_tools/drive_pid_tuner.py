# Copyright 2026 Mowgli Project
#
# SPDX-License-Identifier: GPL-3.0

from __future__ import annotations

import argparse
from dataclasses import dataclass, field, replace
from datetime import datetime, timezone
import math
from pathlib import Path
import statistics
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
    LiveStallDiagnostic,
    SpeedSample,
    TickSample,
    TrialMetrics,
    clamp,
    compute_tick_activity,
    compute_trial_metrics,
    evaluate_live_stall,
    evaluate_live_oscillation_abort,
    finite_or_none,
    integrate_distance,
    recommend_drive_pid_params,
    recommend_pid_only_params,
    resolve_robot_tuning_tier,
    sanitize_finite_data,
)
from .robot_hardware_config import RobotHardwareConfig, extract_robot_hardware_config


# The 8 W profile starts from the theoretical 1964 ticks/m value reduced to
# one hall channel out of three. The motor's 4-pole construction is already
# baked into the theoretical figure, so the live bridge still starts at 1964/3.
_YARDFORCE_12W_1600_PRESET = DrivePidParams(
    ticks_per_meter=533.0,
    wheel_pid_kp=18.0,
    wheel_pid_ki=700.0,
    wheel_pid_kd=0.0,
    wheel_pid_integral_limit=30.0,
    wheel_pid_pwm_per_mps=550.0,
)

PROFILE_PRESETS: dict[str, DrivePidParams] = {
    "yardforce_8w_1964": DrivePidParams(
        ticks_per_meter=655.0,
        wheel_pid_kp=20.0,
        wheel_pid_ki=1000.0,
        wheel_pid_kd=0.0,
        wheel_pid_integral_limit=40.0,
        wheel_pid_pwm_per_mps=450.0,
    ),
    "yardforce_12w_1600": _YARDFORCE_12W_1600_PRESET,
    "yardforce_1600_12w": _YARDFORCE_12W_1600_PRESET,
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
HL_STATE_IDLE = 1
HL_STATE_AUTONOMOUS = 2
HL_STATE_RECORDING = 3

CMD_RATE_HZ = 20.0
CMD_PERIOD_S = 1.0 / CMD_RATE_HZ
RECORDING_ENTRY_TIMEOUT_S = 15.0
RECORDING_EXIT_TIMEOUT_S = 3.0
RECORDING_SETTLE_S = 0.6
ODOM_ACTIVE_SPEED_THRESHOLD_MPS = 0.10
LIVE_STALL_SPEED_WINDOW_S = 0.8


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
    pretrial_last_odom_time: float | None = None
    actual_end_s: float | None = None
    live_oscillation_detected: bool = False
    severe_live_oscillation_detected: bool = False
    warnings: list[str] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)
    speed_samples: list[SpeedSample] = field(default_factory=list)
    rtk_samples: list[RtkPoseSample] = field(default_factory=list)
    tick_samples: list[TickSample] = field(default_factory=list)

    @property
    def command_end_s(self) -> float:
        if self.actual_end_s is not None:
            return self.actual_end_s
        return self.command_start_s + self.command_duration_s


class TrialAbortedError(RuntimeError):
    def __init__(self, message: str, metrics: TrialMetrics | None = None) -> None:
        super().__init__(message)
        self.metrics = metrics


def _default_backup_path() -> Path:
    return Path.home() / ".ros" / "mowgli_tools" / "drive_pid_last_backup.yaml"


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Controlled drive PID tuning for Mowgli wheel motors.")
    parser.add_argument("--dry-run", action="store_true", help="Print the planned actions without moving the robot.")
    parser.add_argument("--mode", choices=("combined", "ff", "pid"), default="combined",
                        help="Run the legacy combined flow, feed-forward/odometry only, or PID-only auto-tune.")
    parser.add_argument("--profile", choices=("yardforce_8w_1964", "yardforce_12w_1600", "yardforce_1600_12w", "custom"),
                        default="yardforce_8w_1964",
                        help="Reference profile for logs/report. It is not applied before the first trial unless --reset-to-profile or --force-profile is set.")
    parser.add_argument("--reset-to-profile", action="store_true",
                        help="Apply the selected preset profile before the first trial instead of starting from the live hardware_bridge parameters.")
    parser.add_argument("--force-profile", action="store_true",
                        help="Alias for --reset-to-profile.")
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
    parser.add_argument("--hardware-config", type=str, default="",
                        help="Optional explicit path to the persisted mowgli_robot.yaml used for robot mass and drivetrain metadata.")
    parser.add_argument("--cmd-topic", type=str, default="",
                        help="TwistStamped topic used for the test commands. Defaults to /cmd_vel_tuning so drive tuning does not share /cmd_vel_teleop.")
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
    parser.add_argument("--odom-timeout", type=float, default=2.0,
                        help="Maximum allowed delay between /wheel_odom samples once motion feedback is flowing.")
    parser.add_argument("--startup-grace", type=float, default=3.0,
                        help="Grace period after the first command before odom-loss checks can fail the trial.")
    live_osc_group = parser.add_mutually_exclusive_group()
    live_osc_group.add_argument("--abort-on-live-oscillation", action="store_true",
                                help="Treat detected live oscillation as a hard abort instead of a calibration warning.")
    live_osc_group.add_argument("--no-abort-on-live-oscillation", action="store_true",
                                help="Never abort on live oscillation; record it as a warning in the trial report.")
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
        self._cmd_topic = self._resolve_cmd_topic(args)
        self._backup_path = Path(args.backup_file).expanduser()
        self._robot_hardware_config = self._load_robot_hardware_config()
        self._tuning_tier = resolve_robot_tuning_tier(
            self._robot_hardware_config.chassis_mass_kg
        )
        self._active_trial: TrialRecorder | None = None
        self._entered_recording = False

        self._latest_status: Status | None = None
        self._latest_emergency: Emergency | None = None
        self._latest_high_level_status: HighLevelStatus | None = None
        self._latest_gnss_status: GnssStatus | None = None
        self._latest_odom_time: float | None = None
        self._latest_wheel_tick_time: float | None = None
        self._latest_wheel_tick_factor: float | None = None
        self._latest_wheel_tick_stamp: str | None = None
        self._failure_message: str | None = None
        self._failure_status_snapshot: dict[str, Any] | None = None

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

        self._cmd_pub = self.create_publisher(TwistStamped, self._cmd_topic, reliable_qos)
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
        self._latest_wheel_tick_time = time.monotonic()
        self._latest_wheel_tick_factor = float(msg.wheel_tick_factor)
        self._latest_wheel_tick_stamp = self._stamp_to_iso(msg.stamp)
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
        self.get_logger().info(
            f"Using cmd_vel topic {self._cmd_topic} for mode {self._args.mode}."
        )
        if self._cmd_topic == "/cmd_vel_tuning":
            self.get_logger().info(
                "Motion path uses the dedicated tuning lane: /cmd_vel_tuning -> twist_mux -> /cmd_vel."
            )
        elif self._cmd_topic == "/cmd_vel_teleop":
            self.get_logger().info(
                "Motion path matches IMU calibration: /cmd_vel_teleop -> twist_mux -> /cmd_vel."
            )
        else:
            self.get_logger().warning(
                "Overriding the dedicated tuning lane; commands will bypass /cmd_vel_tuning."
            )
        if self._robot_hardware_config.chassis_mass_kg is not None:
            self.get_logger().info(
                "Robot mass read from hardware configuration: "
                f"{self._robot_hardware_config.chassis_mass_kg:.2f} kg"
            )
        else:
            self.get_logger().warning(
                "Robot mass is unavailable in the hardware configuration; using the medium internal tuning tier."
            )
        self.get_logger().info(
            f"Internal tuning tier: {self._tuning_tier.report_label}"
        )
        if self._tuning_tier.manual_validation_note is not None:
            self.get_logger().warning(self._tuning_tier.manual_validation_note)
        self._failure_message = None
        self._failure_status_snapshot = None
        if self._args.rollback:
            return self._run_rollback()

        self._wait_for_initial_state()
        current_params = self._get_drive_pid_params()
        self._write_backup(current_params)
        profile_reference_params = self._profile_reference_params()
        working_params = self._resolve_starting_params(current_params)
        self._log_initial_baseline(
            current_params=current_params,
            starting_params=working_params,
            profile_reference_params=profile_reference_params,
        )
        if self._args.dry_run:
            self._print_dry_run(current_params, working_params, profile_reference_params)
            self._write_result_file(
                mode=self._args.mode,
                current_params=current_params,
                starting_params=working_params,
                proposed_params=working_params,
                applied_live=False,
                trials=[],
                reasons=["Dry-run only."],
                status_snapshot=self._build_status_snapshot(),
                profile_reference_params=profile_reference_params,
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
            self._log_motion_gate_state("Before movement")
            session_started = True
            self._enter_recording_if_needed()
            if self._latest_status is not None and self._latest_status.is_charging:
                self._undock_if_requested()

            if working_params != current_params:
                self.get_logger().info(
                    "Applying explicit initial tuning baseline before the first trial."
                )
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
            self._print_summary(
                current_params,
                working_params,
                proposed_params,
                feedforward_trials,
                response_trials,
                keep_final_live,
                reasons,
                profile_reference_params,
            )
            return 0
        except Exception as exc:
            self._remember_failure(str(exc))
            raise
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
                    failure_message=self._failure_message,
                    status_snapshot=self._failure_status_snapshot or self._build_status_snapshot(),
                    profile_reference_params=profile_reference_params,
                )

    # ------------------------------------------------------------------
    # High-level flow helpers
    # ------------------------------------------------------------------

    def _resolve_cmd_topic(self, args: argparse.Namespace) -> str:
        if args.cmd_topic:
            return str(args.cmd_topic)
        return "/cmd_vel_tuning"

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
            if (
                self._latest_status is not None
                and self._latest_emergency is not None
                and self._latest_odom_time is not None
                and self._latest_wheel_tick_time is not None
            ):
                return
        raise RuntimeError(
            "Timed out waiting for hardware_bridge status, emergency, /wheel_odom, and /wheel_ticks."
        )

    def _ensure_safe_to_start(self) -> None:
        if self._latest_emergency is None or self._latest_status is None:
            raise RuntimeError("Robot status is unavailable.")
        if self._latest_emergency.active_emergency or self._latest_emergency.latched_emergency:
            raise RuntimeError(f"Emergency active: {self._latest_emergency.reason}")
        if (
            self._latest_high_level_status is not None
            and self._latest_high_level_status.state == HL_STATE_AUTONOMOUS
        ):
            raise RuntimeError(
                "Refusing to tune while BT is AUTONOMOUS (mowing in progress). "
                "Stop mowing first (HOME command)."
            )
        if self._latest_status.is_charging and self._args.undock_distance <= 0.0:
            raise RuntimeError(
                "Robot is on the dock. Re-run with --undock-distance 2.0 (or another safe value)."
            )

    def _should_reset_to_profile(self) -> bool:
        return bool(self._args.reset_to_profile or self._args.force_profile)

    def _profile_reference_params(self) -> DrivePidParams | None:
        if self._args.profile == "custom":
            return None
        return PROFILE_PRESETS[self._args.profile]

    def _custom_param_updates(self) -> dict[str, float]:
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
        return updates

    def _resolve_starting_params(self, current_params: DrivePidParams) -> DrivePidParams:
        params = current_params
        profile_reference = self._profile_reference_params()
        if self._should_reset_to_profile() and profile_reference is not None:
            params = profile_reference
        updates = self._custom_param_updates()
        if updates:
            params = replace(params, **updates)
        return params

    def _format_drive_params(self, params: DrivePidParams) -> str:
        return (
            f"ticks_per_meter={params.ticks_per_meter:.3f}, "
            f"wheel_pid_pwm_per_mps={params.wheel_pid_pwm_per_mps:.3f}, "
            f"wheel_pid_kp={params.wheel_pid_kp:.3f}, "
            f"wheel_pid_ki={params.wheel_pid_ki:.3f}, "
            f"wheel_pid_kd={params.wheel_pid_kd:.3f}, "
            f"wheel_pid_integral_limit={params.wheel_pid_integral_limit:.3f}"
        )

    def _initial_baseline_source(
        self,
        *,
        current_params: DrivePidParams,
        starting_params: DrivePidParams,
        profile_reference_params: DrivePidParams | None,
    ) -> str:
        if starting_params == current_params:
            return "live_hardware_bridge"
        if self._should_reset_to_profile() and profile_reference_params is not None:
            if self._custom_param_updates():
                return "profile_reset_plus_cli_overrides"
            return "profile_reset"
        return "live_plus_cli_overrides"

    def _log_initial_baseline(
        self,
        *,
        current_params: DrivePidParams,
        starting_params: DrivePidParams,
        profile_reference_params: DrivePidParams | None,
    ) -> None:
        self.get_logger().info("Using live hardware_bridge parameters as initial tuning baseline")
        self.get_logger().info(f"initial ticks_per_meter={current_params.ticks_per_meter:.3f}")
        self.get_logger().info(f"initial wheel_pid_pwm_per_mps={current_params.wheel_pid_pwm_per_mps:.3f}")
        self.get_logger().info(f"initial wheel_pid_kp={current_params.wheel_pid_kp:.3f}")
        self.get_logger().info(f"initial wheel_pid_ki={current_params.wheel_pid_ki:.3f}")
        self.get_logger().info(f"initial wheel_pid_kd={current_params.wheel_pid_kd:.3f}")
        self.get_logger().info(
            f"initial wheel_pid_integral_limit={current_params.wheel_pid_integral_limit:.3f}"
        )
        if profile_reference_params is not None:
            if self._should_reset_to_profile():
                self.get_logger().info(
                    f"Profile {self._args.profile} will be applied before pass 1 because "
                    f"--reset-to-profile/--force-profile was requested: "
                    f"{self._format_drive_params(profile_reference_params)}"
                )
            else:
                self.get_logger().info(
                    f"profile {self._args.profile} reference only, not applied before pass 1: "
                    f"{self._format_drive_params(profile_reference_params)}"
                )
        else:
            self.get_logger().info(
                "profile custom/reference only, not applied before pass 1"
            )
        if starting_params == current_params:
            self.get_logger().info(
                "First trial will use the current live hardware_bridge parameters unchanged."
            )
            return
        self.get_logger().info(
            f"Initial trial params after explicit overrides: {self._format_drive_params(starting_params)}"
        )

    def _robot_config_candidates(self) -> list[Path]:
        candidates: list[Path] = []
        explicit = str(getattr(self._args, "hardware_config", "") or "").strip()
        if explicit:
            candidates.append(Path(explicit).expanduser())
        candidates.extend([
            Path("/ros2_ws/config/mowgli_robot.yaml"),
            Path("/ros2_ws/src/mowglinext/ros2/src/mowgli_bringup/config/mowgli_robot.yaml"),
            Path("/ros2_ws/src/mowgli_bringup/config/mowgli_robot.yaml"),
        ])
        for parent in Path(__file__).resolve().parents:
            candidates.append(parent / "ros2/src/mowgli_bringup/config/mowgli_robot.yaml")
            candidates.append(parent / "src/mowgli_bringup/config/mowgli_robot.yaml")
        unique: list[Path] = []
        seen: set[Path] = set()
        for candidate in candidates:
            if candidate in seen:
                continue
            seen.add(candidate)
            unique.append(candidate)
        return unique

    def _load_robot_hardware_config(self) -> RobotHardwareConfig:
        for candidate in self._robot_config_candidates():
            if not candidate.is_file():
                continue
            try:
                payload = yaml.safe_load(candidate.read_text(encoding="utf-8")) or {}
            except Exception as exc:
                self.get_logger().warning(
                    f"Failed to read hardware configuration from {candidate}: {exc}"
                )
                continue
            return extract_robot_hardware_config(payload, str(candidate))
        return RobotHardwareConfig()

    def _build_drivetrain_diagnostics(
        self,
        proposed_params: DrivePidParams,
    ) -> dict[str, Any]:
        diagnostics: dict[str, Any] = {}
        notes: list[str] = []
        wheel_radius = finite_or_none(
            self._robot_hardware_config.wheel_radius_m,
            positive=True,
        )
        ticks_per_meter = finite_or_none(
            proposed_params.ticks_per_meter,
            positive=True,
        )
        if wheel_radius is not None and ticks_per_meter is not None:
            wheel_circumference = 2.0 * math.pi * wheel_radius
            diagnostics["wheel_radius_m"] = wheel_radius
            diagnostics["wheel_circumference_m"] = wheel_circumference
            diagnostics["estimated_wheel_revolutions_per_meter"] = 1.0 / wheel_circumference
            diagnostics["estimated_encoder_counts_per_wheel_revolution"] = (
                ticks_per_meter * wheel_circumference
            )
        else:
            notes.append(
                "Wheel radius or ticks_per_meter is unavailable/invalid, so wheel-based drivetrain estimates are incomplete."
            )
        if self._robot_hardware_config.ticks_per_revolution is not None:
            diagnostics["configured_ticks_per_revolution"] = (
                self._robot_hardware_config.ticks_per_revolution
            )
        notes.append(
            "Estimated gearbox ratio is unavailable because encoder counts per motor revolution are not stored in the hardware configuration."
        )
        diagnostics["notes"] = notes
        return diagnostics

    def _report_reasons(self, reasons: list[str]) -> list[str]:
        prefix: list[str] = []
        if self._robot_hardware_config.chassis_mass_kg is not None:
            prefix.append(
                "Robot mass read from hardware configuration: "
                f"{self._robot_hardware_config.chassis_mass_kg:.2f} kg"
            )
        else:
            prefix.append(
                "Robot mass was not available in the hardware configuration; medium internal tuning tier was used."
            )
        prefix.append(f"Internal tuning tier: {self._tuning_tier.report_label}")
        if self._tuning_tier.manual_validation_note is not None:
            prefix.append(self._tuning_tier.manual_validation_note)
        return list(dict.fromkeys([*prefix, *reasons]))

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
            try:
                feedforward_trials.append(
                    self._run_speed_trial(
                        name=f"ff_{index}_{speed:.2f}mps",
                        phase="feedforward",
                        initial_speed=0.0,
                        target_speed=speed,
                        params=working_params,
                    )
                )
            except TrialAbortedError as exc:
                if exc.metrics is not None:
                    feedforward_trials.append(exc.metrics)
                raise

        provisional_params, provisional_reasons = recommend_drive_pid_params(
            working_params,
            feedforward_trials,
            [],
            robot_mass_kg=self._robot_hardware_config.chassis_mass_kg,
        )
        reasons.extend(provisional_reasons)
        self._apply_drive_pid_params(provisional_params)

        response_speeds = self._response_speeds(speeds)
        self.get_logger().info(
            f"Response phase at speeds: {', '.join(f'{s:.2f}' for s in response_speeds)}"
        )
        for index, speed in enumerate(response_speeds, start=1):
            try:
                response_trials.append(
                    self._run_speed_trial(
                        name=f"resp_{index}_{speed:.2f}mps",
                        phase="response",
                        initial_speed=0.0,
                        target_speed=speed,
                        params=provisional_params,
                    )
                )
            except TrialAbortedError as exc:
                if exc.metrics is not None:
                    response_trials.append(exc.metrics)
                raise

        proposed_params, final_reasons = recommend_drive_pid_params(
            working_params,
            feedforward_trials,
            response_trials,
            robot_mass_kg=self._robot_hardware_config.chassis_mass_kg,
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
            try:
                trial = self._run_speed_trial(
                    name=f"ff_pass_{pass_index + 1}_{target_speed:.2f}mps",
                    phase="feedforward",
                    initial_speed=0.0,
                    target_speed=target_speed,
                    params=params,
                    duration_s=duration_s,
                )
                trials.append(trial)
            except TrialAbortedError as exc:
                if exc.metrics is not None:
                    trials.append(exc.metrics)
                raise

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
                    f"Pass {pass_index + 1}: ticks_per_meter -> {next_ticks:.3f} from odom/reference "
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
                self._remember_failure(self._stall_message())
                self.get_logger().error(
                    "Feed-forward stall debug "
                    f"(pass_{pass_index + 1}): {self._failure_status_snapshot}"
                )
                raise RuntimeError(self._stall_message())
            next_pwm = clamp(
                params.wheel_pid_pwm_per_mps * (target_speed / max(measured_speed, 1e-6)),
                50.0,
                600.0,
            )
            reasons.append(
                f"Pass {pass_index + 1}: wheel_pid_pwm_per_mps -> {next_pwm:.3f} from target/measured "
                f"{target_speed:.3f}/{measured_speed:.3f} m/s."
            )
            if trial.oscillation_detected:
                reasons.append(
                    f"Pass {pass_index + 1}: oscillation warning recorded; keeping the feed-forward proposal because calibration completed."
                )
            if trial.live_oscillation_detected:
                reasons.append(
                    f"Pass {pass_index + 1}: live oscillation warning recorded during calibration; trial continued by design."
                )

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
                try:
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
                except TrialAbortedError as exc:
                    if exc.metrics is not None:
                        pass_trials.append(exc.metrics)
                        trials.extend(pass_trials)
                    raise
            trials.extend(pass_trials)

            params, pass_reasons = recommend_pid_only_params(
                params,
                pass_trials,
                robot_mass_kg=self._robot_hardware_config.chassis_mass_kg,
            )
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

    def _call_high_level_control(self, command: int, label: str) -> bool:
        if not self._high_level_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warning(
                f"/behavior_tree_node/high_level_control is unavailable, cannot {label}."
            )
            return False
        request = HighLevelControl.Request()
        request.command = int(command)
        future = self._high_level_client.call_async(request)
        response = self._wait_for_future(future, timeout_s=5.0, description=label)
        if not response.success:
            self.get_logger().warning(f"behavior_tree_node rejected request to {label}.")
            return False
        return True

    def _wait_for_bt_state(self, target: int, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._latest_high_level_status is not None and self._latest_high_level_status.state == target:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._latest_high_level_status is not None and self._latest_high_level_status.state == target

    def _log_motion_gate_state(self, label: str) -> None:
        hl_state = None if self._latest_high_level_status is None else int(self._latest_high_level_status.state)
        hl_name = None if self._latest_high_level_status is None else str(self._latest_high_level_status.state_name)
        mower_status = None if self._latest_status is None else int(self._latest_status.mower_status)
        is_charging = None if self._latest_status is None else bool(self._latest_status.is_charging)
        self.get_logger().info(
            f"{label}: high_level_state={hl_state} high_level_name={hl_name!r} "
            f"mower_status={mower_status} is_charging={is_charging} "
            f"wheel_ticks_seen={'yes' if self._latest_wheel_tick_time is not None else 'no'}"
        )

    def _enter_recording_if_needed(self) -> None:
        if self._latest_high_level_status is not None and self._latest_high_level_status.state == HL_STATE_RECORDING:
            self._log_motion_gate_state("Already in RECORDING before movement")
            return
        self.get_logger().info(
            f"Entering RECORDING mode so twist_mux forwards the tuner's {self._cmd_topic} commands."
        )
        if not self._call_high_level_control(HL_CMD_RECORD_AREA, "enter recording"):
            raise RuntimeError(
                "Could not enter RECORDING mode via BT. Check that behavior_tree_node is alive."
            )
        if self._wait_for_bt_state(HL_STATE_RECORDING, RECORDING_ENTRY_TIMEOUT_S):
            self._entered_recording = True
            self._log_motion_gate_state("After entering RECORDING")
            self.get_logger().info(
                f"Holding zero for {RECORDING_SETTLE_S:.1f}s after RECORDING entry so hardware_bridge can forward the non-IDLE mode to STM32."
            )
            self._hold_zero(RECORDING_SETTLE_S)
            return
        self._call_high_level_control(HL_CMD_RECORD_CANCEL, "cancel after failed recording entry")
        state = None if self._latest_high_level_status is None else self._latest_high_level_status.state
        raise RuntimeError(
            "BT did not transition to RECORDING within "
            f"{RECORDING_ENTRY_TIMEOUT_S:.0f}s (stuck at state={state})."
        )

    def _cancel_recording_if_needed(self) -> None:
        if not self._entered_recording:
            return
        self._call_high_level_control(HL_CMD_RECORD_CANCEL, "cancel recording")
        self._wait_for_bt_state(HL_STATE_IDLE, RECORDING_EXIT_TIMEOUT_S)
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
            pretrial_last_odom_time=self._latest_odom_time,
        )
        self._active_trial = recorder
        failure: Exception | None = None
        try:
            self._drive_segment(recorder)
            recorder.actual_end_s = time.monotonic()
            if hold_zero_after:
                self._hold_zero(self._args.stop_between_tests)
        except Exception as exc:
            recorder.actual_end_s = time.monotonic()
            failure = exc
        finally:
            self._active_trial = None
            if stop_after:
                self._stop_robot()
        metrics = self._finalize_trial_with_options(
            recorder,
            allow_incomplete=failure is not None,
            additional_notes=(
                [f"Partial trial metrics recorded after abort: {failure}"]
                if failure is not None
                else ()
            ),
        )
        ground_summary = "n/a" if metrics.ground_speed_mean is None else f"{metrics.ground_speed_mean:.3f} m/s"
        self.get_logger().info(
            f"Trial {metrics.name} summary: wheel={metrics.measured_speed_mean:.3f} m/s "
            f"ground={ground_summary} overshoot={metrics.overshoot:.3f} "
            f"stall={metrics.stall_detected} osc={metrics.oscillation_detected}"
        )
        if failure is not None:
            raise TrialAbortedError(str(failure), metrics) from failure
        return metrics

    def _drive_segment(self, recorder: TrialRecorder) -> None:
        rate_s = CMD_PERIOD_S
        deadline = recorder.command_start_s + recorder.command_duration_s
        while time.monotonic() < deadline:
            now_s = time.monotonic()
            elapsed = now_s - recorder.command_start_s
            vx = self._segment_speed_target(recorder, elapsed)
            self._publish_cmd(vx=vx, wz=0.0)
            rclpy.spin_once(self, timeout_sec=rate_s)
            now_s = time.monotonic()
            elapsed = now_s - recorder.command_start_s
            self._check_live_trial_safety(
                recorder,
                now_s,
                elapsed,
                commanded_speed=vx,
            )
        recorder.actual_end_s = time.monotonic()
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

    def _format_optional_time(self, value: float | None) -> str:
        if value is None:
            return "n/a"
        return f"{value:.3f}"

    def _format_optional_speed(self, value: float | None) -> str:
        if value is None:
            return "n/a"
        return f"{value:.3f}"

    def _trial_last_odom_time(self, recorder: TrialRecorder) -> float | None:
        if recorder.speed_samples:
            return recorder.speed_samples[-1].time_s
        return self._latest_odom_time

    def _recent_speed_samples(
        self,
        recorder: TrialRecorder,
        *,
        now_s: float,
        window_s: float,
    ) -> list[SpeedSample]:
        return [sample for sample in recorder.speed_samples if now_s - sample.time_s <= window_s]

    def _recent_tick_samples(
        self,
        recorder: TrialRecorder,
        *,
        now_s: float,
        window_s: float,
    ) -> list[TickSample]:
        return [sample for sample in recorder.tick_samples if now_s - sample.time_s <= window_s]

    def _movement_is_active(self, commanded_speed: float) -> bool:
        return abs(commanded_speed) >= ODOM_ACTIVE_SPEED_THRESHOLD_MPS

    def _oscillation_grace_elapsed(self, elapsed_s: float) -> bool:
        return elapsed_s >= max(float(self._args.startup_grace), 3.0)

    def _append_trial_warning(
        self,
        recorder: TrialRecorder,
        warning: str,
        *,
        note: str | None = None,
    ) -> None:
        if warning not in recorder.warnings:
            recorder.warnings.append(warning)
        if note is not None and note not in recorder.notes:
            recorder.notes.append(note)

    def _analyze_live_oscillation(
        self,
        *,
        recent: list[SpeedSample],
        commanded_speed: float,
    ) -> tuple[str, int, float] | None:
        if len(recent) < 8:
            return None
        errors = [commanded_speed - sample.speed_mps for sample in recent]
        error_stddev = statistics.pstdev(errors)
        if error_stddev < max(0.015, 0.10 * abs(commanded_speed)):
            return None
        zero_crossings = 0
        previous_sign = 0
        for error in errors:
            if abs(error) < 0.01:
                continue
            sign = 1 if error > 0.0 else -1
            if previous_sign and sign != previous_sign:
                zero_crossings += 1
            previous_sign = sign
        if zero_crossings < 5:
            return None
        if (
            len(recent) >= 10
            and zero_crossings >= 8
            and error_stddev >= max(0.03, 0.18 * abs(commanded_speed))
        ):
            return "severe", zero_crossings, error_stddev
        return "mild", zero_crossings, error_stddev

    def _handle_live_oscillation(
        self,
        *,
        recorder: TrialRecorder,
        now_s: float,
        elapsed_s: float,
        commanded_speed: float,
        recent: list[SpeedSample],
        recent_ticks: list[TickSample],
    ) -> None:
        analysis = self._analyze_live_oscillation(
            recent=recent,
            commanded_speed=commanded_speed,
        )
        if analysis is None:
            return
        severity, zero_crossings, error_stddev = analysis
        decision = evaluate_live_oscillation_abort(
            commanded_speed=commanded_speed,
            recent_speed_samples=recent,
            configured_max_speed=float(self._args.max_speed),
            abort_on_live_oscillation=self._args.abort_on_live_oscillation,
            no_abort_on_live_oscillation=self._args.no_abort_on_live_oscillation,
        )
        will_abort = decision.should_abort
        first_mild_warning = not recorder.live_oscillation_detected
        first_severe_warning = (
            severity == "severe" and not recorder.severe_live_oscillation_detected
        )
        recorder.live_oscillation_detected = True
        if severity == "severe":
            recorder.severe_live_oscillation_detected = True
        recent_speed = sum(sample.speed_mps for sample in recent) / len(recent)
        measured_speed = recent[-1].speed_mps
        tick_activity = compute_tick_activity(recent_ticks)
        overshoot_ratio = max(
            0.0,
            (decision.peak_abs_speed - abs(commanded_speed)) / max(abs(commanded_speed), 1e-6),
        )
        self._append_trial_warning(
            recorder,
            "Live oscillation suspected during calibration; trial continued."
            if not will_abort
            else (
                "Unsafe live speed response detected during calibration."
                if decision.unsafe_speed_detected
                else "Live oscillation suspected during calibration."
            ),
            note=(
                (
                    "Severe live oscillation stayed below unsafe speed thresholds, so PID tuning continued with warning-level trial quality and conservative recommendations."
                    if recorder.phase == "pid" and severity == "severe" and not will_abort
                    else (
                        "Live oscillation is treated as a calibration warning in ff mode."
                        if recorder.phase == "feedforward" and not will_abort
                        else (
                            "Peak live speed exceeded the runaway safety threshold, so the trial was aborted."
                            if decision.unsafe_speed_detected
                            else (
                                "Severe live oscillation triggered strict abort behavior."
                                if will_abort
                                else "Live oscillation was recorded as a calibration warning during PID/response tuning."
                            )
                        )
                    )
                )
            ),
        )
        if first_mild_warning or first_severe_warning:
            log_fn = self.get_logger().error if decision.unsafe_speed_detected else self.get_logger().warning
            absolute_runaway = (
                "n/a"
                if decision.absolute_runaway_threshold is None
                else f"{decision.absolute_runaway_threshold:.3f}"
            )
            log_fn(
                "Live oscillation warning "
                f"({severity}) during {recorder.name}: "
                f"trial_elapsed={elapsed_s:.3f}s "
                f"commanded_speed={commanded_speed:.3f}mps "
                f"measured_speed={measured_speed:.3f}mps "
                f"recent_speed={recent_speed:.3f}mps "
                f"overshoot_ratio={overshoot_ratio:.3f} "
                f"zero_crossings={zero_crossings} "
                f"error_stddev={error_stddev:.4f} "
                f"wheel_ticks_delta={tick_activity.wheel_ticks_delta} "
                f"peak_abs_speed={decision.peak_abs_speed:.3f}mps "
                f"target_runaway_threshold={decision.target_runaway_threshold:.3f}mps "
                f"absolute_runaway_threshold={absolute_runaway}mps "
                f"unsafe_speed_detected={'yes' if decision.unsafe_speed_detected else 'no'} "
                + (
                    "aborting trial."
                    if will_abort
                    else "continuing calibration trial."
                )
            )
        if will_abort:
            if decision.unsafe_speed_detected:
                raise RuntimeError(
                    f"Unsafe live speed response detected during {recorder.name}."
                )
            self._append_trial_warning(
                recorder,
                "Severe live oscillation triggered a strict-mode abort.",
                note="Strict live-oscillation abort mode stopped the trial before completion.",
            )
            raise RuntimeError(f"Live oscillation detected during {recorder.name}.")

    def _log_odom_debug(
        self,
        *,
        recorder: TrialRecorder,
        now_s: float,
        elapsed_s: float,
        commanded_speed: float,
        reason: str,
    ) -> None:
        last_odom_time = self._trial_last_odom_time(recorder)
        time_since_last_odom = None if last_odom_time is None else now_s - last_odom_time
        self.get_logger().error(
            "Odom safety debug "
            f"({reason}) during {recorder.name}: "
            f"current_time={now_s:.3f}s "
            f"trial_elapsed={elapsed_s:.3f}s "
            f"last_odom_timestamp={self._format_optional_time(last_odom_time)}s "
            f"time_since_last_odom={self._format_optional_time(time_since_last_odom)}s "
            f"odom_timeout={self._args.odom_timeout:.3f}s "
            f"startup_grace={self._args.startup_grace:.3f}s "
            f"commanded_speed={commanded_speed:.3f}mps "
            f"trial_odom_received={'yes' if bool(recorder.speed_samples) else 'no'} "
            f"pretrial_last_odom={self._format_optional_time(recorder.pretrial_last_odom_time)}s"
        )

    def _stamp_to_iso(self, stamp: Any) -> str | None:
        if stamp is None:
            return None
        sec = getattr(stamp, "sec", None)
        nanosec = getattr(stamp, "nanosec", None)
        if sec is None or nanosec is None:
            return None
        if sec == 0 and nanosec == 0:
            return None
        return datetime.fromtimestamp(float(sec) + float(nanosec) / 1e9, tz=timezone.utc).isoformat()

    def _build_status_snapshot(self) -> dict[str, Any]:
        return {
            "active_emergency": (
                None if self._latest_emergency is None else bool(self._latest_emergency.active_emergency)
            ),
            "latched_emergency": (
                None if self._latest_emergency is None else bool(self._latest_emergency.latched_emergency)
            ),
            "is_charging": None if self._latest_status is None else bool(self._latest_status.is_charging),
            "mower_status": None if self._latest_status is None else int(self._latest_status.mower_status),
            "esc_power": None if self._latest_status is None else bool(self._latest_status.esc_power),
            "wheel_tick_factor": self._latest_wheel_tick_factor,
            "last_wheel_tick_timestamp": self._latest_wheel_tick_stamp,
        }

    def _remember_failure(self, message: str) -> None:
        if self._failure_message is not None:
            return
        self._failure_message = message
        self._failure_status_snapshot = self._build_status_snapshot()

    def _stall_message(self) -> str:
        return (
            "Drive command produced no wheel motion; check traction motor power, "
            "firmware drive loop, PAC5210 reset/enable, or safety state."
        )

    def _log_stall_diagnostic(
        self,
        *,
        recorder: TrialRecorder,
        elapsed_s: float,
        diagnostic: LiveStallDiagnostic,
        level: str,
    ) -> None:
        message = (
            "Stall safety debug "
            f"({diagnostic.reason}) during {recorder.name}: "
            f"trial_elapsed={elapsed_s:.3f}s "
            f"commanded_speed={diagnostic.commanded_speed:.3f}mps "
            f"measured_speed={self._format_optional_speed(diagnostic.measured_speed)}mps "
            f"recent_speed={self._format_optional_speed(diagnostic.recent_speed)}mps "
            f"wheel_ticks_delta={diagnostic.wheel_ticks_delta} "
            f"left_ticks_delta={diagnostic.left_ticks_delta} "
            f"right_ticks_delta={diagnostic.right_ticks_delta} "
            f"time_window_used={diagnostic.time_window_used:.3f}s "
            f"speed_floor={diagnostic.speed_floor:.3f}mps "
            f"continuous_ticks_observed={'yes' if diagnostic.continuous_ticks_observed else 'no'} "
            f"status_snapshot={self._build_status_snapshot()}"
        )
        if level == "warning":
            self.get_logger().warning(message)
            return
        self.get_logger().error(message)

    def _raise_stall_failure(
        self,
        recorder: TrialRecorder,
        *,
        elapsed_s: float,
        diagnostic: LiveStallDiagnostic,
    ) -> None:
        self._remember_failure(self._stall_message())
        self._log_stall_diagnostic(
            recorder=recorder,
            elapsed_s=elapsed_s,
            diagnostic=diagnostic,
            level="error",
        )
        raise RuntimeError(self._stall_message())

    def _raise_odom_failure(
        self,
        *,
        recorder: TrialRecorder,
        now_s: float,
        elapsed_s: float,
        commanded_speed: float,
        reason: str,
        message: str,
    ) -> None:
        self._log_odom_debug(
            recorder=recorder,
            now_s=now_s,
            elapsed_s=elapsed_s,
            commanded_speed=commanded_speed,
            reason=reason,
        )
        raise RuntimeError(message)

    def _check_live_trial_safety(
        self,
        recorder: TrialRecorder,
        now_s: float,
        elapsed_s: float,
        *,
        commanded_speed: float,
    ) -> None:
        if self._latest_emergency is not None and (
            self._latest_emergency.active_emergency or self._latest_emergency.latched_emergency
        ):
            raise RuntimeError(f"Emergency asserted during {recorder.name}: {self._latest_emergency.reason}")
        if self._latest_status is not None and self._latest_status.is_charging:
            raise RuntimeError(f"Charging detected during {recorder.name}.")
        if (
            self._movement_is_active(commanded_speed)
            and elapsed_s >= self._args.startup_grace
            and recorder.speed_samples
        ):
            last_odom_time = recorder.speed_samples[-1].time_s
            if now_s - last_odom_time > self._args.odom_timeout:
                self._raise_odom_failure(
                    recorder=recorder,
                    now_s=now_s,
                    elapsed_s=elapsed_s,
                    commanded_speed=commanded_speed,
                    reason="odom_received_then_stopped",
                    message=f"Lost /wheel_odom updates during {recorder.name}.",
                )
        if recorder.phase == "feedforward" and self._latest_gnss_status is not None:
            gnss = self._latest_gnss_status
            require_fixed = not self._args.allow_rtk_float
            mode_ok = gnss.rtk_mode == GnssStatus.RTK_MODE_FIXED
            if not require_fixed:
                mode_ok = gnss.rtk_mode in (GnssStatus.RTK_MODE_FIXED, GnssStatus.RTK_MODE_FLOAT)
            if not bool(gnss.fix_valid) or not bool(gnss.corrections_active) or not mode_ok:
                raise RuntimeError(f"Lost RTK/GPS validity during {recorder.name}.")
        recent = self._recent_speed_samples(
            recorder,
            now_s=now_s,
            window_s=LIVE_STALL_SPEED_WINDOW_S,
        )
        if (
            self._movement_is_active(commanded_speed)
            and self._oscillation_grace_elapsed(elapsed_s)
        ):
            recent_ticks = self._recent_tick_samples(
                recorder,
                now_s=now_s,
                window_s=LIVE_STALL_SPEED_WINDOW_S,
            )
            stall_diagnostic = evaluate_live_stall(
                commanded_speed=commanded_speed,
                recent_speed_samples=recent,
                recent_tick_samples=recent_ticks,
                time_window_used=LIVE_STALL_SPEED_WINDOW_S,
            )
            if stall_diagnostic is not None:
                if stall_diagnostic.warning_only:
                    warning = (
                        "Live stall safety saw low odometry speed while wheel ticks kept changing; "
                        "continuing calibration trial."
                    )
                    is_new_warning = warning not in recorder.warnings
                    self._append_trial_warning(
                        recorder,
                        warning,
                        note=(
                            "Continuous wheel ticks were still observed during the live stall check, "
                            "so the event was downgraded to a calibration warning."
                        ),
                    )
                    if is_new_warning:
                        self._log_stall_diagnostic(
                            recorder=recorder,
                            elapsed_s=elapsed_s,
                            diagnostic=stall_diagnostic,
                            level="warning",
                        )
                elif stall_diagnostic.should_abort:
                    self._raise_stall_failure(
                        recorder,
                        elapsed_s=elapsed_s,
                        diagnostic=stall_diagnostic,
                    )
            self._handle_live_oscillation(
                recorder=recorder,
                now_s=now_s,
                elapsed_s=elapsed_s,
                commanded_speed=commanded_speed,
                recent=recent,
                recent_ticks=recent_ticks,
            )

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
            rclpy.spin_once(self, timeout_sec=CMD_PERIOD_S)

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
            rclpy.spin_once(self, timeout_sec=CMD_PERIOD_S)
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
            rclpy.spin_once(self, timeout_sec=CMD_PERIOD_S)

    # ------------------------------------------------------------------
    # Trial post-processing
    # ------------------------------------------------------------------

    def _finalize_trial_with_options(
        self,
        recorder: TrialRecorder,
        *,
        allow_incomplete: bool,
        additional_notes: list[str] | tuple[str, ...],
    ) -> TrialMetrics:
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
        notes = list(additional_notes)
        if not speed_samples:
            if allow_incomplete:
                notes.append(
                    "Trial ended before enough /wheel_odom samples were available for a full steady-state slice."
                )
                speed_samples = recorder.speed_samples
                response_samples = recorder.speed_samples
            else:
                now_s = time.monotonic()
                elapsed_s = now_s - recorder.command_start_s
                if recorder.pretrial_last_odom_time is None:
                    self._raise_odom_failure(
                        recorder=recorder,
                        now_s=now_s,
                        elapsed_s=elapsed_s,
                        commanded_speed=recorder.target_speed,
                        reason="no_odom_sample_ever_received",
                        message=f"No /wheel_odom sample was ever received before or during {recorder.name}.",
                    )
                self._raise_odom_failure(
                    recorder=recorder,
                    now_s=now_s,
                    elapsed_s=elapsed_s,
                    commanded_speed=recorder.target_speed,
                    reason="no_trial_odom_sample_received",
                    message=(
                        f"No /wheel_odom samples were received during {recorder.name}; "
                        "odometry was seen before the trial but never arrived once the test started."
                    ),
                )

        odom_distance_m = integrate_distance(speed_samples)
        ticks_seen, left_ticks_seen, right_ticks_seen = self._ticks_seen(recorder.tick_samples)
        ground_speed_mean, rtk_distance_m, rtk_notes = self._compute_rtk_metrics(
            recorder=recorder,
            steady_start=steady_start,
            steady_end=steady_end,
        )
        notes.extend(rtk_notes)
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
            live_oscillation_detected=recorder.live_oscillation_detected,
            warnings=recorder.warnings,
            notes=notes,
        )

    def _ticks_seen(self, samples: list[TickSample]) -> tuple[int, int, int]:
        activity = compute_tick_activity(samples)
        return (
            activity.wheel_ticks_delta,
            activity.left_ticks_delta,
            activity.right_ticks_delta,
        )

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

    def _print_dry_run(
        self,
        current_params: DrivePidParams,
        working_params: DrivePidParams,
        profile_reference_params: DrivePidParams | None,
    ) -> None:
        print("=== DRIVE PID TUNER DRY RUN ===")
        print(f"mode: {self._args.mode}")
        print(f"profile: {self._args.profile}")
        print(f"profile reset before pass 1: {'yes' if self._should_reset_to_profile() else 'no'}")
        print(f"cmd topic: {self._cmd_topic}")
        print(f"backup file: {self._backup_path}")
        print(f"feedforward speeds: {[f'{s:.2f}' for s in self._phase_speeds()]}")
        print(f"odom timeout: {self._args.odom_timeout:.2f} s")
        print(f"startup grace: {self._args.startup_grace:.2f} s")
        if self._latest_status is not None and self._latest_status.is_charging:
            print(f"robot is charging: yes, requested undock_distance={self._args.undock_distance:.2f} m")
        print(f"current params: {current_params.to_dict()}")
        if profile_reference_params is not None:
            print(f"profile reference params: {profile_reference_params.to_dict()}")
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
        profile_reference_params: DrivePidParams | None,
    ) -> None:
        print("\n=== DRIVE PID TUNER SUMMARY ===")
        print(f"mode: {self._args.mode}")
        print(f"profile: {self._args.profile}")
        print(f"profile reset before pass 1: {'yes' if self._should_reset_to_profile() else 'no'}")
        print(f"backup file: {self._backup_path}")
        print(f"current params:  {current_params.to_dict()}")
        if profile_reference_params is not None:
            print(f"profile ref:    {profile_reference_params.to_dict()}")
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
                f"stall={trial.stall_detected} osc={trial.oscillation_detected} "
                f"live_osc={trial.live_oscillation_detected} quality={trial.trial_quality}"
            )
            for warning in trial.warnings:
                print(f"    warning: {warning}")
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
        failure_message: str | None = None,
        status_snapshot: dict[str, Any] | None = None,
        profile_reference_params: DrivePidParams | None = None,
    ) -> None:
        if not self._args.output:
            return
        output_path = Path(self._args.output).expanduser()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        report_reasons = self._report_reasons(reasons)
        payload = {
            "generated_at": _now_iso(),
            "mode": mode,
            "profile": self._args.profile,
            "hardware_node": self._args.hardware_node,
            "backup_file": str(self._backup_path),
            "cmd_topic": self._cmd_topic,
            "cmd_vel_topic": self._cmd_topic,
            "applied_live": applied_live,
            "requested_apply": bool(self._args.apply),
            "profile_applied_initially": bool(self._should_reset_to_profile() and profile_reference_params is not None),
            "initial_baseline_source": self._initial_baseline_source(
                current_params=current_params,
                starting_params=starting_params,
                profile_reference_params=profile_reference_params,
            ),
            "distance_m": float(self._args.distance),
            "max_speed_mps": float(self._args.max_speed),
            "test_speed_mps": self._args.test_speed,
            "segment_duration_s": float(self._args.duration),
            "passes": int(self._args.passes),
            "odom_timeout_s": float(self._args.odom_timeout),
            "startup_grace_s": float(self._args.startup_grace),
            "auto_turn": bool(self._args.auto_turn),
            "turn_direction": self._args.turn_direction,
            "robot_mass_kg": self._robot_hardware_config.chassis_mass_kg,
            "internal_tuning_tier": self._tuning_tier.report_label,
            "hardware_config_path": self._robot_hardware_config.source_path,
            "drivetrain_diagnostics": self._build_drivetrain_diagnostics(proposed_params),
            "current_params": current_params.to_dict(),
            "starting_params": starting_params.to_dict(),
            "proposed_params": proposed_params.to_dict(),
            "status_snapshot": (
                status_snapshot if status_snapshot is not None else self._build_status_snapshot()
            ),
            "reasons": report_reasons,
            "trials": [trial.to_dict() for trial in trials],
        }
        if profile_reference_params is not None:
            payload["profile_reference_params"] = profile_reference_params.to_dict()
        if failure_message is not None:
            payload["failure_message"] = failure_message
        output_path.write_text(
            yaml.safe_dump(sanitize_finite_data(payload), sort_keys=False),
            encoding="utf-8",
        )
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
    if args.odom_timeout <= 0.0:
        parser.error("--odom-timeout must be positive.")
    if args.startup_grace < 0.0:
        parser.error("--startup-grace must be >= 0.")
    if args.undock_distance < 0.0:
        parser.error("--undock-distance must be >= 0.")
    if args.undock_speed <= 0.0:
        parser.error("--undock-speed must be positive.")
    if args.turn_rate <= 0.0:
        parser.error("--turn-rate must be positive.")
    if args.rollback and args.apply:
        parser.error("--rollback and --apply are mutually exclusive.")
    resolved_cmd_topic = args.cmd_topic if args.cmd_topic else "/cmd_vel_tuning"

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
