# Copyright 2026 Mowgli Project
#
# SPDX-License-Identifier: GPL-3.0

from __future__ import annotations

from dataclasses import dataclass, replace
import math
import statistics
from typing import Any, Sequence


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _mean(values: Sequence[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def _rms(values: Sequence[float]) -> float:
    if not values:
        return 0.0
    return math.sqrt(sum(v * v for v in values) / len(values))


def _median(values: Sequence[float]) -> float:
    return statistics.median(values) if values else 0.0


def finite_or_none(value: Any, *, positive: bool = False) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    if positive and number <= 0.0:
        return None
    return number


def _require_finite_float(name: str, value: Any) -> float:
    number = finite_or_none(value)
    if number is None:
        raise ValueError(f"{name} must be a finite float, got {value!r}")
    return number


def sanitize_finite_data(value: Any) -> Any:
    if isinstance(value, bool) or value is None:
        return value
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {key: sanitize_finite_data(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [sanitize_finite_data(item) for item in value]
    return value


def _adaptive_kp_step(current_kp: float, *, aggressive: bool, max_step: float) -> float:
    magnitude = max(abs(current_kp), 0.02)
    step = magnitude if aggressive else magnitude * 0.5
    return clamp(step, 0.005, max_step)


@dataclass(frozen=True)
class DrivePidParams:
    ticks_per_meter: float
    wheel_pid_kp: float
    wheel_pid_ki: float
    wheel_pid_kd: float
    wheel_pid_integral_limit: float
    wheel_pid_pwm_per_mps: float

    def to_dict(self) -> dict[str, float]:
        return {
            "ticks_per_meter": _require_finite_float("ticks_per_meter", self.ticks_per_meter),
            "wheel_pid_kp": _require_finite_float("wheel_pid_kp", self.wheel_pid_kp),
            "wheel_pid_ki": _require_finite_float("wheel_pid_ki", self.wheel_pid_ki),
            "wheel_pid_kd": _require_finite_float("wheel_pid_kd", self.wheel_pid_kd),
            "wheel_pid_integral_limit": _require_finite_float(
                "wheel_pid_integral_limit",
                self.wheel_pid_integral_limit,
            ),
            "wheel_pid_pwm_per_mps": _require_finite_float(
                "wheel_pid_pwm_per_mps",
                self.wheel_pid_pwm_per_mps,
            ),
        }

    @classmethod
    def from_mapping(cls, mapping: dict[str, Any]) -> "DrivePidParams":
        return cls(
            ticks_per_meter=_require_finite_float("ticks_per_meter", mapping["ticks_per_meter"]),
            wheel_pid_kp=_require_finite_float("wheel_pid_kp", mapping["wheel_pid_kp"]),
            wheel_pid_ki=_require_finite_float("wheel_pid_ki", mapping["wheel_pid_ki"]),
            wheel_pid_kd=_require_finite_float("wheel_pid_kd", mapping["wheel_pid_kd"]),
            wheel_pid_integral_limit=_require_finite_float(
                "wheel_pid_integral_limit",
                mapping["wheel_pid_integral_limit"],
            ),
            wheel_pid_pwm_per_mps=_require_finite_float(
                "wheel_pid_pwm_per_mps",
                mapping["wheel_pid_pwm_per_mps"],
            ),
        )


@dataclass(frozen=True)
class RobotTuningTier:
    report_label: str
    acceptable_overshoot_ratio: float
    severe_overshoot_ratio: float
    low_tracking_error_ratio: float
    max_kp_step_per_pass: float
    post_oscillation_repeat_limit: int
    prefer_ki_trim_before_kp: bool
    manual_validation_note: str | None = None


def resolve_robot_tuning_tier(robot_mass_kg: float | None) -> RobotTuningTier:
    # Light mowers tolerate a bit more transient overshoot and can absorb
    # slightly larger exploratory gain steps. Heavy platforms carry more
    # inertia, so the tuner automatically tightens overshoot expectations
    # and shrinks per-pass gain changes as mass increases.
    mass = finite_or_none(robot_mass_kg, positive=True)
    if mass is None:
        return RobotTuningTier(
            report_label="medium",
            acceptable_overshoot_ratio=0.08,
            severe_overshoot_ratio=0.14,
            low_tracking_error_ratio=0.05,
            max_kp_step_per_pass=0.10,
            post_oscillation_repeat_limit=2,
            prefer_ki_trim_before_kp=True,
        )
    if mass <= 12.0:
        return RobotTuningTier(
            report_label="lightweight",
            acceptable_overshoot_ratio=0.10,
            severe_overshoot_ratio=0.16,
            low_tracking_error_ratio=0.05,
            max_kp_step_per_pass=0.12,
            post_oscillation_repeat_limit=3,
            prefer_ki_trim_before_kp=True,
        )
    if mass <= 30.0:
        return RobotTuningTier(
            report_label="medium",
            acceptable_overshoot_ratio=0.08,
            severe_overshoot_ratio=0.14,
            low_tracking_error_ratio=0.045,
            max_kp_step_per_pass=0.10,
            post_oscillation_repeat_limit=2,
            prefer_ki_trim_before_kp=True,
        )
    if mass <= 60.0:
        return RobotTuningTier(
            report_label="heavy",
            acceptable_overshoot_ratio=0.06,
            severe_overshoot_ratio=0.10,
            low_tracking_error_ratio=0.04,
            max_kp_step_per_pass=0.05,
            post_oscillation_repeat_limit=2,
            prefer_ki_trim_before_kp=True,
        )
    return RobotTuningTier(
        report_label="extra-heavy",
        acceptable_overshoot_ratio=0.05,
        severe_overshoot_ratio=0.08,
        low_tracking_error_ratio=0.035,
        max_kp_step_per_pass=0.03,
        post_oscillation_repeat_limit=1,
        prefer_ki_trim_before_kp=True,
        manual_validation_note="Extra-heavy robot mass detected; manual validation is strongly recommended.",
    )


@dataclass(frozen=True)
class SpeedSample:
    time_s: float
    speed_mps: float


@dataclass(frozen=True)
class TickSample:
    time_s: float
    left_ticks: int
    right_ticks: int


@dataclass(frozen=True)
class TickActivity:
    wheel_ticks_delta: int
    left_ticks_delta: int
    right_ticks_delta: int
    sample_count: int
    time_span_s: float

    @property
    def continuous_ticks_observed(self) -> bool:
        return self.left_ticks_delta > 0 or self.right_ticks_delta > 0


@dataclass(frozen=True)
class LiveStallDiagnostic:
    should_abort: bool
    warning_only: bool
    commanded_speed: float
    measured_speed: float | None
    recent_speed: float | None
    speed_floor: float
    wheel_ticks_delta: int
    left_ticks_delta: int
    right_ticks_delta: int
    time_window_used: float
    continuous_ticks_observed: bool
    reason: str


@dataclass(frozen=True)
class LiveOscillationDecision:
    should_abort: bool
    reason: str
    peak_abs_speed: float
    target_runaway_threshold: float
    absolute_runaway_threshold: float | None
    unsafe_speed_detected: bool


@dataclass(frozen=True)
class TrialMetrics:
    name: str
    phase: str
    target_speed: float
    measured_speed_mean: float
    measured_speed_rms: float
    error_mean: float
    error_rms: float
    overshoot: float
    settling_time: float | None
    ticks_seen: int
    left_ticks_seen: int
    right_ticks_seen: int
    stall_detected: bool
    oscillation_detected: bool
    integral_saturation_suspected: bool
    params_used: dict[str, float]
    live_oscillation_detected: bool = False
    trial_quality: str = "ok"
    warnings: tuple[str, ...] = ()
    ground_speed_mean: float | None = None
    ground_error_mean: float | None = None
    odom_distance_m: float | None = None
    rtk_distance_m: float | None = None
    rtk_accepted: bool = False
    left_right_tick_imbalance: float | None = None
    notes: tuple[str, ...] = ()

    def to_dict(self) -> dict[str, Any]:
        out = {
            "name": self.name,
            "phase": self.phase,
            "target_speed": self.target_speed,
            "measured_speed_mean": self.measured_speed_mean,
            "measured_speed_rms": self.measured_speed_rms,
            "error_mean": self.error_mean,
            "error_rms": self.error_rms,
            "overshoot": self.overshoot,
            "settling_time": self.settling_time,
            "ticks_seen": self.ticks_seen,
            "left_ticks_seen": self.left_ticks_seen,
            "right_ticks_seen": self.right_ticks_seen,
            "stall_detected": self.stall_detected,
            "oscillation_detected": self.oscillation_detected,
            "live_oscillation_detected": self.live_oscillation_detected,
            "trial_quality": self.trial_quality,
            "integral_saturation_suspected": self.integral_saturation_suspected,
            "params_used": dict(self.params_used),
            "warnings": list(self.warnings),
            "ground_speed_mean": self.ground_speed_mean,
            "ground_error_mean": self.ground_error_mean,
            "odom_distance_m": self.odom_distance_m,
            "rtk_distance_m": self.rtk_distance_m,
            "rtk_accepted": self.rtk_accepted,
            "left_right_tick_imbalance": self.left_right_tick_imbalance,
            "notes": list(self.notes),
        }
        return out


def compute_tick_activity(samples: Sequence[TickSample]) -> TickActivity:
    if len(samples) < 2:
        return TickActivity(
            wheel_ticks_delta=0,
            left_ticks_delta=0,
            right_ticks_delta=0,
            sample_count=len(samples),
            time_span_s=0.0,
        )
    first = samples[0]
    last = samples[-1]
    left_delta = abs(last.left_ticks - first.left_ticks)
    right_delta = abs(last.right_ticks - first.right_ticks)
    average_delta = int(round((left_delta + right_delta) * 0.5))
    return TickActivity(
        wheel_ticks_delta=int(average_delta),
        left_ticks_delta=int(left_delta),
        right_ticks_delta=int(right_delta),
        sample_count=len(samples),
        time_span_s=max(0.0, float(last.time_s - first.time_s)),
    )


def evaluate_live_stall(
    *,
    commanded_speed: float,
    recent_speed_samples: Sequence[SpeedSample],
    recent_tick_samples: Sequence[TickSample],
    time_window_used: float,
    min_commanded_speed_mps: float = 0.10,
    min_speed_samples: int = 5,
    speed_floor_ratio: float = 0.20,
    minimum_speed_floor_mps: float = 0.02,
) -> LiveStallDiagnostic | None:
    expected_speed = abs(commanded_speed)
    if expected_speed < min_commanded_speed_mps or len(recent_speed_samples) < min_speed_samples:
        return None
    recent_speed = _mean([sample.speed_mps for sample in recent_speed_samples])
    measured_speed = recent_speed_samples[-1].speed_mps if recent_speed_samples else None
    tick_activity = compute_tick_activity(recent_tick_samples)
    speed_floor = max(minimum_speed_floor_mps, speed_floor_ratio * expected_speed)
    speed_below_floor = recent_speed < speed_floor
    if not speed_below_floor:
        return None
    warning_only = tick_activity.continuous_ticks_observed
    return LiveStallDiagnostic(
        should_abort=not warning_only,
        warning_only=warning_only,
        commanded_speed=float(commanded_speed),
        measured_speed=measured_speed,
        recent_speed=recent_speed,
        speed_floor=speed_floor,
        wheel_ticks_delta=tick_activity.wheel_ticks_delta,
        left_ticks_delta=tick_activity.left_ticks_delta,
        right_ticks_delta=tick_activity.right_ticks_delta,
        time_window_used=float(time_window_used),
        continuous_ticks_observed=tick_activity.continuous_ticks_observed,
        reason=(
            "live_recent_speed_below_floor_with_ticks"
            if warning_only
            else "live_recent_speed_below_floor"
        ),
    )


def evaluate_live_oscillation_abort(
    *,
    commanded_speed: float,
    recent_speed_samples: Sequence[SpeedSample],
    configured_max_speed: float | None,
    abort_on_live_oscillation: bool = False,
    no_abort_on_live_oscillation: bool = False,
) -> LiveOscillationDecision:
    peak_abs_speed = max((abs(sample.speed_mps) for sample in recent_speed_samples), default=0.0)
    target_runaway_threshold = abs(commanded_speed) * 1.8
    max_speed_value = finite_or_none(configured_max_speed, positive=True)
    absolute_runaway_threshold = (
        None if max_speed_value is None else max_speed_value * 1.5
    )
    unsafe_speed_detected = peak_abs_speed > target_runaway_threshold
    if absolute_runaway_threshold is not None and peak_abs_speed > absolute_runaway_threshold:
        unsafe_speed_detected = True
    if unsafe_speed_detected:
        return LiveOscillationDecision(
            should_abort=True,
            reason="unsafe_speed_runaway",
            peak_abs_speed=peak_abs_speed,
            target_runaway_threshold=target_runaway_threshold,
            absolute_runaway_threshold=absolute_runaway_threshold,
            unsafe_speed_detected=True,
        )
    if no_abort_on_live_oscillation:
        return LiveOscillationDecision(
            should_abort=False,
            reason="warning_only_explicit",
            peak_abs_speed=peak_abs_speed,
            target_runaway_threshold=target_runaway_threshold,
            absolute_runaway_threshold=absolute_runaway_threshold,
            unsafe_speed_detected=False,
        )
    if abort_on_live_oscillation:
        return LiveOscillationDecision(
            should_abort=True,
            reason="strict_live_oscillation_abort",
            peak_abs_speed=peak_abs_speed,
            target_runaway_threshold=target_runaway_threshold,
            absolute_runaway_threshold=absolute_runaway_threshold,
            unsafe_speed_detected=False,
        )
    return LiveOscillationDecision(
        should_abort=False,
        reason="warning_only_default",
        peak_abs_speed=peak_abs_speed,
        target_runaway_threshold=target_runaway_threshold,
        absolute_runaway_threshold=absolute_runaway_threshold,
        unsafe_speed_detected=False,
    )


def compute_settling_time(
    samples: Sequence[SpeedSample],
    target_speed: float,
    tolerance_mps: float,
    min_hold_s: float,
) -> float | None:
    if len(samples) < 2:
        return None
    for idx, sample in enumerate(samples):
        remaining = samples[idx:]
        if remaining[-1].time_s - sample.time_s < min_hold_s:
            continue
        if all(abs(s.speed_mps - target_speed) <= tolerance_mps for s in remaining):
            return sample.time_s - samples[0].time_s
    return None


def detect_oscillation(samples: Sequence[SpeedSample], target_speed: float) -> bool:
    if len(samples) < 5:
        return False
    errors = [target_speed - sample.speed_mps for sample in samples]
    if statistics.pstdev(errors) < max(0.015, 0.12 * abs(target_speed)):
        return False
    zero_crossings = 0
    previous_sign = 0
    for error in errors:
        if abs(error) < 0.01:
            continue
        sign = 1 if error > 0.0 else -1
        if previous_sign and sign != previous_sign:
            zero_crossings += 1
        previous_sign = sign
    return zero_crossings >= 4


def integrate_distance(samples: Sequence[SpeedSample]) -> float:
    if len(samples) < 2:
        return 0.0
    distance = 0.0
    for previous, current in zip(samples[:-1], samples[1:]):
        dt = current.time_s - previous.time_s
        if dt <= 0.0 or dt > 1.0:
            continue
        distance += 0.5 * (previous.speed_mps + current.speed_mps) * dt
    return distance


def compute_trial_metrics(
    *,
    name: str,
    phase: str,
    target_speed: float,
    speed_samples: Sequence[SpeedSample],
    response_samples: Sequence[SpeedSample] | None,
    ticks_seen: int,
    left_ticks_seen: int,
    right_ticks_seen: int,
    params_used: DrivePidParams,
    ground_speed_mean: float | None,
    odom_distance_m: float | None,
    rtk_distance_m: float | None,
    live_oscillation_detected: bool = False,
    warnings: Sequence[str] = (),
    notes: Sequence[str] = (),
) -> TrialMetrics:
    speeds = [sample.speed_mps for sample in speed_samples]
    errors = [target_speed - speed for speed in speeds]
    measured_speed_mean = _mean(speeds)
    measured_speed_rms = _rms(speeds)
    error_mean = _mean(errors)
    error_rms = _rms(errors)
    response = list(response_samples) if response_samples else list(speed_samples)
    is_stop_trial = abs(target_speed) < 0.05
    overshoot = max(0.0, max((sample.speed_mps - target_speed) for sample in response)) if response else 0.0
    response_peak_abs_speed = max((abs(sample.speed_mps) for sample in response), default=0.0)
    settling_time = compute_settling_time(
        response,
        target_speed=target_speed,
        tolerance_mps=max(0.02, 0.1 * abs(target_speed)),
        min_hold_s=1.0,
    )
    oscillation_detected = detect_oscillation(response, target_speed)
    speed_floor = max(0.02, 0.3 * abs(target_speed))
    stall_detected = False
    if not is_stop_trial:
        stall_detected = ticks_seen <= 2 or measured_speed_mean < speed_floor
        if ground_speed_mean is not None and ground_speed_mean < speed_floor:
            stall_detected = True
    ground_error_mean = None
    if ground_speed_mean is not None:
        ground_error_mean = target_speed - ground_speed_mean
    left_right_tick_imbalance = None
    tick_average = 0.5 * (abs(left_ticks_seen) + abs(right_ticks_seen))
    if tick_average > 1e-6:
        left_right_tick_imbalance = abs(left_ticks_seen - right_ticks_seen) / tick_average
    integral_enabled = (
        params_used.wheel_pid_ki > 0.0 and params_used.wheel_pid_integral_limit > 0.0
    )
    # Treat integral saturation as a sustained underspeed problem only. Mild
    # overshoot or a still-settling but otherwise responsive trial should not
    # poison bring-up calibration before the tuner has enough data to improve it.
    persistent_lag_threshold = max(0.03, 0.12 * abs(target_speed))
    severe_underspeed_threshold = max(0.02, 0.88 * abs(target_speed))
    never_settled_under_load = settling_time is None and abs(target_speed) >= 0.20
    persistent_underspeed = measured_speed_mean < severe_underspeed_threshold
    positive_steady_state_error = error_mean > persistent_lag_threshold
    integral_saturation_suspected = (
        integral_enabled
        and (
            stall_detected
            or (
                never_settled_under_load
                and persistent_underspeed
                and positive_steady_state_error
            )
            or (
                abs(target_speed) >= 0.25
                and measured_speed_mean < max(0.02, 0.75 * abs(target_speed))
                and positive_steady_state_error
            )
        )
    )
    warning_list = list(dict.fromkeys(warnings))
    stop_behavior_warning = (
        is_stop_trial
        and (
            abs(measured_speed_mean) > 0.03
            or response_peak_abs_speed > 0.05
        )
    )
    stop_behavior_severe = (
        is_stop_trial
        and (
            abs(measured_speed_mean) > 0.06
            or response_peak_abs_speed > 0.10
        )
    )
    if stop_behavior_warning:
        warning_list.append("Stop behavior warning: residual motion detected after zero-speed command.")
        notes = [
            *notes,
            (
                "Residual motion after stop command: "
                f"mean={abs(measured_speed_mean):.3f} m/s, peak={response_peak_abs_speed:.3f} m/s."
            ),
        ]
    if oscillation_detected:
        warning_list.append("Post-trial oscillation signature detected in the recorded speed response.")
    if live_oscillation_detected:
        warning_list.append("Live oscillation suspected during calibration; trial continued.")
    warning_list = list(dict.fromkeys(warning_list))
    trial_quality = "ok"
    if stall_detected or stop_behavior_severe:
        trial_quality = "poor"
    elif warning_list or oscillation_detected or integral_saturation_suspected:
        trial_quality = "warning"
    return TrialMetrics(
        name=name,
        phase=phase,
        target_speed=target_speed,
        measured_speed_mean=measured_speed_mean,
        measured_speed_rms=measured_speed_rms,
        error_mean=error_mean,
        error_rms=error_rms,
        overshoot=overshoot,
        settling_time=settling_time,
        ticks_seen=ticks_seen,
        left_ticks_seen=left_ticks_seen,
        right_ticks_seen=right_ticks_seen,
        stall_detected=stall_detected,
        oscillation_detected=oscillation_detected,
        integral_saturation_suspected=integral_saturation_suspected,
        params_used=params_used.to_dict(),
        live_oscillation_detected=live_oscillation_detected,
        trial_quality=trial_quality,
        warnings=tuple(warning_list),
        ground_speed_mean=ground_speed_mean,
        ground_error_mean=ground_error_mean,
        odom_distance_m=odom_distance_m,
        rtk_distance_m=rtk_distance_m,
        rtk_accepted=ground_speed_mean is not None and rtk_distance_m is not None,
        left_right_tick_imbalance=left_right_tick_imbalance,
        notes=tuple(notes),
    )


def recommend_feedforward_scale(
    current_pwm_per_mps: float,
    trials: Sequence[TrialMetrics],
) -> tuple[float, list[str]]:
    ratios: list[float] = []
    sources: list[str] = []
    for trial in trials:
        measured = (
            trial.ground_speed_mean
            if trial.rtk_accepted and trial.ground_speed_mean is not None
            else trial.measured_speed_mean
        )
        if measured <= 0.02 or trial.stall_detected:
            continue
        ratios.append(clamp(trial.target_speed / measured, 0.7, 1.3))
        sources.append("rtk" if trial.rtk_accepted and trial.ground_speed_mean is not None else "wheel_odom")
    if not ratios:
        return current_pwm_per_mps, ["No trustworthy speed samples for feed-forward update."]
    median_ratio = _median(ratios)
    blended_ratio = clamp(1.0 + 0.5 * (median_ratio - 1.0), 0.85, 1.15)
    recommended = clamp(current_pwm_per_mps * blended_ratio, 50.0, 600.0)
    source_summary = " + ".join(sorted(set(sources)))
    reason = (
        f"Feed-forward scaled by {blended_ratio:.3f} from median target/measured ratio "
        f"{median_ratio:.3f} using {source_summary}."
    )
    return recommended, [reason]


def recommend_ticks_per_meter(
    current_ticks_per_meter: float,
    trials: Sequence[TrialMetrics],
) -> tuple[float, list[str]]:
    ratios: list[float] = []
    for trial in trials:
        if (
            not trial.rtk_accepted
            or trial.odom_distance_m is None
            or trial.rtk_distance_m is None
            or trial.rtk_distance_m < 1.0
            or abs(trial.target_speed) < 0.12
        ):
            continue
        ratios.append(clamp(trial.odom_distance_m / trial.rtk_distance_m, 0.5, 2.5))
    if not ratios:
        return current_ticks_per_meter, ["RTK distance checks were unavailable or too short for ticks_per_meter."]
    median_ratio = _median(ratios)
    recommended = clamp(current_ticks_per_meter * median_ratio, 100.0, 2500.0)
    return recommended, [
        "ticks_per_meter scaled from the median odom/RTK distance ratio "
        f"{median_ratio:.3f}."
    ]


def recommend_feedforward_params(
    base_params: DrivePidParams,
    feedforward_trials: Sequence[TrialMetrics],
) -> tuple[DrivePidParams, list[str]]:
    reasons: list[str] = []
    recommended_pwm_per_mps, ff_reasons = recommend_feedforward_scale(
        base_params.wheel_pid_pwm_per_mps,
        feedforward_trials,
    )
    recommended_ticks_per_meter, tick_reasons = recommend_ticks_per_meter(
        base_params.ticks_per_meter,
        feedforward_trials,
    )
    reasons.extend(ff_reasons)
    reasons.extend(tick_reasons)
    return replace(
        base_params,
        ticks_per_meter=recommended_ticks_per_meter,
        wheel_pid_pwm_per_mps=recommended_pwm_per_mps,
    ), reasons


def recommend_pid_only_params(
    base_params: DrivePidParams,
    response_trials: Sequence[TrialMetrics],
    *,
    robot_mass_kg: float | None = None,
) -> tuple[DrivePidParams, list[str]]:
    reasons: list[str] = []
    params = replace(base_params)
    tier = resolve_robot_tuning_tier(robot_mass_kg)
    stop_trials = [trial for trial in response_trials if abs(trial.target_speed) < 0.05]
    usable_trials = [trial for trial in response_trials if abs(trial.target_speed) >= 0.05]
    if stop_trials:
        reasons.append(
            "Zero-speed stop trial excluded from PID gain selection; it is kept only as a stop behavior diagnostic."
        )
    if any(
        any("Stop behavior warning:" in warning for warning in trial.warnings)
        for trial in stop_trials
    ):
        reasons.append("Stop behavior warning: residual motion detected after zero-speed command.")
    if not usable_trials:
        reasons.append("No response trials available, keeping KP/KI/integral limit unchanged.")
        return params, reasons

    max_target = max(abs(trial.target_speed) for trial in usable_trials)
    median_error = _median([trial.error_mean for trial in usable_trials])
    median_abs_error = _median([abs(trial.error_mean) for trial in usable_trials])
    median_overshoot = _median([trial.overshoot for trial in usable_trials])
    max_overshoot = max((trial.overshoot for trial in usable_trials), default=0.0)
    live_oscillation_count = sum(
        1
        for trial in usable_trials
        if trial.live_oscillation_detected
    )
    post_analysis_oscillation_count = sum(
        1
        for trial in usable_trials
        if trial.oscillation_detected and not trial.live_oscillation_detected
    )
    slow_count = sum(
        1
        for trial in usable_trials
        if trial.settling_time is None or trial.settling_time > 2.5
    )
    stall_count = sum(1 for trial in usable_trials if trial.stall_detected)
    imbalance_count = sum(
        1
        for trial in usable_trials
        if trial.left_right_tick_imbalance is not None and trial.left_right_tick_imbalance > 0.12
    )
    imbalance_samples = [
        trial.left_right_tick_imbalance
        for trial in usable_trials
        if trial.left_right_tick_imbalance is not None
    ]
    max_imbalance = max(imbalance_samples, default=0.0)
    integral_sat_count = sum(1 for trial in usable_trials if trial.integral_saturation_suspected)
    overshoot_warning_threshold = max(0.02, tier.acceptable_overshoot_ratio * max_target)
    severe_overshoot_threshold = max(0.04, tier.severe_overshoot_ratio * max_target)
    low_tracking_error_threshold = max(0.015, tier.low_tracking_error_ratio * max_target)
    steady_state_error_threshold = max(0.02, max(0.08, tier.low_tracking_error_ratio * 1.7) * max_target)
    lag_error_threshold = max(0.03, max(0.12, tier.low_tracking_error_ratio * 2.2) * max_target)
    overspeed_threshold = max(0.015, 0.5 * overshoot_warning_threshold)
    good_ff_tracking = median_abs_error <= low_tracking_error_threshold and stall_count == 0
    dangerous_overshoot = max_overshoot > severe_overshoot_threshold
    overshoot_warning_present = (
        max_overshoot > overshoot_warning_threshold
        or median_overshoot > overshoot_warning_threshold
    )
    severe_post_analysis_oscillation = sum(
        1
        for trial in usable_trials
        if trial.oscillation_detected
        and not trial.live_oscillation_detected
        and (
            trial.overshoot > severe_overshoot_threshold
            or abs(trial.error_mean) > steady_state_error_threshold
        )
    )
    repeated_post_analysis_oscillation = (
        post_analysis_oscillation_count >= tier.post_oscillation_repeat_limit
    )
    severe_oscillation = (
        live_oscillation_count > 0
        or repeated_post_analysis_oscillation
        or severe_post_analysis_oscillation > 0
    )
    mild_post_analysis_oscillation = (
        post_analysis_oscillation_count > 0 and not severe_oscillation
    )
    if mild_post_analysis_oscillation:
        reasons.append(
            "Post-analysis oscillation detected but no live oscillation observed; the response remains usable for conservative tuning."
        )
    if dangerous_overshoot:
        reasons.append(
            f"Worst-step overshoot reached {(max_overshoot / max(max_target, 1e-6)) * 100.0:.1f}% of target speed, so the recommendation stays conservative."
        )
    warning_count = sum(
        1
        for trial in usable_trials
        if trial.trial_quality == "poor"
        or trial.live_oscillation_detected
        or trial.integral_saturation_suspected
        or (
            trial.oscillation_detected
            and not trial.live_oscillation_detected
            and severe_oscillation
        )
        or dangerous_overshoot
    )
    fine_gain_mode = base_params.wheel_pid_kp <= 1.0
    fine_gain_cap = 0.5 if base_params.wheel_pid_kp <= 0.2 else 1.0
    low_speed_trials = [
        trial
        for trial in usable_trials
        if abs(trial.target_speed) <= min(max_target, 0.20)
    ]
    overspeed_trials = [
        trial
        for trial in low_speed_trials
        if (trial.measured_speed_mean - trial.target_speed) > overspeed_threshold
    ]
    consistent_low_speed_overspeed = (
        bool(low_speed_trials)
        and len(overspeed_trials) >= max(1, math.ceil(len(low_speed_trials) / 2.0))
    )

    recommended_kp = params.wheel_pid_kp
    recommended_ki = params.wheel_pid_ki
    recommended_kd = params.wheel_pid_kd
    recommended_integral_limit = params.wheel_pid_integral_limit

    if tier.manual_validation_note is not None:
        reasons.append(tier.manual_validation_note)
    if imbalance_samples and max_imbalance <= 0.01:
        reasons.append("Drivetrain symmetry is good.")
        reasons.append("Left/right tick balance is within expected limits.")

    if fine_gain_mode:
        if params.wheel_pid_kp < 0.02:
            recommended_kp = max(recommended_kp, 0.02)
            reasons.append("Starting KP is near zero; seeding a conservative low-gain P-only baseline at 0.020.")

        step_reference = max(recommended_kp, params.wheel_pid_kp, 0.02)
        mild_kp_step = _adaptive_kp_step(
            step_reference,
            aggressive=False,
            max_step=tier.max_kp_step_per_pass,
        )
        strong_kp_step = _adaptive_kp_step(
            step_reference,
            aggressive=(median_error > lag_error_threshold or slow_count > 1),
            max_step=tier.max_kp_step_per_pass,
        )

        if consistent_low_speed_overspeed and params.wheel_pid_ki > 0.0:
            recommended_ki = params.wheel_pid_ki * 0.90
            if params.wheel_pid_integral_limit > 0.0:
                recommended_integral_limit = params.wheel_pid_integral_limit * 0.95
            reasons.append(
                "Low-speed tracking indicates possible excessive integral action, so KI is trimmed before touching KP."
            )
        elif integral_sat_count > 0 and params.wheel_pid_ki > 0.0:
            recommended_ki = params.wheel_pid_ki * 0.88
            if params.wheel_pid_integral_limit > 0.0:
                recommended_integral_limit = params.wheel_pid_integral_limit * 0.90
            reasons.append(
                "True integral saturation is suspected, so KI and the integral limit are trimmed conservatively."
            )
        elif severe_oscillation or dangerous_overshoot or warning_count > 0:
            if params.wheel_pid_ki > 0.0 and tier.prefer_ki_trim_before_kp:
                recommended_ki = params.wheel_pid_ki * 0.92
                reasons.append(
                    "Live or repeated oscillation was observed, so integral action is softened before any stronger KP move."
                )
            else:
                recommended_kp = max(0.0, recommended_kp - mild_kp_step)
            recommended_kp = min(max(recommended_kp, 0.0), fine_gain_cap)
            reasons.append(
                "Oscillation or overshoot warnings were present, so low-gain PID tuning keeps KP conservative and avoids adding I/D terms."
            )
        elif median_error > steady_state_error_threshold or slow_count > 0:
            recommended_kp = min(max(recommended_kp, params.wheel_pid_kp + strong_kp_step), fine_gain_cap)
            reasons.append(
                f"Feed-forward is already close; nudging KP by {strong_kp_step:.3f} for a conservative low-gain P-only trial."
            )
        elif median_error < -steady_state_error_threshold:
            recommended_kp = max(0.0, params.wheel_pid_kp - mild_kp_step)
            reasons.append(
                f"Average overspeed remained negative, trimming KP by {mild_kp_step:.3f}."
            )
        elif good_ff_tracking:
            recommended_kp = min(max(recommended_kp, params.wheel_pid_kp), fine_gain_cap)
            reasons.append(
                "Feed-forward already tracks target speed reasonably well; keeping the PID proposal in the low-gain fine-tuning range."
            )
    else:
        kp_scale = 1.0
        ki_scale = 1.0
        kd_scale = 1.0
        integral_scale = 1.0

        if consistent_low_speed_overspeed and params.wheel_pid_ki > 0.0:
            ki_scale *= 0.90
            integral_scale *= 0.95
            reasons.append("Low-speed tracking indicates possible excessive integral action, trimming KI before KP.")
        elif integral_sat_count > 0 and params.wheel_pid_ki > 0.0:
            ki_scale *= 0.88
            integral_scale *= 0.90
            reasons.append("True integral saturation is suspected, reducing KI and integral support.")
        elif severe_oscillation or dangerous_overshoot:
            kp_scale *= 0.92
            ki_scale *= 0.90
            integral_scale *= 0.92
            reasons.append("Oscillation or strong overshoot detected, softening KP/KI/integral support.")
        else:
            if median_error > steady_state_error_threshold:
                kp_scale *= 1.05
                reasons.append("Average underspeed remained positive, nudging KP upward.")
            elif median_error < -steady_state_error_threshold:
                kp_scale *= 0.95
                reasons.append("Average overspeed remained negative, trimming KP slightly.")

            if slow_count > 0 or median_error > max(0.015, 0.08 * max_target):
                ki_scale *= 1.08
                reasons.append("Slow settling or steady-state lag detected, nudging KI upward.")

            if overshoot_warning_present:
                kp_scale *= 0.97
                ki_scale *= 0.97
                if params.wheel_pid_kd > 0.0:
                    kd_scale *= 1.10
                reasons.append("Mild overshoot detected, trimming KP/KI and damping gently.")

        recommended_kp = clamp(params.wheel_pid_kp * kp_scale, 0.0, 80.0)
        if params.wheel_pid_ki > 0.0:
            recommended_ki = clamp(params.wheel_pid_ki * ki_scale, 0.0, 8000.0)
        if params.wheel_pid_kd > 0.0:
            recommended_kd = clamp(params.wheel_pid_kd * kd_scale, 0.0, 50.0)
        if params.wheel_pid_integral_limit > 0.0:
            recommended_integral_limit = clamp(
                params.wheel_pid_integral_limit * integral_scale,
                0.0,
                200.0,
            )

    if params.wheel_pid_ki <= 0.0:
        recommended_ki = 0.0
        if good_ff_tracking and ((mild_post_analysis_oscillation or overshoot_warning_present) or warning_count > 0):
            reasons.append(
                "Feed-forward already tracks target speed reasonably well, and oscillation/overshoot warnings were present; keeping KI at 0.0 for this low-gain PID proposal."
            )
        else:
            reasons.append(
                "Starting KI is zero, so the next PID proposal keeps integral gain disabled until P-only trials are stable."
            )

    if params.wheel_pid_integral_limit <= 0.0:
        recommended_integral_limit = 0.0
        reasons.append(
            "Integral limit remains 0.0 because integral action is still intentionally disabled at this stage."
        )

    if params.wheel_pid_kd <= 0.0:
        recommended_kd = 0.0
        reasons.append(
            "Kd remains 0.0 during low-gain P-only validation; add derivative only after stable P-only trials if it is still needed."
        )

    if imbalance_count > 0:
        reasons.append("Left/right wheel response diverged noticeably on at least one pass; re-check tire pressure and mechanical drag.")

    recommended = replace(
        params,
        wheel_pid_kp=clamp(recommended_kp, 0.0, 80.0),
        wheel_pid_ki=clamp(recommended_ki, 0.0, 8000.0),
        wheel_pid_kd=clamp(recommended_kd, 0.0, 50.0),
        wheel_pid_integral_limit=clamp(recommended_integral_limit, 0.0, 200.0),
    )
    return recommended, reasons


def recommend_drive_pid_params(
    base_params: DrivePidParams,
    feedforward_trials: Sequence[TrialMetrics],
    response_trials: Sequence[TrialMetrics],
    *,
    robot_mass_kg: float | None = None,
) -> tuple[DrivePidParams, list[str]]:
    ff_params, ff_reasons = recommend_feedforward_params(base_params, feedforward_trials)
    pid_params, pid_reasons = recommend_pid_only_params(
        ff_params,
        response_trials,
        robot_mass_kg=robot_mass_kg,
    )
    return pid_params, [*ff_reasons, *pid_reasons]
