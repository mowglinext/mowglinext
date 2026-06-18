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
            "ticks_per_meter": float(self.ticks_per_meter),
            "wheel_pid_kp": float(self.wheel_pid_kp),
            "wheel_pid_ki": float(self.wheel_pid_ki),
            "wheel_pid_kd": float(self.wheel_pid_kd),
            "wheel_pid_integral_limit": float(self.wheel_pid_integral_limit),
            "wheel_pid_pwm_per_mps": float(self.wheel_pid_pwm_per_mps),
        }

    @classmethod
    def from_mapping(cls, mapping: dict[str, Any]) -> "DrivePidParams":
        return cls(
            ticks_per_meter=float(mapping["ticks_per_meter"]),
            wheel_pid_kp=float(mapping["wheel_pid_kp"]),
            wheel_pid_ki=float(mapping["wheel_pid_ki"]),
            wheel_pid_kd=float(mapping["wheel_pid_kd"]),
            wheel_pid_integral_limit=float(mapping["wheel_pid_integral_limit"]),
            wheel_pid_pwm_per_mps=float(mapping["wheel_pid_pwm_per_mps"]),
        )


@dataclass(frozen=True)
class SpeedSample:
    time_s: float
    speed_mps: float


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
            "integral_saturation_suspected": self.integral_saturation_suspected,
            "params_used": dict(self.params_used),
            "ground_speed_mean": self.ground_speed_mean,
            "ground_error_mean": self.ground_error_mean,
            "odom_distance_m": self.odom_distance_m,
            "rtk_distance_m": self.rtk_distance_m,
            "rtk_accepted": self.rtk_accepted,
            "left_right_tick_imbalance": self.left_right_tick_imbalance,
            "notes": list(self.notes),
        }
        return out


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
    notes: Sequence[str] = (),
) -> TrialMetrics:
    speeds = [sample.speed_mps for sample in speed_samples]
    errors = [target_speed - speed for speed in speeds]
    measured_speed_mean = _mean(speeds)
    measured_speed_rms = _rms(speeds)
    error_mean = _mean(errors)
    error_rms = _rms(errors)
    response = list(response_samples) if response_samples else list(speed_samples)
    overshoot = max(0.0, max((sample.speed_mps - target_speed) for sample in response)) if response else 0.0
    settling_time = compute_settling_time(
        response,
        target_speed=target_speed,
        tolerance_mps=max(0.02, 0.1 * abs(target_speed)),
        min_hold_s=1.0,
    )
    oscillation_detected = detect_oscillation(response, target_speed)
    speed_floor = max(0.02, 0.3 * abs(target_speed))
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
    integral_saturation_suspected = (
        stall_detected
        or (settling_time is None and abs(target_speed) >= 0.15)
        or (
            measured_speed_mean < max(0.02, 0.85 * abs(target_speed))
            and params_used.wheel_pid_integral_limit <= 100.0
        )
    )
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
) -> tuple[DrivePidParams, list[str]]:
    reasons: list[str] = []
    params = replace(base_params)
    usable_trials = [trial for trial in response_trials if abs(trial.target_speed) >= 0.05]
    if not usable_trials:
        reasons.append("No response trials available, keeping KP/KI/integral limit unchanged.")
        return params, reasons

    max_target = max(abs(trial.target_speed) for trial in usable_trials)
    median_error = _median([trial.error_mean for trial in usable_trials])
    median_overshoot = _median([trial.overshoot for trial in usable_trials])
    oscillation_count = sum(1 for trial in usable_trials if trial.oscillation_detected)
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
    integral_sat_count = sum(1 for trial in usable_trials if trial.integral_saturation_suspected)

    kp_scale = 1.0
    ki_scale = 1.0
    kd_scale = 1.0
    integral_scale = 1.0

    if oscillation_count > 0 or median_overshoot > max(0.04, 0.2 * max_target):
        kp_scale *= 0.90
        ki_scale *= 0.85
        kd_scale *= 1.35
        integral_scale *= 0.90
        reasons.append("Oscillation or overshoot detected, softening KP/KI/integral limit and nudging Kd upward.")
    else:
        if median_error > max(0.02, 0.10 * max_target):
            kp_scale *= 1.08
            reasons.append("Average underspeed remained positive, nudging KP upward.")
        elif median_error < -max(0.02, 0.10 * max_target):
            kp_scale *= 0.94
            reasons.append("Average overspeed remained negative, trimming KP slightly.")

        if slow_count > 0 or median_error > max(0.015, 0.08 * max_target):
            ki_scale *= 1.12
            reasons.append("Slow settling or steady-state lag detected, nudging KI upward.")

        if stall_count > 0 or integral_sat_count > 0:
            ki_scale *= 1.05
            integral_scale *= 1.15
            reasons.append("Stall behaviour observed, widening integral support modestly.")

        if median_overshoot > max(0.02, 0.10 * max_target):
            kp_scale *= 0.96
            ki_scale *= 0.96
            kd_scale *= 1.20
            reasons.append("Mild overshoot detected, trimming KP/KI and adding a little derivative damping.")

    if imbalance_count > 0:
        reasons.append("Left/right wheel response diverged noticeably on at least one pass; re-check tire pressure and mechanical drag.")

    recommended = replace(
        params,
        wheel_pid_kp=clamp(params.wheel_pid_kp * kp_scale, 5.0, 80.0),
        wheel_pid_ki=clamp(params.wheel_pid_ki * ki_scale, 50.0, 8000.0),
        wheel_pid_kd=clamp(
            (
                params.wheel_pid_kd * kd_scale
                if params.wheel_pid_kd > 0.0
                else (0.02 if (oscillation_count > 0 or median_overshoot > max(0.02, 0.10 * max_target)) else 0.0)
            ),
            0.0,
            50.0,
        ),
        wheel_pid_integral_limit=clamp(
            params.wheel_pid_integral_limit * integral_scale,
            10.0,
            200.0,
        ),
    )
    return recommended, reasons


def recommend_drive_pid_params(
    base_params: DrivePidParams,
    feedforward_trials: Sequence[TrialMetrics],
    response_trials: Sequence[TrialMetrics],
) -> tuple[DrivePidParams, list[str]]:
    ff_params, ff_reasons = recommend_feedforward_params(base_params, feedforward_trials)
    pid_params, pid_reasons = recommend_pid_only_params(ff_params, response_trials)
    return pid_params, [*ff_reasons, *pid_reasons]
