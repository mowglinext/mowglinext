import json

import pytest
import yaml

from mowgli_tools.drive_pid_math import (
    DrivePidParams,
    LiveOscillationDecision,
    LiveStallDiagnostic,
    SpeedSample,
    TickSample,
    TrialMetrics,
    compute_settling_time,
    compute_tick_activity,
    compute_trial_metrics,
    detect_oscillation,
    evaluate_live_stall,
    evaluate_live_oscillation_abort,
    finite_or_none,
    recommend_drive_pid_params,
    recommend_pid_only_params,
    sanitize_finite_data,
)


def _params() -> DrivePidParams:
    return DrivePidParams(
        ticks_per_meter=533.0,
        wheel_pid_kp=18.0,
        wheel_pid_ki=700.0,
        wheel_pid_kd=0.0,
        wheel_pid_integral_limit=30.0,
        wheel_pid_pwm_per_mps=550.0,
    )


def _early_pid_params(kp: float = 0.0) -> DrivePidParams:
    return DrivePidParams(
        ticks_per_meter=533.0,
        wheel_pid_kp=kp,
        wheel_pid_ki=0.0,
        wheel_pid_kd=0.0,
        wheel_pid_integral_limit=0.0,
        wheel_pid_pwm_per_mps=550.0,
    )


def _trial(
    *,
    params: DrivePidParams,
    name: str,
    target_speed: float,
    measured_speed_mean: float,
    overshoot: float,
    error_mean: float,
    oscillation_detected: bool = False,
    live_oscillation_detected: bool = False,
    integral_saturation_suspected: bool = False,
    trial_quality: str = "ok",
    warnings: tuple[str, ...] = (),
    left_right_tick_imbalance: float | None = 0.006,
    settling_time: float | None = 1.2,
) -> TrialMetrics:
    return TrialMetrics(
        name=name,
        phase="pid",
        target_speed=target_speed,
        measured_speed_mean=measured_speed_mean,
        measured_speed_rms=measured_speed_mean,
        error_mean=error_mean,
        error_rms=abs(error_mean),
        overshoot=overshoot,
        settling_time=settling_time,
        ticks_seen=120,
        left_ticks_seen=118,
        right_ticks_seen=122,
        stall_detected=False,
        oscillation_detected=oscillation_detected,
        integral_saturation_suspected=integral_saturation_suspected,
        params_used=params.to_dict(),
        live_oscillation_detected=live_oscillation_detected,
        trial_quality=trial_quality,
        warnings=warnings,
        ground_speed_mean=None,
        ground_error_mean=None,
        odom_distance_m=0.9,
        rtk_distance_m=None,
        rtk_accepted=False,
        left_right_tick_imbalance=left_right_tick_imbalance,
        notes=(),
    )


def test_compute_settling_time_returns_first_stable_point() -> None:
    samples = [
        SpeedSample(0.0, 0.00),
        SpeedSample(0.5, 0.12),
        SpeedSample(1.0, 0.19),
        SpeedSample(1.5, 0.20),
        SpeedSample(2.0, 0.20),
        SpeedSample(2.5, 0.20),
    ]
    settling = compute_settling_time(samples, target_speed=0.20, tolerance_mps=0.02, min_hold_s=1.0)
    assert settling == 1.0


def test_detect_oscillation_flags_repeated_crossings() -> None:
    samples = [
        SpeedSample(0.0, 0.10),
        SpeedSample(0.2, 0.28),
        SpeedSample(0.4, 0.12),
        SpeedSample(0.6, 0.29),
        SpeedSample(0.8, 0.11),
        SpeedSample(1.0, 0.30),
    ]
    assert detect_oscillation(samples, target_speed=0.20)


def test_evaluate_live_stall_downgrades_to_warning_when_ticks_continue() -> None:
    diagnostic = evaluate_live_stall(
        commanded_speed=0.30,
        recent_speed_samples=[
            SpeedSample(0.0, 0.01),
            SpeedSample(0.2, 0.01),
            SpeedSample(0.4, 0.02),
            SpeedSample(0.6, 0.02),
            SpeedSample(0.8, 0.01),
        ],
        recent_tick_samples=[
            TickSample(0.0, 100, 100),
            TickSample(0.4, 103, 104),
            TickSample(0.8, 107, 109),
        ],
        time_window_used=0.8,
    )

    assert isinstance(diagnostic, LiveStallDiagnostic)
    assert diagnostic is not None
    assert diagnostic.warning_only
    assert not diagnostic.should_abort
    assert diagnostic.reason == "live_recent_speed_below_floor_with_ticks"
    assert diagnostic.wheel_ticks_delta == 8
    assert diagnostic.left_ticks_delta == 7
    assert diagnostic.right_ticks_delta == 9
    assert diagnostic.time_window_used == 0.8


def test_evaluate_live_stall_keeps_abort_when_ticks_stop() -> None:
    diagnostic = evaluate_live_stall(
        commanded_speed=0.30,
        recent_speed_samples=[
            SpeedSample(0.0, 0.01),
            SpeedSample(0.2, 0.01),
            SpeedSample(0.4, 0.02),
            SpeedSample(0.6, 0.02),
            SpeedSample(0.8, 0.01),
        ],
        recent_tick_samples=[
            TickSample(0.0, 100, 100),
            TickSample(0.4, 100, 100),
            TickSample(0.8, 100, 100),
        ],
        time_window_used=0.8,
    )

    assert isinstance(diagnostic, LiveStallDiagnostic)
    assert diagnostic is not None
    assert not diagnostic.warning_only
    assert diagnostic.should_abort
    assert diagnostic.reason == "live_recent_speed_below_floor"


def test_compute_tick_activity_uses_average_wheel_delta() -> None:
    activity = compute_tick_activity(
        [
            TickSample(0.0, 10, 20),
            TickSample(0.5, 13, 26),
            TickSample(1.0, 16, 30),
        ]
    )

    assert activity.wheel_ticks_delta == 8
    assert activity.left_ticks_delta == 6
    assert activity.right_ticks_delta == 10
    assert activity.continuous_ticks_observed


def test_evaluate_live_oscillation_abort_defaults_to_warning_below_runaway_threshold() -> None:
    decision = evaluate_live_oscillation_abort(
        commanded_speed=0.30,
        recent_speed_samples=[
            SpeedSample(0.0, 0.29),
            SpeedSample(0.2, 0.32),
            SpeedSample(0.4, 0.28),
            SpeedSample(0.6, 0.43),
            SpeedSample(0.8, 0.307),
        ],
        configured_max_speed=0.30,
    )

    assert isinstance(decision, LiveOscillationDecision)
    assert not decision.should_abort
    assert not decision.unsafe_speed_detected
    assert decision.reason == "warning_only_default"
    assert decision.peak_abs_speed == 0.43
    assert decision.target_runaway_threshold == 0.54
    assert decision.absolute_runaway_threshold == pytest.approx(0.45)


def test_evaluate_live_oscillation_abort_rejects_unsafe_speed_runaway() -> None:
    decision = evaluate_live_oscillation_abort(
        commanded_speed=0.30,
        recent_speed_samples=[
            SpeedSample(0.0, 0.29),
            SpeedSample(0.2, 0.35),
            SpeedSample(0.4, 0.56),
            SpeedSample(0.6, 0.31),
        ],
        configured_max_speed=0.30,
    )

    assert decision.should_abort
    assert decision.unsafe_speed_detected
    assert decision.reason == "unsafe_speed_runaway"


def test_evaluate_live_oscillation_abort_still_supports_explicit_strict_mode() -> None:
    decision = evaluate_live_oscillation_abort(
        commanded_speed=0.30,
        recent_speed_samples=[
            SpeedSample(0.0, 0.29),
            SpeedSample(0.2, 0.34),
            SpeedSample(0.4, 0.33),
            SpeedSample(0.6, 0.31),
        ],
        configured_max_speed=0.30,
        abort_on_live_oscillation=True,
    )

    assert decision.should_abort
    assert not decision.unsafe_speed_detected
    assert decision.reason == "strict_live_oscillation_abort"


def test_recommend_drive_pid_params_increases_feedforward_when_robot_is_slow() -> None:
    base_params = _params()
    feedforward_trial = compute_trial_metrics(
        name="ff",
        phase="feedforward",
        target_speed=0.30,
        speed_samples=[
            SpeedSample(0.0, 0.18),
            SpeedSample(0.5, 0.20),
            SpeedSample(1.0, 0.21),
            SpeedSample(1.5, 0.22),
        ],
        response_samples=None,
        ticks_seen=120,
        left_ticks_seen=118,
        right_ticks_seen=122,
        params_used=base_params,
        ground_speed_mean=0.21,
        odom_distance_m=1.10,
        rtk_distance_m=1.45,
        notes=(),
    )
    response_trial = compute_trial_metrics(
        name="resp",
        phase="response",
        target_speed=0.30,
        speed_samples=[
            SpeedSample(0.0, 0.20),
            SpeedSample(0.5, 0.22),
            SpeedSample(1.0, 0.24),
            SpeedSample(1.5, 0.25),
        ],
        response_samples=None,
        ticks_seen=140,
        left_ticks_seen=138,
        right_ticks_seen=142,
        params_used=base_params,
        ground_speed_mean=0.23,
        odom_distance_m=1.20,
        rtk_distance_m=1.50,
        notes=(),
    )
    recommended, reasons = recommend_drive_pid_params(base_params, [feedforward_trial], [response_trial])
    assert recommended.wheel_pid_pwm_per_mps > base_params.wheel_pid_pwm_per_mps
    assert recommended.wheel_pid_kp >= base_params.wheel_pid_kp
    assert recommended.ticks_per_meter < base_params.ticks_per_meter
    assert reasons


def test_recommend_drive_pid_params_softens_oscillatory_response() -> None:
    base_params = _params()
    feedforward_trial = compute_trial_metrics(
        name="ff",
        phase="feedforward",
        target_speed=0.20,
        speed_samples=[
            SpeedSample(0.0, 0.20),
            SpeedSample(0.5, 0.21),
            SpeedSample(1.0, 0.20),
        ],
        response_samples=None,
        ticks_seen=100,
        left_ticks_seen=100,
        right_ticks_seen=100,
        params_used=base_params,
        ground_speed_mean=None,
        odom_distance_m=0.5,
        rtk_distance_m=None,
        notes=(),
    )
    response_trial = compute_trial_metrics(
        name="resp",
        phase="response",
        target_speed=0.20,
        speed_samples=[
            SpeedSample(0.0, 0.10),
            SpeedSample(0.2, 0.28),
            SpeedSample(0.4, 0.12),
            SpeedSample(0.6, 0.30),
            SpeedSample(0.8, 0.11),
            SpeedSample(1.0, 0.31),
        ],
        response_samples=None,
        ticks_seen=130,
        left_ticks_seen=126,
        right_ticks_seen=134,
        params_used=base_params,
        ground_speed_mean=None,
        odom_distance_m=0.7,
        rtk_distance_m=None,
        notes=(),
    )
    recommended, _ = recommend_drive_pid_params(base_params, [feedforward_trial], [response_trial])
    assert recommended.wheel_pid_kp < base_params.wheel_pid_kp
    assert recommended.wheel_pid_ki < base_params.wheel_pid_ki
    assert recommended.wheel_pid_integral_limit <= base_params.wheel_pid_integral_limit


def test_compute_trial_metrics_does_not_flag_integral_saturation_with_integral_disabled() -> None:
    params = _early_pid_params(kp=0.02)
    trial = compute_trial_metrics(
        name="pid_step",
        phase="pid",
        target_speed=0.30,
        speed_samples=[
            SpeedSample(0.0, 0.22),
            SpeedSample(0.4, 0.24),
            SpeedSample(0.8, 0.25),
            SpeedSample(1.2, 0.26),
            SpeedSample(1.6, 0.27),
            SpeedSample(2.0, 0.27),
        ],
        response_samples=None,
        ticks_seen=120,
        left_ticks_seen=118,
        right_ticks_seen=122,
        params_used=params,
        ground_speed_mean=None,
        odom_distance_m=0.8,
        rtk_distance_m=None,
        notes=(),
    )

    assert not trial.integral_saturation_suspected


def test_compute_trial_metrics_does_not_flag_integral_saturation_for_mild_overspeed() -> None:
    params = DrivePidParams(
        ticks_per_meter=533.0,
        wheel_pid_kp=0.2,
        wheel_pid_ki=0.4,
        wheel_pid_kd=0.0,
        wheel_pid_integral_limit=5.0,
        wheel_pid_pwm_per_mps=345.0,
    )
    trial = compute_trial_metrics(
        name="pid_step",
        phase="pid",
        target_speed=0.30,
        speed_samples=[
            SpeedSample(0.0, 0.29),
            SpeedSample(0.4, 0.31),
            SpeedSample(0.8, 0.312),
            SpeedSample(1.2, 0.311),
            SpeedSample(1.6, 0.309),
            SpeedSample(2.0, 0.310),
        ],
        response_samples=None,
        ticks_seen=120,
        left_ticks_seen=119,
        right_ticks_seen=121,
        params_used=params,
        ground_speed_mean=None,
        odom_distance_m=0.9,
        rtk_distance_m=None,
        notes=(),
    )

    assert not trial.integral_saturation_suspected


def test_compute_trial_metrics_stop_trial_warns_on_residual_motion_without_flagging_stall() -> None:
    params = DrivePidParams(
        ticks_per_meter=345.0,
        wheel_pid_kp=0.2,
        wheel_pid_ki=0.25,
        wheel_pid_kd=0.0,
        wheel_pid_integral_limit=5.0,
        wheel_pid_pwm_per_mps=345.0,
    )
    trial = compute_trial_metrics(
        name="pid_stop",
        phase="pid",
        target_speed=0.0,
        speed_samples=[
            SpeedSample(0.0, 0.09),
            SpeedSample(0.5, 0.07),
            SpeedSample(1.0, 0.06),
            SpeedSample(1.5, 0.04),
            SpeedSample(2.0, 0.03),
            SpeedSample(2.5, 0.03),
        ],
        response_samples=None,
        ticks_seen=120,
        left_ticks_seen=118,
        right_ticks_seen=122,
        params_used=params,
        ground_speed_mean=None,
        odom_distance_m=0.2,
        rtk_distance_m=None,
        notes=(),
    )

    assert not trial.stall_detected
    assert trial.trial_quality == "warning"
    assert any("Stop behavior warning:" in warning for warning in trial.warnings)


def test_recommend_pid_only_params_keeps_early_pid_conservative_after_good_ff() -> None:
    base_params = _early_pid_params()
    response_trial = compute_trial_metrics(
        name="pid_step",
        phase="pid",
        target_speed=0.30,
        speed_samples=[
            SpeedSample(0.0, 0.29),
            SpeedSample(0.2, 0.31),
            SpeedSample(0.4, 0.29),
            SpeedSample(0.6, 0.30),
            SpeedSample(0.8, 0.31),
            SpeedSample(1.0, 0.29),
        ],
        response_samples=None,
        ticks_seen=120,
        left_ticks_seen=118,
        right_ticks_seen=122,
        params_used=base_params,
        ground_speed_mean=None,
        odom_distance_m=0.9,
        rtk_distance_m=None,
        live_oscillation_detected=True,
        notes=(),
    )

    recommended, reasons = recommend_pid_only_params(base_params, [response_trial])

    assert 0.0 < recommended.wheel_pid_kp <= 0.05
    assert recommended.wheel_pid_ki == 0.0
    assert recommended.wheel_pid_kd == 0.0
    assert recommended.wheel_pid_integral_limit == 0.0
    assert any("keeping KI at 0.0" in reason for reason in reasons)
    assert any("Kd remains 0.0" in reason for reason in reasons)


def test_recommend_pid_only_params_uses_small_kp_steps_from_low_gain_start() -> None:
    base_params = _early_pid_params(kp=0.2)
    response_trial = compute_trial_metrics(
        name="pid_step",
        phase="pid",
        target_speed=0.30,
        speed_samples=[
            SpeedSample(0.0, 0.22),
            SpeedSample(0.4, 0.24),
            SpeedSample(0.8, 0.27),
            SpeedSample(1.2, 0.28),
            SpeedSample(1.8, 0.28),
            SpeedSample(2.4, 0.28),
        ],
        response_samples=None,
        ticks_seen=120,
        left_ticks_seen=118,
        right_ticks_seen=122,
        params_used=base_params,
        ground_speed_mean=None,
        odom_distance_m=0.9,
        rtk_distance_m=None,
        notes=(),
    )

    recommended, _ = recommend_pid_only_params(base_params, [response_trial])

    assert 0.2 < recommended.wheel_pid_kp <= 0.5
    assert 0.0 < round(recommended.wheel_pid_kp - base_params.wheel_pid_kp, 3) <= 0.1
    assert recommended.wheel_pid_ki == 0.0
    assert recommended.wheel_pid_kd == 0.0
    assert recommended.wheel_pid_integral_limit == 0.0


def test_recommend_pid_only_params_uses_fine_steps_for_sub_tenth_kp() -> None:
    base_params = _early_pid_params(kp=0.01)
    response_trial = compute_trial_metrics(
        name="pid_step",
        phase="pid",
        target_speed=0.30,
        speed_samples=[
            SpeedSample(0.0, 0.21),
            SpeedSample(0.4, 0.23),
            SpeedSample(0.8, 0.25),
            SpeedSample(1.2, 0.26),
            SpeedSample(1.6, 0.27),
            SpeedSample(2.0, 0.27),
        ],
        response_samples=None,
        ticks_seen=120,
        left_ticks_seen=118,
        right_ticks_seen=122,
        params_used=base_params,
        ground_speed_mean=None,
        odom_distance_m=0.9,
        rtk_distance_m=None,
        notes=(),
    )

    recommended, _ = recommend_pid_only_params(base_params, [response_trial])

    assert base_params.wheel_pid_kp < recommended.wheel_pid_kp < 0.1
    assert round(recommended.wheel_pid_kp - base_params.wheel_pid_kp, 3) <= 0.05
    assert recommended.wheel_pid_ki == 0.0
    assert recommended.wheel_pid_kd == 0.0
    assert recommended.wheel_pid_integral_limit == 0.0


def test_recommend_pid_only_params_flags_imbalance_without_forcing_derivative() -> None:
    base_params = _params()
    response_trial = compute_trial_metrics(
        name="pid_step",
        phase="pid",
        target_speed=0.30,
        speed_samples=[
            SpeedSample(0.0, 0.18),
            SpeedSample(0.2, 0.36),
            SpeedSample(0.4, 0.17),
            SpeedSample(0.6, 0.35),
            SpeedSample(0.8, 0.18),
            SpeedSample(1.0, 0.34),
        ],
        response_samples=None,
        ticks_seen=120,
        left_ticks_seen=96,
        right_ticks_seen=144,
        params_used=base_params,
        ground_speed_mean=None,
        odom_distance_m=0.9,
        rtk_distance_m=None,
        notes=(),
    )

    recommended, reasons = recommend_pid_only_params(base_params, [response_trial])

    assert recommended.wheel_pid_kd == base_params.wheel_pid_kd
    assert any("Left/right wheel response diverged" in reason for reason in reasons)


def test_recommend_pid_only_params_uses_post_analysis_oscillation_without_blocking_light_robot() -> None:
    base_params = DrivePidParams(
        ticks_per_meter=345.0,
        wheel_pid_kp=0.2,
        wheel_pid_ki=0.25,
        wheel_pid_kd=0.0,
        wheel_pid_integral_limit=5.0,
        wheel_pid_pwm_per_mps=345.0,
    )
    low_speed_trial = _trial(
        params=base_params,
        name="pid_step_1",
        target_speed=0.20,
        measured_speed_mean=0.218,
        overshoot=0.018,
        error_mean=-0.018,
    )
    step_trial = _trial(
        params=base_params,
        name="pid_step_2",
        target_speed=0.30,
        measured_speed_mean=0.312,
        overshoot=0.020,
        error_mean=-0.012,
        oscillation_detected=True,
        trial_quality="warning",
        warnings=("Post-trial oscillation signature detected in the recorded speed response.",),
    )
    stop_trial = _trial(
        params=base_params,
        name="pid_step_stop",
        target_speed=0.0,
        measured_speed_mean=0.0,
        overshoot=0.0,
        error_mean=0.0,
        trial_quality="warning",
        warnings=("Stop behavior captured for diagnostics only.",),
        settling_time=0.4,
    )

    recommended, reasons = recommend_pid_only_params(
        base_params,
        [low_speed_trial, step_trial, stop_trial],
        robot_mass_kg=10.0,
    )

    assert recommended.wheel_pid_ki < base_params.wheel_pid_ki
    assert recommended.wheel_pid_kp == base_params.wheel_pid_kp
    assert recommended.wheel_pid_kd == 0.0
    assert any("Post-analysis oscillation detected but no live oscillation observed" in reason for reason in reasons)
    assert any("Zero-speed stop trial excluded" in reason for reason in reasons)
    assert any("Low-speed tracking indicates possible excessive integral action" in reason for reason in reasons)
    assert any("Drivetrain symmetry is good." in reason for reason in reasons)
    assert any("Left/right tick balance is within expected limits." in reason for reason in reasons)


def test_recommend_pid_only_params_exposes_stop_warning_even_when_motion_trial_looks_valid() -> None:
    base_params = DrivePidParams(
        ticks_per_meter=345.0,
        wheel_pid_kp=0.2,
        wheel_pid_ki=0.25,
        wheel_pid_kd=0.0,
        wheel_pid_integral_limit=5.0,
        wheel_pid_pwm_per_mps=345.0,
    )
    motion_trial = _trial(
        params=base_params,
        name="pid_step_motion",
        target_speed=0.30,
        measured_speed_mean=0.300,
        overshoot=0.010,
        error_mean=0.000,
    )
    stop_trial = _trial(
        params=base_params,
        name="pid_step_stop",
        target_speed=0.0,
        measured_speed_mean=0.07,
        overshoot=0.11,
        error_mean=-0.07,
        trial_quality="warning",
        warnings=("Stop behavior warning: residual motion detected after zero-speed command.",),
        settling_time=1.5,
    )

    recommended, reasons = recommend_pid_only_params(
        base_params,
        [motion_trial, stop_trial],
        robot_mass_kg=10.0,
    )

    assert recommended == base_params
    assert any("Zero-speed stop trial excluded" in reason for reason in reasons)
    assert any("Stop behavior warning: residual motion detected after zero-speed command." in reason for reason in reasons)


def test_recommend_pid_only_params_does_not_ignore_one_dangerous_overshoot_when_median_is_calm() -> None:
    base_params = DrivePidParams(
        ticks_per_meter=345.0,
        wheel_pid_kp=0.2,
        wheel_pid_ki=0.25,
        wheel_pid_kd=0.0,
        wheel_pid_integral_limit=5.0,
        wheel_pid_pwm_per_mps=345.0,
    )
    trials = [
        _trial(
            params=base_params,
            name="pid_step_1",
            target_speed=0.20,
            measured_speed_mean=0.20,
            overshoot=0.005,
            error_mean=0.0,
        ),
        _trial(
            params=base_params,
            name="pid_step_2",
            target_speed=0.30,
            measured_speed_mean=0.305,
            overshoot=0.090,
            error_mean=-0.005,
        ),
        _trial(
            params=base_params,
            name="pid_step_3",
            target_speed=0.10,
            measured_speed_mean=0.10,
            overshoot=0.000,
            error_mean=0.0,
        ),
    ]

    recommended, reasons = recommend_pid_only_params(
        base_params,
        trials,
        robot_mass_kg=10.0,
    )

    assert recommended.wheel_pid_ki < base_params.wheel_pid_ki
    assert any("Worst-step overshoot reached" in reason for reason in reasons)


def test_finite_or_none_rejects_nan_and_inf_values() -> None:
    assert finite_or_none(float("nan")) is None
    assert finite_or_none(float("inf")) is None
    assert finite_or_none(float("-inf")) is None
    assert finite_or_none(8.76, positive=True) == 8.76


def test_drive_pid_params_from_mapping_rejects_non_finite_values() -> None:
    try:
        DrivePidParams.from_mapping(
            {
                "ticks_per_meter": float("nan"),
                "wheel_pid_kp": 0.2,
                "wheel_pid_ki": 0.0,
                "wheel_pid_kd": 0.0,
                "wheel_pid_integral_limit": 0.0,
                "wheel_pid_pwm_per_mps": 345.0,
            }
        )
    except ValueError as exc:
        assert "ticks_per_meter" in str(exc)
    else:
        raise AssertionError("Expected non-finite drive parameters to be rejected")


def test_sanitize_finite_data_replaces_non_finite_values_before_yaml_or_json_serialization() -> None:
    sanitized = sanitize_finite_data(
        {
            "robot_mass_kg": float("nan"),
            "drivetrain_diagnostics": {
                "wheel_radius_m": float("inf"),
                "configured_ticks_per_revolution": 1964.0 / 3.0,
            },
            "trials": [
                {
                    "overshoot": float("-inf"),
                    "warnings": ["Stop behavior warning: residual motion detected after zero-speed command."],
                }
            ],
        }
    )

    assert sanitized["robot_mass_kg"] is None
    assert sanitized["drivetrain_diagnostics"]["wheel_radius_m"] is None
    assert sanitized["trials"][0]["overshoot"] is None
    yaml_text = yaml.safe_dump(sanitized, sort_keys=False)
    assert "nan" not in yaml_text.lower()
    assert "inf" not in yaml_text.lower()
    json.dumps(sanitized, allow_nan=False)


def test_recommend_pid_only_params_uses_mass_tier_aware_worst_overshoot_threshold() -> None:
    base_params = DrivePidParams(
        ticks_per_meter=345.0,
        wheel_pid_kp=0.2,
        wheel_pid_ki=0.25,
        wheel_pid_kd=0.0,
        wheel_pid_integral_limit=5.0,
        wheel_pid_pwm_per_mps=345.0,
    )
    borderline_trial = _trial(
        params=base_params,
        name="pid_step_borderline",
        target_speed=0.30,
        measured_speed_mean=0.304,
        overshoot=0.045,
        error_mean=-0.004,
    )

    _, light_reasons = recommend_pid_only_params(
        base_params,
        [borderline_trial],
        robot_mass_kg=10.0,
    )
    _, heavy_reasons = recommend_pid_only_params(
        base_params,
        [borderline_trial],
        robot_mass_kg=45.0,
    )

    assert not any("Worst-step overshoot reached" in reason for reason in light_reasons)
    assert any("Worst-step overshoot reached" in reason for reason in heavy_reasons)


def test_recommend_pid_only_params_uses_smaller_kp_steps_for_heavy_robot() -> None:
    base_params = _early_pid_params(kp=0.2)
    lagging_trial = _trial(
        params=base_params,
        name="pid_step",
        target_speed=0.30,
        measured_speed_mean=0.255,
        overshoot=0.008,
        error_mean=0.045,
        settling_time=3.2,
        left_right_tick_imbalance=0.004,
    )

    recommended_light, _ = recommend_pid_only_params(
        base_params,
        [lagging_trial],
        robot_mass_kg=8.0,
    )
    recommended_heavy, _ = recommend_pid_only_params(
        base_params,
        [lagging_trial],
        robot_mass_kg=45.0,
    )

    assert recommended_light.wheel_pid_kp > base_params.wheel_pid_kp
    assert recommended_heavy.wheel_pid_kp > base_params.wheel_pid_kp
    assert (recommended_light.wheel_pid_kp - base_params.wheel_pid_kp) > (
        recommended_heavy.wheel_pid_kp - base_params.wheel_pid_kp
    )
