import json
from pathlib import Path
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import yaml

pytest.importorskip(
    "geometry_msgs.msg", reason="Source ROS setup to test the real tuner methods"
)

from mowgli_tools.drive_pid_math import (  # noqa: E402
    DrivePidParams,
    OdometryParams,
    MotorCounterSample,
    SpeedSample,
    TickSample,
    TrialMetrics,
    resolve_robot_tuning_tier,
    motor_revolutions_between,
)
from mowgli_tools.drive_pid_tuner import (  # noqa: E402
    DrivePidTuner, RtkPoseSample, TrialRecorder, TrialAbortedError, _build_parser,
)
from mowgli_interfaces.msg import GnssStatus  # noqa: E402
from mowgli_tools.robot_hardware_config import extract_robot_hardware_config  # noqa: E402


@pytest.fixture
def tuner(tmp_path):
    node = object.__new__(DrivePidTuner)
    node._args = _build_parser().parse_args(
        [
            "--hardware-backend",
            "mavros",
            "--mode",
            "ff",
            "--passes",
            "3",
            "--auto-turn",
            "--output",
            str(tmp_path / "report.yaml"),
        ]
    )
    node._cmd_topic = "/cmd_vel_tuning"
    node._backup_path = tmp_path / "backup.yaml"
    node._robot_hardware_config = extract_robot_hardware_config({}, "/robot.yaml")
    node._tuning_tier = resolve_robot_tuning_tier(None)
    node._latest_status = None
    node._latest_gnss_status = None
    node.get_logger = Mock(return_value=Mock())
    node._build_status_snapshot = Mock(return_value={})
    node._apply_drive_pid_params = Mock()
    node._turn_around = Mock()
    node._hold_zero = Mock()
    return node


def wheel_trial(coefficient=343.596, distance=3.0):
    count = round(coefficient * distance * 1000)
    return TrialMetrics(
        name="odom_pass",
        phase="odometry",
        target_speed=0.3,
        measured_speed_mean=0.3,
        measured_speed_rms=0.3,
        error_mean=0.0,
        error_rms=0.0,
        overshoot=0.0,
        settling_time=None,
        ticks_seen=2 * count,
        left_ticks_seen=count,
        right_ticks_seen=count,
        stall_detected=False,
        oscillation_detected=False,
        integral_saturation_suspected=False,
        params_used={"ticks_per_meter": 300.125},
        rtk_accepted=True,
        rtk_distance_m=distance,
        odom_distance_m=3.0,
        calibration_window={"ticks_per_meter": coefficient, "counter_unit": "motor_revolution"},
    )


def test_mavros_passes_keep_live_reference_and_recommend_float_ticks_only(tuner):
    measured = [wheel_trial(343.581), wheel_trial(343.607), wheel_trial(343.599)]
    tuner._run_speed_trial = Mock(side_effect=measured)
    original = OdometryParams(300.125)
    recommendation, trials, reasons = tuner._run_feedforward_session(original)
    assert recommendation.ticks_per_meter == pytest.approx(
        (343.581 + 343.607 + 343.599) / 3
    )
    assert recommendation.to_dict().keys() == {"ticks_per_meter"}
    assert trials == measured
    assert all(
        call.kwargs["params"] == original
        for call in tuner._run_speed_trial.call_args_list
    )
    tuner._apply_drive_pid_params.assert_not_called()
    assert tuner._turn_around.call_count == 2
    assert not any("PWM" in reason or "wheel_pid" in reason for reason in reasons)


@pytest.mark.parametrize("apply", [False, True])
def test_mavros_report_only_writes_no_live_parameters_and_apply_writes_only_ticks(
    tuner, apply
):
    tuner._args.apply = apply
    original, recommended = OdometryParams(300.125), OdometryParams(343.596123456789)
    tuner._get_drive_pid_params = Mock(return_value=original)
    tuner._run_feedforward_session = Mock(return_value=(recommended, [wheel_trial(recommended.ticks_per_meter)], []))
    for name in (
        "_wait_for_initial_state",
        "_ensure_safe_to_start",
        "_log_motion_gate_state",
        "_enter_recording_if_needed",
        "_stop_robot",
        "_cancel_recording_if_needed",
        "_print_summary",
    ):
        setattr(tuner, name, Mock())
    assert tuner.run() == 0
    if apply:
        tuner._apply_drive_pid_params.assert_called_once_with(recommended)
    else:
        tuner._apply_drive_pid_params.assert_not_called()
    with open(tuner._args.output) as stream:
        report = yaml.safe_load(stream)
    assert report["proposed_params"] == {"ticks_per_meter": 343.596123456789}
    assert report["recommended"] == report["proposed_params"]
    assert report["applied_live"] is apply
    assert "wheel_pid" not in yaml.safe_dump(report)
    assert "feed-forward" not in yaml.safe_dump(report)
    backup = yaml.safe_load(tuner._backup_path.read_text())
    assert backup["parameters"] == original.to_dict()


def test_mavros_report_retains_separate_wheels_and_each_pass_coefficient(tuner):
    trial = replace(wheel_trial(), right_ticks_seen=1_030_800,
                    calibration_window={"ticks_per_meter": 343.598, "counter_unit": "motor_revolution"})
    tuner._write_result_file(
        mode="ff",
        current_params=OdometryParams(300.125),
        starting_params=OdometryParams(300.125),
        proposed_params=OdometryParams(343.598),
        applied_live=False,
        trials=[trial],
        reasons=[],
    )
    with open(tuner._args.output) as stream:
        report = yaml.safe_load(stream)
    assert report["trials"][0]["left_ticks_seen"] == 1_030_788
    assert report["trials"][0]["right_ticks_seen"] == 1_030_800
    assert report["trials"][0]["ticks_per_meter"] == pytest.approx(343.598)
    assert report["trials"][0]["params_used"] == {"ticks_per_meter": 300.125}
    assert "wheel_pid" not in yaml.safe_dump(report)


def test_mavros_parameter_client_reads_and_applies_only_double_ticks(tuner):
    response = SimpleNamespace(values=[SimpleNamespace(double_value=343.596123456789)])
    client = Mock()
    client.get_parameters.return_value.result.return_value = response
    client.set_parameters.return_value.result.return_value = SimpleNamespace(
        results=[SimpleNamespace(successful=True)]
    )
    tuner._odometry_parameter_client = client
    tuner._parameter_client = Mock()
    tuner._wait_for_future = lambda future, **_: future.result()
    params = DrivePidTuner._get_drive_pid_params(tuner)
    assert params.to_dict() == {"ticks_per_meter": 343.596123456789}
    client.get_parameters.assert_called_once_with(["ticks_per_meter"])
    DrivePidTuner._apply_drive_pid_params(tuner, params)
    updates = client.set_parameters.call_args.args[0]
    assert len(updates) == 1
    assert updates[0].name == "ticks_per_meter"
    assert updates[0].value == 343.596123456789
    assert updates[0].type_.name == "DOUBLE"
    tuner._parameter_client.get_parameters.assert_not_called()
    tuner._parameter_client.set_parameters.assert_not_called()


def test_mavros_legacy_backup_and_cli_overrides_cannot_restore_drive_gains(tuner):
    tuner._backup_path.write_text(
        "parameters:\n  ticks_per_meter: 343.596\n  wheel_pid_pwm_per_mps: 999.0\n"
    )
    assert tuner._load_backup().to_dict() == {"ticks_per_meter": 343.596}
    tuner._args.reset_to_profile = True
    tuner._args.custom_pwm_per_mps = 999
    original = OdometryParams(300.125)
    assert tuner._resolve_starting_params(original) == original


def test_stm32_session_still_refines_odometry_and_feedforward(tuner):
    tuner._args.hardware_backend = "mowgli"
    tuner._args.passes = 1
    original = DrivePidParams(300.125, 10.0, 2000.0, 0.0, 45.0, 200.0)
    tuner._run_speed_trial = Mock(return_value=replace(wheel_trial(), measured_speed_mean=0.2))
    recommended, _, _ = tuner._run_feedforward_session(original)
    assert recommended.ticks_per_meter == 300.125
    assert recommended.wheel_pid_pwm_per_mps == pytest.approx(300.0)
    assert recommended.wheel_pid_kp == original.wheel_pid_kp
    assert recommended.wheel_pid_ki == original.wheel_pid_ki


@pytest.mark.parametrize("rtk_fixed", [True, False])
def test_mavros_real_metrics_session_to_report_without_motion(tuner, monkeypatch, rtk_fixed):
    tuner._args.passes = 1
    tuner._args.test_speed = 0.4
    tuner._latest_odom_time = 100.0
    tuner._latest_gnss_status = object()
    tuner._get_drive_pid_params = Mock(return_value=OdometryParams(343.0))
    for name in (
        "_wait_for_initial_state", "_ensure_safe_to_start", "_log_motion_gate_state",
        "_enter_recording_if_needed", "_stop_robot", "_cancel_recording_if_needed",
    ):
        setattr(tuner, name, Mock())
    # No ROS node is initialized; all motion/BT endpoints are disconnected mocks.
    tuner._publish_cmd = Mock(side_effect=AssertionError("Unexpected motion command"))
    clock = [100.0]
    monkeypatch.setattr("mowgli_tools.drive_pid_tuner.time.monotonic", lambda: clock[0])

    def simulated_segment(recorder):
        recorder.speed_samples = [SpeedSample(100 + i, 0.4) for i in range(1, 8)]
        recorder.tick_samples = [
            TickSample(101.0, 0, 0), TickSample(107.0, 823200, 823200),
        ]
        recorder.rtk_samples = [
            RtkPoseSample(100 + i, (i - 1) * 0.4, 0.0, 0.01, True, True,
                          GnssStatus.RTK_MODE_FIXED if rtk_fixed else GnssStatus.RTK_MODE_FLOAT,
                          "FIXED" if rtk_fixed else "FLOAT",
                          observation_stamp_ns=(100 + i) * 1_000_000_000,
                          observation_sequence=i, backend="mavros_gps2", provenance_valid=True)
            for i in range(1, 8)
        ]
        recorder.left_motor_samples = [
            MotorCounterSample((100 + i) * 1_000_000_000, (i - 1) * 137.2,
                               "ardupilot_legacy", 1, 1) for i in range(1, 8)
        ]
        recorder.right_motor_samples = list(recorder.left_motor_samples)
        clock[0] = 110.0

    tuner._drive_segment = simulated_segment
    if rtk_fixed:
        assert tuner.run() == 0
    else:
        with pytest.raises(RuntimeError, match="No trustworthy aligned motor/RTK window"):
            tuner.run()
    with open(tuner._args.output) as stream:
        report = yaml.safe_load(stream)
    assert report["calibration_scope"] == "odometry"
    assert report["proposed_params"] == ({"ticks_per_meter": pytest.approx(343.0)} if rtk_fixed else {})
    assert "wheel_pid" not in yaml.safe_dump(report)
    if rtk_fixed:
        trial = report["trials"][0]
        assert trial["rtk_accepted"] is True
        assert trial["rtk_distance_m"] == pytest.approx(2.4)
        assert trial["odom_distance_m"] == pytest.approx(2.4)
        assert trial["stall_detected"] is False
        assert trial["params_used"] == {"ticks_per_meter": 343.0}
        assert "integral_saturation_suspected" not in trial
    else:
        assert "No trustworthy aligned motor/RTK window" in report["failure_message"]
    tuner._publish_cmd.assert_not_called()
    tuner._apply_drive_pid_params.assert_not_called()
    tuner._stop_robot.assert_called()
    tuner._cancel_recording_if_needed.assert_called_once()


def test_mavros_aborted_speed_trial_keeps_metrics_without_pid_exception(tuner):
    tuner._latest_odom_time = 0.0
    tuner._stop_robot = Mock()

    def aborted_segment(recorder):
        recorder.speed_samples = [SpeedSample(i * 0.2, 0.051) for i in range(16)]
        recorder.tick_samples = [TickSample(0, 0, 0), TickSample(3, 12534, 12818)]
        raise RuntimeError("live_recent_speed_below_floor_with_ticks")

    tuner._drive_segment = aborted_segment
    with pytest.raises(TrialAbortedError, match="live_recent_speed_below_floor_with_ticks") as caught:
        tuner._run_speed_trial(
            name="odom_pass", phase="odometry", initial_speed=0.0,
            target_speed=0.4, params=OdometryParams(343.0), duration_s=8.5,
        )
    metrics = caught.value.metrics
    assert metrics.stall_detected is False  # underspeed alone is not physical immobilisation
    assert metrics.rtk_accepted is False
    assert metrics.integral_saturation_suspected is False
    assert metrics.left_ticks_seen == 12534
    assert metrics.right_ticks_seen == 12818
    assert metrics.params_used == {"ticks_per_meter": 343.0}
    assert any("Partial trial metrics recorded after abort" in note for note in metrics.notes)
    tuner._stop_robot.assert_called_once()
    assert tuner._active_trial is None


@pytest.fixture
def recorded_trial(tuner):
    data = json.loads((Path(__file__).parent / "fixtures/odom_second_trial.json").read_text())
    tuner._latest_gnss_status = GnssStatus(
        fix_valid=True, corrections_active=True, rtk_mode=GnssStatus.RTK_MODE_FIXED,
    )
    rec = TrialRecorder(
        "second_real_trial", "odometry", 0, .4, OdometryParams(343),
        data["trial_start_s"], 1, 8.5, actual_end_s=data["trial_start_s"] + 8.75,
    )
    rec.speed_samples = [SpeedSample(*row) for row in data["speed_samples"]]
    rec.tick_samples = [TickSample(*row) for row in data["tick_samples"]]
    rec.rtk_samples = [
        RtkPoseSample(t, x, y, .014, True, True, GnssStatus.RTK_MODE_FIXED,
                      "FIXED", stamp, seq, "mavros_gps2", True)
        for t, x, y, stamp, seq in data["rtk_samples"]
    ]
    for wheel in ("left", "right"):
        setattr(rec, wheel + "_motor_samples", [
            MotorCounterSample(stamp, rev, "ardupilot_legacy", epoch, segment, True, delivery)
            for stamp, rev, epoch, segment, delivery in data[wheel + "_motor_samples"]
        ])
    return rec


def finalize(tuner, rec):
    return tuner._finalize_trial_with_options(rec, allow_incomplete=False, additional_notes=[])


def test_recorded_trial_low_odometry_does_not_mean_stall_and_window_excludes_ramps(tuner, recorded_trial):
    rec = recorded_trial
    metrics = finalize(tuner, rec)
    assert metrics.stall_detected is False
    assert metrics.rtk_accepted is True
    assert any("not proof of physical stall" in warning for warning in metrics.warnings)
    assert metrics.measured_speed_mean == pytest.approx(.0504521119541996)
    assert metrics.ground_speed_mean == pytest.approx(.40400444440119526)
    assert metrics.calibration_window["ticks_per_meter"] == pytest.approx(43.96782262055455)
    assert metrics.calibration_window["left_epoch"] == 2
    assert metrics.calibration_window["left_segment"] == 1
    # Huge unrelated unsigned tick totals do not enter the motor-revolution fit.
    rec.tick_samples[-1] = replace(rec.tick_samples[-1], left_ticks=99_000_000, right_ticks=99_000_000)
    assert finalize(tuner, rec).calibration_window == metrics.calibration_window
    assert metrics.integral_saturation_suspected is False
    plateau_start = rec.command_start_s + rec.ramp_time_s
    plateau_end = rec.command_start_s + rec.command_duration_s - rec.ramp_time_s
    # Bad measurements outside the plateau cannot pollute or extend the fit.
    rec.rtk_samples = [p if plateau_start <= p.time_s <= plateau_end
                       else replace(p, provenance_valid=False, x_m=999) for p in rec.rtk_samples]
    assert finalize(tuner, rec).calibration_window == metrics.calibration_window


@pytest.mark.parametrize("fault", [
    "absent", "float", "immobile", "slip", "one_wheel", "cached_gnss", "rtk_gap", "rtk_restart",
    "epoch", "segment", "counter_reset", "counter_gap", "cached_counter", "stale_pose", "missing_identity",
])
@pytest.mark.parametrize("apply", [False, True])
def test_odometry_rejects_real_motion_and_provenance_failures(tuner, recorded_trial, fault, apply):
    rec = recorded_trial
    if fault == "absent":
        tuner._latest_gnss_status = None
    if fault == "float":
        rec.rtk_samples = [replace(p, rtk_mode=GnssStatus.RTK_MODE_FLOAT) for p in rec.rtk_samples]
    if fault in ("immobile", "slip"):
        rec.rtk_samples = [replace(p, x_m=0, y_m=0) for p in rec.rtk_samples]
    if fault in ("immobile", "one_wheel"):
        rec.tick_samples = [replace(p, left_ticks=0, right_ticks=0 if fault == "immobile" else p.right_ticks)
                            for p in rec.tick_samples]
    if fault == "missing_identity":
        rec.rtk_samples = [replace(p, observation_sequence=0, backend="") for p in rec.rtk_samples]
    if fault == "cached_gnss":
        rec.rtk_samples = [replace(p, observation_sequence=1) for p in rec.rtk_samples]
    if fault == "rtk_gap":
        rec.rtk_samples = [rec.rtk_samples[3], rec.rtk_samples[13]]
    if fault == "rtk_restart":
        rec.rtk_samples[7] = replace(rec.rtk_samples[7], observation_sequence=1)
    if fault == "stale_pose":
        rec.rtk_samples[7] = replace(rec.rtk_samples[7], provenance_valid=False)
    if fault in ("epoch", "segment"):
        rec.left_motor_samples[7] = replace(rec.left_motor_samples[7], **{fault: 99})
    if fault == "counter_reset":
        rec.left_motor_samples[7] = replace(rec.left_motor_samples[7], revolutions=0)
    if fault == "counter_gap":
        rec.left_motor_samples = [rec.left_motor_samples[3], rec.left_motor_samples[14]]
    if fault == "cached_counter":
        rec.left_motor_samples = [rec.left_motor_samples[3]] * 20
    metrics = finalize(tuner, rec)
    assert metrics.rtk_accepted is False
    assert metrics.calibration_window is None
    tuner._args.passes = 1
    tuner._args.apply = apply
    tuner._get_drive_pid_params = Mock(return_value=OdometryParams(343))
    tuner._run_speed_trial = Mock(return_value=metrics)
    for name in ("_wait_for_initial_state", "_ensure_safe_to_start", "_log_motion_gate_state",
                 "_enter_recording_if_needed", "_stop_robot", "_cancel_recording_if_needed", "_print_summary"):
        setattr(tuner, name, Mock())
    with pytest.raises(TrialAbortedError):
        tuner.run()
    report = yaml.safe_load(Path(tuner._args.output).read_text())
    assert report["trials"][0]["rtk_accepted"] is False
    assert "calibration_window" not in report["trials"][0]
    assert "ticks_per_meter" not in report["trials"][0]
    assert report["recommended"] == report["proposed_params"] == {}
    assert report["applied_live"] is False
    assert report["failure_message"]
    assert report["trials"][0]["notes"] or metrics.stall_detected
    if fault in ("immobile", "slip", "one_wheel"):
        assert metrics.stall_detected is True
    if apply:
        # Existing failure cleanup restores the baseline, never a rejected fit.
        tuner._apply_drive_pid_params.assert_called_once_with(OdometryParams(343))
    else:
        tuner._apply_drive_pid_params.assert_not_called()


@pytest.mark.parametrize("failed", [False, True])
def test_recorded_calibration_report_retains_success_and_rejected_trial(tuner, recorded_trial, failed):
    tuner._args.passes = 1
    if failed:
        recorded_trial.left_motor_samples[7] = replace(recorded_trial.left_motor_samples[7], segment=99)
    metrics = finalize(tuner, recorded_trial)
    tuner._get_drive_pid_params = Mock(return_value=OdometryParams(343))
    tuner._run_speed_trial = Mock(return_value=metrics)
    for name in ("_wait_for_initial_state", "_ensure_safe_to_start", "_log_motion_gate_state",
                 "_enter_recording_if_needed", "_stop_robot", "_cancel_recording_if_needed", "_print_summary"):
        setattr(tuner, name, Mock())
    if failed:
        with pytest.raises(TrialAbortedError):
            tuner.run()
    else:
        assert tuner.run() == 0
    report = yaml.safe_load(Path(tuner._args.output).read_text())
    assert len(report["trials"]) == 1
    assert report["trials"][0]["params_used"] == {"ticks_per_meter": 343.0}
    assert report["applied_live"] is False
    assert "wheel_pid" not in yaml.safe_dump(report)
    if failed:
        assert report["proposed_params"] == {}
        assert report["recommended"] == {}
        assert report["trials"][0]["rtk_accepted"] is False
        assert "discontinuity" in " ".join(report["trials"][0]["notes"])
    else:
        assert report["recommended"]["ticks_per_meter"] == pytest.approx(43.96782262055455)
    tuner._stop_robot.assert_called()
    tuner._cancel_recording_if_needed.assert_called()
    tuner._apply_drive_pid_params.assert_not_called()


@pytest.mark.parametrize("age, matching, valid", [(2, True, True), (5, True, False),
                                                  (-1, True, False), (2, False, False)])
def test_pose_callback_uses_matching_observation_stamp_and_real_age(
    tuner, recorded_trial, monkeypatch, age, matching, valid,
):
    from mowgli_interfaces.msg import AbsolutePose
    stamp_ns = int((100 - age) * 1e9)
    tuner._gnss_status_history = {}
    status = GnssStatus(fix_valid=True, corrections_active=True, rtk_mode=GnssStatus.RTK_MODE_FIXED,
                        horizontal_accuracy_m=.014, position_observation_sequence=9, backend="mavros_gps2")
    status.header.stamp.sec = stamp_ns // 1_000_000_000 + (0 if matching else 1)
    tuner._on_gnss_status(status)
    pose = AbsolutePose(source=AbsolutePose.SOURCE_GPS, flags=3, position_accuracy=.014)
    pose.header.stamp.sec = stamp_ns // 1_000_000_000
    tuner._active_trial = recorded_trial
    tuner.get_clock = Mock(return_value=Mock(now=Mock(return_value=Mock(nanoseconds=100_000_000_000))))
    monkeypatch.setattr("mowgli_tools.drive_pid_tuner.time.monotonic", lambda: 200)
    tuner._on_absolute_pose(pose)
    sample = recorded_trial.rtk_samples[-1]
    assert sample.provenance_valid is valid
    assert sample.time_s == 200 - age  # explicit clock-domain conversion, not receipt as acquisition


@pytest.mark.parametrize("fault", ["emergency", "charging", "odom_lost", "rtk_lost"])
def test_existing_live_safety_paths_remain_active(tuner, recorded_trial, fault):
    tuner._failure_message = None
    tuner._latest_odom_time = 0
    tuner._latest_emergency = SimpleNamespace(active_emergency=fault == "emergency",
                                              latched_emergency=False, reason="TEST")
    tuner._latest_status = SimpleNamespace(is_charging=fault == "charging")
    if fault == "rtk_lost":
        tuner._latest_gnss_status.fix_valid = False
    now = recorded_trial.speed_samples[-1].time_s + (.1 if fault != "odom_lost" else 5)
    with pytest.raises(RuntimeError):
        tuner._check_live_trial_safety(recorded_trial, now, 5, commanded_speed=.4)


@pytest.mark.parametrize("outside", [False, True])
def test_second_rtk_validation_owns_final_counter_bounds(tuner, recorded_trial, monkeypatch, outside):
    original = tuner._compute_rtk_metrics
    calls = []
    validated = []

    def reselect(**kwargs):
        calls.append(kwargs)
        if len(calls) == 2 and not outside:
            kwargs["steady_start"] += .6
            kwargs["steady_end"] -= .6
        result = original(**kwargs)
        if len(calls) == 2:
            if outside:
                recorded_trial.accepted_rtk_samples.insert(0, recorded_trial.rtk_samples[0])
            validated.extend(recorded_trial.accepted_rtk_samples)
        return result

    monkeypatch.setattr(tuner, "_compute_rtk_metrics", reselect)
    metrics = finalize(tuner, recorded_trial)
    assert len(calls) == 2
    if outside:
        assert metrics.rtk_accepted is False
        assert metrics.calibration_window is None
        assert "outside common motor window" in " ".join(metrics.notes)
        assert metrics.rtk_distance_m is not None  # original raw RTK diagnostics preserved
        return
    w = metrics.calibration_window
    assert metrics.rtk_accepted is True
    assert w["start_stamp_ns"] == validated[0].observation_stamp_ns
    assert w["end_stamp_ns"] == validated[-1].observation_stamp_ns
    assert validated[0].time_s > calls[1]["steady_start"]
    assert validated[-1].time_s < calls[1]["steady_end"]
    assert w["left_motor_revolutions"] == pytest.approx(motor_revolutions_between(
        recorded_trial.left_motor_samples, w["start_stamp_ns"], w["end_stamp_ns"], tuner._args.odom_timeout,
    ))
    assert w["right_motor_revolutions"] == pytest.approx(motor_revolutions_between(
        recorded_trial.right_motor_samples, w["start_stamp_ns"], w["end_stamp_ns"], tuner._args.odom_timeout,
    ))
    assert w["rtk_distance_m"] == metrics.rtk_distance_m
    assert w["ticks_per_meter"] == pytest.approx(
        (w["left_motor_revolutions"] + w["right_motor_revolutions"]) / 2 / metrics.rtk_distance_m
    )
    assert w["ticks_per_meter"] != pytest.approx(43.96782262055455)
    assert validated[0].time_s >= recorded_trial.command_start_s + recorded_trial.ramp_time_s
    assert validated[-1].time_s <= (recorded_trial.command_start_s + recorded_trial.command_duration_s
                                    - recorded_trial.ramp_time_s)


def test_second_pass_rejection_retains_both_trials_without_global_proposal(tuner, recorded_trial):
    good = finalize(tuner, recorded_trial)
    recorded_trial.left_motor_samples[7] = replace(recorded_trial.left_motor_samples[7], segment=99)
    bad = finalize(tuner, recorded_trial)
    tuner._args.passes = 2
    tuner._get_drive_pid_params = Mock(return_value=OdometryParams(48))
    tuner._run_speed_trial = Mock(side_effect=[good, bad])
    for name in ("_wait_for_initial_state", "_ensure_safe_to_start", "_log_motion_gate_state",
                 "_enter_recording_if_needed", "_stop_robot", "_cancel_recording_if_needed", "_print_summary"):
        setattr(tuner, name, Mock())
    with pytest.raises(TrialAbortedError):
        tuner.run()
    report = yaml.safe_load(Path(tuner._args.output).read_text())
    assert len(report["trials"]) == 2
    assert report["trials"][0]["rtk_accepted"] is True  # useful per-pass evidence retained
    assert report["trials"][1]["rtk_accepted"] is False
    assert report["trials"][1]["rtk_distance_m"] is not None
    assert report["proposed_params"] == report["recommended"] == {}
    assert report["current_params"] == {"ticks_per_meter": 48}
    tuner._apply_drive_pid_params.assert_not_called()
