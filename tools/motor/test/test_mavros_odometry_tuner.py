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
    TrialMetrics,
    resolve_robot_tuning_tier,
)
from mowgli_tools.drive_pid_tuner import DrivePidTuner, _build_parser  # noqa: E402
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
    tuner._run_feedforward_session = Mock(return_value=(recommended, [], []))
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
    trial = replace(wheel_trial(), right_ticks_seen=1_030_800)
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
