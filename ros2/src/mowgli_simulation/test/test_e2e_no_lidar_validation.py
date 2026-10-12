# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0

import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


def _load_module():
    path = Path(__file__).resolve().parents[2] / 'e2e_test_no_lidar.py'
    spec = importlib.util.spec_from_file_location('no_lidar_validation', path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


e2e = _load_module()


def _node():
    node = SimpleNamespace(
        metrics=e2e.Metrics(start_time=e2e.time.time()), current_bt_state='',
        current_phase=e2e.TestPhase.WAITING, phase_start_time=0.0,
        mowing_started=True, test_complete=False, current_is_charging=False,
        target_path_count=0, completed_paths=0, skipped_paths=0,
        coverage_incomplete=False,
        get_logger=lambda: SimpleNamespace(info=lambda _: None),
    )
    node._track_phase_transition = lambda prev, curr, t: (
        e2e.NoLidarE2ETestNode._track_phase_transition(node, prev, curr, t)
    )
    node._complete_phase = lambda phase, passed, details: (
        e2e.NoLidarE2ETestNode._complete_phase(node, phase, passed, details)
    )
    return node


def _state(node, name, completed=0, skipped=0, sub_state=''):
    msg = SimpleNamespace(
        state_name=name, total_swaths=2,
        completed_swaths=completed, skipped_swaths=skipped,
        sub_state_name=sub_state,
    )
    e2e.NoLidarE2ETestNode._on_bt_status(node, msg)


def _begin_cycle(node):
    for name in ('UNDOCKING', 'PLANNING', 'MOWING'):
        _state(node, name)


def test_mowing_completion_waits_for_actual_charging_dock():
    node = _node()
    _begin_cycle(node)
    _state(node, 'MOWING_COMPLETE', completed=2)
    assert not node.test_complete
    _state(node, 'IDLE_DOCKED', completed=2)
    assert not node.test_complete
    node.current_is_charging = True
    _state(node, 'IDLE_DOCKED', completed=2)
    assert node.test_complete
    assert {p.name for p in node.metrics.phase_results if p.passed} == {
        'UNDOCKING', 'PLANNING', 'MOWING', 'DOCKING',
    }


@pytest.mark.parametrize('completed,skipped', [(0, 0), (1, 0), (2, 1)])
def test_incomplete_or_skipped_coverage_cannot_pass_mowing(completed, skipped):
    node = _node()
    _begin_cycle(node)
    _state(node, 'MOWING_COMPLETE', completed, skipped)
    results = [p for p in node.metrics.phase_results if p.name == 'MOWING']
    assert len(results) == 1
    assert not results[0].passed
    assert not node.test_complete


def test_manual_charging_during_mowing_does_not_complete_cycle():
    node = _node()
    _begin_cycle(node)
    node.current_is_charging = True
    _state(node, 'MANUAL_CHARGING', completed=1)
    assert not node.test_complete


def test_coverage_warning_cannot_be_cleared_by_complete_counters():
    node = _node()
    _begin_cycle(node)
    _state(node, 'MOWING', completed=2, sub_state='COVERAGE_INCOMPLETE')
    _state(node, 'MOWING_COMPLETE', completed=2)
    results = [p for p in node.metrics.phase_results if p.name == 'MOWING']
    assert not results[0].passed
    assert node.coverage_incomplete


def test_late_coverage_warning_invalidates_otherwise_successful_report():
    node = _node()
    _begin_cycle(node)
    _state(node, 'MOWING_COMPLETE', completed=2)
    node.current_is_charging = True
    _state(node, 'IDLE_DOCKED', completed=2, sub_state='COVERAGE_INCOMPLETE')
    _state(node, 'IDLE_DOCKED', completed=2)
    node.metrics.path_deviations = [(0.0, 0.0)]
    assert not e2e.NoLidarE2ETestNode.print_final_report(node)


def test_post_mux_command_uses_stamped_twist_payload():
    node = _node()
    msg = SimpleNamespace(twist=SimpleNamespace(
        linear=SimpleNamespace(x=0.15), angular=SimpleNamespace(z=-0.2),
    ))
    e2e.NoLidarE2ETestNode._on_cmd_vel(node, msg)
    assert node.metrics.cmd_vels[0][1:] == (0.15, -0.2)


def test_partial_successful_phase_list_cannot_pass_final_report():
    node = _node()
    node.metrics.phase_results = [e2e.PhaseResult(name='PLANNING', passed=True)]
    node.metrics.path_deviations = [(0.0, 0.0)]
    assert not e2e.NoLidarE2ETestNode.print_final_report(node)


def test_median_above_stated_tracking_threshold_cannot_pass():
    node = _node()
    node.metrics.phase_results = [
        e2e.PhaseResult(name=name, passed=True)
        for name in ('UNDOCKING', 'PLANNING', 'MOWING', 'DOCKING')
    ]
    node.metrics.path_deviations = [(0.0, 0.15)]
    assert not e2e.NoLidarE2ETestNode.print_final_report(node)


@pytest.mark.parametrize('verdict,exit_code', [(False, 1), (True, 0)])
def test_sigterm_preserves_actual_mission_verdict(monkeypatch, verdict, exit_code):
    cleaned = []
    node = SimpleNamespace(
        get_logger=lambda: SimpleNamespace(info=lambda _: None),
        send_start_command=lambda: True,
        print_final_report=lambda: verdict,
        destroy_node=lambda: cleaned.append('node'),
    )
    monkeypatch.setattr(e2e, 'NoLidarE2ETestNode', lambda: node)
    monkeypatch.setattr(e2e, 'wait_for_tf', lambda *_args, **_kwargs: True)
    monkeypatch.setattr(e2e.time, 'sleep', lambda _: None)
    monkeypatch.setattr(e2e.rclpy, 'init', lambda: None)
    monkeypatch.setattr(e2e.rclpy, 'shutdown', lambda: cleaned.append('context'))
    # Deliver termination through the actual registered production callback.
    monkeypatch.setattr(e2e.signal, 'signal', lambda sig, cb: cb(sig, None))
    with pytest.raises(SystemExit) as raised:
        e2e.main()
    assert raised.value.code == exit_code
    assert cleaned == ['node', 'context']
