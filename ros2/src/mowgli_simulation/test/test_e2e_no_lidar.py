# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0

import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

import rclpy
from rclpy.qos import DurabilityPolicy


def _load_module():
    path = Path(__file__).resolve().parents[2] / 'e2e_test_no_lidar.py'
    spec = importlib.util.spec_from_file_location('mowgli_e2e_no_lidar', path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


e2e = _load_module()


class _Logger:

    def info(self, _message: str) -> None:
        pass

    def warn(self, _message: str) -> None:
        pass

    def error(self, _message: str) -> None:
        pass


def _node():
    node = e2e.NoLidarE2ETestNode.__new__(e2e.NoLidarE2ETestNode)
    node.metrics = e2e.Metrics(start_time=e2e.time.time())
    node.get_logger = lambda: _Logger()
    node.current_phase = e2e.TestPhase.WAITING
    node.phase_start_time = 0.0
    node.current_bt_state = ''
    node.current_is_charging = False
    node.test_complete = False
    node.mowing_started = True
    return node


def _status(state_name, is_charging=False):
    return SimpleNamespace(state_name=state_name, is_charging=is_charging)


def _drive(node, *states):
    for state in states:
        if isinstance(state, str):
            state = _status(state)
        node._on_bt_status(state)


def _result(name, passed=True):
    return e2e.PhaseResult(name=name, passed=passed, details='x')


# The first status only initialises the BT state; a real run starts docked.
START = _status('IDLE_DOCKED', is_charging=True)
FULL_CYCLE = (START, 'UNDOCKING', 'PLANNING', 'MOWING', 'MOWING_COMPLETE')


def test_cycle_verdict_requires_every_phase_recorded():
    results = [_result('UNDOCKING'), _result('PLANNING')]

    passed, problems = e2e._cycle_verdict(results)

    assert not passed
    assert problems == ['MOWING never completed', 'DOCKING never completed']


def test_cycle_verdict_passes_a_complete_cycle():
    results = [_result(name) for name in e2e.REQUIRED_PHASES]

    assert e2e._cycle_verdict(results) == (True, [])


def test_cycle_verdict_fails_a_failed_phase():
    results = [_result(name) for name in e2e.REQUIRED_PHASES]
    results[2] = _result('MOWING', passed=False)

    passed, problems = e2e._cycle_verdict(results)

    assert not passed
    assert problems == ['MOWING failed: x']


def test_cycle_verdict_fails_when_nothing_was_recorded():
    assert not e2e._cycle_verdict([])[0]


def test_mowing_complete_does_not_end_the_cycle_before_docking():
    node = _node()

    _drive(node, *FULL_CYCLE)

    assert not node.test_complete
    assert node.current_phase == e2e.TestPhase.DOCKING


def test_docked_and_charging_completes_a_passing_cycle():
    node = _node()

    _drive(node, *FULL_CYCLE, _status('IDLE_DOCKED', is_charging=True))

    assert node.test_complete
    assert e2e._cycle_verdict(node.metrics.phase_results) == (True, [])


def test_docked_without_charging_fails_the_cycle():
    node = _node()

    _drive(node, *FULL_CYCLE, _status('IDLE_DOCKED', is_charging=False))

    assert node.test_complete
    passed, problems = e2e._cycle_verdict(node.metrics.phase_results)
    assert not passed
    assert problems == ['DOCKING failed: Docking sequence ended without charging']


def test_returning_to_the_dock_without_finishing_the_mow_fails():
    node = _node()

    _drive(
        node, START, 'UNDOCKING', 'PLANNING', 'MOWING', _status('IDLE_DOCKED', True)
    )

    assert node.test_complete
    passed, problems = e2e._cycle_verdict(node.metrics.phase_results)
    assert not passed
    assert problems[0].startswith('MOWING failed')


def test_idle_docked_before_the_robot_leaves_is_not_the_end_of_the_cycle():
    node = _node()

    _drive(node, START, _status('IDLE_DOCKED', True))

    assert not node.test_complete
    assert node.metrics.phase_results == []


def test_coverage_failure_is_a_failed_mowing_phase():
    node = _node()

    _drive(
        node,
        START, 'UNDOCKING', 'PLANNING', 'MOWING', 'COVERAGE_FAILED_DOCKING',
        _status('IDLE_DOCKED', True),
    )

    passed, problems = e2e._cycle_verdict(node.metrics.phase_results)
    assert not passed
    assert problems == ['MOWING failed: Mowing failed']


def test_final_report_fails_a_run_that_stopped_before_docking():
    node = _node()
    node.fusion_pose = None
    node.current_pose = None
    # Path tracking looks excellent, the cycle never finished.
    node.metrics.path_deviations = [(float(i), 0.01) for i in range(20)]
    _drive(node, START, 'UNDOCKING', 'PLANNING', 'MOWING')

    assert not node.print_final_report()


def test_final_report_passes_a_complete_tracked_cycle():
    node = _node()
    node.metrics.path_deviations = [(float(i), 0.01) for i in range(20)]
    _drive(node, *FULL_CYCLE, _status('IDLE_DOCKED', True))

    assert node.print_final_report()


def test_cmd_vel_callback_reads_the_stamped_twist():
    node = _node()
    msg = SimpleNamespace(
        twist=SimpleNamespace(
            linear=SimpleNamespace(x=0.3), angular=SimpleNamespace(z=0.1)
        )
    )

    node._on_cmd_vel(msg)

    assert node.metrics.cmd_vels[0][1:] == (0.3, 0.1)


def test_exit_code_follows_the_verdict(monkeypatch):
    monkeypatch.setattr(e2e.rclpy, 'shutdown', lambda: None)

    class _FakeNode:

        def __init__(self, verdict):
            self.verdict = verdict

        def print_final_report(self):
            return self.verdict

        def destroy_node(self):
            pass

    assert e2e._finish(_FakeNode(True)) == 0
    assert e2e._finish(_FakeNode(False)) == 1


def test_subscriptions_match_the_published_topics_and_types():
    rclpy.init()
    try:
        node = e2e.NoLidarE2ETestNode()
        try:
            cmd_vel = node.get_subscriptions_info_by_topic('/cmd_vel')
            assert {i.topic_type for i in cmd_vel} == {
                'geometry_msgs/msg/TwistStamped'
            }
            plan = node.get_subscriptions_info_by_topic('/coverage/full_plan')
            assert {i.topic_type for i in plan} == {'nav_msgs/msg/Path'}
            # Matches behavior_tree_node's QoS(1).transient_local() publisher.
            assert plan[0].qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
            assert not node.get_subscriptions_info_by_topic(
                '/mowgli/coverage/path'
            )
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()
