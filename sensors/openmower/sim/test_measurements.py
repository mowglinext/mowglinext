# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
"""Deterministic regressions for measurement errors, without a ROS executor."""

import threading
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from measurements import ClockEpoch, MotionHistory, calibration_attempt, drives_running, odometry_mean, run_scenario
from openmower_v1_emulator import OpenMowerV1Rig


class StopTimingTest(unittest.TestCase):
    def rig(self, left, right, now=150):
        return SimpleNamespace(
            lock=threading.RLock(), now_ms=lambda: now,
            escs={'left': SimpleNamespace(commands=left),
                  'right': SimpleNamespace(commands=right)})

    def test_late_service_response_preserves_actual_stop_time(self):
        rig = self.rig([(0, .5), (30, 0)], [(0, -.5), (40, 0)])
        actual = OpenMowerV1Rig.first_time_all_zero(rig, ('left', 'right'), 1, 1000)
        self.assertEqual(actual, 40)
        self.assertLess(actual - 1, 100)

    def test_already_stopped_or_only_one_running_drive_cannot_test_stop_latency(self):
        self.assertTrue(drives_running({'left': .3, 'right': -.3}))
        for left, right in ((0, 0), (.3, 0), (0, -.3), (float('nan'), -.3)):
            with self.subTest(left=left, right=right):
                self.assertFalse(drives_running({'left': left, 'right': right}))

    def test_actual_late_stop_still_fails_one_hundred_ms(self):
        rig = self.rig([(0, .5), (130, 0)], [(0, -.5), (140, 0)])
        actual = OpenMowerV1Rig.first_time_all_zero(rig, ('left', 'right'), 1, 1000)
        self.assertEqual(actual, 140)
        self.assertGreaterEqual(actual - 1, 100)

    def test_one_drive_still_running_is_not_a_stop(self):
        rig = self.rig([(0, .5), (30, 0)], [(0, -.5)], now=1001)
        self.assertIsNone(OpenMowerV1Rig.first_time_all_zero(rig, ('left', 'right'), 1, 1000))

    def test_zero_after_measurement_deadline_does_not_count(self):
        rig = self.rig([(0, .5), (1100, 0)], [(0, -.5), (1100, 0)], now=1200)
        self.assertIsNone(OpenMowerV1Rig.first_time_all_zero(rig, ('left', 'right'), 1, 1000))

    def test_an_earlier_scenario_stop_does_not_count(self):
        rig = self.rig([(0, 0), (10, .5)], [(0, 0), (10, -.5)], now=1100)
        self.assertIsNone(OpenMowerV1Rig.first_time_all_zero(rig, ('left', 'right'), 20, 1000))

    def test_buffered_commands_with_identical_times_keep_uart_order(self):
        rig = self.rig([(0, .5), (60, 0)], [(0, -.5), (30, 0), (30, -.5)], now=1001)
        self.assertIsNone(OpenMowerV1Rig.first_time_all_zero(rig, ('left', 'right'), 1, 1000))
        rig.escs['right'].commands = [(0, -.5), (30, -.5), (30, 0)]
        self.assertEqual(OpenMowerV1Rig.first_time_all_zero(rig, ('left', 'right'), 1, 1000), 60)


class MotionComparisonTest(unittest.TestCase):
    def test_delivery_delay_does_not_change_comparison_interval(self):
        history = MotionHistory()
        history.append(0, 0, 0, (0, 0))
        history.append(.35, .509 * .35, 0, (285, -285))
        history.append(1.85, .509 * .35 + .239 * 1.5, 0, (859, -859))
        rows = [(.6, .509, 0, .35), (1.1, .239, 0, .85),
                (1.6, .239, 0, 1.35), (2.1, .239, 0, 1.85)]
        (odom, _), start, end = odometry_mean(rows)
        ground, _ = history.mean(start, end)
        self.assertAlmostEqual(odom, ground)
        # The old comparison incorrectly included .35 s absent from odom.
        old_ground, _ = history.mean(0, 1.85)
        self.assertGreater(abs(old_ground - odom), .05)

    def test_real_underreading_still_exceeds_unchanged_tolerance(self):
        history = MotionHistory()
        history.append(0, 0, 0, (0, 0))
        history.append(2, .6, 0, (960, -960))
        (odom, _), start, end = odometry_mean([(0, .239, 0, 0), (2, .239, 0, 2)])
        ground, _ = history.mean(start, end)
        self.assertGreaterEqual(abs(odom - ground), .05)

    def test_yaw_is_unwrapped_and_interpolated_over_the_same_window(self):
        history = MotionHistory()
        history.append(0, 0, 3, (0, 0))
        history.append(2, 0, 4.2, (0, 0))
        self.assertAlmostEqual(history.mean(.5, 1.5)[1], .6)

    def test_frozen_reversed_or_nonfinite_stamps_are_rejected(self):
        for end in (0, -1, float('nan')):
            with self.subTest(end=end), self.assertRaises(ValueError):
                odometry_mean([(0, .3, 0, 0), (1, .3, 0, end)])

    def test_missing_history_is_not_extrapolated(self):
        history = MotionHistory()
        history.append(1, 0, 0, (0, 0))
        history.append(2, .3, 0, (480, -480))
        with self.assertRaises(ValueError):
            history.mean(0, 2)

    def test_stationary_time_uses_wheel_ticks_before_dock_receipt(self):
        history = MotionHistory()
        history.append(0, 0, 0, (0, 0))
        history.append(1, .3, 0, (480, -480))
        history.append(3, .3, 0, (480, -480))
        self.assertEqual(history.stationary_for(3), 2)
        self.assertLess(history.stationary_for(1.5), 1)


class ScenarioFailureTest(unittest.TestCase):
    def report(self):
        report = SimpleNamespace(failed=[])
        report.check = lambda *args: report.failed.append(args)
        return report

    def test_failed_prerequisite_stops_following_scenarios_and_keeps_failure(self):
        report, executed = self.report(), []

        def fail():
            executed.append('first')
            report.check('first', 'unexpected latch', False)

        def following():
            executed.append('second')

        for scenario in (fail, following):
            if not run_scenario(scenario, report):
                break
        self.assertEqual(executed, ['first'])
        self.assertEqual(len(report.failed), 1)

    def test_exception_is_recorded_as_failure(self):
        report = self.report()

        def crash():
            raise RuntimeError('disconnected fixture')

        self.assertFalse(run_scenario(crash, report))
        self.assertEqual(len(report.failed), 1)

    def test_existing_other_controller_failure_does_not_abort_success(self):
        report = self.report()
        report.failed.append(('previous controller failure',))
        self.assertTrue(run_scenario(lambda: None, report))


class CalibrationEventsTest(unittest.TestCase):
    def line(self, stamp, event):
        return f'[INFO] [{stamp}] [hardware_bridge]: IMU bias {event}'

    def test_uses_start_after_abort_not_earlier_attempt(self):
        lines = [self.line(1, 'calibration started (docked, wheels still)'),
                 self.line(2, 'calibration aborted'),
                 self.line(3, 'calibration started (docked, wheels still)'),
                 self.line(7, 'ready: gyro')]
        self.assertEqual(calibration_attempt(lines, 0), (3, 7))

    def test_old_or_unpaired_completion_does_not_count(self):
        for lines in ([self.line(1, 'ready: gyro')],
                      [self.line(1, 'calibration started (docked, wheels still)'),
                       self.line(2, 'ready: gyro')]):
            with self.subTest(lines=lines), self.assertRaises(ValueError):
                calibration_attempt(lines, 3)

    def test_started_but_frozen_or_aborted_calibration_does_not_complete(self):
        start = self.line(1, 'calibration started (docked, wheels still)')
        for lines in ([start], [start, self.line(2, 'calibration aborted'), self.line(3, 'ready: gyro')]):
            with self.subTest(lines=lines), self.assertRaises(ValueError):
                calibration_attempt(lines, 0)


class ClockEpochTest(unittest.TestCase):
    def test_overlapping_measurement_bounds_keep_the_epoch(self):
        clock = ClockEpoch()
        self.assertFalse(clock.observe(100, 1000, 110))
        self.assertFalse(clock.observe(200, 1105, 220))

    def test_long_read_bracket_does_not_invent_a_clock_step(self):
        clock = ClockEpoch()
        self.assertFalse(clock.observe(100, 1000, 110))
        self.assertFalse(clock.observe(200, 1150, 300))

    def test_asymmetric_clock_read_pause_preserves_integration_order(self):
        clock = ClockEpoch()
        clock.observe(100_000_000, 995_000_000, 100_000_000)
        with patch('measurements.time.monotonic_ns', side_effect=[200_000_000, 300_000_000]), \
                patch('measurements.time.time_ns', return_value=1_195_000_000):
            mono, host, changed = clock.sample()
        self.assertFalse(changed)
        self.assertAlmostEqual(mono, .3)
        self.assertAlmostEqual(host, 1.195)
        with patch('measurements.time.monotonic_ns', side_effect=[310_000_000, 310_000_000]), \
                patch('measurements.time.time_ns', return_value=1_205_000_000):
            next_mono, next_host, changed = clock.sample()
        self.assertFalse(changed)
        self.assertAlmostEqual(next_mono - mono, next_host - host)
        history = MotionHistory()
        history.append(host, 0, 0, (0, 0))
        history.append(next_host - (next_mono - mono) / 2, .0015, 0, (2, -2))
        history.append(next_host, .003, 0, (5, -5))

    def test_narrowing_initial_uncertainty_cannot_reorder_ground_history(self):
        clock = ClockEpoch()
        with patch('measurements.time.monotonic_ns', side_effect=[100_000_000, 200_000_000]), \
                patch('measurements.time.time_ns', return_value=1_100_000_000):
            mono, _, changed = clock.sample()
        history = MotionHistory()
        history.append(mono, 0, 0, (0, 0))
        with patch('measurements.time.monotonic_ns', side_effect=[209_000_000, 211_000_000]), \
                patch('measurements.time.time_ns', return_value=1_110_000_000):
            next_mono, _, changed = clock.sample()
        self.assertFalse(changed)
        history.append(next_mono, .0033, 0, (5, -5))
        self.assertAlmostEqual(clock.to_monotonic(1.1005), .2)
        self.assertAlmostEqual(history.mean(.2, .211)[0], .3)

    def test_both_directions_of_clock_change_begin_a_new_epoch(self):
        for host in (2000, 500):
            clock = ClockEpoch()
            clock.observe(100, 1000, 110)
            self.assertTrue(clock.observe(200, host, 210))
            self.assertFalse(clock.observe(300, host + 100, 310))

    def test_clock_change_keeps_an_otherwise_passing_scenario_red(self):
        report = SimpleNamespace(failed=[])
        report.check = lambda *args: report.failed.append(args)
        fixture = SimpleNamespace(clock_steps=[])

        def step():
            fixture.clock_steps.append(('system clock stepped',))

        self.assertFalse(run_scenario(step, report, fixture=fixture))
        self.assertEqual(len(report.failed), 1)


if __name__ == '__main__':
    unittest.main()
