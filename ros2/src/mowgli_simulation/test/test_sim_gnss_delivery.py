# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later

"""Generated GNSS pairs retain acquisition ownership until causal delivery."""

import importlib.util
import os
from pathlib import Path
import sys
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped
import pytest
import rclpy
from rclpy.parameter import Parameter
from rclpy.serialization import serialize_message
from sensor_msgs.msg import NavSatFix


def load_relay():
    path = Path(os.environ.get(
        'SIM_GNSS_SOURCE',
        Path(__file__).resolve().parents[1] / 'scripts' / 'sim_navsat_rtk_fix.py',
    ))
    spec = importlib.util.spec_from_file_location('gnss_delivery_under_test', path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


relay = load_relay()


@pytest.fixture
def node():
    rclpy.init(args=['--ros-args', '-p', 'delivery_latency_s:=0.02'])
    sensor = relay.SimNavSatRtkFix()
    clock = SimpleNamespace(nanoseconds=1_000_000_000)
    sensor.get_clock = lambda: SimpleNamespace(now=lambda: clock)
    fixes, statuses = [], []
    sensor._pub = SimpleNamespace(publish=fixes.append)
    sensor._status_pub = SimpleNamespace(publish=statuses.append)
    try:
        yield sensor, clock, fixes, statuses
    finally:
        sensor.destroy_node()
        rclpy.shutdown()


def acquire(sensor, stamp_ns, x=0.0, raw_stamp_ns=None):
    truth = PoseStamped()
    truth.header.frame_id = 'map'
    truth.header.stamp.sec, truth.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
    truth.pose.position.x = x
    truth.pose.orientation.w = 1.0
    sensor._on_ground_truth(truth)
    raw = NavSatFix()
    raw.header.frame_id = 'gps_link'
    raw.header.stamp.sec, raw.header.stamp.nanosec = divmod(
        stamp_ns if raw_stamp_ns is None else raw_stamp_ns, 1_000_000_000)
    raw.altitude = 12.5
    sensor._on_fix(raw)
    return truth, raw


def test_pair_is_generated_once_and_final_pending_drains_without_gps(node):
    sensor, clock, fixes, statuses = node
    truth, raw = acquire(sensor, clock.nanoseconds)
    assert fixes == statuses == []
    pending = sensor._pending_deliveries[0]
    original = tuple(serialize_message(msg) for msg in pending[1:])
    rng_state = sensor._rng.getstate()
    sensor._on_ground_truth(truth)
    sensor._on_fix(raw)
    acquire(sensor, clock.nanoseconds, x=10.0)  # duplicate truth cannot replace pair
    clock.nanoseconds += 19_999_999
    sensor._drain_deliveries()
    assert fixes == statuses == []
    clock.nanoseconds += 1
    sensor._drain_deliveries()
    sensor._drain_deliveries()
    assert len(fixes) == len(statuses) == 1
    assert tuple(serialize_message(msg) for msg in (fixes[0], statuses[0])) == original
    assert sensor._rng.getstate() == rng_state
    assert sensor._position_observation_sequence == 1
    assert statuses[0].position_observation_sequence == 1
    assert fixes[0].altitude == 12.5
    assert fixes[0].header == statuses[0].header
    assert fixes[0].header.stamp.sec == 1


def test_newer_truth_and_raw_do_not_replace_queued_pair(node):
    sensor, clock, fixes, statuses = node
    acquire(sensor, clock.nanoseconds, x=1.0)
    clock.nanoseconds += 10_000_000
    acquire(sensor, clock.nanoseconds, x=7.0)
    assert len(sensor._pending_deliveries) == 2
    clock.nanoseconds += 10_000_000
    sensor._drain_deliveries()
    assert len(fixes) == 1
    clock.nanoseconds += 10_000_000
    sensor._drain_deliveries()
    assert [status.position_observation_sequence for status in statuses] == [1, 2]
    assert fixes[0].longitude != fixes[1].longitude


@pytest.mark.parametrize('raw_ns', [0, 1_000_000_001])
def test_invalid_raw_never_reserves_identity_or_delivery(node, raw_ns):
    sensor, clock, fixes, statuses = node
    acquire(sensor, clock.nanoseconds, raw_stamp_ns=raw_ns)
    assert sensor._position_observation_sequence == 0
    assert not sensor._pending_deliveries
    acquire(sensor, clock.nanoseconds)
    assert sensor._position_observation_sequence == 1


def test_malformed_raw_never_reserves_identity_or_delivery(node):
    sensor, clock, fixes, statuses = node
    truth, raw = acquire(sensor, clock.nanoseconds, raw_stamp_ns=0)
    raw.header.stamp.sec = 1
    raw.header.stamp.nanosec = 1_000_000_000
    sensor._on_fix(raw)
    assert sensor._last_fix_stamp_ns is None
    assert sensor._position_observation_sequence == 0
    assert not sensor._pending_deliveries


def test_pause_resume_and_forward_jump_keep_original_epoch(node):
    sensor, clock, fixes, statuses = node
    acquire(sensor, clock.nanoseconds)
    for _ in range(5):
        sensor._drain_deliveries()
    assert fixes == statuses == []
    clock.nanoseconds = 100_000_000_000
    sensor._drain_deliveries()
    assert len(fixes) == 1
    assert fixes[0].header.stamp.sec == 1  # stale stays stale, never restamped
    assert statuses[0].position_observation_sequence == 1


def test_overflow_drops_new_identity_without_retry_or_replacing_fifo(node):
    sensor, clock, fixes, statuses = node
    sensor._delivery_latency_ns = 10_000_000_000
    for i in range(11):
        clock.nanoseconds = 1_000_000_000 + i * 20_000_000
        truth, raw = acquire(sensor, clock.nanoseconds, x=float(i))
    assert len(sensor._pending_deliveries) == 10
    assert sensor._delivery_overflow_count == 1
    assert sensor._position_observation_sequence == 11
    sensor._on_ground_truth(truth)
    sensor._on_fix(raw)
    assert sensor._delivery_overflow_count == 1
    clock.nanoseconds = 20_000_000_000
    sensor._drain_deliveries()
    assert [status.position_observation_sequence for status in statuses] == list(range(1, 11))
    acquire(sensor, clock.nanoseconds)
    clock.nanoseconds += 10_000_000_000
    sensor._drain_deliveries()
    assert statuses[-1].position_observation_sequence == 12


def test_source_quality_pattern_and_seeded_payload_survive_boundary_release(node):
    sensor, clock, fixes, statuses = node
    sensor._segments = relay._parse_pattern('65,RTK_FIXED;3,RTK_FLOAT;3,NO_FIX')
    sensor._cycle_total_s = 71.0
    epochs = [1.0, 65.99, 66.0, 68.99, 69.0, 71.99, 72.0]
    expected = ['RTK_FIXED', 'RTK_FIXED', 'RTK_FLOAT', 'RTK_FLOAT',
                'NO_FIX', 'NO_FIX', 'RTK_FIXED']
    originals = []
    for epoch in epochs:
        clock.nanoseconds = round(epoch * 1_000_000_000)
        acquire(sensor, clock.nanoseconds)
        pending = sensor._pending_deliveries[-1]
        originals.append(tuple(serialize_message(msg) for msg in pending[1:]))
    rng_state = sensor._rng.getstate()
    clock.nanoseconds += 20_000_000
    sensor._drain_deliveries()
    assert sensor._rng.getstate() == rng_state
    assert [tuple(serialize_message(msg) for msg in pair)
            for pair in zip(fixes, statuses)] == originals
    for fix, status, regime in zip(fixes, statuses, expected):
        assert fix.status.status == relay.QUALITY_REGIMES[regime][0]
        assert status.fix_valid == (regime != 'NO_FIX')
    assert [status.position_observation_sequence for status in statuses] == list(range(1, 8))


@pytest.mark.parametrize('latency', [-0.01, float('nan'), float('inf')])
def test_latency_rejects_nonfinite_or_negative_values(latency):
    rclpy.init()
    # Constructor gets a real ROS parameter override, including nonfinite values.
    original = relay.Node.__init__

    def initialize(self, *args, **kwargs):
        kwargs['parameter_overrides'] = [Parameter('delivery_latency_s', value=latency)]
        original(self, *args, **kwargs)

    relay.Node.__init__ = initialize
    try:
        with pytest.raises(ValueError, match='finite and nonnegative'):
            relay.SimNavSatRtkFix()
    finally:
        relay.Node.__init__ = original
        rclpy.shutdown()


def test_latency_is_immutable_after_construction(node):
    sensor, _, _, _ = node
    result = sensor.set_parameters([Parameter('delivery_latency_s', value=0.2)])
    assert not result[0].successful
    assert sensor._delivery_latency_ns == 20_000_000
