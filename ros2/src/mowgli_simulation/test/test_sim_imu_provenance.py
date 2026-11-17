# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later

import importlib.util
import math
import os
from pathlib import Path
from types import SimpleNamespace

from geometry_msgs.msg import TwistStamped
import pytest
import rclpy
from sensor_msgs.msg import Imu


def _load_module():
    source = Path(os.environ.get(
        'SIM_IMU_SOURCE',
        Path(__file__).resolve().parents[1] / 'scripts' / 'sim_imu_noise.py',
    ))
    spec = importlib.util.spec_from_file_location('sim_imu_provenance_under_test', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


sim_imu = _load_module()


@pytest.fixture
def make_node(monkeypatch):
    nodes = []

    def create(applied=True):
        args = ['--ros-args']
        parameters = {
            'gyro_white_std': 0.0, 'gyro_bias_walk_std': 0.0,
            'gyro_bias_init_std': 0.0, 'accel_white_std': 0.0,
            'accel_bias_walk_std': 0.0, 'accel_bias_init_std': 0.0,
        }
        if applied is not None:
            parameters['synthesize_from_applied_twist'] = applied
        for name, value in parameters.items():
            args.extend(['-p', f'{name}:={str(value).lower()}'])
        rclpy.init(args=args)
        node = sim_imu.SimImuNoise()
        nodes.append(node)
        clock = SimpleNamespace(nanoseconds=2_000_000_000)
        monkeypatch.setattr(node, 'get_clock', lambda: SimpleNamespace(now=lambda: clock))
        outputs = []
        node._pub = SimpleNamespace(publish=outputs.append)
        return node, clock, outputs

    yield create
    for node in nodes:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def _applied(stamp_ns, wz, frame='base_footprint', vx=0.15):
    msg = TwistStamped()
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
    msg.header.frame_id = frame
    msg.twist.linear.x = vx
    msg.twist.angular.z = wz
    return msg


def _stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def test_constructor_subscribes_only_to_selected_source(make_node):
    node, _, _ = make_node()
    assert node.get_parameter('synthesize_from_applied_twist').value is True
    assert node.get_parameter('applied_twist_topic').value == '/sim/applied_twist'
    assert [sub.topic_name for sub in node.subscriptions] == ['/sim/applied_twist']
    assert any(timer.timer_period_ns == 10_000_000 for timer in node.timers)


def test_applied_mode_ignores_raw_device_callbacks(make_node):
    node, _, outputs = make_node()
    raw = Imu()
    raw.header.stamp.sec = 2
    raw.angular_velocity.z = 99.0
    node._on_imu(raw)
    assert outputs == []


def test_applied_value_and_source_epoch_stay_paired_including_applied_stop(make_node):
    node, _, outputs = make_node()
    turn = _applied(1_960_000_000, 0.6)
    node._on_applied_twist(turn)
    # The plant may apply zero after its existing command timeout even though
    # an older command requested a turn. The applied observation is authoritative.
    node._on_applied_twist(_applied(1_980_000_000, 0.0, vx=0.0))
    assert [_stamp_ns(msg) for msg in outputs] == [1_960_000_000, 1_980_000_000]
    assert [msg.angular_velocity.z for msg in outputs] == [0.6, 0.0]
    assert [msg.header.frame_id for msg in outputs] == ['imu_link', 'imu_link']
    assert outputs[0].orientation.w == 1.0
    assert outputs[0].orientation_covariance[8] == 99.0
    assert outputs[0].linear_acceleration.z == 9.80665
    assert turn.header.frame_id == 'base_footprint'


def test_duplicate_older_and_invalid_applied_samples_do_not_publish(make_node):
    node, _, outputs = make_node()
    node._on_applied_twist(_applied(1_980_000_000, 0.2))
    for msg in (
        _applied(1_980_000_000, 0.9),
        _applied(1_960_000_000, 0.9),
        _applied(0, 0.9),
        _applied(2_000_000_000, math.nan),
        _applied(2_000_000_000, 0.9, vx=math.inf),
        _applied(2_000_000_000, 0.9, frame='map'),
    ):
        node._on_applied_twist(msg)
    assert len(outputs) == 1
    assert node._last_applied_stamp_ns == 1_980_000_000


def test_future_sample_waits_for_clock_and_flushes_once_without_new_input(make_node):
    node, clock, outputs = make_node()
    msg = _applied(2_020_000_000, -0.4)
    node._on_applied_twist(msg)
    node._on_applied_twist(msg)
    assert outputs == []
    assert node._last_applied_stamp_ns is None
    # Pending messages are snapshots, not mutable callback-owned aliases.
    msg.twist.angular.z = 42.0
    clock.nanoseconds = 2_020_000_000
    node._flush_pending_applied()
    node._flush_pending_applied()
    assert [_stamp_ns(out) for out in outputs] == [2_020_000_000]
    assert outputs[0].angular_velocity.z == -0.4


def test_previous_pending_is_published_before_current_sample(make_node):
    node, clock, outputs = make_node()
    node._on_applied_twist(_applied(2_020_000_000, 0.2))
    clock.nanoseconds = 2_040_000_000
    node._on_applied_twist(_applied(2_040_000_000, 0.4))
    assert [_stamp_ns(msg) for msg in outputs] == [2_020_000_000, 2_040_000_000]
    assert [msg.angular_velocity.z for msg in outputs] == [0.2, 0.4]


def test_future_turn_and_final_zero_both_flush_without_another_input(make_node):
    node, clock, outputs = make_node()
    node._on_applied_twist(_applied(2_020_000_000, 0.6))
    node._on_applied_twist(_applied(2_040_000_000, 0.0, vx=0.0))
    assert outputs == []
    clock.nanoseconds = 2_040_000_000
    node._flush_pending_applied()
    node._flush_pending_applied()
    assert [_stamp_ns(msg) for msg in outputs] == [2_020_000_000, 2_040_000_000]
    assert [msg.angular_velocity.z for msg in outputs] == [0.6, 0.0]


def test_pending_future_cannot_block_eligible_current_sample(make_node):
    node, _, outputs = make_node()
    node._on_applied_twist(_applied(100_000_000_000, 9.0))
    node._on_applied_twist(_applied(2_000_000_000, 0.2))
    assert [_stamp_ns(msg) for msg in outputs] == [2_000_000_000]
    assert node._last_applied_stamp_ns == 2_000_000_000


def test_pending_samples_drain_in_source_order_without_duplicate_replacement(make_node):
    node, clock, outputs = make_node()
    node._on_applied_twist(_applied(2_060_000_000, 0.6))
    node._on_applied_twist(_applied(2_020_000_000, 0.2))
    node._on_applied_twist(_applied(2_040_000_000, 0.4))
    node._on_applied_twist(_applied(2_040_000_000, 9.9))
    assert len(node._pending_applied) == 3
    clock.nanoseconds = 2_060_000_000
    node._flush_pending_applied()
    assert [_stamp_ns(msg) for msg in outputs] == [
        2_020_000_000, 2_040_000_000, 2_060_000_000,
    ]
    assert [msg.angular_velocity.z for msg in outputs] == [0.2, 0.4, 0.6]
    assert node._pending_applied == []


def test_pending_capacity_matches_qos_and_overflow_retains_newest_samples(make_node):
    node, clock, outputs = make_node()
    for step in range(1, 13):
        wz = 0.0 if step == 12 else 0.6
        node._on_applied_twist(_applied(2_000_000_000 + step * 20_000_000, wz))
    assert len(node._pending_applied) == 10
    assert node._sub.qos_profile.depth == 10
    clock.nanoseconds = 2_240_000_000
    node._flush_pending_applied()
    assert [_stamp_ns(msg) for msg in outputs] == [
        2_000_000_000 + step * 20_000_000 for step in range(3, 13)
    ]
    assert outputs[-1].angular_velocity.z == 0.0
    assert node._pending_applied == []


def test_failed_publish_does_not_advance_accepted_identity(make_node):
    node, _, outputs = make_node()

    def fail(_):
        raise RuntimeError('publication failed')

    node._pub = SimpleNamespace(publish=fail)
    msg = _applied(2_000_000_000, 0.2)
    with pytest.raises(RuntimeError, match='publication failed'):
        node._on_applied_twist(msg)
    assert node._last_applied_stamp_ns is None
    assert node._prev_t is None
    node._pub = SimpleNamespace(publish=outputs.append)
    node._on_applied_twist(msg)
    assert len(outputs) == 1
    assert node._last_applied_stamp_ns == 2_000_000_000


def test_clock_rewind_clears_pending_and_accepted_source_epoch(make_node):
    node, clock, outputs = make_node()
    node._on_applied_twist(_applied(2_000_000_000, 0.2))
    node._on_applied_twist(_applied(2_020_000_000, 0.9))
    node._on_applied_twist(_applied(2_040_000_000, 0.0))
    clock.nanoseconds = 1_000_000_000
    node._flush_pending_applied()
    assert node._pending_applied == []
    assert node._last_applied_stamp_ns is None
    assert node._prev_t is None
    node._on_applied_twist(_applied(1_000_000_000, -0.2))
    assert [_stamp_ns(msg) for msg in outputs] == [2_000_000_000, 1_000_000_000]
    assert outputs[-1].angular_velocity.z == -0.2


def test_raw_mode_preserves_raw_payload_header_and_noise_policy(make_node):
    node, _, outputs = make_node(applied=None)
    assert node.get_parameter('synthesize_from_applied_twist').value is False
    assert [sub.topic_name for sub in node.subscriptions] == ['/imu/data_gz']
    raw = Imu()
    raw.header.stamp.sec = 1
    raw.header.frame_id = 'raw_imu_link'
    raw.orientation.w = 1.0
    raw.orientation_covariance[8] = 0.25
    raw.angular_velocity.z = -0.7
    raw.linear_acceleration.x = 0.3
    node._on_imu(raw)
    assert len(outputs) == 1
    assert outputs[0].header == raw.header
    assert outputs[0].orientation == raw.orientation
    assert outputs[0].orientation_covariance[8] == 0.25
    assert outputs[0].angular_velocity.z == -0.7
    assert outputs[0].linear_acceleration.x == 0.3
