# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later

"""Check generated GNSS acquisition ownership using the actual ROS node."""

import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped
import pytest
import rclpy
from sensor_msgs.msg import NavSatFix


def _load_module():
    path = (
        Path(__file__).resolve().parents[1]
        / 'scripts'
        / 'sim_navsat_rtk_fix.py'
    )
    spec = importlib.util.spec_from_file_location(
        'sim_navsat_rtk_fix_under_test', path
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


sim_navsat = _load_module()


def _relay():
    fixes, statuses = [], []
    clock = SimpleNamespace(nanoseconds=2_000_000_000)
    rclpy.init()
    node = sim_navsat.SimNavSatRtkFix()
    node.get_clock = lambda: SimpleNamespace(now=lambda: clock)
    node._pub = SimpleNamespace(publish=fixes.append)
    node._status_pub = SimpleNamespace(publish=statuses.append)
    return node, clock, fixes, statuses


@pytest.fixture(autouse=True)
def cleanup_ros():
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _truth(node, stamp):
    msg = PoseStamped()
    msg.header.stamp.sec = stamp
    msg.header.frame_id = 'map'
    msg.pose.orientation.w = 1.0
    sim_navsat.SimNavSatRtkFix._on_ground_truth(node, msg)


def _fix(node, sec, nanosec=0):
    msg = NavSatFix()
    msg.header.stamp.sec, msg.header.stamp.nanosec = sec, nanosec
    msg.header.frame_id = 'gps_link'
    sim_navsat.SimNavSatRtkFix._on_fix(node, msg)
    return msg


def test_new_stationary_acquisitions_have_paired_nonzero_identity():
    node, clock, fixes, statuses = _relay()
    _truth(node, 1)
    first = _fix(node, 1)
    _truth(node, 2)
    second = _fix(node, 2)
    assert [status.position_observation_sequence for status in statuses] == [1, 2]
    assert [fix.header for fix in fixes] == [first.header, second.header]
    assert [status.header for status in statuses] == [first.header, second.header]
    assert fixes[0].latitude == fixes[1].latitude
    assert fixes[0].longitude == fixes[1].longitude


@pytest.mark.parametrize('raw_stamp', [(0, 0), (3, 0), (1, 0), (0, 900_000_000)])
def test_unset_future_duplicate_and_replayed_fixes_do_not_refresh_identity(raw_stamp):
    node, clock, fixes, statuses = _relay()
    _truth(node, 1)
    _fix(node, 1)
    _truth(node, 2)
    _fix(node, *raw_stamp)
    assert len(fixes) == len(statuses) == 1
    assert node._position_observation_sequence == 1


def test_new_raw_stamp_cannot_authorize_cached_ground_truth():
    node, clock, fixes, statuses = _relay()
    _truth(node, 1)
    _fix(node, 1)
    _truth(node, 1)
    _fix(node, 2)
    assert len(fixes) == len(statuses) == 1
    _truth(node, 2)
    _fix(node, 2)
    assert [status.position_observation_sequence for status in statuses] == [1, 2]


def test_sim_clock_restart_accepts_new_acquisition_without_reusing_sequence():
    node, clock, fixes, statuses = _relay()
    _truth(node, 2)
    _fix(node, 2)
    clock.nanoseconds = 1_000_000_000
    _truth(node, 1)
    _fix(node, 1)
    assert [status.position_observation_sequence for status in statuses] == [1, 2]


def test_raw_fix_after_clock_restart_waits_for_new_ground_truth():
    node, clock, fixes, statuses = _relay()
    _truth(node, 2)
    _fix(node, 2)
    clock.nanoseconds = 1_000_000_000
    _fix(node, 1)
    assert len(fixes) == len(statuses) == 1
    assert node._ground_truth is None
    _truth(node, 1)
    _fix(node, 1)
    assert [status.position_observation_sequence for status in statuses] == [1, 2]


def test_projected_pose_keeps_its_acquisition_epoch_and_sensor_frame():
    node, clock, fixes, statuses = _relay()
    epochs = []
    clock.nanoseconds = 3_000_000_000
    node._current_regime = lambda epoch: epochs.append(epoch) or 'RTK_FIXED'
    truth = PoseStamped()
    truth.header.stamp.sec = 2
    truth.header.frame_id = 'map'
    truth.pose.position.x, truth.pose.position.y = 1.25, 0.5
    truth.pose.orientation.z = math.sin(math.pi / 4)
    truth.pose.orientation.w = math.cos(math.pi / 4)
    sim_navsat.SimNavSatRtkFix._on_ground_truth(node, truth)
    raw = _fix(node, 1)
    assert fixes[0].header.stamp.sec == 2
    assert statuses[0].header == fixes[0].header
    assert raw.header.stamp.sec == 1
    assert fixes[0].header.frame_id == raw.header.frame_id
    assert epochs == [2.0]

    projected_x = ((fixes[0].longitude - node._datum_lon)
                   * math.cos(math.radians(node._datum_lat)) * sim_navsat.METERS_PER_DEG)
    projected_y = (fixes[0].latitude - node._datum_lat) * sim_navsat.METERS_PER_DEG
    assert projected_x == pytest.approx(1.25, abs=1e-7)
    assert projected_y == pytest.approx(0.8, abs=1e-7)


def test_native_truth_waits_for_clock_without_poisoning_current_acquisition():
    node, clock, fixes, statuses = _relay()
    _truth(node, 100)
    assert node._last_truth_stamp_ns is None
    _truth(node, 1)
    _fix(node, 1)
    assert fixes[0].header.stamp.sec == 1
    _truth(node, 3)
    _fix(node, 2)
    assert len(fixes) == 1
    clock.nanoseconds = 3_000_000_000
    _fix(node, 3)
    assert [fix.header.stamp.sec for fix in fixes] == [1, 3]


def test_pending_truth_is_cleared_on_clock_rewind():
    node, clock, fixes, statuses = _relay()
    _truth(node, 3)
    clock.nanoseconds = 1_000_000_000
    _fix(node, 1)
    assert node._pending_ground_truth is None
    assert fixes == []
    _truth(node, 1)
    _fix(node, 1)
    assert statuses[0].position_observation_sequence == 1


def test_malformed_future_truth_does_not_consume_trigger_or_poison_watermark():
    node, clock, fixes, statuses = _relay()
    bad = PoseStamped()
    bad.header.stamp.sec = 100
    bad.header.frame_id = 'map'
    bad.pose.position.x = math.nan
    sim_navsat.SimNavSatRtkFix._on_ground_truth(node, bad)
    _fix(node, 1)
    assert node._last_fix_stamp_ns is None
    assert node._pending_ground_truth is None
    _truth(node, 1)
    _fix(node, 1)
    assert len(fixes) == len(statuses) == 1


def test_clock_before_truth_accepts_without_pending_and_duplicate_truth_cannot_refresh():
    node, clock, fixes, statuses = _relay()
    _truth(node, 2)
    _fix(node, 1)
    assert node._pending_ground_truth is None
    _truth(node, 2)
    _fix(node, 2)
    assert len(fixes) == len(statuses) == 1


@pytest.mark.parametrize('invalid', ['frame', 'nanosec'])
def test_invalid_truth_identity_does_not_replace_current_truth(invalid):
    node, clock, fixes, statuses = _relay()
    _truth(node, 1)
    bad = PoseStamped()
    bad.header.stamp.sec = 2
    bad.header.frame_id = 'map'
    bad.pose.orientation.w = 1.0
    if invalid == 'frame':
        bad.header.frame_id = 'odom'
    else:
        bad.header.stamp.nanosec = 1_000_000_000
    sim_navsat.SimNavSatRtkFix._on_ground_truth(node, bad)
    _fix(node, 1)
    assert fixes[0].header.stamp.sec == 1
    assert node._pending_ground_truth is None


def test_full_native_quaternion_uses_projected_yaw_for_planar_antenna():
    node, clock, fixes, statuses = _relay()
    truth = PoseStamped()
    truth.header.stamp.sec = 1
    truth.header.frame_id = 'map'
    # Rz(0.7) Ry(-0.4) Rx(0.3); with nonzero roll and pitch,
    # 2*atan2(q.z,q.w) differs from the actual projected body-X yaw.
    roll, pitch, yaw = 0.3, -0.4, 0.7
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    q = truth.pose.orientation
    q.x, q.y = sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy
    q.z, q.w = cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy
    sim_navsat.SimNavSatRtkFix._on_ground_truth(node, truth)
    _fix(node, 1)
    projected_x = ((fixes[0].longitude - node._datum_lon)
                   * math.cos(math.radians(node._datum_lat)) * sim_navsat.METERS_PER_DEG)
    projected_y = (fixes[0].latitude - node._datum_lat) * sim_navsat.METERS_PER_DEG
    assert projected_x == pytest.approx(0.3*math.cos(yaw), rel=0.0, abs=1e-15)
    assert projected_y == pytest.approx(0.3*math.sin(yaw), rel=0.0, abs=1e-15)
    assert statuses[0].header == fixes[0].header == truth.header.__class__(
        stamp=truth.header.stamp, frame_id='gps_link')


@pytest.mark.parametrize('component', ['x', 'y'])
def test_nonfinite_full_quaternion_cannot_poison_truth_identity(component):
    node, clock, fixes, statuses = _relay()
    bad = PoseStamped()
    bad.header.stamp.sec = 1
    bad.header.frame_id = 'map'
    bad.pose.orientation.w = 1.0
    setattr(bad.pose.orientation, component, math.nan)
    sim_navsat.SimNavSatRtkFix._on_ground_truth(node, bad)
    _fix(node, 1)
    assert fixes == statuses == []
    assert node._last_truth_stamp_ns is None
    assert node._last_fix_stamp_ns is None
    _truth(node, 1)
    _fix(node, 1)
    assert statuses[0].position_observation_sequence == 1
