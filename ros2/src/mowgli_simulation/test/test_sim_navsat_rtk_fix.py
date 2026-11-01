import importlib.util
import math
from pathlib import Path
import random
import sys
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped
from mowgli_interfaces.msg import GnssStatus
import pytest
import rclpy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header


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


def test_empty_quality_pattern_preserves_always_fixed_default():
    assert sim_navsat._parse_pattern('') == []
    node = SimpleNamespace(_segments=[], _cycle_total_s=0.0, _start_t=None)
    assert sim_navsat.SimNavSatRtkFix._current_regime(node, 123.0) == 'RTK_FIXED'


def test_valid_quality_pattern_preserves_segment_timing_and_cycle():
    segments = sim_navsat._parse_pattern(
        ' 1.5,rtk_fixed ; 2, RTK_FLOAT ; 3,NO_FIX '
    )
    node = SimpleNamespace(
        _segments=segments,
        _cycle_total_s=sum(segment.duration_s for segment in segments),
        _start_t=None,
    )

    assert sim_navsat.SimNavSatRtkFix._current_regime(node, 10.0) == 'RTK_FIXED'
    assert sim_navsat.SimNavSatRtkFix._current_regime(node, 11.499) == 'RTK_FIXED'
    assert sim_navsat.SimNavSatRtkFix._current_regime(node, 11.5) == 'RTK_FLOAT'
    assert sim_navsat.SimNavSatRtkFix._current_regime(node, 13.5) == 'NO_FIX'
    assert sim_navsat.SimNavSatRtkFix._current_regime(node, 16.5) == 'RTK_FIXED'


@pytest.mark.parametrize(
    'pattern',
    [
        'RTK_FIXED',
        'bad-duration,RTK_FIXED',
        '1,UNKNOWN',
        '1,RTK_FIXED;;2,NO_FIX',
        'nan,RTK_FIXED',
        'inf,RTK_FIXED',
        '-inf,RTK_FIXED',
        '1e308,RTK_FIXED;1e308,NO_FIX',
        '0,RTK_FIXED',
        '-1,RTK_FIXED',
    ],
)
def test_invalid_quality_patterns_are_rejected(pattern):
    with pytest.raises(ValueError):
        sim_navsat._parse_pattern(pattern)


def test_node_constructor_rejects_invalid_quality_pattern():
    rclpy.init(args=['--ros-args', '-p', 'quality_pattern:=nan,RTK_FLOAT'])
    node = None
    try:
        with pytest.raises(ValueError, match='Invalid quality_pattern'):
            node = sim_navsat.SimNavSatRtkFix()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize(
    ('base_x', 'base_y', 'yaw', 'expected_x', 'expected_y'),
    [
        (1.0, 2.0, 0.0, 1.30, 2.0),
        (1.0, 2.0, math.pi / 2.0, 1.0, 2.30),
        (1.0, 2.0, math.pi, 0.70, 2.0),
    ],
)
def test_ground_truth_pose_is_projected_to_rotated_antenna_position(
    base_x, base_y, yaw, expected_x, expected_y
):
    datum_lat = 48.137154
    datum_lon = 11.576124

    latitude, longitude = sim_navsat._antenna_wgs84(
        base_x,
        base_y,
        yaw,
        datum_lat,
        datum_lon,
        0.30,
        0.0,
    )

    projected_x = (
        (longitude - datum_lon)
        * math.cos(math.radians(datum_lat))
        * sim_navsat.METERS_PER_DEG
    )
    projected_y = (latitude - datum_lat) * sim_navsat.METERS_PER_DEG
    assert projected_x == pytest.approx(expected_x, abs=1e-7)
    assert projected_y == pytest.approx(expected_y, abs=1e-7)


@pytest.mark.parametrize(
    (
        'regime',
        'sigma_xy',
        'sigma_z',
        'fix_type',
        'rtk_mode',
        'fix_valid',
        'quality_percent',
    ),
    [
        (
            'RTK_FIXED',
            0.003,
            0.006,
            GnssStatus.FIX_TYPE_RTK_FIXED,
            GnssStatus.RTK_MODE_FIXED,
            True,
            100.0,
        ),
        (
            'RTK_FLOAT',
            0.30,
            0.60,
            GnssStatus.FIX_TYPE_RTK_FLOAT,
            GnssStatus.RTK_MODE_FLOAT,
            True,
            70.0,
        ),
        (
            'NO_FIX',
            2.0,
            4.0,
            GnssStatus.FIX_TYPE_NO_FIX,
            GnssStatus.RTK_MODE_NONE,
            False,
            0.0,
        ),
    ],
)
def test_typed_status_matches_simulated_quality_regime(
    regime,
    sigma_xy,
    sigma_z,
    fix_type,
    rtk_mode,
    fix_valid,
    quality_percent,
):
    status = sim_navsat._build_gnss_status(
        Header(), regime, sigma_xy, sigma_z, 42
    )

    assert status.backend == 'simulation'
    assert status.position_observation_sequence == 42
    assert status.receiver_vendor == 'Webots'
    assert status.fix_type == fix_type
    assert status.rtk_mode == rtk_mode
    assert status.fix_valid is fix_valid
    assert status.differential_corrections is fix_valid
    assert status.corrections_active is fix_valid
    assert status.quality_percent == quality_percent
    assert status.horizontal_accuracy_m == pytest.approx(sigma_xy)
    assert status.vertical_accuracy_m == pytest.approx(sigma_z)
    assert status.capability_flags == sim_navsat.GNSS_STATUS_CAPABILITIES
    assert status.value_flags == sim_navsat.GNSS_STATUS_CAPABILITIES


def _relay():
    fixes, statuses = [], []
    clock = SimpleNamespace(nanoseconds=2_000_000_000)
    node = SimpleNamespace(
        _ground_truth=None, _pending_ground_truth=None, _last_truth_stamp_ns=None,
        _ground_truth_generation=0, _last_published_truth_generation=0,
        _last_fix_stamp_ns=None, _last_ros_now_ns=None,
        _position_observation_sequence=0,
        get_clock=lambda: SimpleNamespace(now=lambda: clock),
        _current_regime=lambda now: 'RTK_FIXED',
        _regime_counts={'RTK_FIXED': 0},
        _datum_lat=48.137154, _datum_lon=11.576124,
        _lever_arm_x=0.30, _lever_arm_y=0.0,
        _rng=random.Random(42),
        _cov=sim_navsat.SimNavSatRtkFix._cov,
        _pub=SimpleNamespace(publish=fixes.append),
        _status_pub=SimpleNamespace(publish=statuses.append),
    )
    node._check_clock_epoch = lambda: sim_navsat.SimNavSatRtkFix._check_clock_epoch(node)
    for name in ('_accept_ground_truth', '_promote_pending_truth'):
        setattr(node, name, lambda *args, _name=name: getattr(
            sim_navsat.SimNavSatRtkFix, _name)(node, *args))
    return node, clock, fixes, statuses


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
