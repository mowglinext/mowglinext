import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped
from mowgli_interfaces.msg import GnssStatus
import pytest
import rclpy
from sensor_msgs.msg import NavSatFix, NavSatStatus
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
        ';',
        '1,RTK_FIXED;',
        ';1,RTK_FIXED',
        '1,RTK_FIXED,NO_FIX',
        ',RTK_FIXED',
        '1e309,RTK_FIXED',
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


@pytest.mark.parametrize(
    'pattern',
    ['bad', 'nan,RTK_FLOAT', '0,NO_FIX',
     '1e308,RTK_FIXED;1e308,NO_FIX'],
)
def test_node_constructor_rejects_invalid_quality_pattern(pattern):
    rclpy.init(args=['--ros-args', '-p', f'quality_pattern:={pattern}'])
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
        Header(), regime, sigma_xy, sigma_z
    )

    assert status.backend == 'simulation'
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


@pytest.mark.parametrize('pattern', ['', '1,RTK_FIXED;2,RTK_FLOAT;3,NO_FIX'])
def test_relay_outputs_quality_cycle_and_fixed_control(monkeypatch, pattern):
    rclpy.init(args=(['--ros-args', '-p', f'quality_pattern:={pattern}']
                     if pattern else []))
    node = sim_navsat.SimNavSatRtkFix()
    fixes, statuses = [], []
    try:
        monkeypatch.setattr(node, '_pub', SimpleNamespace(publish=fixes.append))
        monkeypatch.setattr(
            node, '_status_pub', SimpleNamespace(publish=statuses.append)
        )
        truth = PoseStamped()
        truth.pose.orientation.w = 1.0
        node._on_ground_truth(truth)
        regimes = (['RTK_FIXED'] * 4 if not pattern else
                   ['RTK_FIXED', 'RTK_FLOAT', 'NO_FIX', 'RTK_FIXED'])
        for seconds, regime in zip([10, 11, 13, 16], regimes):
            monkeypatch.setattr(node, 'get_clock', lambda: SimpleNamespace(
                now=lambda: SimpleNamespace(nanoseconds=seconds * 10**9)
            ))
            raw = NavSatFix()
            raw.header.stamp.sec = seconds
            node._on_fix(raw)
            fix, status = fixes[-1], statuses[-1]
            code, sigma_xy, sigma_z = sim_navsat.QUALITY_REGIMES[regime]
            assert fix.header == status.header == raw.header
            assert fix.status.status == code
            assert status.fix_valid == (regime != 'NO_FIX')
            assert fix.position_covariance_type == (
                NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            )
            assert list(fix.position_covariance) == pytest.approx([
                sigma_xy**2, 0, 0, 0, sigma_xy**2, 0, 0, 0, sigma_z**2,
            ])
            assert status.horizontal_accuracy_m == pytest.approx(sigma_xy)
            assert status.vertical_accuracy_m == pytest.approx(sigma_z)
            if regime == 'NO_FIX':
                assert fix.status.status == NavSatStatus.STATUS_NO_FIX
        # NO_FIX still produces a sample: it is a quality profile, not silence.
        assert len(fixes) == len(statuses) == 4
    finally:
        node.destroy_node()
        rclpy.shutdown()
