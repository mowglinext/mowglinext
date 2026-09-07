import importlib.util
import math
from pathlib import Path
import sys

from mowgli_interfaces.msg import GnssStatus
import pytest
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
