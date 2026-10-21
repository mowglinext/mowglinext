# SPDX-License-Identifier: MIT
"""UM960 parser state -> /gps/status (GnssStatus) projection."""
import math

from um960_gps_driver.gnss_status import (CAP_HORIZONTAL_ACCURACY, FIX_TYPE_GPS_FIX,
                                          FIX_TYPE_NO_FIX, FIX_TYPE_RTK_FIXED,
                                          FIX_TYPE_RTK_FLOAT, gnss_status_fields)
from um960_gps_driver.parsers import NmeaParser


def _gga(quality):
    body = 'GNGGA,120000.00,4807.038,N,01131.000,E,%d,18,0.8,545.4,M,46.9,M,1.0,0001' % quality
    cs = 0
    for c in body:
        cs ^= ord(c)
    return '$%s*%02X' % (body, cs)


def test_rtk_fixed_from_gga():
    p = NmeaParser()
    p.feed(_gga(4))
    f = gnss_status_fields(p.fix, True, [0.0004, 0, 0, 0, 0.0004, 0, 0, 0, 0.001],
                           {'flow': 'active', 'source': 'lora', 'corr_age_s': 1.0}, 7)
    assert f['fix_type'] == FIX_TYPE_RTK_FIXED and f['fix_valid'] and f['quality_percent'] == 100.0
    assert math.isclose(f['horizontal_accuracy_m'], 0.02)
    assert f['value_flags'] & CAP_HORIZONTAL_ACCURACY and f['corrections_active']
    assert f['position_observation_sequence'] == 7 and f['satellites_used'] == 18


def test_float_single_and_none():
    for q, ft in ((5, FIX_TYPE_RTK_FLOAT), (1, FIX_TYPE_GPS_FIX)):
        p = NmeaParser()
        p.feed(_gga(q))
        assert gnss_status_fields(p.fix, True, None)['fix_type'] == ft
    f = gnss_status_fields(NmeaParser().fix, False, None)
    assert f['fix_type'] == FIX_TYPE_NO_FIX and not f['fix_valid']


def test_bestnav_position_type_wins():
    fix = {'position_type': 'NARROW_FLOAT', 'quality': 4}
    assert gnss_status_fields(fix, True, None)['fix_type'] == FIX_TYPE_RTK_FLOAT
