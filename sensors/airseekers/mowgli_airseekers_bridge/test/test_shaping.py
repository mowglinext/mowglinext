# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
import math

import pytest

from mowgli_airseekers_bridge.drive_shaper import DriveShaper
from mowgli_airseekers_bridge.imu_bias import ImuBiasEstimator, RestCalibrator
from mowgli_airseekers_bridge.tilt import TiltBands


def test_shaper_neutral_by_default():
    s = DriveShaper()
    assert s.shape(0.3, 0.01, yaw=0.5) == 0.01


def test_trim_signed_with_direction():
    s = DriveShaper(angular_trim_radps=0.02)
    assert s.shape(0.3, 0.0) == pytest.approx(0.02)
    assert s.shape(-0.3, 0.0) == pytest.approx(-0.02)
    assert s.shape(0.0, 0.0) == 0.0


def test_heading_hold_steers_back():
    s = DriveShaper(heading_hold=True)
    assert s.shape(0.3, 0.0, yaw=0.0) == 0.0
    assert s.shape(0.3, 0.0, yaw=-0.1) > 0.0
    assert s.shape(0.3, 0.2, yaw=-0.1) == 0.2 and not s.holding


def test_bias_estimator():
    e = ImuBiasEstimator(10)
    e.start()
    done = [e.feed((0.1, 0.2, 9.8), (0.01, 0.0, -0.02)) for _ in range(10)]
    assert done[-1] and e.ready
    acc, gyro = e.apply((0.1, 0.2, 9.8), (0.01, 0.0, -0.02))
    assert acc == pytest.approx((0.0, 0.0, 9.8)) and gyro == pytest.approx((0.0, 0.0, 0.0))


def test_dock_armed_calibration_waits_for_still_and_retries():
    e = ImuBiasEstimator(5)
    c = RestCalibrator(e, still_s=1.0, auto_rest_s=15.0)
    assert c.step(0.0, True, True) == ''         # arrived, still creeping
    assert c.step(0.5, True, False) == ''
    assert c.step(1.6, True, False) == 'started'
    assert c.step(1.7, True, True) == 'aborted'  # moved: retried
    c.step(1.8, True, False)
    assert c.step(2.9, True, False) == 'started'


def test_auto_rest_off_dock_once():
    e = ImuBiasEstimator(1)
    c = RestCalibrator(e, auto_rest_s=15.0)
    c.step(0.0, False, False)
    assert c.step(14.0, False, False) == ''
    assert c.step(15.1, False, False) == 'started'
    e.feed((0, 0, 9.8), (0, 0, 0))
    assert c.step(40.0, False, False) == ''


def test_tilt_critical_after_debounce():
    t = TiltBands(roll_thresholds_deg=(15, 22, 30), pitch_thresholds_deg=(18, 25, 32),
                  debounce_s=(1.0, 0.6, 0.3))
    band = None
    for i in range(100):
        band = t.update(i * 0.01, math.radians(35), 0.0)
    assert band == 'critical'
    t2 = TiltBands()
    assert t2.update(0.0, math.radians(35), 0.0) != 'critical'   # not on one sample
