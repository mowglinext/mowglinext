# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
"""The emulated MCU's rules (no ROS):  python3 -m pytest sensors/airseekers/sim"""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from airseekers_emulator import DockPads, Mcu  # noqa: E402
from kinematics import DriveParams, Plant, wheel_speeds  # noqa: E402
from mowgli_airseekers_bridge import protocol as P  # noqa: E402


def fed(mcu, t, *frames):
    mcu.receive(b''.join(frames), t)
    mcu.tick(t)


def upright(mcu, t):
    fed(mcu, t, P.imu_frame(0.0, 0.0, 0.0, (0, 0, 9.8), (0, 0, 0)))


def test_setpoint_held_while_heartbeats_continue():
    m = Mcu()
    upright(m, 0)
    fed(m, 1, P.speed_frame(0.2, 0.0))
    for t in range(100, 1900, 100):
        upright(m, t)
        fed(m, t, P.heartbeat_frame())
    assert m.setpoint[0] == pytest.approx(0.2)  # silence on SpeedData != stop


def test_host_watchdog_zeroes_after_timeout():
    m = Mcu(host_timeout_ms=2000)
    upright(m, 0)
    fed(m, 0, P.speed_frame(0.2, 0.0), P.cutter_frame(True, 100, 50))
    assert m.cutter_on
    for t in range(0, 1900, 20):
        m.imu_t = t
        m.tick(t)
    assert m.setpoint[0] == pytest.approx(0.2)
    m.imu_t = 2100
    m.tick(2100)
    assert m.setpoint == (0.0, 0.0) and not m.cutter_on and m.watchdog_fired_t == 2100


def test_missing_imu_stream_reads_as_lift():
    m = Mcu()
    m.tick(1000)
    assert m.lift
    upright(m, 1001)
    assert not m.lift


def test_speed_ignored_while_stop_or_lift():
    m = Mcu()
    upright(m, 0)
    m.stop_button = True
    fed(m, 1, P.speed_frame(0.2, 0.0))
    assert m.setpoint == (0.0, 0.0)
    m.stop_button = False
    fed(m, 2, P.imu_frame(0.0, 0.8, 0.0, (0, 0, 9.8), (0, 0, 0)))   # 46 deg pitch
    assert m.lift
    fed(m, 3, P.speed_frame(0.2, 0.0))
    assert m.setpoint == (0.0, 0.0)


def test_lift_latches_and_clear_frame_releases_only_in_window():
    m = Mcu(lift_latch_s=1.0)
    fed(m, 0, P.imu_frame(0.9, 0.0, 0.0, (0, 0, 9.8), (0, 0, 0)))
    fed(m, 1100, P.imu_frame(0.9, 0.0, 0.0, (0, 0, 9.8), (0, 0, 0)))
    assert m.lift_latched
    fed(m, 1200, P.clear_latch_frame())                 # still tilted: refused
    upright(m, 1300)
    assert m.lift
    fed(m, 1400, P.clear_latch_frame())
    upright(m, 1401)
    assert not m.lift


def test_speed_frames_counted_on_dock():
    m = Mcu()
    m.docked = True
    upright(m, 0)
    fed(m, 1, P.speed_frame(0.0, 0.0))
    assert m.speed_frames_while_docked == 1


def test_plant_clamps_and_floors():
    p = DriveParams()
    assert wheel_speeds(1.0, 0.0, p) == (0.3, 0.3)
    assert wheel_speeds(0.0, 0.2, p) == (0.0, 0.0)       # 0.048 m/s per wheel < floor
    plant = Plant()
    for _ in range(200):
        plant.step(0.01, 0.2, 0.0)
    assert abs(plant.v - 0.2) < 1e-3 and plant.x > 0.3


def test_dock_pads_open_only_after_driving_forward_off_them():
    plant = Plant(DriveParams())
    pads = DockPads(plant.x, plant.y, plant.yaw, pad_m=0.10)
    for _ in range(100):                      # reverse into the dock: a hard stop
        plant.step(0.01, -0.15, 0.0)
        assert pads.step(plant)
    assert pads.pushes > 0 and abs(pads.along(plant)) < 1e-9
    for _ in range(400):                      # forward undock at 0.16 m/s
        plant.step(0.01, 0.16, 0.0)
        if not pads.step(plant):
            break
    assert not pads.seated and pads.along(plant) > 0.10
    plant.step(0.5, -0.15, 0.0)               # left for good: no stop behind any more
    assert not pads.step(plant)
