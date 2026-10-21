# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
"""Framing and payload codec.

The golden frames below were produced by the Airseekers ROS 2 port's
mower_mcu_driver (the driver that ran on the Tron, validated against ~170k
captured frames), which is the oracle for this re-implementation. Set
AIRSEEKERS_ORACLE_DRIVER to that package's directory to also run the
randomised comparison against it.
"""
import importlib
import math
import os
import random
import struct
import sys
from datetime import datetime

import pytest

from mowgli_airseekers_bridge import protocol as P

GOLDEN = {
    'speed': 'a50b0800040000803ecdccccbdf75a',
    'hb': 'a50b08ff001a0a050c22380315b95a',
    'cut': 'a50f0c000b010064000000000000003200bd5a',
    'cutoff': 'a50f0c000b00000000000000000000f50200805a',
    'clear': 'a50b08000a00000000000000001d5a',
    'imu': 'a515120009daf713041e3d1500d6ffff0709001300e4ff625a',
    'charge': 'a50401000301095a',
    'esc': 'a506030813f501f502f503185a',
}


def test_golden_host_frames():
    assert P.speed_frame(0.25, -0.1).hex() == GOLDEN['speed']
    assert P.heartbeat_frame(datetime(2026, 10, 5, 12, 34, 56, 789000)).hex() == GOLDEN['hb']
    assert P.cutter_frame(True, 100, 50).hex() == GOLDEN['cut']
    assert P.cutter_frame(False, 0, 90).hex() == GOLDEN['cutoff']
    assert P.clear_latch_frame().hex() == GOLDEN['clear']
    assert P.imu_frame(0.1, -0.2, 1.5, (0.1, -0.2, 9.8), (0.01, 0.02, -0.03)).hex() == GOLDEN['imu']
    assert P.charge_frame(True).hex() == GOLDEN['charge']
    assert P.build_frame([(8, 19, b'\xa5\x5a\xf5')]).hex() == GOLDEN['esc']


def test_cutoff_frame_needs_escaping_and_round_trips():
    # height 90 = 0x5A, the EOF byte: it must be stuffed.
    frame = P.cutter_frame(False, 0, 90)
    assert b'\x5a' not in frame[1:-1]
    sub = P.FrameParser().feed(frame)
    assert P.decode_cutter(sub[0][2]) == (False, False, 0, 90)


def test_round_trip_and_multi_subpacket():
    subs = [(8, P.MOD_BATTERY, b'\x01' * 11), (8, P.MOD_VERSION, b'\x02' * 9),
            (8, P.MOD_MOTORS, b'\x03' * 32)]
    assert P.FrameParser().feed(P.build_frame(subs)) == subs


def test_byte_dribble_reassembly():
    subs = [(8, P.MOD_SENSOR, bytes(range(10))), (8, P.MOD_SPEED, struct.pack('<ff', -1.25, 0.5))]
    stream = P.build_frame(subs) * 3
    parser = P.FrameParser()
    got = []
    for i in range(0, len(stream), 3):
        got.extend(parser.feed(stream[i:i + 3]))
    assert got == subs * 3


def test_bad_checksum_rejected_and_resync():
    good = P.speed_frame(1.0, 0.0)
    broken = bytearray(good)
    broken[-2] ^= 0xFF
    parser = P.FrameParser()
    assert parser.feed(bytes(broken) + good) == [(0, P.MOD_SPEED, P.speed_payload(1.0, 0.0))]
    assert parser.frames_bad >= 1


def test_leading_garbage_skipped():
    got = P.FrameParser().feed(b'\x00\x13garbage\xA5\x00' + P.heartbeat_frame())
    assert len(got) == 1 and got[0][0] == P.TYPE_HEARTBEAT


def test_payload_sizes_match_the_wire():
    for fmt, size in ((P.SPEED_FMT, 8), (P.BATTERY_FMT, 11), (P.SENSOR_FMT, 10),
                      (P.IMU_FMT, 18), (P.VERSION_FMT, 9), (P.BMS_FMT, 3), (P.MOTOR_FMT, 8),
                      (P.HEARTBEAT_FMT, 8), (P.SENSOR_CTRL_FMT, 8), (P.MOTOR_CTRL_FMT, 6)):
        assert struct.calcsize(fmt) == size


# Raw payloads captured on a Tron at rest (2026-10-06, battery 19.7 V).
CAPTURED_MOTORS = '00000000b1071e000000fe0dfe312801000097fdfe312201ef4c60ea00000000'
CAPTURED_BATTERY = 'c50007001f002000000000'


def test_decode_captured_motors():
    m = P.decode_motors(bytes.fromhex(CAPTURED_MOTORS))
    assert abs(m['cutter'].voltage_v - 19.69) < 1e-6 and m['cutter'].status == 0
    assert m['cutter'].temperature_c == 30
    for drive in (m['left'], m['right']):
        assert 19.0 < drive.voltage_v < 20.2
        assert drive.status_text == 'running'
    assert abs(m['left'].current_a - 3.582) < 1e-9
    assert abs(m['right'].current_a + 0.617) < 1e-9
    assert P.decode_motors(b'\x00' * 31) is None


def test_decode_captured_battery():
    b = P.decode_battery(bytes.fromhex(CAPTURED_BATTERY))
    assert abs(b.voltage_v - 19.7) < 1e-9 and abs(b.discharge_a - 0.7) < 1e-9
    assert b.percentage == 31 and not b.dock_ok and b.temperature_c == 32
    assert P.encode_battery(b).hex() == CAPTURED_BATTERY


def test_sensor_round_trip():
    s = P.SensorInfo(False, True, True, False, False, False, 1, True, False, True)
    assert P.decode_sensor(P.encode_sensor(s)) == s
    assert s.any_bumper


def test_imu_round_trip():
    payload = P.imu_payload(0.3, -0.2, 1.0, (0.5, -0.5, 9.81), (0.1, -0.1, 0.2))
    d = P.decode_imu(payload)
    assert abs(d.roll - 0.3) < 1e-3 and abs(d.pitch + 0.2) < 1e-3 and abs(d.yaw - 1.0) < 1e-3
    assert abs(d.acc[2] - 9.81) < 0.01 and abs(d.gyro[2] - 0.2) < 1e-3


def test_height_clamped_never_zero():
    assert P.decode_cutter(P.cutter_payload(True, 100, 0)) [3] == 30
    assert P.decode_cutter(P.cutter_payload(True, 100, 200))[3] == 90
    assert P.decode_cutter(P.cutter_payload(False, 100, 55)) == (False, False, 0, 55)


def _oracle():
    path = os.environ.get('AIRSEEKERS_ORACLE_DRIVER')
    if not path:
        pytest.skip('AIRSEEKERS_ORACLE_DRIVER not set (path to mower_mcu_driver)')
    sys.path.insert(0, os.path.join(path, 'test'))
    sys.path.insert(0, path)
    import ros_stubs  # noqa: F401
    ros_stubs.install()
    return importlib.import_module('mower_mcu_driver.mcu_node')


def test_randomised_against_oracle_driver():
    mn = _oracle()
    rng = random.Random(7)
    for _ in range(500):
        lin, ang = rng.uniform(-1, 1), rng.uniform(-1, 1)
        assert P.speed_frame(lin, ang) == mn.build_frame([(0, 4, mn.speed_payload(lin, ang))])
        payload = bytes(rng.randrange(256) for _ in range(rng.randrange(0, 60)))
        assert P.build_frame([(8, 19, payload)]) == mn.build_frame([(8, 19, payload)])
        h = rng.randrange(0, 200)
        assert P.cutter_payload(True, 100, h) == mn.cutter_payload(
            True, False, 100, False, False, mn.clamp_height_mm(h))
        r, p, y = (rng.uniform(-math.pi, math.pi) for _ in range(3))
        acc = tuple(rng.uniform(-30, 30) for _ in range(3))
        gyro = tuple(rng.uniform(-5, 5) for _ in range(3))
        assert P.imu_payload(r, p, y, acc, gyro) == mn.imu_payload(r, p, y, acc, gyro)
    stream = b''.join(P.build_frame([(8, rng.randrange(30), bytes(rng.randrange(256)
                                                                    for _ in range(9)))])
                      for _ in range(200))
    stream = bytes(b ^ 0xFF if rng.random() < 0.002 else b for b in stream)
    ours, theirs = P.FrameParser(), mn.FrameParser()
    a, b = [], []
    for i in range(0, len(stream), 37):
        a.extend(ours.feed(stream[i:i + 37]))
        b.extend(theirs.feed(stream[i:i + 37]))
    assert a == b and ours.frames_bad == theirs.frames_bad
