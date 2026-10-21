# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
# Ported from the Airseekers ROS 2 port (wit_imu_driver/wit_protocol.py); same author, relicensed GPL-3.0.
"""WIT-Motion (JY61P / JY901 family) serial protocol, ROS-free.

Standard WIT output frame, 11 bytes, little-endian:

    0x55 | type | d0 d1 | d2 d3 | d4 d5 | d6 d7 | sum

``sum`` is the low byte of the sum of the first 10 bytes. Each data pair is an
int16. Frame types used here:

    0x50 time          (not decoded)
    0x51 acceleration  ax ay az  temp     raw/32768*16 g
    0x52 angular rate  wx wy wz  temp     raw/32768*2000 deg/s
    0x53 angle         roll pitch yaw ver raw/32768*180 deg
    0x54 magnetic      hx hy hz  temp     raw counts
    0x59 quaternion    q0 q1 q2 q3        raw/32768

Temperature in 0x51/0x52 is raw/100 degC.

The previous driver assumed 5-byte frames carrying one axis each; that format
does not exist in the WIT protocol and would have mis-decoded every frame.
"""

import math
import struct
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

FRAME_LEN = 11
SYNC = 0x55

TYPE_TIME = 0x50
TYPE_ACC = 0x51
TYPE_GYRO = 0x52
TYPE_ANGLE = 0x53
TYPE_MAG = 0x54
TYPE_QUAT = 0x59

RAW_ANGLE_TO_RAD = math.radians(180.0) / 32768.0
RAW_GYRO_TO_RAD = math.radians(2000.0) / 32768.0
RAW_ACC_TO_MS2 = 16.0 * 9.80665 / 32768.0
RAW_QUAT = 1.0 / 32768.0

_PAYLOAD = struct.Struct('<4h')


def checksum(frame: bytes) -> int:
    return sum(frame[:10]) & 0xFF


def build_frame(frame_type: int, values: Tuple[int, int, int, int]) -> bytes:
    """Encode a frame (used by tests and the fake device)."""
    body = bytes([SYNC, frame_type]) + _PAYLOAD.pack(*values)
    return body + bytes([sum(body) & 0xFF])


@dataclass
class ImuSample:
    """Latest decoded values in SI units; ``None`` until the frame type was seen."""
    acc: Optional[Tuple[float, float, float]] = None        # m/s^2
    gyro: Optional[Tuple[float, float, float]] = None       # rad/s
    rpy: Optional[Tuple[float, float, float]] = None        # rad (roll, pitch, yaw)
    quat: Optional[Tuple[float, float, float, float]] = None  # (x, y, z, w)
    mag: Optional[Tuple[int, int, int]] = None              # raw counts
    temperature_c: Optional[float] = None
    counts: Dict[int, int] = field(default_factory=dict)

    @property
    def complete(self) -> bool:
        return self.acc is not None and self.gyro is not None and self.rpy is not None


class WitParser:
    """Incremental frame parser. Feed bytes, get a list of decoded frame types."""

    def __init__(self):
        self._buf = bytearray()
        self.sample = ImuSample()
        self.bad_checksums = 0

    def feed(self, data: bytes) -> List[int]:
        self._buf += data
        seen: List[int] = []
        buf = self._buf
        while len(buf) >= FRAME_LEN:
            if buf[0] != SYNC:
                del buf[0]
                continue
            frame = bytes(buf[:FRAME_LEN])
            if checksum(frame) != frame[10]:
                self.bad_checksums += 1
                del buf[0]
                continue
            del buf[:FRAME_LEN]
            if self._decode(frame):
                seen.append(frame[1])
        return seen

    def _decode(self, frame: bytes) -> bool:
        ftype = frame[1]
        a, b, c, d = _PAYLOAD.unpack(frame[2:10])
        s = self.sample
        s.counts[ftype] = s.counts.get(ftype, 0) + 1
        if ftype == TYPE_ACC:
            s.acc = (a * RAW_ACC_TO_MS2, b * RAW_ACC_TO_MS2, c * RAW_ACC_TO_MS2)
            s.temperature_c = d / 100.0
        elif ftype == TYPE_GYRO:
            s.gyro = (a * RAW_GYRO_TO_RAD, b * RAW_GYRO_TO_RAD, c * RAW_GYRO_TO_RAD)
        elif ftype == TYPE_ANGLE:
            s.rpy = (a * RAW_ANGLE_TO_RAD, b * RAW_ANGLE_TO_RAD, c * RAW_ANGLE_TO_RAD)
        elif ftype == TYPE_MAG:
            s.mag = (a, b, c)
        elif ftype == TYPE_QUAT:
            # WIT order is q0=w, q1=x, q2=y, q3=z
            s.quat = (b * RAW_QUAT, c * RAW_QUAT, d * RAW_QUAT, a * RAW_QUAT)
        else:
            return False
        return True


def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    """Body-fixed ZYX rotation (rad) -> (x, y, z, w)."""
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


# Configuration commands (unlock, set, save) are all 5 bytes: FF AA reg lo hi.
CMD_UNLOCK = bytes([0xFF, 0xAA, 0x69, 0x88, 0xB5])
CMD_SAVE = bytes([0xFF, 0xAA, 0x00, 0x00, 0x00])


def cmd_set_rate(rate_hz: int) -> bytes:
    """RRATE register 0x03. Supported: 1,2,5,10,20,50,100,200 Hz."""
    table = {1: 0x03, 2: 0x04, 5: 0x05, 10: 0x06, 20: 0x07, 50: 0x08, 100: 0x09, 200: 0x0B}
    return bytes([0xFF, 0xAA, 0x03, table[rate_hz], 0x00])


def cmd_set_output(acc=True, gyro=True, angle=True, mag=False, quat=False) -> bytes:
    """RSW register 0x02: bitmask of enabled frame types."""
    mask = (acc << 1) | (gyro << 2) | (angle << 3) | (mag << 4) | (quat << 9)
    return bytes([0xFF, 0xAA, 0x02, mask & 0xFF, (mask >> 8) & 0xFF])
