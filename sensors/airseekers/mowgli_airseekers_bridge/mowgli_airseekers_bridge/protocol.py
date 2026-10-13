# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
"""Wire protocol of the Airseekers Tron chassis/cutter MCU. No ROS imports.

The MCU (an AT32 "cutter" board that also gateways the chassis board) sits on
the host UART ``/dev/serial_mower`` (RK3588 ttyS9), 115200 8N1, no flow
control. Every frame:

    0xA5 | total_len | { sub_len | type_id | mod_id | payload[sub_len] }+ | checksum | 0x5A

* ``total_len`` counts the bytes from the first ``sub_len`` through the last
  payload byte; ``checksum`` is ``sum(total_len .. last payload byte) & 0xFF``.
* One frame may carry several sub-packets back to back (the MCU batches
  battery + versions + BMS + motors once per second).
* Byte stuffing between SOF and EOF (checksum included):
  ``A5 -> F5 01``, ``5A -> F5 02``, ``F5 -> F5 03``.
* Multi-byte fields are little-endian packed structs.

Type ids: host->MCU ``0``, MCU->host ``8``, host keep-alive (heartbeat) ``255``.

Origin: recovered by observing the stock vendor software on a Tron (serial
captures, ~170k frames, zero checksum errors) and the vendor's host SDK. This
module is a clean re-implementation of that behaviour; it carries no vendor
code. See README.md "Origin of the protocol".

Everything here is pure and unit-tested (test/test_protocol.py).
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

# ---------------------------------------------------------------------------
# Framing constants
# ---------------------------------------------------------------------------
SOF, EOF, ESC = 0xA5, 0x5A, 0xF5
_UNESCAPE = {0x01: SOF, 0x02: EOF, 0x03: ESC}

TYPE_HOST_TO_MCU = 0
TYPE_MCU_TO_HOST = 8
TYPE_HEARTBEAT = 255

MOD_ALL = 0           # heartbeat target
MOD_CHARGE = 3        # host->MCU ChargeControl (1 B)
MOD_SPEED = 4         # both: SpeedData (8 B)
MOD_BATTERY = 5       # MCU->host BatteryInfo (11 B)
MOD_UPTIME = 7        # MCU->host 4 B counter (~600/s), not decoded
MOD_IMU = 9           # host->MCU ImuData (18 B): the host's IMU, forwarded
MOD_SENSOR = 10       # MCU->host SensorInfo (10 B); host->MCU SensorInfoControl (8 B)
MOD_CUTTER = 11       # host->MCU CutterControl (12 B)
MOD_VERSION = 12      # MCU->host VersionInfo (9 B)
MOD_MOTORS = 13       # MCU->host MotorsInfo (32 B)
MOD_CALIB = 18        # MCU->host MCCalib (3 B), factory test only
MOD_LOG = 19          # MCU->host ASCII text
MOD_BMS_VERSION = 24  # MCU->host Version (3 B)

SPEED_FMT = '<ff'            # linear m/s, angular rad/s
CHARGE_FMT = '<B'            # 1 = charger on, 0 = off
MOTOR_CTRL_FMT = '<BBHH'     # enable, direction (0 fwd, 1 rev), speed, position
SENSOR_CTRL_FMT = '<8B'      # all zero = clear latched lift / stop
BATTERY_FMT = '<HhBbbI'      # voltage 0.1 V, current 0.1 A (+ = discharge), %, dock_ok, degC, error
SENSOR_FMT = '<10B'          # see SENSOR_FIELDS
IMU_FMT = '<9h'              # pitch, roll, yaw, accx..z, gyrox..z (WIT raw counts)
VERSION_FMT = '<9B'          # cutter, chassis, rtk boards (major, minor, patch)
BMS_FMT = '<3B'
CALIB_FMT = '<3B'
MOTOR_FMT = '<hhhbb'         # speed rpm, current, voltage, degC, signed status
HEARTBEAT_FMT = '<8B'        # year-2000, month, day, hour, minute, second, ms_hi, ms_lo

SENSOR_FIELDS = ('bumper', 'rain', 'lift', 'stop', 'power_off', 'battery_gate',
                 'cutter_size', 'press_module', 'bumper_r', 'bumper_l')
MOTOR_NAMES = ('cutter', 'left', 'right', 'height')

# MotorInfo units. The cutter board reports current in 10 mA and voltage in
# 10 mV; the two drive boards use other scalings (voltage ~655.36 counts/V,
# empirical from two battery levels; current read as mA, unverified while
# driving). The height motor slot is not populated on this hardware.
MOTOR_CURRENT_A_PER_COUNT = {'cutter': 0.01, 'left': 0.001, 'right': 0.001, 'height': 0.01}
MOTOR_VOLTAGE_V_PER_COUNT = {'cutter': 0.01, 'left': 1.0 / 655.36, 'right': 1.0 / 655.36,
                             'height': 0.01}
MOTOR_STATUS_TEXT = {0: 'idle', 1: 'running', 2: 'locking', -1: 'error', -2: 'over-current',
                     -3: 'over-voltage', -4: 'under-voltage', -5: 'over-temperature',
                     -6: 'stalled', -7: 'overload'}

# WIT-Motion raw scaling used by ImuData: angle*32768/180 deg, acc*32768/16 g,
# gyro*32768/2000 dps.
WIT_RAW_ANGLE_TO_RAD = math.pi / 32768.0
WIT_RAW_GYRO_TO_RAD = math.radians(2000.0) / 32768.0
WIT_RAW_ACC_TO_MS2 = 16.0 * 9.80665 / 32768.0

# Deck height: CutterControl.height.position is an ABSOLUTE height in mm.
# Never send 0 (out of range, drives the deck to its stop).
CUTTER_HEIGHT_MIN_MM = 30
CUTTER_HEIGHT_MAX_MM = 90

# Battery scaling.
BATTERY_VOLTAGE_V_PER_COUNT = 0.1
BATTERY_DISCHARGE_A_PER_COUNT = 0.1   # positive raw = discharging


# ---------------------------------------------------------------------------
# Framing
# ---------------------------------------------------------------------------
def escape(data: bytes) -> bytes:
    out = bytearray()
    for b in data:
        if b == SOF:
            out += b'\xF5\x01'
        elif b == EOF:
            out += b'\xF5\x02'
        elif b == ESC:
            out += b'\xF5\x03'
        else:
            out.append(b)
    return bytes(out)


def unescape(data: bytes) -> bytes:
    out = bytearray()
    i = 0
    while i < len(data):
        if data[i] == ESC and i + 1 < len(data):
            out.append(_UNESCAPE.get(data[i + 1], data[i + 1]))
            i += 2
        else:
            out.append(data[i])
            i += 1
    return bytes(out)


SubPacket = Tuple[int, int, bytes]


def build_frame(subpackets: Iterable[SubPacket]) -> bytes:
    """``[(type_id, mod_id, payload), ...]`` -> one wire-ready frame."""
    body = bytearray()
    for type_id, mod_id, payload in subpackets:
        if len(payload) > 255:
            raise ValueError('sub-packet payload too long: %d bytes' % len(payload))
        body += bytes((len(payload), type_id, mod_id)) + bytes(payload)
    if len(body) > 255:
        raise ValueError('frame body too long: %d bytes' % len(body))
    inner = bytes((len(body),)) + bytes(body)
    return bytes((SOF,)) + escape(inner + bytes((sum(inner) & 0xFF,))) + bytes((EOF,))


def split_subpackets(inner: bytes) -> List[SubPacket]:
    """``inner`` = total_len .. checksum (unescaped)."""
    body = inner[1:-1]
    out = []
    k = 0
    while k + 3 <= len(body):
        length, type_id, mod_id = body[k], body[k + 1], body[k + 2]
        out.append((type_id, mod_id, bytes(body[k + 3:k + 3 + length])))
        k += 3 + length
    return out


class FrameParser:
    """Incremental RX parser: feed serial bytes, get validated sub-packets.

    Resynchronises on 0xA5, takes the next 0x5A as the end (a raw 0x5A never
    occurs inside a frame), unescapes, then checks length and checksum before
    accepting anything; a bad frame costs one byte of resync.
    """

    MAX_BUFFER = 8192

    def __init__(self) -> None:
        self._buf = bytearray()
        self.frames_ok = 0
        self.frames_bad = 0

    def feed(self, data: bytes) -> List[SubPacket]:
        self._buf += data
        if len(self._buf) > self.MAX_BUFFER:
            del self._buf[:-self.MAX_BUFFER]
        out: List[SubPacket] = []
        buf = self._buf
        i = consumed = 0
        while True:
            sof = buf.find(b'\xA5', i)
            if sof < 0:
                consumed = len(buf)
                break
            eof = buf.find(b'\x5A', sof + 1)
            if eof < 0:
                consumed = sof
                break
            inner = unescape(bytes(buf[sof + 1:eof]))
            if (len(inner) >= 5 and inner[0] == len(inner) - 2
                    and (sum(inner[:-1]) & 0xFF) == inner[-1]):
                out.extend(split_subpackets(inner))
                self.frames_ok += 1
                consumed = i = eof + 1
            else:
                self.frames_bad += 1
                consumed = i = sof + 1
        if consumed:
            del buf[:consumed]
        return out


# ---------------------------------------------------------------------------
# Host -> MCU payloads
# ---------------------------------------------------------------------------
def heartbeat_payload(now: Optional[datetime] = None) -> bytes:
    """8-byte wall-clock heartbeat (year is year-2000). The vendor host sends
    one every 100 ms."""
    now = now or datetime.now()
    ms = now.microsecond // 1000
    return struct.pack(HEARTBEAT_FMT, (now.year - 2000) & 0xFF, now.month, now.day,
                       now.hour, now.minute, now.second, (ms >> 8) & 0xFF, ms & 0xFF)


def speed_payload(linear: float, angular: float) -> bytes:
    return struct.pack(SPEED_FMT, float(linear), float(angular))


def charge_payload(enable: bool) -> bytes:
    return struct.pack(CHARGE_FMT, 1 if enable else 0)


def clear_latch_payload() -> bytes:
    """SensorInfoControl, all zero: the MCU releases a latched lift / stop
    (only while the trigger itself is released and the IMU is in-window)."""
    return bytes(8)


def clamp_height_mm(mm: float) -> int:
    return max(CUTTER_HEIGHT_MIN_MM, min(CUTTER_HEIGHT_MAX_MM, int(round(mm))))


def cutter_payload(enable: bool, speed_percent: int, height_mm: float,
                   reverse: bool = False) -> bytes:
    """CutterControl = cutter MotorControl + height MotorControl (12 B).

    The height motor's enable/direction/speed stay 0 (as the vendor sends);
    only its absolute position in mm matters, clamped to 30..90.
    """
    def u16(v: float) -> int:
        return max(0, min(0xFFFF, int(v)))
    cutter = struct.pack(MOTOR_CTRL_FMT, 1 if enable else 0, 1 if (enable and reverse) else 0,
                         u16(speed_percent) if enable else 0, 0)
    height = struct.pack(MOTOR_CTRL_FMT, 0, 0, 0, clamp_height_mm(height_mm))
    return cutter + height


def _i16(v: float) -> int:
    return max(-32768, min(32767, int(round(v))))


def imu_payload(roll: float, pitch: float, yaw: float,
                acc: Sequence[float], gyro: Sequence[float]) -> bytes:
    """ImuData (18 B) in WIT raw counts from SI (rad, m/s^2, rad/s).

    The MCU needs this stream (~45 Hz): it computes ``lift`` from the
    forwarded attitude, and without the stream it reports the mower lifted.
    Field order on the wire is pitch, roll, yaw.
    """
    ang = lambda r: _i16(r / math.pi * 32768.0)                       # noqa: E731
    acc_c = lambda a: _i16(a / (16.0 * 9.80665) * 32768.0)            # noqa: E731
    gyr_c = lambda g: _i16(g / math.radians(2000.0) * 32768.0)        # noqa: E731
    return struct.pack(IMU_FMT, ang(pitch), ang(roll), ang(yaw),
                       acc_c(acc[0]), acc_c(acc[1]), acc_c(acc[2]),
                       gyr_c(gyro[0]), gyr_c(gyro[1]), gyr_c(gyro[2]))


def heartbeat_frame(now: Optional[datetime] = None) -> bytes:
    return build_frame([(TYPE_HEARTBEAT, MOD_ALL, heartbeat_payload(now))])


def speed_frame(linear: float, angular: float) -> bytes:
    return build_frame([(TYPE_HOST_TO_MCU, MOD_SPEED, speed_payload(linear, angular))])


def cutter_frame(enable: bool, speed_percent: int, height_mm: float, reverse: bool = False) -> bytes:
    return build_frame([(TYPE_HOST_TO_MCU, MOD_CUTTER,
                         cutter_payload(enable, speed_percent, height_mm, reverse))])


def clear_latch_frame() -> bytes:
    return build_frame([(TYPE_HOST_TO_MCU, MOD_SENSOR, clear_latch_payload())])


def charge_frame(enable: bool) -> bytes:
    return build_frame([(TYPE_HOST_TO_MCU, MOD_CHARGE, charge_payload(enable))])


def imu_frame(roll, pitch, yaw, acc, gyro) -> bytes:
    return build_frame([(TYPE_HOST_TO_MCU, MOD_IMU, imu_payload(roll, pitch, yaw, acc, gyro))])


# ---------------------------------------------------------------------------
# MCU -> host payloads
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class SpeedData:
    linear: float
    angular: float


@dataclass(frozen=True)
class BatteryInfo:
    voltage_v: float
    discharge_a: float       # + = discharging, - = charging (MCU convention)
    percentage: int
    dock_ok: bool            # charger contacts live = on the dock
    temperature_c: int
    error: int


@dataclass(frozen=True)
class SensorInfo:
    bumper: bool
    rain: bool
    lift: bool
    stop: bool
    power_off: bool
    battery_gate: bool
    cutter_size: int
    press_module: bool
    bumper_r: bool
    bumper_l: bool

    @property
    def any_bumper(self) -> bool:
        return self.bumper or self.bumper_l or self.bumper_r


@dataclass(frozen=True)
class MotorInfo:
    speed_rpm: int
    current_a: float
    voltage_v: float
    temperature_c: int
    status: int

    @property
    def status_text(self) -> str:
        return MOTOR_STATUS_TEXT.get(self.status, 'status %d' % self.status)


@dataclass(frozen=True)
class VersionInfo:
    cutter: Tuple[int, int, int]
    chassis: Tuple[int, int, int]
    rtk: Tuple[int, int, int]


@dataclass(frozen=True)
class ImuData:
    roll: float
    pitch: float
    yaw: float
    acc: Tuple[float, float, float]
    gyro: Tuple[float, float, float]


def _sized(payload: bytes, fmt: str) -> bool:
    return len(payload) == struct.calcsize(fmt)


def decode_speed(payload: bytes) -> Optional[SpeedData]:
    if not _sized(payload, SPEED_FMT):
        return None
    lin, ang = struct.unpack(SPEED_FMT, payload)
    return SpeedData(lin, ang)


def decode_battery(payload: bytes) -> Optional[BatteryInfo]:
    if not _sized(payload, BATTERY_FMT):
        return None
    v, i, pct, dock, temp, err = struct.unpack(BATTERY_FMT, payload)
    return BatteryInfo(v * BATTERY_VOLTAGE_V_PER_COUNT, i * BATTERY_DISCHARGE_A_PER_COUNT,
                       pct, dock != 0, temp, err)


def encode_battery(info: BatteryInfo) -> bytes:
    return struct.pack(BATTERY_FMT, int(round(info.voltage_v / BATTERY_VOLTAGE_V_PER_COUNT)),
                       int(round(info.discharge_a / BATTERY_DISCHARGE_A_PER_COUNT)),
                       int(info.percentage), 1 if info.dock_ok else 0, int(info.temperature_c),
                       int(info.error))


def decode_sensor(payload: bytes) -> Optional[SensorInfo]:
    if not _sized(payload, SENSOR_FMT):
        return None
    v = struct.unpack(SENSOR_FMT, payload)
    d = dict(zip(SENSOR_FIELDS, v))
    return SensorInfo(**{k: (val if k == 'cutter_size' else bool(val)) for k, val in d.items()})


def encode_sensor(info: SensorInfo) -> bytes:
    return struct.pack(SENSOR_FMT, *[int(getattr(info, k)) for k in SENSOR_FIELDS])


def decode_motors(payload: bytes) -> Optional[Dict[str, MotorInfo]]:
    size = struct.calcsize(MOTOR_FMT)
    if len(payload) != size * len(MOTOR_NAMES):
        return None
    out = {}
    for idx, name in enumerate(MOTOR_NAMES):
        speed, current, voltage, temp, status = struct.unpack(
            MOTOR_FMT, payload[idx * size:(idx + 1) * size])
        out[name] = MotorInfo(speed, current * MOTOR_CURRENT_A_PER_COUNT[name],
                              voltage * MOTOR_VOLTAGE_V_PER_COUNT[name], temp, status)
    return out


def encode_motors(motors: Dict[str, Tuple[int, int, int, int, int]]) -> bytes:
    """Raw wire tuples ``{name: (speed, current, voltage, temp, status)}``."""
    return b''.join(struct.pack(MOTOR_FMT, *motors[name]) for name in MOTOR_NAMES)


def decode_version(payload: bytes) -> Optional[VersionInfo]:
    if not _sized(payload, VERSION_FMT):
        return None
    v = struct.unpack(VERSION_FMT, payload)
    return VersionInfo(tuple(v[0:3]), tuple(v[3:6]), tuple(v[6:9]))


def decode_imu(payload: bytes) -> Optional[ImuData]:
    if not _sized(payload, IMU_FMT):
        return None
    p, r, y, ax, ay, az, gx, gy, gz = struct.unpack(IMU_FMT, payload)
    a = WIT_RAW_ANGLE_TO_RAD
    return ImuData(r * a, p * a, y * a,
                   (ax * WIT_RAW_ACC_TO_MS2, ay * WIT_RAW_ACC_TO_MS2, az * WIT_RAW_ACC_TO_MS2),
                   (gx * WIT_RAW_GYRO_TO_RAD, gy * WIT_RAW_GYRO_TO_RAD, gz * WIT_RAW_GYRO_TO_RAD))


def decode_cutter(payload: bytes):
    """CutterControl -> (enable, reverse, speed, height_mm); for the emulator."""
    if len(payload) != 2 * struct.calcsize(MOTOR_CTRL_FMT):
        return None
    en, direction, speed, _ = struct.unpack(MOTOR_CTRL_FMT, payload[:6])
    _, _, _, height = struct.unpack(MOTOR_CTRL_FMT, payload[6:])
    return bool(en), bool(direction), speed, height


def version_string(v: Tuple[int, int, int]) -> str:
    return '%d.%d.%d' % tuple(v)
