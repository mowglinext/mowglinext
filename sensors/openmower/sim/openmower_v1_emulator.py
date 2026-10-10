#!/usr/bin/env python3
# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
"""Emulated OpenMower v1 electronics for the mowgli_openmower_bridge simulation.

Four pseudo-terminals stand in for the Raspberry Pi UARTs: the LowLevel (Pico)
board and three xESC motor controllers. A small differential-drive plant turns
the duty the controllers receive into wheel ticks, IMU rates and a pose.

Every rule below is taken from the hardware's own sources — NOT from the
bridge — so a wrong sign or timeout in the bridge cannot be mirrored here and
pass unnoticed:

LowLevel board — OpenMower ``Firmware/LowLevel/src/main.cpp`` on branch
``v1-fw`` (current v1 firmware; tag v0.13.2 for the older charging rule):
  * boots with ``emergency_latch = true``;
  * no heartbeat for HEARTBEAT_MILLIS (500 ms) -> latch, ``ROS_running = false``;
  * heartbeat ``emergency_release_requested`` clears the latch
    UNCONDITIONALLY, ``emergency_requested`` (checked after) sets it;
  * ``updateEmergency``: stop debounced BUTTON_EMERGENCY_MILLIS (20 ms), lift
    when >= 2 wheels for ``lift_period``, tilt when >= 1 for ``tilt_period``,
    any trigger -> latch; a latch change is sent immediately;
  * status every STATUS_CYCLETIME (100 ms), IMU every IMU_CYCLETIME (20 ms);
  * ``applyConfig``: start from the board's compiled defaults and take over
    only fields that are not -1 / 0xFFFF / UNDEFINED; the result is saved to
    flash and echoed in a CONFIG_RSP when the packet was a REQ;
  * ``updateChargingEnabled``: v1-fw forces ``charging_allowed = false`` below
    3 V of charge voltage; v0.13.x has no such early return, so its relay bit
    reads 1 OFF the dock. Both open the relay once ``checkShouldCharge`` fails
    and retry after CHARGING_RETRY_MILLIS (10 s).

xESC 2040 — ``ClemensElflein/xESC2040`` firmware: status every
STATUS_UPDATE_MILLIS (20 ms), ``FAULT_UNINITIALIZED`` until a settings packet,
``FAULT_WATCHDOG`` (motor off) after WATCHDOG_TIMEOUT_MILLIS (500 ms) without
a control packet, ``direction = hall_diff < 0``.

xESC mini — ``ClemensElflein/xesc_firmware`` (VESC fork): request/response,
``APPCONF_TIMEOUT_MSEC`` 1000 ms without a command -> coast
(``APPCONF_TIMEOUT_BRAKE_CURRENT`` 0).

Wiring — ``open_mower_ros`` ``mower_comms_v1.cpp``: ``left.setDutyCycle(
speed_l); right.setDutyCycle(-speed_r)`` ("the ESC has the same config as the
left one, so the motor is running in the wrong direction"). A positive duty on
the LEFT controller drives the left wheel forward; the RIGHT wheel goes
forward on a NEGATIVE duty.

No ROS import: this module only speaks bytes.
"""

from __future__ import annotations

import collections
import math
import os
import random
import struct
import threading
import time
import tty

# ---------------------------------------------------------------------------
# Framing
# ---------------------------------------------------------------------------


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def crc16_xmodem(data: bytes) -> int:
    crc = 0x0000
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    out = bytearray([0])
    code_idx, code = 0, 1
    for b in data:
        if b == 0:
            out[code_idx] = code
            code_idx, code = len(out), 1
            out.append(0)
        else:
            out.append(b)
            code += 1
            if code == 0xFF:
                out[code_idx] = code
                code_idx, code = len(out), 1
                out.append(0)
    out[code_idx] = code
    return bytes(out)


def cobs_decode(data: bytes):
    out = bytearray()
    i, n = 0, len(data)
    while i < n:
        code = data[i]
        if code == 0:
            return None
        i += 1
        end = i + code - 1
        if end > n:
            return None
        out += data[i:end]
        i = end
        if code < 0xFF and i < n:
            out.append(0)
    return bytes(out)


def cobs_packet(payload: bytes) -> bytes:
    """type + fields (no CRC) -> COBS frame + 0x00, CRC-16/CCITT-FALSE LE."""
    body = payload + struct.pack('<H', crc16_ccitt_false(payload))
    return cobs_encode(body) + b'\x00'


class CobsReader:
    """0x00-delimited COBS frames -> CRC-verified payloads (CRC stripped)."""

    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0

    def feed(self, data: bytes):
        frames = []
        for b in data:
            if b != 0:
                self.buf.append(b)
                if len(self.buf) > 4096:
                    self.buf.clear()
                continue
            if not self.buf:
                continue  # leading / repeated delimiter
            dec = cobs_decode(bytes(self.buf))
            self.buf.clear()
            if dec is None or len(dec) < 3:
                continue
            body, crc = dec[:-2], struct.unpack('<H', dec[-2:])[0]
            if crc16_ccitt_false(body) == crc:
                frames.append(body)
            else:
                self.crc_errors += 1
        return frames


def vesc_frame(payload: bytes) -> bytes:
    assert len(payload) < 256
    crc = crc16_xmodem(payload)
    return bytes([2, len(payload)]) + payload + struct.pack('>H', crc) + b'\x03'


class VescReader:
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes):
        self.buf += data
        out = []
        while len(self.buf) >= 5:
            if self.buf[0] != 2:
                del self.buf[0]
                continue
            n = self.buf[1]
            if len(self.buf) < n + 5:
                break
            payload = bytes(self.buf[2:2 + n])
            crc = struct.unpack('>H', self.buf[2 + n:4 + n])[0]
            eof = self.buf[4 + n]
            if eof == 3 and crc == crc16_xmodem(payload) and n > 0:
                out.append(payload)
                del self.buf[:n + 5]
            else:
                del self.buf[0]
        return out


# ---------------------------------------------------------------------------
# Pseudo-terminals
# ---------------------------------------------------------------------------


class PtyPort:
    """A UART: data the emulator writes appears on the slave the bridge opens.

    Writes never block. When nobody drains the slave (bridge not running), the
    pty buffer fills and further bytes are dropped — like a UART with nobody
    listening.
    """

    def __init__(self, link_path: str):
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)  # no echo before the bridge configures the port
        os.set_blocking(self.master, False)
        self.link_path = link_path
        if os.path.lexists(link_path):
            os.unlink(link_path)
        os.symlink(os.ttyname(self.slave), link_path)
        self.tx_dropped = 0

    def write(self, data: bytes):
        try:
            n = os.write(self.master, data)
            self.tx_dropped += len(data) - n
        except OSError:
            self.tx_dropped += len(data)

    def read(self) -> bytes:
        try:
            return os.read(self.master, 8192)
        except OSError:
            return b''

    def close(self):
        for fd in (self.master, self.slave):
            try:
                os.close(fd)
            except OSError:
                pass
        if os.path.islink(self.link_path):
            os.unlink(self.link_path)


# ---------------------------------------------------------------------------
# Plant
# ---------------------------------------------------------------------------

TICKS_PER_M = 1600.0  # YardForce 500 OM_WHEEL_TICKS_PER_M
WHEEL_TRACK = 0.325   # OM_WHEEL_DISTANCE_M


class DriveMotor:
    """One xESC and its wheel, in the CONTROLLER's rotation frame.

    Deliberately imperfect, so an open-loop bridge visibly misses its target:
    0.85 m/s of wheel surface per unit duty (OpenMower's own comms assumes
    1.0), a 0.04 duty deadband and a 120 ms first-order lag.
    """

    GAIN = 0.85
    DEADBAND = 0.04
    TAU = 0.12

    def __init__(self):
        self.duty = 0.0     # last commanded duty as the controller received it
        self.speed = 0.0    # controller-frame wheel surface speed [m/s]
        self.ticks = 0.0    # controller-frame signed cumulative ticks

    def step(self, dt: float, effective_duty: float):
        target = 0.0 if abs(effective_duty) < self.DEADBAND else self.GAIN * effective_duty
        self.speed += (target - self.speed) * min(1.0, dt / self.TAU)
        self.ticks += self.speed * dt * TICKS_PER_M


class BladeMotor:
    RPM_PER_DUTY = 3600.0
    TAU = 0.4
    POLE_PAIRS = 4  # the bridge's motor_pole_pairs default
    # xESC2040 main.cpp adds one to tacho_absolute per hall state change.
    HALL_STEPS_PER_REV = 6 * POLE_PAIRS

    def __init__(self):
        self.duty = 0.0
        self.rpm = 0.0    # signed: negative while the duty is
        self.ticks = 0.0  # signed hall steps

    def step(self, dt: float, effective_duty: float):
        self.rpm += (effective_duty * self.RPM_PER_DUTY - self.rpm) * min(1.0, dt / self.TAU)
        self.ticks += self.rpm / 60.0 * dt * self.HALL_STEPS_PER_REV


class Plant:
    def __init__(self):
        self.x = self.y = self.theta = 0.0
        self.v = self.w = 0.0

    def step(self, dt: float, v_left: float, v_right: float):
        self.v = 0.5 * (v_left + v_right)
        self.w = (v_right - v_left) / WHEEL_TRACK
        self.theta += self.w * dt
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt


# ---------------------------------------------------------------------------
# LowLevel (Pico) board
# ---------------------------------------------------------------------------

LL_STATUS, LL_IMU, LL_UI_EVENT = 0x01, 0x02, 0x03
LL_CONFIG_REQ, LL_CONFIG_RSP = 0x11, 0x12
LL_HEARTBEAT, LL_HL_STATE = 0x42, 0x43
BIT_LATCH, BIT_STOP, BIT_LIFT = 1 << 0, 1 << 1, 1 << 2
ST_INITIALIZED, ST_RASPI_POWER, ST_CHARGING, ST_RAIN, ST_UI_AVAIL = 1, 2, 4, 16, 128

HEARTBEAT_MILLIS = 500
STATUS_CYCLETIME = 100
IMU_CYCLETIME = 20
BUTTON_EMERGENCY_MILLIS = 20
CHARGING_RETRY_MILLIS = 10000

OPT_OFF, OPT_ON, OPT_UNDEFINED = 0, 1, 2
HALL_OFF, HALL_LIFT_TILT, HALL_STOP, HALL_UNDEFINED = 0, 1, 2, 3
CONFIG_FMT = '<HHfffffHHB2sB10s'  # ll_high_level_config, 42 bytes
assert struct.calcsize(CONFIG_FMT) == 42


def board_default_config():
    """The v1-fw board's compiled defaults (datatypes.h ll_high_level_config)."""
    return {
        'dfp_is_5v': OPT_OFF, 'background_sounds': OPT_OFF, 'ignore_charging_current': OPT_OFF,
        'rain_threshold': 700,
        'v_charge_cutoff': 30.0, 'i_charge_cutoff': 1.5, 'v_battery_cutoff': 29.0,
        'v_battery_empty': 21.7 + 0.3, 'v_battery_full': 28.7 - 0.3,
        'lift_period': 100, 'tilt_period': 2500, 'shutdown_esc_max_pitch': 0,
        'language': b'en', 'volume': 80,
        # [0..1] OM Hall-1/2 = lift, [2..3] OM Hall-3/4 = stop (YF-C500 default),
        # [4..9] CoverUI inputs: not initialised -> UNDEFINED, i.e. unused.
        'halls': [(HALL_LIFT_TILT, True), (HALL_LIFT_TILT, True),
                  (HALL_STOP, True), (HALL_STOP, True)] + [(HALL_UNDEFINED, False)] * 6,
    }


def unpack_config(raw: bytes, base: dict) -> dict:
    """Flexible-length receive: bytes the sender did not send keep `base`."""
    full = bytearray(pack_config(base))
    full[:min(len(raw), 42)] = raw[:42]
    (opts, rain, vcc, icc, vbc, vbe, vbf, lift, tilt, pitch, lang, vol, halls) = \
        struct.unpack(CONFIG_FMT, bytes(full))
    return {
        'dfp_is_5v': opts & 3, 'background_sounds': (opts >> 2) & 3,
        'ignore_charging_current': (opts >> 4) & 3,
        'rain_threshold': rain, 'v_charge_cutoff': vcc, 'i_charge_cutoff': icc,
        'v_battery_cutoff': vbc, 'v_battery_empty': vbe, 'v_battery_full': vbf,
        'lift_period': lift, 'tilt_period': tilt, 'shutdown_esc_max_pitch': pitch,
        'language': lang, 'volume': vol,
        'halls': [(h & 7, bool(h & 8)) for h in halls],
    }


def pack_config(c: dict) -> bytes:
    opts = (c['dfp_is_5v'] | (c['background_sounds'] << 2) | (c['ignore_charging_current'] << 4)
            | (OPT_UNDEFINED << 6) | (OPT_UNDEFINED << 8) | (OPT_UNDEFINED << 10)
            | (OPT_UNDEFINED << 12) | (OPT_UNDEFINED << 14))
    halls = bytes((m & 7) | (8 if al else 0) for m, al in c['halls'])
    return struct.pack(CONFIG_FMT, opts, c['rain_threshold'], c['v_charge_cutoff'],
                       c['i_charge_cutoff'], c['v_battery_cutoff'], c['v_battery_empty'],
                       c['v_battery_full'], c['lift_period'], c['tilt_period'],
                       c['shutdown_esc_max_pitch'], c['language'], c['volume'], halls)


def apply_config(rcv: dict) -> dict:
    """v1-fw applyConfig(): fresh defaults, take over only defined fields."""
    new = board_default_config()
    new['language'] = rcv['language']
    for k in ('dfp_is_5v', 'background_sounds', 'ignore_charging_current'):
        if rcv[k] != OPT_UNDEFINED:
            new[k] = rcv[k]
    if rcv['rain_threshold'] != 0xFFFF:
        new['rain_threshold'] = rcv['rain_threshold']
    if rcv['v_charge_cutoff'] >= 0:
        new['v_charge_cutoff'] = min(rcv['v_charge_cutoff'], 36.0)
    if rcv['i_charge_cutoff'] >= 0:
        new['i_charge_cutoff'] = min(rcv['i_charge_cutoff'], 5.0)
    for k in ('v_battery_cutoff', 'v_battery_empty', 'v_battery_full'):
        if rcv[k] >= 0:
            new[k] = rcv[k]
    for k in ('lift_period', 'tilt_period'):
        if rcv[k] != 0xFFFF:
            new[k] = rcv[k]
    if rcv['shutdown_esc_max_pitch'] != 0xFF:
        new['shutdown_esc_max_pitch'] = rcv['shutdown_esc_max_pitch']
    if rcv['volume'] != 0xFF:
        new['volume'] = rcv['volume']
    new['halls'] = [r if r[0] != HALL_UNDEFINED else d for r, d in zip(rcv['halls'], new['halls'])]
    return new


class LowLevelBoard:
    def __init__(self, port: PtyPort, plant: Plant, generation: str = 'v1-fw'):
        assert generation in ('v1-fw', 'v0.13')
        self.port, self.plant, self.generation = port, plant, generation
        self.reader = CobsReader()
        self.config = board_default_config()   # what is live (and in flash)
        self.flash_writes = 0
        self.config_requests = []              # (t_ms, received config dict)
        self.heartbeats = collections.deque(maxlen=4000)  # (t_ms, request, release)
        self.hl_states = collections.deque(maxlen=500)    # (t_ms, mode, gps_quality)
        self.latch = True                      # boots latched
        self.latch_events = collections.deque(maxlen=200)  # t_ms of each latch
        self.last_heartbeat_ms = -1e12
        self.ros_running = False
        self.emergency_bitmask = BIT_LATCH
        self.stop_pressed = False
        self.lifted_wheels = 0
        self._btn_started = None
        self._lift_started = None
        self._tilt_started = None
        self.v_charge = 0.2
        self.v_battery = 27.0
        self.charging_allowed = False
        self.charge_current_offset = 0.0  # the real sensor reads ~0.05 A of nothing
        self._charging_disabled_ms = -1e12
        self.gyro_bias_z = 0.0
        self.paused = False                    # cable unplugged: no bytes either way
        self._last_status_ms = 0.0
        self._last_imu_ms = 0.0
        self.status_sent = 0

    # --- physical stimuli -------------------------------------------------
    def hall_asserted(self, i: int) -> bool:
        if i == 0:
            return self.lifted_wheels >= 1
        if i == 1:
            return self.lifted_wheels >= 2
        if i in (2, 3):
            return self.stop_pressed
        return False

    def charging_current(self) -> float:
        docked = self.v_charge >= 3.0
        return (1.0 if (docked and self.charging_allowed) else 0.0) + self.charge_current_offset

    # --- firmware loop ----------------------------------------------------
    def service(self, now: float):
        latched_before = self.latch
        data = self.port.read()
        if not self.paused:
            for body in self.reader.feed(data):
                self._on_frame(body, now)
        self._update_emergency(now)
        if self.latch and not latched_before:
            self.latch_events.append(now)  # when the board latched, whatever the cause
        self._update_charging(now)
        if self.paused:
            return
        if now - self._last_imu_ms > IMU_CYCLETIME:
            self._send_imu(now)
        if now - self._last_status_ms > STATUS_CYCLETIME:
            self._send_status()
            self._last_status_ms = now

    def _on_frame(self, body: bytes, now: float):
        t = body[0]
        if t == LL_HEARTBEAT and len(body) == 3:
            self.last_heartbeat_ms = now
            req, rel = body[1], body[2]
            self.heartbeats.append((now, bool(req), bool(rel)))
            if rel:
                self.latch = False
            if req:
                self.latch = True
            self.ros_running = True
        elif t == LL_HL_STATE and len(body) == 3:
            self.hl_states.append((now, body[1], body[2]))
        elif t in (LL_CONFIG_REQ, LL_CONFIG_RSP):
            rcv = unpack_config(body[1:], board_default_config())
            self.config_requests.append((now, rcv))
            self.config = apply_config(rcv)
            if t == LL_CONFIG_REQ:
                self.port.write(cobs_packet(bytes([LL_CONFIG_RSP]) + pack_config(self.config)))
            self.flash_writes += 1

    def _update_emergency(self, now: float):
        if now - self.last_heartbeat_ms > HEARTBEAT_MILLIS:
            self.latch = True
            self.ros_running = False
        last = self.emergency_bitmask & BIT_LATCH
        stop, lifted = False, 0
        for i, (mode, _active_low) in enumerate(self.config['halls'][:4]):
            if not self.hall_asserted(i):
                continue
            if mode == HALL_STOP:
                stop = True
            elif mode == HALL_LIFT_TILT:
                lifted += 1
        state = 0
        if stop:
            if self._btn_started is None:
                self._btn_started = now
            elif now - self._btn_started >= BUTTON_EMERGENCY_MILLIS:
                state |= BIT_STOP
        else:
            self._btn_started = None
        if lifted >= 2:
            self._lift_started = self._lift_started if self._lift_started is not None else now
        else:
            self._lift_started = None
        if lifted >= 1:
            self._tilt_started = self._tilt_started if self._tilt_started is not None else now
        else:
            self._tilt_started = None
        lp, tp = self.config['lift_period'], self.config['tilt_period']
        if ((lp > 0 and self._lift_started is not None and now - self._lift_started >= lp)
                or (tp > 0 and self._tilt_started is not None and now - self._tilt_started >= tp)):
            state |= BIT_LIFT
        if state or self.latch:
            self.latch = True
            state |= BIT_LATCH
        self.emergency_bitmask = state
        if last != (state & BIT_LATCH) and not self.paused:
            self._send_status()  # "a new emergency is sent instantly"

    def _update_charging(self, now: float):
        if self.generation == 'v1-fw':
            if self.v_charge < 3.0:
                self.charging_allowed = False  # "Always enable for regen when not docked"
                return
            should = (self.v_charge < self.config['v_charge_cutoff']
                      and self.charging_current() < self.config['i_charge_cutoff']
                      and self.v_battery < self.config['v_battery_cutoff'])
        else:  # v0.13.x: hard-coded limits, no "off the dock" early return
            should = self.v_charge < 30.0 and self.charging_current() < 1.5 and self.v_battery < 29.0
        if self.charging_allowed:
            if not should:
                self.charging_allowed = False
                self._charging_disabled_ms = now
        elif now - self._charging_disabled_ms > CHARGING_RETRY_MILLIS:
            if should:
                self.charging_allowed = True
            else:
                self._charging_disabled_ms = now

    def _send_status(self):
        bits = ST_INITIALIZED | ST_RASPI_POWER | ST_UI_AVAIL
        if self.charging_allowed:
            bits |= ST_CHARGING
        empty, full = self.config['v_battery_empty'], self.config['v_battery_full']
        pct = int(max(0.0, min(100.0, (self.v_battery - empty) / max(full - empty, 0.1) * 100)))
        payload = struct.pack('<BB5fBfffB', LL_STATUS, bits, 0, 0, 0, 0, 0,
                              self.emergency_bitmask, self.v_charge, self.v_battery,
                              self.charging_current(), pct)
        self.port.write(cobs_packet(payload))
        self.status_sent += 1

    def _send_imu(self, now: float):
        dt_ms = int(now - self._last_imu_ms) if self._last_imu_ms else IMU_CYCLETIME
        self._last_imu_ms = now
        n = random.gauss
        accel = (n(0, 0.02), n(0, 0.02), 9.81 + n(0, 0.02))
        gyro = (n(0, 0.002), n(0, 0.002), self.plant.w + self.gyro_bias_z + n(0, 0.002))
        mag = (20.0, 0.0, -40.0)
        payload = struct.pack('<BH9f', LL_IMU, min(dt_ms, 0xFFFF), *accel, *gyro, *mag)
        self.port.write(cobs_packet(payload))

    def send_ui_event(self, button_id: int, press_duration: int):
        self.port.write(cobs_packet(struct.pack('<BBB', LL_UI_EVENT, button_id, press_duration)))


# ---------------------------------------------------------------------------
# Motor controllers
# ---------------------------------------------------------------------------


class _Esc:
    def __init__(self, port: PtyPort, motor):
        self.port, self.motor = port, motor
        self.paused = False          # TX wire cut: the controller still runs
        self.commands = collections.deque(maxlen=20000)  # (t_ms, duty)
        self.last_command_ms = -1e12

    def effective_duty(self, now: float) -> float:
        raise NotImplementedError

    def _record(self, now: float, duty: float):
        self.motor.duty = duty
        self.last_command_ms = now
        self.commands.append((now, duty))


class Xesc2040(_Esc):
    STATUS_UPDATE_MILLIS = 20
    WATCHDOG_TIMEOUT_MILLIS = 500
    FAULT_UNINITIALIZED, FAULT_WATCHDOG = 1, 2

    def __init__(self, port: PtyPort, motor):
        super().__init__(port, motor)
        self.reader = CobsReader()
        self.settings_received = False
        self.settings_count = 0
        self.seq = 0
        self.tacho_abs = 0
        self.direction = False
        self._last_tick_int = 0
        self._last_status_ms = 0.0

    def faults(self, now: float) -> int:
        f = 0
        if not self.settings_received:
            f |= self.FAULT_UNINITIALIZED
        if now - self.last_command_ms > self.WATCHDOG_TIMEOUT_MILLIS:
            f |= self.FAULT_WATCHDOG
        return f

    def effective_duty(self, now: float) -> float:
        return 0.0 if self.faults(now) else self.motor.duty

    def reboot(self):
        """Brown-out: settings and counters are gone."""
        self.settings_received = False
        self.motor.ticks = 0.0
        self.tacho_abs = 0
        self._last_tick_int = 0

    def receive(self, now: float):
        for body in self.reader.feed(self.port.read()):
            if body[0] == 2 and len(body) == 9:
                self._record(now, struct.unpack('<d', body[1:9])[0])
            elif body[0] == 3 and len(body) == 34:
                self.settings_received = True
                self.settings_count += 1

    def transmit(self, now: float):
        tick = int(self.motor.ticks)
        diff = tick - self._last_tick_int
        if diff:
            self.tacho_abs = (self.tacho_abs + abs(diff)) & 0xFFFFFFFF
            self.direction = diff < 0
            self._last_tick_int = tick
        if not self.paused and now - self._last_status_ms > self.STATUS_UPDATE_MILLIS:
            self._last_status_ms = now
            self.seq = (self.seq + 1) & 0xFFFFFFFF
            duty = self.effective_duty(now)
            payload = struct.pack('<BIBBdddddIIBi', 1, self.seq, 1, 4, 26.5, 35.0, 30.0,
                                  abs(duty) * 1.5, duty, tick & 0xFFFFFFFF, self.tacho_abs,
                                  1 if self.direction else 0, self.faults(now))
            self.port.write(cobs_packet(payload))


class XescMini(_Esc):
    TIMEOUT_MSEC = 1000  # APPCONF_TIMEOUT_MSEC, brake current 0 -> coast

    def __init__(self, port: PtyPort, motor):
        super().__init__(port, motor)
        self.reader = VescReader()
        self._pending_values = 0
        self._ticks_at_receive = 0.0

    def effective_duty(self, now: float) -> float:
        return self.motor.duty if now - self.last_command_ms <= self.TIMEOUT_MSEC else 0.0

    def reboot(self):
        self.motor.ticks = 0.0
        self.last_command_ms = -1e12

    def receive(self, now: float):
        self._ticks_at_receive = self.motor.ticks
        for p in self.reader.feed(self.port.read()):
            if p[0] == 5 and len(p) >= 5:
                self._record(now, struct.unpack('>i', p[1:5])[0] / 100000.0)
            if self.paused:
                continue
            if p[0] == 0:
                self.port.write(vesc_frame(bytes([0, 5, 3]) + b'xESC-mini\x00'))
            elif p[0] == 4:
                self._pending_values += 1

    def transmit(self, now: float):
        # Requests that queued up while this thread was starved arrived spread
        # over the stall; the real controller answers each one at once, so
        # answer the k-th of n with the count reached k/n of the way through
        # the elapsed time, not all of them with the count from its end.
        n, self._pending_values = self._pending_values, 0
        before, after = self._ticks_at_receive, self.motor.ticks
        for k in range(1, n + 1):
            self.port.write(vesc_frame(self._values(now, int(before + (after - before) * k / n))))

    def _values(self, now: float, tacho: int) -> bytes:
        duty = self.effective_duty(now)
        speed = getattr(self.motor, 'speed', 0.0)
        p = bytearray(60)
        p[0] = 4
        struct.pack_into('>h', p, 1, 350)                               # temp_mos 35.0
        struct.pack_into('>h', p, 3, 300)                               # temp_motor 30.0
        struct.pack_into('>i', p, 5, int(abs(duty) * 150))              # current_motor
        struct.pack_into('>i', p, 9, int(abs(duty) * 150))              # current_in
        struct.pack_into('>h', p, 21, int(duty * 1000))                 # duty_now
        erpm = int(speed * 2000) if hasattr(self.motor, 'speed') else int(self.motor.rpm * BladeMotor.POLE_PAIRS)
        struct.pack_into('>i', p, 23, erpm)
        struct.pack_into('>h', p, 27, 265)                              # v_in 26.5
        struct.pack_into('>i', p, 45, tacho)                            # tacho (signed)
        struct.pack_into('>i', p, 49, abs(tacho))                       # tacho_abs
        p[53] = 0                                                       # fault
        return bytes(p)


# ---------------------------------------------------------------------------
# The rig
# ---------------------------------------------------------------------------


class OpenMowerV1Rig:
    """LowLevel board + three xESC + plant, serviced by one 500 Hz thread."""

    PERIOD_S = 0.002

    def __init__(self, workdir: str, esc_type: str, ll_generation: str = 'v1-fw'):
        assert esc_type in ('xesc_mini', 'xesc_2040')
        self.esc_type = esc_type
        self.paths = {n: os.path.join(workdir, n) for n in ('ll', 'left', 'right', 'mow')}
        self.ports = {n: PtyPort(p) for n, p in self.paths.items()}
        self.plant = Plant()
        self.motors = {'left': DriveMotor(), 'right': DriveMotor(), 'mow': BladeMotor()}
        self.ll = LowLevelBoard(self.ports['ll'], self.plant, ll_generation)
        cls = Xesc2040 if esc_type == 'xesc_2040' else XescMini
        self.escs = {n: cls(self.ports[n], self.motors[n]) for n in ('left', 'right', 'mow')}
        self.lock = threading.RLock()
        self._t0 = time.monotonic()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name='om-v1-rig', daemon=True)

    def now_ms(self) -> float:
        return (time.monotonic() - self._t0) * 1000.0

    def start(self):
        self._thread.start()

    def close(self):
        self._stop.set()
        self._thread.join(timeout=2)
        for p in self.ports.values():
            p.close()

    def _run(self):
        last = time.monotonic()
        while not self._stop.is_set():
            time.sleep(self.PERIOD_S)
            t = time.monotonic()
            elapsed = t - last
            last = t
            now = (t - self._t0) * 1000.0
            with self.lock:
                for esc in self.escs.values():
                    esc.receive(now)
                self.ll.service(now)
                # Integrate ALL the elapsed wall time, in small substeps. A
                # starved thread (a 2-vCPU CI runner, the GIL shared with the
                # ROS probe) used to cap the step at 50 ms and silently drop
                # the rest: the simulated robot then moved slower than wall
                # time, and every speed check compared two different clocks.
                steps = max(1, math.ceil(elapsed / self.PERIOD_S))
                dt = elapsed / steps
                duty = {n: self.escs[n].effective_duty(now) for n in ('left', 'right', 'mow')}
                for _ in range(steps):
                    for name in ('left', 'right', 'mow'):
                        self.motors[name].step(dt, duty[name])
                    # Real wiring: right wheel forward on a NEGATIVE controller duty.
                    self.plant.step(dt, self.motors['left'].speed, -self.motors['right'].speed)
                # Report AFTER integrating: a status sent before it would carry
                # the counts from before a stall, and the next one the whole
                # stall's ticks at a normal 20 ms spacing — a jump no real
                # controller produces, which the bridge rightly drops as a
                # counter reset (and the speed loop then overshoots).
                for esc in self.escs.values():
                    esc.transmit(now)

    # --- observation helpers (thread-safe) --------------------------------
    def snapshot(self) -> dict:
        with self.lock:
            now = self.now_ms()
            return {
                't': now,
                'x': self.plant.x, 'y': self.plant.y, 'theta': self.plant.theta,
                'v': self.plant.v, 'w': self.plant.w,
                'duty': {n: e.motor.duty for n, e in self.escs.items()},
                'effective': {n: e.effective_duty(now) for n, e in self.escs.items()},
                'blade_rpm': self.motors['mow'].rpm,
                'latch': self.ll.latch, 'emergency_bitmask': self.ll.emergency_bitmask,
            }

    def first_time_all_zero(self, names, since_ms: float):
        """Earliest t >= since where every named controller's latest command is 0."""
        with self.lock:
            events = sorted((t, n, d) for n in names for (t, d) in self.escs[n].commands
                            if t >= since_ms - 1000)
        latest = {}
        for t, n, d in events:
            latest[n] = d
            if t >= since_ms and len(latest) == len(names) and all(abs(v) < 1e-9 for v in latest.values()):
                return t
        return None

    def first_zero_command(self, name: str, since_ms: float, timeout_ms: float):
        """When the controller RECEIVED its first zero duty since a moment.

        Read from the command log, so a caller that only starts looking after
        a blocking service call returns still gets the arrival time, not the
        moment it happened to look."""
        deadline = since_ms + timeout_ms
        while True:
            with self.lock:
                hit = next((t for (t, d) in self.escs[name].commands
                            if t >= since_ms and abs(d) < 1e-9), None)
            if hit is not None or self.now_ms() >= deadline:
                return hit
            time.sleep(0.002)

    def first_time_effective_zero(self, names, since_ms: float, timeout_ms: float):
        """Poll the controllers' OWN view (watchdogs included) until all are 0."""
        deadline = since_ms + timeout_ms
        while self.now_ms() < deadline:
            with self.lock:
                now = self.now_ms()
                if all(abs(self.escs[n].effective_duty(now)) < 1e-9 for n in names):
                    return now
            time.sleep(0.002)
        return None
