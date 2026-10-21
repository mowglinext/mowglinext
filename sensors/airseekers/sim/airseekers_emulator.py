#!/usr/bin/env python3
# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
"""Emulated Airseekers Tron electronics for the mowgli_airseekers_bridge simulation.

Pseudo-terminals stand in for the RK3588 UARTs: the vendor MCU
(``/dev/serial_mower``), the WIT JY61P IMU (``/dev/serial_imu``) and the UM960
(``/dev/serial_rtk``); a FIFO stands in for the panel-key evdev device. A
differential-drive plant (sim/kinematics.py, the Tron model the port used)
turns the SpeedData the MCU accepts into motion, measured speed, IMU rates and
a GNSS position.

The MCU rules below come from observing the stock vendor system on a Tron, NOT
from the bridge, so a wrong rule in the bridge cannot be mirrored here.
Each rule is tagged: [measured] on the wire / on the robot, [decompile] read
from the vendor firmware, [ASSUMED] not characterised yet (README "Open
measurements"):

  * MCU -> host: SpeedData + SensorInfo at ~100 Hz; battery, versions, BMS
    version and motors batched once per second [measured];
  * the host heartbeat (type 255) arrives every 100 ms from the vendor host
    [measured];
  * SpeedData sets the drive setpoint, which is HELD while host traffic
    continues -- silence does not stop the wheels [decompile];
  * SpeedData is ignored while ``lift`` or ``stop`` is set, and those flags
    zero the setpoint and the cutter [decompile];
  * after ``host_timeout_ms`` without host traffic (SpeedData or heartbeat)
    the setpoint is zeroed and the cutter stopped [decompile: "about 200
    ticks"; tick period ASSUMED 10 ms -> 2000 ms default; the real value is
    the heartbeat-failsafe measurement still open];
  * ``lift`` is asserted while the forwarded IMU attitude is outside pitch
    [-36.8, +40.1] deg / |roll| <= 38.5 deg, while the lift halls read lifted,
    and while no ImuData has been forwarded recently (without the stream the
    MCU reports lifted) [measured + decompile; the "recently" window of 500 ms
    is ASSUMED]; after ``lift_latch_s`` (~17 s) it stays latched until an
    all-zero SensorInfoControl arrives while the trigger is gone [decompile,
    clear frame measured];
  * the stop button is reported as ``stop`` while pressed [measured];
  * any SpeedData (zeros included) takes the chassis out of its position
    hold; the emulator counts frames that arrive while docked [measured on the
    dock: creep].

No ROS import: this module only speaks bytes.
"""

from __future__ import annotations

import math
import os
import struct
import threading
import time
import tty
from typing import Dict, List, Optional, Tuple

import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..',
                                'mowgli_airseekers_bridge'))

from kinematics import DriveParams, Plant  # noqa: E402
from mowgli_airseekers_bridge import protocol as P  # noqa: E402

WIT_SYNC = 0x55


def wit_frame(ftype: int, values) -> bytes:
    body = bytes([WIT_SYNC, ftype]) + struct.pack('<4h', *[max(-32768, min(32767, int(v)))
                                                           for v in values])
    return body + bytes([sum(body) & 0xFF])


def nmea(body: str) -> bytes:
    cs = 0
    for c in body:
        cs ^= ord(c)
    return ('$%s*%02X\r\n' % (body, cs)).encode('ascii')


class PtyPort:
    """A UART: data the emulator writes appears on the slave the bridge opens."""

    def __init__(self, link_path: str):
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)
        os.set_blocking(self.master, False)
        self.link_path = link_path
        if os.path.lexists(link_path):
            os.unlink(link_path)
        os.symlink(os.ttyname(self.slave), link_path)

    def write(self, data: bytes) -> None:
        try:
            os.write(self.master, data)
        except OSError:
            pass

    def read(self) -> bytes:
        try:
            return os.read(self.master, 65536)
        except OSError:
            return b''

    def close(self) -> None:
        for fd in (self.master, self.slave):
            try:
                os.close(fd)
            except OSError:
                pass
        if os.path.islink(self.link_path):
            os.unlink(self.link_path)


class Mcu:
    """The vendor cutter (gateway) + chassis boards, as seen from the host UART."""

    LIFT_PITCH_MIN, LIFT_PITCH_MAX, LIFT_ROLL_MAX = -36.8, 40.1, 38.5

    def __init__(self, host_timeout_ms: float = 2000.0, imu_missing_ms: float = 500.0,
                 lift_latch_s: float = 17.0) -> None:
        self.host_timeout_ms = host_timeout_ms
        self.imu_missing_ms = imu_missing_ms
        self.lift_latch_s = lift_latch_s
        self.parser = P.FrameParser()
        self.setpoint = (0.0, 0.0)
        self.host_t = -1e9            # last host traffic (ms)
        self.imu_t = -1e9
        self.imu_rp = (0.0, 0.0)      # forwarded roll, pitch (deg)
        self.stop_button = False
        self.halls_lifted = False
        self.lift = False
        self.lift_since: Optional[float] = None
        self.lift_latched = False
        self.cutter_on = False
        self.cutter_speed = 0
        self.height_mm = 90
        self.charge_enabled = None
        self.docked = False
        self.battery_v = 24.0
        self.watchdog_fired_t: Optional[float] = None
        # observation log: (t_ms, kind, value)
        self.log: List[Tuple[float, str, object]] = []
        self.speed_frames_while_docked = 0

    # -- host -> MCU -------------------------------------------------------------
    def receive(self, data: bytes, now: float) -> None:
        for type_id, mod_id, payload in self.parser.feed(data):
            if type_id == P.TYPE_HEARTBEAT:
                self.host_t = now
                self.log.append((now, 'heartbeat', None))
                continue
            if type_id != P.TYPE_HOST_TO_MCU:
                continue
            if mod_id == P.MOD_SPEED:
                s = P.decode_speed(payload)
                if s is None:
                    continue
                self.host_t = now
                self.log.append((now, 'speed', (s.linear, s.angular)))
                if self.docked:
                    self.speed_frames_while_docked += 1
                if not (self.lift or self.stop_button):
                    self.setpoint = (s.linear, s.angular)
            elif mod_id == P.MOD_IMU:
                d = P.decode_imu(payload)
                if d is not None:
                    self.imu_t = now
                    self.imu_rp = (math.degrees(d.roll), math.degrees(d.pitch))
                    self.log.append((now, 'imu', self.imu_rp))
            elif mod_id == P.MOD_CUTTER:
                c = P.decode_cutter(payload)
                if c is not None:
                    en, _rev, speed, height = c
                    self.height_mm = height
                    self.cutter_on = en and not (self.lift or self.stop_button)
                    self.cutter_speed = speed if en else 0
                    self.log.append((now, 'cutter', (en, speed, height)))
            elif mod_id == P.MOD_SENSOR and payload == bytes(8):
                self.log.append((now, 'clear', None))
                if self.lift_latched and not self._lift_trigger(now):
                    self.lift_latched = False
            elif mod_id == P.MOD_CHARGE and len(payload) == 1:
                self.charge_enabled = payload[0] == 1
                self.log.append((now, 'charge', self.charge_enabled))

    # -- firmware tick -----------------------------------------------------------
    def _lift_trigger(self, now: float) -> bool:
        roll, pitch = self.imu_rp
        out_of_window = not (self.LIFT_PITCH_MIN <= pitch <= self.LIFT_PITCH_MAX
                             and abs(roll) <= self.LIFT_ROLL_MAX)
        imu_missing = now - self.imu_t > self.imu_missing_ms
        return out_of_window or imu_missing or self.halls_lifted

    def tick(self, now: float) -> None:
        trig = self._lift_trigger(now)
        if trig:
            if self.lift_since is None:
                self.lift_since = now
            if now - self.lift_since >= self.lift_latch_s * 1000.0:
                self.lift_latched = True
        else:
            self.lift_since = None
        self.lift = trig or self.lift_latched
        if self.lift or self.stop_button:
            self.setpoint = (0.0, 0.0)
            self.cutter_on = False
        if now - self.host_t > self.host_timeout_ms:
            if self.setpoint != (0.0, 0.0) or self.cutter_on:
                self.watchdog_fired_t = now
            self.setpoint = (0.0, 0.0)
            self.cutter_on = False

    # -- MCU -> host ------------------------------------------------------------
    def fast_frame(self, v: float, w: float) -> bytes:
        sensor = P.SensorInfo(False, False, self.lift, self.stop_button, False, False, 1, True,
                              False, False)
        return P.build_frame([(P.TYPE_MCU_TO_HOST, P.MOD_SPEED, P.speed_payload(v, w)),
                              (P.TYPE_MCU_TO_HOST, P.MOD_SENSOR, P.encode_sensor(sensor))])

    def slow_frame(self, blade_rpm: int) -> bytes:
        current = -41 if (self.docked and self.charge_enabled is not False) else 7
        batt = P.BatteryInfo(self.battery_v, current * 0.1, 80, self.docked, 25, 0)
        motors = {'cutter': (blade_rpm, 150 if blade_rpm else 0, 2400, 30, 1 if blade_rpm else 0),
                  'left': (0, 300, 15700, 32, 1), 'right': (0, -200, 15700, 33, 1),
                  'height': (0, 0, 0, 0, 0)}
        return P.build_frame([
            (P.TYPE_MCU_TO_HOST, P.MOD_BATTERY, P.encode_battery(batt)),
            (P.TYPE_MCU_TO_HOST, P.MOD_VERSION, struct.pack(P.VERSION_FMT, 0, 6, 36, 0, 6, 34,
                                                             0, 0, 0)),
            (P.TYPE_MCU_TO_HOST, P.MOD_BMS_VERSION, struct.pack(P.BMS_FMT, 1, 18, 0)),
            (P.TYPE_MCU_TO_HOST, P.MOD_MOTORS, P.encode_motors(motors)),
        ])

    # -- queries (tests) ---------------------------------------------------------
    def events(self, kind: str, since: float = -1e18):
        return [(t, v) for (t, k, v) in self.log if k == kind and t >= since]


class DockPads:
    """The charge contacts of the Tron's REVERSE-entry dock.

    The robot is seated at a pose facing out of the dock. The contacts (the
    MCU's ``dock_ok``) stay closed until it has driven ``pad_m`` FORWARD off
    them; the dock body behind it is a hard stop, so reversing only pushes into
    it (counted in ``pushes``) [ASSUMED pad length; the geometry is not
    measured]. Once off the pads the dock stays left.
    """

    def __init__(self, x: float, y: float, yaw: float, pad_m: float = 0.10) -> None:
        self.x, self.y, self.yaw, self.pad_m = x, y, yaw, pad_m
        self.seated = True
        self.pushes = 0

    def along(self, plant: Plant) -> float:
        """Signed distance travelled out of the dock along the seated heading."""
        return (plant.x - self.x) * math.cos(self.yaw) + (plant.y - self.y) * math.sin(self.yaw)

    def step(self, plant: Plant) -> bool:
        if not self.seated:
            return False
        a = self.along(plant)
        if a < 0.0:
            plant.x -= a * math.cos(self.yaw)
            plant.y -= a * math.sin(self.yaw)
            self.pushes += 1
        elif a > self.pad_m:
            self.seated = False
        return self.seated


class AirseekersRig:
    """MCU + IMU + GNSS + keys + plant, serviced by one 500 Hz thread."""

    PERIOD_S = 0.002

    def __init__(self, workdir: str, host_timeout_ms: float = 2000.0, gnss: bool = True):
        self.paths = {n: os.path.join(workdir, n) for n in ('mcu', 'imu', 'gnss')}
        self.ports = {'mcu': PtyPort(self.paths['mcu']), 'imu': PtyPort(self.paths['imu'])}
        if gnss:
            self.ports['gnss'] = PtyPort(self.paths['gnss'])
        self.paths['keys'] = os.path.join(workdir, 'keys')
        if os.path.exists(self.paths['keys']):
            os.unlink(self.paths['keys'])
        os.mkfifo(self.paths['keys'])
        self._keys_fd: Optional[int] = None
        self.mcu = Mcu(host_timeout_ms=host_timeout_ms)
        self.plant = Plant(DriveParams())
        self.extra_roll_deg = 0.0      # injected chassis attitude (lift / tilt tests)
        self.blade_rpm = 0.0
        self.mcu_silent = False        # simulate a dead MCU link
        self.dock: Optional[DockPads] = None   # seat_on_dock(); None = no dock model
        self.lock = threading.RLock()
        self._t0 = time.monotonic()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name='airseekers-rig', daemon=True)
        self.lat0, self.lon0 = 48.1173, 11.5167

    def now_ms(self) -> float:
        return (time.monotonic() - self._t0) * 1000.0

    def start(self) -> None:
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2)
        for p in self.ports.values():
            p.close()
        if self._keys_fd is not None:
            os.close(self._keys_fd)
        if os.path.exists(self.paths['keys']):
            os.unlink(self.paths['keys'])

    def press_key(self, code: int) -> None:
        """Write an EV_KEY press + release to the keys FIFO (struct input_event)."""
        if self._keys_fd is None:
            # O_RDWR: opening a FIFO never blocks or fails with ENXIO this way.
            self._keys_fd = os.open(self.paths['keys'], os.O_RDWR | os.O_NONBLOCK)
        t = time.time()
        sec, usec = int(t), int((t % 1) * 1e6)
        data = (struct.pack('llHHi', sec, usec, 1, code, 1) + struct.pack('llHHi', sec, usec, 0, 0, 0)
                + struct.pack('llHHi', sec, usec, 1, code, 0))
        os.write(self._keys_fd, data)

    def _run(self) -> None:
        last = time.monotonic()
        next_fast = next_slow = next_imu = next_gnss = 0.0
        while not self._stop.is_set():
            time.sleep(self.PERIOD_S)
            t = time.monotonic()
            elapsed, last = t - last, t
            now = (t - self._t0) * 1000.0
            with self.lock:
                self.mcu.receive(self.ports['mcu'].read(), now)
                self.ports['imu'].read()
                if 'gnss' in self.ports:
                    self.ports['gnss'].read()
                self.mcu.tick(now)
                steps = max(1, math.ceil(elapsed / self.PERIOD_S))
                for _ in range(steps):
                    self.plant.step(elapsed / steps, *self.mcu.setpoint)
                    if self.dock is not None and self.dock.seated:
                        self.mcu.docked = self.dock.step(self.plant)
                target_rpm = 3200.0 if self.mcu.cutter_on else 0.0
                self.blade_rpm += (target_rpm - self.blade_rpm) * min(1.0, elapsed / 0.3)
                if now >= next_fast and not self.mcu_silent:
                    next_fast = now + 10.0
                    self.ports['mcu'].write(self.mcu.fast_frame(self.plant.v, self.plant.w))
                if now >= next_slow and not self.mcu_silent:
                    next_slow = now + 1000.0
                    self.ports['mcu'].write(self.mcu.slow_frame(int(self.blade_rpm)))
                if now >= next_imu:
                    next_imu = now + 10.0
                    self._send_imu()
                if 'gnss' in self.ports and now >= next_gnss:
                    next_gnss = now + 200.0
                    self._send_gnss()

    def _send_imu(self) -> None:
        p = self.plant
        roll = math.radians(self.extra_roll_deg)
        g = 9.80665
        acc = (0.0, g * math.sin(roll), g * math.cos(roll))
        acc_raw = [a / (16.0 * g) * 32768.0 for a in acc] + [2500]
        gyro_raw = [0.0, 0.0, p.w / math.radians(2000.0) * 32768.0, 2500]
        ang_raw = [math.degrees(roll) / 180.0 * 32768.0, 0.0,
                   math.degrees(p.yaw) / 180.0 * 32768.0, 0]
        self.ports['imu'].write(wit_frame(0x51, acc_raw) + wit_frame(0x52, gyro_raw)
                                + wit_frame(0x53, ang_raw))

    def _send_gnss(self) -> None:
        lat = self.lat0 + self.plant.y / 111320.0
        lon = self.lon0 + self.plant.x / (111320.0 * math.cos(math.radians(self.lat0)))

        def dm(v, deg_digits):
            d = int(abs(v))
            return '%0*d%08.5f' % (deg_digits, d, (abs(v) - d) * 60.0)
        body = 'GNGGA,120000.00,%s,%s,%s,%s,4,20,0.7,500.0,M,47.0,M,1.0,0001' % (
            dm(lat, 2), 'N' if lat >= 0 else 'S', dm(lon, 3), 'E' if lon >= 0 else 'W')
        self.ports['gnss'].write(nmea(body))

    def seat_on_dock(self, pad_m: float = 0.10) -> None:
        """Seat the robot (at rest, where it stands) on a reverse-entry dock."""
        with self.lock:
            p = self.plant
            self.dock = DockPads(p.x, p.y, p.yaw, pad_m)
            self.mcu.docked = True

    # -- observation -------------------------------------------------------------
    def snapshot(self) -> Dict[str, object]:
        with self.lock:
            return {'t': self.now_ms(), 'x': self.plant.x, 'y': self.plant.y,
                    'yaw': self.plant.yaw, 'v': self.plant.v, 'w': self.plant.w,
                    'setpoint': self.mcu.setpoint, 'lift': self.mcu.lift,
                    'cutter_on': self.mcu.cutter_on, 'height_mm': self.mcu.height_mm,
                    'blade_rpm': self.blade_rpm,
                    'dock_along': self.dock.along(self.plant) if self.dock else None,
                    'dock_pushes': self.dock.pushes if self.dock else 0}

    def events(self, kind: str, since: float = -1e18):
        with self.lock:
            return self.mcu.events(kind, since)
