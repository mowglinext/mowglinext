# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
"""Causal plant integration across a delayed emulator wake (no ROS/hardware)."""

import struct
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from measurements import MotionHistory
from openmower_v1_emulator import (
    BladeMotor, DriveMotor, OpenMowerV1Rig, Plant, VescReader, Xesc2040, XescMini,
    cobs_packet, vesc_frame,
)


class Port:
    def __init__(self, data=b''):
        self.data = data
        self.written = []

    def read(self):
        data, self.data = self.data, b''
        return data

    def write(self, data):
        self.written.append(data)


class CausalIntegrationTest(unittest.TestCase):
    def rig(self, esc_type, start, speed=.30, next_duty=None):
        # An already-recorded steady trajectory up to start. The only new
        # packet becomes available at the next actual receive pass, not before.
        rig = object.__new__(OpenMowerV1Rig)
        rig._t0 = 0.0
        rig.lock = threading.RLock()
        rig._stop = threading.Event()
        rig.clock_steps = []
        rig.plant = Plant()
        rig.plant.x = speed * start
        rig.plant.v = speed
        rig.distance = speed * start
        rig.motors = {'left': DriveMotor(), 'right': DriveMotor(), 'mow': BladeMotor()}
        rig.escs = {}
        cls = XescMini if esc_type == 'xesc_mini' else Xesc2040
        for name, motor in rig.motors.items():
            sign = -1 if name == 'right' else 1
            wheel_speed = sign * speed if name != 'mow' else 0
            if name != 'mow':
                motor.speed = wheel_speed
            motor.ticks = wheel_speed * start * 1600
            data = b''
            if next_duty is not None:
                duty = sign * next_duty if name != 'mow' else 0
                data = (vesc_frame(bytes([5]) + struct.pack('>i', round(duty * 100000)))
                        if esc_type == 'xesc_mini' else cobs_packet(struct.pack('<Bd', 2, duty)))
            esc = cls(Port(data), motor)
            if esc_type == 'xesc_2040':
                esc.settings_received = True
            esc._record(start * 1000, wheel_speed / DriveMotor.GAIN)
            rig.escs[name] = esc
        rig.ll = SimpleNamespace(service=lambda now: None, latch=False, emergency_bitmask=0)
        rig.motion = MotionHistory()
        rig.motion.append(0, 0, 0, (0, 0))
        rig.motion.append(start, rig.distance, 0, (int(speed * start * 1600), -int(speed * start * 1600)))
        return rig

    def wake(self, rig, start, end):
        def sample():
            rig._stop.set()  # exactly one real _run iteration
            return end, end + 1000, False
        rig.clock = SimpleNamespace(sample=sample)
        with patch('time.monotonic', return_value=start), patch('time.sleep'):
            rig._run()

    def test_late_zero_does_not_rewrite_the_previous_528_ms(self):
        # CI's reported Mini gap: 528 ms. Over a 2 s window, retroactive
        # braking removes ~0.123 m and yields ~0.238 m/s instead of 0.30.
        # For 2040, the unchanged 500 ms watchdog must still stop its final
        # 28 ms; it must not brake the preceding 500 ms retroactively.
        for esc in ('xesc_mini', 'xesc_2040'):
            with self.subTest(esc=esc):
                rig = self.rig(esc, 1.472, next_duty=0)
                self.wake(rig, 1.472, 2.0)
                measured = rig.distance / 2.0
                print(f'{esc}: 528 ms delayed zero, distance={rig.distance:.9f} m, '
                      f'2 s mean={measured:.9f} m/s')
                self.assertLess(abs(measured - .30), .03)  # original s03 tolerance
                self.assertEqual(rig.motors['left'].duty, 0)
                self.assertEqual(rig.motors['right'].duty, 0)
                self.assertEqual(rig.escs['left'].commands[-1], (2000, 0))
                # The newly received zero applies to the NEXT interval.
                previous = rig.distance
                rig._stop.clear()
                self.wake(rig, 2.0, 2.6)
                self.assertLess(rig.distance - previous, .3 * DriveMotor.TAU)
                self.assertLess(abs(rig.plant.v), .003)

    def test_a_late_start_cannot_invent_motion_before_reception(self):
        for esc in ('xesc_mini', 'xesc_2040'):
            with self.subTest(esc=esc):
                rig = self.rig(esc, 1.0, speed=0, next_duty=.4)
                self.wake(rig, 1.0, 1.5)
                self.assertEqual(rig.distance, 0)
                self.assertEqual(rig.plant.x, 0)
                self.assertEqual(rig.motors['left'].ticks, 0)
                self.assertGreater(rig.motors['left'].duty, 0)
                self.assertLess(rig.motors['right'].duty, 0)

    def test_watchdog_expiration_does_not_erase_motion_before_its_deadline(self):
        for esc, timeout in (('xesc_mini', 1.0), ('xesc_2040', .5)):
            with self.subTest(esc=esc):
                rig = self.rig(esc, 1.0)
                self.wake(rig, 1.0, 1.0 + timeout + .6)
                travelled = rig.distance - .3
                self.assertGreater(travelled, .3 * timeout)
                # Remaining travel is a physical coast, bounded by the
                # unchanged first-order motor time constant (+ one step).
                self.assertLess(travelled, .3 * (timeout + DriveMotor.TAU + rig.PERIOD_S))
                self.assertEqual(rig.escs['left'].effective_duty((1 + timeout + .6) * 1000), 0)
                self.assertLess(abs(rig.plant.v), .003)

    def test_snapshot_dates_recorded_motion_not_the_observers_later_clock(self):
        rig = self.rig('xesc_mini', 1.472)
        with patch('time.monotonic', return_value=2.0):
            snap = rig.snapshot()
        self.assertEqual(snap['t'], 1472)
        self.assertAlmostEqual(snap['x'], .3 * 1.472)

    def test_queued_mini_requests_report_actual_counts_without_interpolation(self):
        port = Port(vesc_frame(bytes([4])) * 2)
        motor = DriveMotor()
        motor.ticks = 100
        esc = XescMini(port, motor)
        esc.receive(1000)
        motor.ticks = 400
        esc.transmit(1500)
        replies = VescReader().feed(b''.join(port.written))
        counts = [struct.unpack_from('>i', body, 45)[0] for body in replies]
        # No UART arrival timestamps exist for the stalled interval. Inventing
        # one historical count per request masks that missing information.
        self.assertEqual(counts, [400, 400])


if __name__ == '__main__':
    unittest.main()
