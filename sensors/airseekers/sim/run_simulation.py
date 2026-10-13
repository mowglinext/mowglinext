#!/usr/bin/env python3
# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
"""Closed-loop simulation of mowgli_airseekers_bridge on emulated Tron electronics.

Launches the REAL bridge (and the UM960 GNSS node) through the production
launch file against airseekers_emulator.py, drives it over ROS the way
MowgliNext does, and checks what the emulated MCU actually received, timed on
the emulator's clock. The last scenario SIGKILLs the bridge mid-mow.

  python3 run_simulation.py [--only s02_round_trip ...] [--logdir DIR]

Needs a sourced workspace with mowgli_interfaces, um960_gps_driver and
mowgli_airseekers_bridge. Exit status 0 only when every check passes.
"""

from __future__ import annotations

import argparse
import math
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time
import traceback

import rclpy
from geometry_msgs.msg import TwistStamped
from mowgli_interfaces.msg import Emergency, GnssStatus, HighLevelStatus, Power, Status, WheelTick
from mowgli_interfaces.srv import EmergencyStop, HighLevelControl, MowerControl
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, Imu, NavSatFix
from std_msgs.msg import Bool, Int16
from std_srvs.srv import SetBool, Trigger

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from airseekers_emulator import AirseekersRig  # noqa: E402

IDLE, AUTONOMOUS = 1, 2
HERE = os.path.dirname(os.path.abspath(__file__))
ROBOT_TEMPLATE = os.path.normpath(os.path.join(
    HERE, '..', '..', '..', 'ros2', 'src', 'mowgli_bringup', 'config', 'mowgli_robot.yaml'))
BACKEND_DEFAULTS = os.path.join(os.path.dirname(ROBOT_TEMPLATE), 'backends', 'airseekers.yaml')
HOST_TIMEOUT_MS = 2000.0   # emulator's MCU host watchdog (ASSUMED, see the emulator)
KEY_S = 31                 # top-panel "go dock" key


class Report:
    def __init__(self):
        self.rows = []

    def check(self, scenario, name, ok, detail=''):
        self.rows.append((scenario, name, bool(ok), detail))
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f' — {detail}' if detail else ''),
              flush=True)
        return ok

    @property
    def failed(self):
        return [r for r in self.rows if not r[2]]


class Probe(Node):
    def __init__(self):
        super().__init__('as_sim_probe')
        self.lock = threading.Lock()
        self.last = {}
        self.counts = {}
        self.hlc_commands = []
        self.cmd = None
        self.hl_state = None
        rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', rel)
        self.hl_pub = self.create_publisher(HighLevelStatus,
                                            '/behavior_tree_node/high_level_status', rel)
        for topic, typ, key, qos in [
                ('/hardware_bridge/status', Status, 'status', rel),
                ('/hardware_bridge/emergency', Emergency, 'emergency', rel),
                ('/hardware_bridge/power', Power, 'power', rel),
                ('/battery_state', BatteryState, 'battery', rel),
                ('/wheel_odom', Odometry, 'odom', rel),
                ('/wheel_ticks', WheelTick, 'ticks', rel),
                ('/imu/data', Imu, 'imu', rel),
                ('/gps/fix', NavSatFix, 'fix', rel),
                ('/gps/status', GnssStatus, 'gnss', rel),
                ('/hardware_bridge/cmd_vel_applied', TwistStamped, 'applied', rel),
                ('/hardware_bridge/dig_escalated', Bool, 'dig', latched),
                ('/hardware_bridge/airseekers/cutter_height_mm', Int16, 'height', latched)]:
            self.create_subscription(typ, topic, self._keeper(key), qos)
        self.create_service(HighLevelControl, '/behavior_tree_node/high_level_control', self._hlc)
        self.estop_cli = self.create_client(EmergencyStop, '/hardware_bridge/emergency_stop')
        self.mower_cli = self.create_client(MowerControl, '/hardware_bridge/mower_control')
        self.reboot_cli = self.create_client(Trigger, '/hardware_bridge/reboot_board')
        self.debug_cli = self.create_client(SetBool, '/hardware_bridge/set_firmware_debug')
        self.dig_cli = self.create_client(Trigger, '/hardware_bridge/clear_dig_escalation')
        self.param_cli = self.create_client(SetParameters, '/hardware_bridge/set_parameters')
        self.create_timer(0.05, self._publish_cmd)
        self.create_timer(0.5, self._publish_hl)

    def _keeper(self, key):
        def cb(msg):
            with self.lock:
                self.last[key] = msg
                self.counts[key] = self.counts.get(key, 0) + 1
        return cb

    def _hlc(self, req, res):
        with self.lock:
            self.hlc_commands.append(req.command)
        res.success = True
        return res

    def _publish_cmd(self):
        if self.cmd is not None:
            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.twist.linear.x, m.twist.angular.z = self.cmd
            self.cmd_pub.publish(m)

    def _publish_hl(self):
        if self.hl_state is not None:
            m = HighLevelStatus()
            m.state = self.hl_state
            self.hl_pub.publish(m)

    def get(self, key):
        with self.lock:
            return self.last.get(key)

    def count(self, key):
        with self.lock:
            return self.counts.get(key, 0)

    def call(self, client, req, timeout=5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        fut = client.call_async(req)
        end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < end:
            time.sleep(0.01)
        return fut.result() if fut.done() else None


def wait_for(pred, timeout, period=0.02):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(period)
    return bool(pred())


def estop(probe, value):
    r = EmergencyStop.Request()
    r.emergency = value
    return probe.call(probe.estop_cli, r)


def mower(probe, enabled):
    r = MowerControl.Request()
    r.mow_enabled, r.mow_direction = enabled, 0
    return probe.call(probe.mower_cli, r)


def reset_emergency(probe):
    res = estop(probe, 0)
    return bool(res and res.success) and wait_for(
        lambda: probe.get('emergency') is not None and not probe.get('emergency').latched_emergency,
        2.0)


def speed_frames(rig, since):
    return rig.events('speed', since)


def start_bridge(rig, workdir, log_path, extra_yaml=''):
    cfg = os.path.join(workdir, 'mowgli_robot.yaml')
    with open(cfg, 'w', encoding='utf-8') as fh:
        # A fresh install's SPARSE file: the Tron defaults must come from the
        # template + backends/airseekers.yaml.
        fh.write('mowgli:\n  ros__parameters:\n    mowing_enabled: true\n'
                 f"    airseekers_mcu_port: \"{rig.paths['mcu']}\"\n"
                 f"    airseekers_imu_port: \"{rig.paths['imu']}\"\n"
                 f"    airseekers_keys_device: \"{rig.paths['keys']}\"\n"
                 f"    gnss_serial_device: \"{rig.paths['gnss']}\"\n" + extra_yaml)
    env = dict(os.environ, AIRSEEKERS_ROBOT_CONFIG=cfg, AIRSEEKERS_ROBOT_TEMPLATE=ROBOT_TEMPLATE,
               AIRSEEKERS_BACKEND_DEFAULTS=BACKEND_DEFAULTS)
    env.pop('AIRSEEKERS_MCU_PORT', None)
    env.pop('AIRSEEKERS_IMU_PORT', None)
    log = open(log_path, 'w', encoding='utf-8')
    return subprocess.Popen(['ros2', 'launch', 'mowgli_airseekers_bridge',
                             'airseekers_bridge.launch.py'],
                            env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)


def drive(probe, vx, wz, seconds):
    probe.hl_state = AUTONOMOUS
    probe.cmd = (vx, wz)
    time.sleep(seconds)


def stop_driving(probe, rig):
    probe.cmd = (0.0, 0.0)
    wait_for(lambda: abs(rig.snapshot()['v']) < 0.005 and abs(rig.snapshot()['w']) < 0.01, 3)
    probe.cmd = None


# ---------------------------------------------------------------------------
# Scenarios
# ---------------------------------------------------------------------------
def s01_boot(probe, rig, R):
    S = 'boot'
    ok = wait_for(lambda: probe.get('status') is not None
                  and probe.get('status').firmware_compatible, 20)
    st = probe.get('status')
    R.check(S, 'firmware_compatible (MCU stream + versions + IMU live)', ok,
            st.firmware_version if st else 'no Status')
    t0 = rig.now_ms()
    time.sleep(2.0)
    hb = rig.events('heartbeat', t0)
    imu = rig.events('imu', t0)
    R.check(S, 'heartbeat every 100 ms', 17 <= len(hb) <= 23, f'{len(hb)} in 2 s')
    R.check(S, 'IMU forwarded to the MCU at ~45 Hz', 70 <= len(imu) <= 100, f'{len(imu)} in 2 s')
    R.check(S, 'MCU does not report lift with the IMU forwarded', not rig.snapshot()['lift'])
    R.check(S, 'no SpeedData at rest (idle = silence)', not speed_frames(rig, 0))
    em = probe.get('emergency')
    R.check(S, 'emergency latched at start-up', em is not None and em.latched_emergency,
            em.reason if em else '')
    R.check(S, 'reset service clears it', reset_emergency(probe))
    R.check(S, 'dig_escalated latched false', probe.get('dig') is not None
            and probe.get('dig').data is False)
    for name, cli in (('reboot_board', probe.reboot_cli), ('set_firmware_debug', probe.debug_cli),
                      ('clear_dig_escalation', probe.dig_cli)):
        req = SetBool.Request() if cli is probe.debug_cli else Trigger.Request()
        res = probe.call(cli, req)
        R.check(S, f'{name} answers success=false with a reason',
                res is not None and not res.success and bool(res.message))
    R.check(S, '/imu/data published', wait_for(lambda: probe.count('imu') > 50, 3))
    R.check(S, '/gps/fix published', wait_for(lambda: probe.get('fix') is not None, 5))
    g = probe.get('gnss')
    R.check(S, '/gps/status RTK fixed with an accuracy', wait_for(
        lambda: probe.get('gnss') is not None and probe.get('gnss').fix_type == 3
        and probe.get('gnss').horizontal_accuracy_m > 0.0, 5),
        f'fix_type={g.fix_type if g else None}')
    b = probe.get('battery')
    R.check(S, '/battery_state off the dock: current 0 (docking convention)',
            b is not None and b.current == 0.0 and abs(b.voltage - 24.0) < 0.05)


def s02_round_trip(probe, rig, R):
    S = 'round trip'
    s0 = rig.snapshot()
    t0 = rig.now_ms()
    ticks0 = probe.get('ticks')
    drive(probe, 0.2, 0.0, 4.0)
    frames = speed_frames(rig, t0)
    R.check(S, 'cmd_vel reaches the MCU as SpeedData', bool(frames), f'{len(frames)} frames')
    R.check(S, 'SpeedData streamed at the control rate', 60 <= len(frames) <= 90,
            f'{len(frames)} in 4 s')
    last = frames[-1][1] if frames else (0, 0)
    R.check(S, 'reaches 0.2 m/s after the slew ramp', abs(last[0] - 0.2) < 1e-3, f'{last}')
    rises = [b[1][0] - a[1][0] for a, b in zip(frames, frames[1:])]
    R.check(S, 'acceleration limited (<= 0.3 m/s^2 per 50 ms)',
            max(rises or [0]) <= 0.3 * 0.05 + 0.01, f'max step {max(rises or [0]):.3f}')
    odom = probe.get('odom')
    R.check(S, '/wheel_odom reports the MCU-measured speed',
            odom is not None and abs(odom.twist.twist.linear.x - 0.2) < 0.02,
            f'{odom.twist.twist.linear.x if odom else None}')
    s1 = rig.snapshot()
    dist = math.hypot(s1['x'] - s0['x'], s1['y'] - s0['y'])
    R.check(S, 'the emulated robot moved', 0.4 < dist < 0.8, f'{dist:.2f} m')
    t1 = probe.get('ticks')
    R.check(S, '/wheel_ticks advance with the motion (derived)',
            t1 is not None and ticks0 is not None and t1.wheel_ticks_rl > ticks0.wheel_ticks_rl)
    drive(probe, 0.0, 0.3, 3.0)
    w = probe.get('odom').twist.twist.angular.z
    R.check(S, 'pivot: yaw rate from the IMU gyro on /wheel_odom', abs(w - 0.3) < 0.05, f'{w:.3f}')
    drive(probe, 1.0, 2.0, 2.0)
    last = speed_frames(rig, rig.now_ms() - 300)[-1][1]
    R.check(S, 'clamped to 0.3 m/s and 0.3 rad/s before the MCU',
            abs(last[0]) <= 0.3 + 1e-6 and abs(last[1]) <= 0.3 + 1e-6, f'{last}')
    stop_driving(probe, rig)


def s03_stop_sequence(probe, rig, R):
    S = 'stop / stale cmd_vel'
    drive(probe, 0.2, 0.0, 2.0)
    probe.cmd = None                        # publisher goes silent
    t0 = rig.now_ms()
    time.sleep(1.5)
    after = speed_frames(rig, t0)
    nonzero = [t for t, v in after if v != (0.0, 0.0)]
    zeros = [t for t, v in after if v == (0.0, 0.0)]
    R.check(S, 'stale cmd_vel -> zeros within cmd_vel_timeout', bool(zeros)
            and zeros[0] - t0 < 650, f'first zero {zeros[0] - t0 if zeros else float("nan"):.0f} ms')
    R.check(S, 'exactly 3 zeros, then silence', len(zeros) == 3 and all(t < zeros[0] for t in nonzero),
            f'{len(zeros)} zeros')
    R.check(S, 'robot stopped', wait_for(lambda: abs(rig.snapshot()['v']) < 0.005, 2))
    drive(probe, 0.2, 0.0, 1.5)
    t0 = rig.now_ms()
    probe.cmd = (0.0, 0.0)
    time.sleep(1.0)
    probe.cmd = None
    zeros = [v for t, v in speed_frames(rig, t0) if v == (0.0, 0.0)]
    R.check(S, 'explicit zero cmd_vel -> 3 zeros then silence (not a zero stream)', len(zeros) == 3,
            f'{len(zeros)}')


def s04_idle_hold(probe, rig, R):
    S = 'IDLE holds the wheels'
    probe.hl_state = IDLE
    time.sleep(1.0)
    t0 = rig.now_ms()
    probe.cmd = (0.2, 0.0)
    time.sleep(1.5)
    probe.cmd = None
    R.check(S, 'no motion frame in IDLE', not [v for t, v in speed_frames(rig, t0)
                                               if v != (0.0, 0.0)])
    probe.hl_state = AUTONOMOUS
    time.sleep(0.6)


def s05_stop_button(probe, rig, R):
    S = 'stop button'
    drive(probe, 0.2, 0.0, 1.5)
    with rig.lock:
        rig.mcu.stop_button = True
    ok = wait_for(lambda: probe.get('emergency').active_emergency, 1.0)
    R.check(S, 'emergency active + latched', ok and probe.get('emergency').latched_emergency,
            probe.get('emergency').reason)
    R.check(S, 'robot stops', wait_for(lambda: abs(rig.snapshot()['v']) < 0.005, 2))
    res = estop(probe, 0)
    R.check(S, 'release refused while the button is held', res is not None and not res.success)
    with rig.lock:
        rig.mcu.stop_button = False
    time.sleep(0.5)
    t0 = rig.now_ms()
    time.sleep(1.0)
    R.check(S, 'still latched after the button is let go; no motion',
            probe.get('emergency').latched_emergency and not [
                v for t, v in speed_frames(rig, t0) if v != (0.0, 0.0)])
    t0 = rig.now_ms()
    R.check(S, 'reset clears it', reset_emergency(probe))
    R.check(S, 'reset sends the MCU clear frame', wait_for(lambda: bool(rig.events('clear', t0)), 1))
    probe.cmd = None
    time.sleep(0.6)


def s06_tilt(probe, rig, R):
    S = 'tilt / lift'
    probe.cmd = None
    with rig.lock:
        rig.extra_roll_deg = 33.0          # past the host critical band, inside the MCU window
    ok = wait_for(lambda: probe.get('emergency').latched_emergency
                  and 'Tilt' in probe.get('emergency').reason, 3)
    R.check(S, 'host tilt monitor latches an emergency before the MCU lift window', ok
            and not rig.snapshot()['lift'], probe.get('emergency').reason)
    with rig.lock:
        rig.extra_roll_deg = 45.0
    R.check(S, 'MCU asserts lift from the forwarded attitude',
            wait_for(lambda: rig.snapshot()['lift'], 2))
    R.check(S, 'lift reported', wait_for(lambda: 'Lift' in probe.get('emergency').reason, 2),
            probe.get('emergency').reason)
    with rig.lock:
        rig.extra_roll_deg = 0.0
    time.sleep(3.0)
    R.check(S, 'reset after levelling', reset_emergency(probe))


def s07_blade(probe, rig, R):
    S = 'blade'
    probe.hl_state = IDLE
    time.sleep(0.6)
    mower(probe, 1)
    time.sleep(1.0)
    R.check(S, 'no blade in IDLE', not rig.snapshot()['cutter_on'])
    probe.hl_state = AUTONOMOUS
    R.check(S, 'blade on in AUTONOMOUS', wait_for(lambda: rig.snapshot()['cutter_on'], 2))
    R.check(S, 'Status mow_enabled + rpm', wait_for(
        lambda: probe.get('status').mow_enabled and probe.get('status').mower_motor_rpm > 1000, 3))
    probe.hl_state = None                   # behaviour tree dies
    t0 = time.monotonic()
    ok = wait_for(lambda: not rig.snapshot()['cutter_on'], 8)
    R.check(S, 'blade off when HighLevelStatus goes stale (5 s)', ok,
            f'{time.monotonic() - t0:.1f} s')
    probe.hl_state = AUTONOMOUS
    wait_for(lambda: rig.snapshot()['cutter_on'], 2)
    estop(probe, 1)
    R.check(S, 'blade off on software e-stop', wait_for(lambda: not rig.snapshot()['cutter_on'], 1))
    reset_emergency(probe)
    R.check(S, 'blade back once released (still AUTONOMOUS + requested)',
            wait_for(lambda: rig.snapshot()['cutter_on'], 2))
    req = SetParameters.Request()
    req.parameters = [Parameter(name='cutter_height_mm', value=ParameterValue(
        type=ParameterType.PARAMETER_INTEGER, integer_value=45))]
    probe.call(probe.param_cli, req)
    R.check(S, 'deck height change reaches the MCU (live parameter)',
            wait_for(lambda: rig.snapshot()['height_mm'] == 45, 2))
    R.check(S, 'height published', wait_for(lambda: probe.get('height') is not None
                                            and probe.get('height').data == 45, 2))
    mower(probe, 0)
    R.check(S, 'blade off on request', wait_for(lambda: not rig.snapshot()['cutter_on'], 1))


def s08_docked(probe, rig, R):
    S = 'docked'
    probe.hl_state = IDLE
    probe.cmd = None
    with rig.lock:
        rig.mcu.docked = True
        rig.mcu.speed_frames_while_docked = 0
    t_dock = rig.now_ms()
    ok = wait_for(lambda: probe.get('status').is_charging, 3)
    R.check(S, 'is_charging = on the dock', ok)
    with rig.lock:
        early = rig.mcu.speed_frames_while_docked
        rig.mcu.speed_frames_while_docked = 0
    R.check(S, 'nothing sent while the dock contact was being reported', early == 0,
            f't_dock={t_dock:.0f} {speed_frames(rig, t_dock - 3000)}')
    b = probe.get('battery')
    R.check(S, '/battery_state current = abs(charge current)', b is not None and b.current > 1.0,
            f'{b.current if b else None}')
    R.check(S, 'charger enabled on dock arrival', wait_for(lambda: bool(rig.events('charge')), 2))
    t0 = rig.now_ms()
    probe.cmd = (0.2, 0.0)
    probe.hl_state = None
    time.sleep(1.5)
    with rig.lock:
        n = rig.mcu.speed_frames_while_docked
    R.check(S, 'no SpeedData at all on the dock without an undock (position hold kept)', n == 0,
            f'{n} frames: {speed_frames(rig, t0)}')
    odom = probe.get('odom')
    R.check(S, '/wheel_odom zeroed while docked', odom.twist.twist.linear.x == 0.0)
    estop(probe, 1)
    time.sleep(0.5)
    reset_emergency(probe)
    with rig.lock:
        n = rig.mcu.speed_frames_while_docked
    R.check(S, 'an e-stop at rest on the dock sends no SpeedData either', n == 0, f'{n}')
    probe.hl_state = AUTONOMOUS
    probe.cmd = (-0.1, 0.0)
    time.sleep(1.5)
    with rig.lock:
        n = rig.mcu.speed_frames_while_docked
    R.check(S, 'reverse refused on the dock (would push into the charger)', n == 0,
            f'{n}: {speed_frames(rig, t0)}')
    probe.cmd = (0.15, 0.0)
    R.check(S, 'forward undock allowed once the BT is active',
            wait_for(lambda: rig.mcu.speed_frames_while_docked > 0, 2))
    with rig.lock:
        rig.mcu.docked = False
    stop_driving(probe, rig)


UNDOCK_SPEED = 0.16       # template undock_speed
UNDOCK_DISTANCE = 0.6     # shorter than the template's 1.5 m: only the dock exit matters here


def s12_undock_forward(probe, rig, R):
    """The Tron reverses onto its charger, so it undocks FORWARD
    (backends/airseekers.yaml undock_forward: true -> the BT's undock BackUps
    use /drive_on_heading). Closed loop on a dock-pad model: the contacts open
    only once the robot has driven off them, and the dock body is a hard stop
    behind it."""
    S = 'undock forward'
    probe.hl_state = IDLE
    probe.cmd = None
    at_rest = wait_for(lambda: rig.snapshot()['setpoint'] == (0.0, 0.0)
                       and abs(rig.snapshot()['v']) < 1e-3, 5)
    R.check(S, 'at rest before seating', at_rest, f"{rig.snapshot()['setpoint']}")
    rig.seat_on_dock(pad_m=0.10)
    R.check(S, 'seated: is_charging', wait_for(lambda: probe.get('status').is_charging, 3))

    # The BT leaves IDLE; a stock (reverse) BackUp undock is refused outright.
    probe.hl_state = AUTONOMOUS
    t0 = rig.now_ms()
    probe.cmd = (-UNDOCK_SPEED, 0.0)
    time.sleep(1.5)
    frames = speed_frames(rig, t0)
    R.check(S, 'reverse undock refused while docked: no SpeedData', not frames, f'{frames[:5]}')
    snap = rig.snapshot()
    R.check(S, 'reverse: robot did not move and is still charging',
            abs(snap['dock_along']) < 0.005 and snap['dock_pushes'] == 0
            and probe.get('status').is_charging, f"along={snap['dock_along']:.4f}")

    # undock_forward: the same leg driven forward at the undock speed.
    t1 = rig.now_ms()
    probe.cmd = (UNDOCK_SPEED, 0.0)
    left = wait_for(lambda: not probe.get('status').is_charging, 5)
    snap = rig.snapshot()
    R.check(S, 'forward undock leaves the pads: is_charging drops', left,
            f"along={snap['dock_along']:.3f}")
    R.check(S, 'Power.charger_enabled and /battery_state current follow',
            wait_for(lambda: probe.get('power') is not None
                     and not probe.get('power').charger_enabled
                     and probe.get('battery').current == 0.0, 2))
    R.check(S, 'undock distance covered forward',
            wait_for(lambda: rig.snapshot()['dock_along'] >= UNDOCK_DISTANCE, 8),
            f"along={rig.snapshot()['dock_along']:.3f}")
    frames = [v for _, v in speed_frames(rig, t1)]
    R.check(S, 'forward SpeedData only, up to the undock speed',
            bool(frames) and all(lin >= 0.0 for lin, _ in frames)
            and abs(max(lin for lin, _ in frames) - UNDOCK_SPEED) < 0.01,
            f'{len(frames)} frames, max {max((f[0] for f in frames), default=0):.3f}')
    R.check(S, 'never pushed into the dock', rig.snapshot()['dock_pushes'] == 0)
    R.check(S, '/wheel_odom live again off the dock',
            wait_for(lambda: probe.get('odom').twist.twist.linear.x > 0.05, 2))

    # Off the dock the normal driving rules apply: reverse is allowed again.
    t2 = rig.now_ms()
    probe.cmd = (-0.1, 0.0)
    R.check(S, 'reverse allowed once off the dock',
            wait_for(lambda: any(lin < 0.0 for _, (lin, _a) in speed_frames(rig, t2)), 3))
    stop_driving(probe, rig)
    with rig.lock:
        rig.dock = None


def s09_link_loss(probe, rig, R):
    S = 'MCU link loss'
    drive(probe, 0.2, 0.0, 1.5)
    with rig.lock:
        rig.mcu_silent = True
    t0 = rig.now_ms()
    time.sleep(1.5)
    after = speed_frames(rig, t0)
    nonzero = [t for t, v in after if v != (0.0, 0.0)]
    R.check(S, 'motion frames stop within the link timeout (0.5 s)',
            not nonzero or nonzero[-1] - t0 < 650, f'last {nonzero[-1] - t0 if nonzero else 0:.0f} ms')
    R.check(S, 'zeros sent', any(v == (0.0, 0.0) for t, v in after))
    R.check(S, 'firmware_compatible drops', wait_for(
        lambda: not probe.get('status').firmware_compatible, 2))
    with rig.lock:
        rig.mcu_silent = False
    probe.cmd = None
    R.check(S, 'recovers', wait_for(lambda: probe.get('status').firmware_compatible, 5))


def s10_keys(probe, rig, R):
    S = 'panel keys'
    with probe.lock:
        probe.hlc_commands.clear()
    rig.press_key(KEY_S)
    R.check(S, 'go-dock key -> high_level_control HOME',
            wait_for(lambda: 2 in probe.hlc_commands, 7), f'{probe.hlc_commands}')


def s11_kill(probe, rig, R, bridge):
    S = 'bridge SIGKILL'
    drive(probe, 0.3, 0.0, 3.0)
    mower(probe, 1)
    wait_for(lambda: rig.snapshot()['cutter_on'], 2)
    snap = rig.snapshot()
    R.check(S, 'driving and mowing at the kill', snap['v'] > 0.25 and snap['cutter_on'], f'{snap}')
    with rig.lock:
        # Worst case: measure the host-traffic watchdog alone. (The MCU also
        # asserts lift once the forwarded IMU stream stops, which would stop
        # it sooner -- but that window is not characterised either.)
        rig.mcu.imu_missing_ms = 1e9
    t0 = rig.now_ms()
    os.killpg(bridge.pid, signal.SIGKILL)   # no destructor, no last zero
    time.sleep(0.3)
    t_last = max([t for t, _ in rig.events('speed')] + [t for t, _ in rig.events('heartbeat')])
    R.check(S, 'host traffic stops at the kill', t_last - t0 < 150, f'{t_last - t0:.0f} ms')
    ok = wait_for(lambda: rig.snapshot()['setpoint'] == (0.0, 0.0)
                  and not rig.snapshot()['cutter_on'], (HOST_TIMEOUT_MS + 1000) / 1000.0)
    fired = rig.mcu.watchdog_fired_t
    dt = (fired - t_last) if fired else float('nan')
    R.check(S, f'MCU watchdog zeroes wheels AND blade within its window (~{HOST_TIMEOUT_MS:.0f} ms '
            'after the last host frame; window ASSUMED, see README)',
            ok and fired is not None and dt <= HOST_TIMEOUT_MS + 100,
            f'{dt:.0f} ms; at kill: {snap}')
    R.check(S, 'robot physically stopped', wait_for(lambda: abs(rig.snapshot()['v']) < 0.005, 2))


SCENARIOS = [s01_boot, s02_round_trip, s03_stop_sequence, s04_idle_hold, s05_stop_button,
             s06_tilt, s07_blade, s08_docked, s09_link_loss, s10_keys, s12_undock_forward]


def main():
    sys.setswitchinterval(0.0005)
    ap = argparse.ArgumentParser()
    ap.add_argument('--only', nargs='*')
    ap.add_argument('--logdir', default=tempfile.gettempdir())
    args = ap.parse_args()
    rclpy.init()
    R = Report()
    workdir = tempfile.mkdtemp(prefix='assim-')
    rig = AirseekersRig(workdir, host_timeout_ms=HOST_TIMEOUT_MS)
    rig.start()
    bridge = start_bridge(rig, workdir, os.path.join(args.logdir, 'bridge-airseekers.log'))
    probe = Probe()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(probe)
    spinner = threading.Thread(target=ex.spin, daemon=True)
    spinner.start()
    try:
        for sc in SCENARIOS:
            if args.only and sc.__name__ not in args.only:
                continue
            print(f'-- {sc.__name__}', flush=True)
            try:
                sc(probe, rig, R)
            except Exception as exc:  # noqa: BLE001
                R.check(sc.__name__, 'scenario raised', False, repr(exc))
                traceback.print_exc()
            probe.cmd = None
        if not args.only or 's11_kill' in args.only:
            print('-- s11_kill', flush=True)
            try:
                s11_kill(probe, rig, R, bridge)
            except Exception as exc:  # noqa: BLE001
                R.check('s11_kill', 'scenario raised', False, repr(exc))
                traceback.print_exc()
    finally:
        try:
            os.killpg(bridge.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        ex.shutdown()
        spinner.join(timeout=5.0)
        probe.destroy_node()
        rig.close()
        rclpy.shutdown()
    total, bad = len(R.rows), R.failed
    print(f'\n===== {total - len(bad)}/{total} checks passed =====')
    for s, n, _, d in bad:
        print(f'  FAIL {s}: {n}' + (f' — {d}' if d else ''))
    sys.exit(1 if bad or total == 0 else 0)


if __name__ == '__main__':
    main()
