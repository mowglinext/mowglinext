#!/usr/bin/env python3
# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
"""Closed-loop simulation of mowgli_openmower_bridge on emulated OpenMower v1.

Launches the REAL bridge through its production launch file against the
emulated LowLevel board + xESC controllers (openmower_v1_emulator.py), drives
it over ROS the way MowgliNext does, and checks what the emulated hardware
actually received. Timings are measured on the emulator's clock.

  python3 run_simulation.py --esc xesc_2040        # one controller type
  python3 run_simulation.py                        # both (default)

Needs a sourced workspace with mowgli_interfaces + mowgli_openmower_bridge.
Exit status 0 only when every check passes.
"""

from __future__ import annotations

import argparse
import math
import os
import signal
import statistics
import subprocess
import sys
import tempfile
import threading
import time
import traceback

import rclpy
from geometry_msgs.msg import TwistStamped
from mowgli_interfaces.msg import Emergency, HighLevelStatus, Power, Status
from mowgli_interfaces.srv import EmergencyStop, HighLevelControl, MowerControl
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from openmower_v1_emulator import OpenMowerV1Rig  # noqa: E402

AUTONOMOUS, IDLE = 2, 1
# xesc_odometry.hpp kMaxPlausibleWheelSpeedMps: no wheel of this robot goes faster.
MAX_PLAUSIBLE_WHEEL_SPEED_MPS = 2.0
# The robot-config template the image ships (Dockerfile COPY), from this checkout.
ROBOT_TEMPLATE = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', '..', '..',
    'ros2', 'src', 'mowgli_bringup', 'config', 'mowgli_robot.yaml'))
BACKEND_DEFAULTS = os.path.join(os.path.dirname(ROBOT_TEMPLATE), 'backends', 'openmower.yaml')
DRIVES = ('left', 'right')


# ---------------------------------------------------------------------------
# Result bookkeeping
# ---------------------------------------------------------------------------


class Report:
    def __init__(self):
        self.rows = []

    def check(self, scenario: str, name: str, ok: bool, detail: str = ''):
        self.rows.append((scenario, name, bool(ok), detail))
        mark = 'PASS' if ok else 'FAIL'
        print(f'  [{mark}] {name}' + (f' — {detail}' if detail else ''), flush=True)
        return ok

    @property
    def failed(self):
        return [r for r in self.rows if not r[2]]


# ---------------------------------------------------------------------------
# ROS side: what the rest of MowgliNext would be doing
# ---------------------------------------------------------------------------


class Probe(Node):
    def __init__(self):
        super().__init__('om_sim_probe')
        self.lock = threading.Lock()
        self.last = {}
        self.odom = []
        self.imu = []
        self.applied = []
        self.hlc_commands = []
        self.cmd = None          # (vx, wz) published at 20 Hz while set
        self.hl_state = None     # published at 2 Hz while set
        rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', rel)
        self.hl_pub = self.create_publisher(HighLevelStatus, '/behavior_tree_node/high_level_status', rel)
        for topic, typ, key in [('/hardware_bridge/status', Status, 'status'),
                                ('/hardware_bridge/emergency', Emergency, 'emergency'),
                                ('/hardware_bridge/power', Power, 'power'),
                                ('/battery_state', BatteryState, 'battery')]:
            self.create_subscription(typ, topic, self._keeper(key), rel)
        self.create_subscription(Bool, '/hardware_bridge/dig_escalated', self._keeper('dig'), latched)
        self.create_subscription(Odometry, '/wheel_odom', self._on_odom, rel)
        self.create_subscription(Imu, '/imu/data', self._on_imu, rel)
        self.create_subscription(TwistStamped, '/hardware_bridge/cmd_vel_applied', self._on_applied, rel)
        self.create_service(HighLevelControl, '/behavior_tree_node/high_level_control', self._on_hlc)
        self.estop = self.create_client(EmergencyStop, '/hardware_bridge/emergency_stop')
        self.mower = self.create_client(MowerControl, '/hardware_bridge/mower_control')
        self.clear_dig = self.create_client(Trigger, '/hardware_bridge/clear_dig_escalation')
        self.reboot = self.create_client(Trigger, '/hardware_bridge/reboot_board')
        self.fw_debug = self.create_client(SetBool, '/hardware_bridge/set_firmware_debug')
        self.get_params = self.create_client(GetParameters, '/hardware_bridge/get_parameters')
        self.create_timer(0.05, self._publish_cmd)
        self.create_timer(0.5, self._publish_hl)

    def _keeper(self, key):
        def cb(msg):
            with self.lock:
                self.last[key] = (time.monotonic(), msg)
        return cb

    def _on_odom(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            self.odom.append((time.monotonic(), msg.twist.twist.linear.x, msg.twist.twist.angular.z, stamp))
            self.odom = self.odom[-2000:]

    def _on_imu(self, msg):
        with self.lock:
            self.imu.append((time.monotonic(), msg.angular_velocity.z))
            self.imu = self.imu[-3000:]

    def _on_applied(self, msg):
        with self.lock:
            self.applied.append((time.monotonic(), msg.twist.linear.x, msg.twist.angular.z))
            self.applied = self.applied[-3000:]

    def _on_hlc(self, req, res):
        with self.lock:
            self.hlc_commands.append((time.monotonic(), req.command))
        res.success = True
        return res

    # The scenario thread sets cmd / hl_state at any time: read each ONCE. A
    # check-then-read let `stop_driving`'s `cmd = None` land in between, the
    # unpack raised inside the executor, and spin() died with every
    # subscription (a whole controller run then failed in cascade).
    def _publish_cmd(self):
        cmd = self.cmd
        if cmd is None:
            return
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.twist.linear.x, m.twist.angular.z = cmd
        self.cmd_pub.publish(m)

    def _publish_hl(self):
        state = self.hl_state
        if state is None:
            return
        m = HighLevelStatus()
        m.state = state
        self.hl_pub.publish(m)

    # helpers
    def get(self, key):
        with self.lock:
            v = self.last.get(key)
        return v[1] if v else None

    def mean_since(self, series: str, since: float, idx: int):
        with self.lock:
            data = [s[idx] for s in getattr(self, series) if s[0] >= since]
        return (statistics.fmean(data), len(data)) if data else (float('nan'), 0)

    def odom_distance_mean(self, since: float, idx: int):
        """Distance over time from /wheel_odom: each message's rate times the
        interval its header stamp covers, which is what a consumer integrating
        the odometry gets. A plain mean of jittery windows is biased up (0.03 +
        0.71 averages 0.37), and receive times carry the probe's own latency."""
        with self.lock:
            data = [(s[3], s[idx]) for s in self.odom if s[0] >= since]
        if len(data) < 2:
            return float('nan'), len(data)
        num = sum(v * (t - t_prev) for (t_prev, _), (t, v) in zip(data, data[1:]))
        return num / (data[-1][0] - data[0][0]), len(data)

    def call(self, client, request, timeout=5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        fut = client.call_async(request)
        t_end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < t_end:
            time.sleep(0.01)
        return fut.result() if fut.done() else None


def spin_in_background(executor):
    """Spin on a daemon thread; a callback exception is a harness bug, so say
    so loudly instead of silently ending every subscription."""
    def run():
        try:
            executor.spin()
        except Exception:  # noqa: BLE001 — report, then stop
            print('!!! probe executor crashed — every later check is void:', flush=True)
            traceback.print_exc()
            raise
    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    return thread


def wait_for(pred, timeout: float, period: float = 0.02) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(period)
    return bool(pred())


def estop(probe, value: int):
    r = EmergencyStop.Request()
    r.emergency = value
    return probe.call(probe.estop, r)


def mower(probe, enabled: int, direction: int):
    r = MowerControl.Request()
    r.mow_enabled, r.mow_direction = enabled, direction
    return probe.call(probe.mower, r)


# ---------------------------------------------------------------------------
# Bridge process
# ---------------------------------------------------------------------------


def start_bridge(rig, esc_type, workdir, log_path, extra_yaml='', env_override=None):
    cfg = os.path.join(workdir, 'mowgli_robot.yaml')
    with open(cfg, 'w', encoding='utf-8') as fh:
        # The operator's SPARSE file as a fresh install leaves it: nothing
        # about the drive or the board. ticks_per_meter and the board's
        # safety values must come from the OpenMower backend defaults.
        fh.write('mowgli:\n  ros__parameters:\n    mowing_enabled: true\n' + extra_yaml)
    env = dict(os.environ,
               OPENMOWER_LL_PORT=rig.paths['ll'], OPENMOWER_XESC_LEFT_PORT=rig.paths['left'],
               OPENMOWER_XESC_RIGHT_PORT=rig.paths['right'], OPENMOWER_XESC_MOW_PORT=rig.paths['mow'],
               OPENMOWER_XESC_TYPE=esc_type, OPENMOWER_ROBOT_CONFIG=cfg,
               OPENMOWER_ROBOT_TEMPLATE=ROBOT_TEMPLATE,
               OPENMOWER_BACKEND_DEFAULTS=BACKEND_DEFAULTS)
    env.update(env_override or {})
    log = open(log_path, 'w', encoding='utf-8')
    return subprocess.Popen(['ros2', 'launch', 'mowgli_openmower_bridge', 'openmower_bridge.launch.py'],
                            env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)


# ---------------------------------------------------------------------------
# Scenarios
# ---------------------------------------------------------------------------


def _bridge_param(probe, name):
    """A parameter of the running bridge node (None if unavailable)."""
    if not probe.get_params.wait_for_service(timeout_sec=5.0):
        return None
    req = GetParameters.Request()
    req.names = [name]
    fut = probe.get_params.call_async(req)
    if not wait_for(fut.done, 5.0) or fut.result() is None or not fut.result().values:
        return None
    v = fut.result().values[0]
    return {1: v.bool_value, 2: v.integer_value, 3: v.double_value, 4: v.string_value}.get(v.type)


def drive(probe, rig, vx, wz, seconds):
    probe.hl_state = AUTONOMOUS
    probe.cmd = (vx, wz)
    time.sleep(seconds)


def stop_driving(probe, rig, settle=1.0):
    probe.cmd = (0.0, 0.0)
    wait_for(lambda: abs(rig.snapshot()['v']) < 0.01 and abs(rig.snapshot()['w']) < 0.02, settle + 2)
    probe.cmd = None


def reset_emergency(probe, rig):
    estop(probe, 0)
    return wait_for(lambda: not rig.snapshot()['latch'], 2.0) and wait_for(
        lambda: probe.get('emergency') is not None and not probe.get('emergency').latched_emergency, 2.0)


def s01_boot(probe, rig, R, esc):
    S = 'boot'
    ok = wait_for(lambda: probe.get('status') is not None and probe.get('status').firmware_compatible, 20)
    R.check(S, 'Status.firmware_compatible once the LowLevel + both drives answer', ok,
            probe.get('status').firmware_version if probe.get('status') else 'no Status')
    st = probe.get('status')
    R.check(S, 'mower_status OK', st is not None and st.mower_status == Status.MOWER_STATUS_OK)
    R.check(S, 'board received a config request', len(rig.ll.config_requests) >= 1,
            f'{len(rig.ll.config_requests)} request(s)')
    if rig.ll.config_requests:
        rcv = rig.ll.config_requests[0][1]
        R.check(S, 'board config comes from the shared settings + OpenMower defaults '
                '(lift 100 / tilt 2500 ms, 29.0 V, 1.2 A)',
                rcv['lift_period'] == 100 and rcv['tilt_period'] == 2500
                and abs(rcv['v_battery_cutoff'] - 29.0) < 1e-3 and abs(rcv['i_charge_cutoff'] - 1.2) < 1e-3,
                f"lift={rcv['lift_period']} tilt={rcv['tilt_period']} "
                f"v_bat={rcv['v_battery_cutoff']:.2f} i={rcv['i_charge_cutoff']:.2f}")
        R.check(S, 'no MowgliNext equivalent -> left undefined (charger-input cutoff, halls, options)',
                rcv['v_charge_cutoff'] < 0 and all(h[0] == 3 for h in rcv['halls'])
                and rcv['dfp_is_5v'] == 2 and rcv['ignore_charging_current'] == 2,
                f"v_charge={rcv['v_charge_cutoff']}")
    c = rig.ll.config
    R.check(S, "board runs OpenMower's own lift/tilt periods (100 / 2500 ms)",
            c['lift_period'] == 100 and c['tilt_period'] == 2500,
            f"lift={c['lift_period']} tilt={c['tilt_period']}")
    R.check(S, 'board kept its own charger-input cutoff (30 V)', abs(c['v_charge_cutoff'] - 30) < 1e-3)
    R.check(S, 'ticks_per_meter from the OpenMower defaults (1600, not the STM32 399)',
            _bridge_param(probe, 'ticks_per_meter') == 1600.0, f"{_bridge_param(probe, 'ticks_per_meter')}")
    hb = [t for (t, _, _) in rig.ll.heartbeats if t > rig.now_ms() - 3000]
    gaps = [b - a for a, b in zip(hb, hb[1:])]
    R.check(S, 'heartbeat never gaps near the 500 ms board timeout',
            bool(gaps) and max(gaps) < 150, f'max gap {max(gaps):.0f} ms over {len(gaps)}' if gaps else 'none')
    if esc == 'xesc_2040':
        R.check(S, 'every xESC 2040 received its settings',
                all(rig.escs[n].settings_received for n in ('left', 'right', 'mow')))


def s02_boot_latch(probe, rig, R, esc):
    S = 'boot latch'
    em = probe.get('emergency')
    R.check(S, 'board boots latched and the bridge reports it',
            rig.snapshot()['latch'] and em is not None and em.latched_emergency and not em.active_emergency,
            em.reason if em else 'no Emergency')
    drive(probe, rig, 0.3, 0.0, 1.5)
    snap = rig.snapshot()
    R.check(S, 'wheels held at zero while latched, even with cmd_vel',
            all(abs(snap['duty'][n]) < 1e-9 for n in DRIVES) and abs(snap['v']) < 0.01,
            f"duty L={snap['duty']['left']:.3f} R={snap['duty']['right']:.3f} v={snap['v']:.3f}")
    R.check(S, 'operator reset clears the latch (bounded release)', reset_emergency(probe, rig))
    ok = wait_for(lambda: abs(rig.snapshot()['v']) > 0.1, 3.0)
    R.check(S, 'wheels move once released', ok, f"v={rig.snapshot()['v']:.3f}")
    stop_driving(probe, rig)


def forward_speed(a: dict, b: dict) -> float:
    """Mean speed along the robot's heading between two snapshots."""
    dx, dy = b['x'] - a['x'], b['y'] - a['y']
    heading = 0.5 * (a['theta'] + b['theta'])
    return (dx * math.cos(heading) + dy * math.sin(heading)) / ((b['t'] - a['t']) / 1000.0)


def _drive_diagnostics(probe, rig, since_mono: float, since_ms: float, x_since: float) -> str:
    """What the controllers and the plant actually saw since a moment — printed
    with a failing speed check, so a CI-only miss shows its cause."""
    snap = rig.snapshot()
    with rig.lock:
        cmds = [(t, d) for (t, d) in rig.escs['left'].commands if t >= since_ms]
    span_s = max((snap['t'] - since_ms) / 1000.0, 1e-6)
    gaps = [b[0] - a[0] for a, b in zip(cmds, cmds[1:])]
    duty = statistics.fmean(d for _, d in cmds) if cmds else float('nan')
    applied, _ = probe.mean_since('applied', since_mono, 1)
    odom, n_odom = probe.mean_since('odom', since_mono, 1)
    return (f"left cmds {len(cmds) / span_s:.0f}/s (max gap {max(gaps, default=float('nan')):.0f} ms), "
            f"mean duty {duty:+.3f}, applied {applied:.3f}, odom {odom:.3f} ({n_odom} msgs), "
            f"plant mean {(snap['x'] - x_since) / span_s:.3f} m/s")


def s03_straight(probe, rig, R, esc):
    S = 'straight line'
    x0 = rig.snapshot()['x']
    drive(probe, rig, 0.30, 0.0, 4.0)
    s0 = rig.snapshot()
    t_meas, t_meas_ms, x_meas = time.monotonic(), s0['t'], s0['x']
    time.sleep(2.0)
    snap = rig.snapshot()
    # Means over the window: an instantaneous plant speed or a single odometry
    # window is just jitter on a loaded runner.
    plant_v = forward_speed(s0, snap)
    odom_vx, n = probe.odom_distance_mean(t_meas, 1)
    if abs(plant_v - 0.30) >= 0.03 or abs(odom_vx - plant_v) >= 0.03:
        print('    diag: ' + _drive_diagnostics(probe, rig, t_meas, t_meas_ms, x_meas), flush=True)
    R.check(S, 'true ground speed tracks 0.30 m/s (closed loop beats the 0.85 motor gain)',
            abs(plant_v - 0.30) < 0.03, f"plant mean v={plant_v:.3f} m/s")
    R.check(S, '/wheel_odom vx matches the ground', abs(odom_vx - plant_v) < 0.03,
            f'odom vx={odom_vx:.3f} over {n} msgs')
    R.check(S, 'no spurious rotation', abs(snap['w']) < 0.05, f"w={snap['w']:.3f}")
    R.check(S, 'real wiring: left duty > 0, right duty < 0',
            snap['duty']['left'] > 0.05 and snap['duty']['right'] < -0.05,
            f"L={snap['duty']['left']:+.3f} R={snap['duty']['right']:+.3f}")
    R.check(S, 'robot actually moved forward', snap['x'] - x0 > 1.2, f"dx={snap['x'] - x0:.2f} m")
    stop_driving(probe, rig)


def s04_pivot(probe, rig, R, esc):
    S = 'pivot'
    drive(probe, rig, 0.0, 0.6, 3.0)
    t_meas, s0 = time.monotonic(), rig.snapshot()
    time.sleep(2.0)
    snap = rig.snapshot()
    # Mean yaw rate over the window (2 s at 0.6 rad/s never wraps past pi).
    plant_w = math.atan2(math.sin(snap['theta'] - s0['theta']), math.cos(snap['theta'] - s0['theta'])) \
        / ((snap['t'] - s0['t']) / 1000.0)
    odom_w, _ = probe.odom_distance_mean(t_meas, 2)
    imu_w, _ = probe.mean_since('imu', t_meas, 1)
    R.check(S, 'true yaw rate tracks +0.6 rad/s, counter-clockwise', abs(plant_w - 0.6) < 0.08,
            f"plant mean w={plant_w:+.3f}")
    R.check(S, '/wheel_odom angular.z agrees in sign and size', abs(odom_w - plant_w) < 0.08,
            f'odom w={odom_w:+.3f}')
    R.check(S, '/imu/data gyro z agrees in sign and size', abs(imu_w - plant_w) < 0.08,
            f'imu gz={imu_w:+.3f}')
    R.check(S, 'no translation', abs(snap['v']) < 0.03, f"v={snap['v']:+.3f}")
    stop_driving(probe, rig)


def s05_slew(probe, rig, R, esc):
    S = 'acceleration limit'
    t0 = time.monotonic()
    drive(probe, rig, 0.4, 0.0, 2.5)
    with probe.lock:
        pts = [(t, v) for (t, v, _) in probe.applied if t >= t0]
    slopes = [(v2 - v1) / (t2 - t1) for (t1, v1), (t2, v2) in zip(pts, pts[1:]) if t2 - t1 > 0.015]
    worst = max(slopes) if slopes else float('nan')
    R.check(S, 'applied command ramps at <= 0.30 m/s^2 (+ jitter)', bool(slopes) and worst < 0.45,
            f'max {worst:.2f} m/s^2 over {len(slopes)} steps')
    stop_driving(probe, rig)


def s06_cmd_timeout(probe, rig, R, esc):
    S = 'cmd_vel timeout'
    drive(probe, rig, 0.3, 0.0, 3.0)
    probe.cmd = None  # the publisher dies mid-drive
    t0 = rig.now_ms()
    tz = rig.first_time_effective_zero(DRIVES, t0, 3000)
    R.check(S, 'drives commanded to zero within 1.0 s timeout + one tick',
            tz is not None and tz - t0 < 1250, f'{(tz - t0) if tz else float("nan"):.0f} ms')
    wait_for(lambda: abs(rig.snapshot()['v']) < 0.01, 3)


def s07_idle(probe, rig, R, esc):
    S = 'IDLE holds the wheels'
    probe.hl_state = IDLE
    time.sleep(1.0)
    probe.cmd = (0.3, 0.0)
    time.sleep(1.5)
    snap = rig.snapshot()
    R.check(S, 'no motion in IDLE despite cmd_vel', abs(snap['v']) < 0.01 and
            all(abs(snap['duty'][n]) < 1e-9 for n in DRIVES))
    probe.hl_state = AUTONOMOUS
    R.check(S, 'motion resumes in AUTONOMOUS', wait_for(lambda: rig.snapshot()['v'] > 0.1, 3))
    stop_driving(probe, rig)


def s08_stop_button(probe, rig, R, esc):
    S = 'stop button'
    drive(probe, rig, 0.3, 0.0, 2.5)
    with rig.lock:
        rig.ll.stop_pressed = True
        t0 = rig.now_ms()
    tz = rig.first_time_effective_zero(DRIVES, t0, 2000)
    R.check(S, 'drives at zero within 250 ms of the press (20 ms debounce included)',
            tz is not None and tz - t0 < 250, f'{(tz - t0) if tz else float("nan"):.0f} ms')
    ok = wait_for(lambda: probe.get('emergency') is not None and probe.get('emergency').active_emergency, 1)
    em = probe.get('emergency')
    R.check(S, 'Emergency active, reason "STOP button"', ok and em.reason == 'STOP button',
            em.reason if em else '')
    with rig.lock:
        rig.ll.stop_pressed = False
    time.sleep(1.5)
    snap = rig.snapshot()
    R.check(S, 'button released: still latched, wheels still held', snap['latch'] and abs(snap['v']) < 0.01)
    R.check(S, 'operator reset releases it', reset_emergency(probe, rig))
    R.check(S, 'motion resumes after reset', wait_for(lambda: rig.snapshot()['v'] > 0.1, 3))
    stop_driving(probe, rig)


def s09_release_refused(probe, rig, R, esc):
    S = 'reset while the button is held'
    probe.hl_state = AUTONOMOUS
    with rig.lock:
        rig.ll.stop_pressed = True
    wait_for(lambda: rig.snapshot()['emergency_bitmask'] & 2, 1)
    estop(probe, 0)  # operator resets while the button is still pressed
    time.sleep(0.6)
    with rig.lock:
        rig.ll.stop_pressed = False
    probe.cmd = (0.3, 0.0)
    time.sleep(2.0)
    snap = rig.snapshot()
    R.check(S, 'release does NOT fire by itself when the button is let go',
            snap['latch'] and abs(snap['v']) < 0.01, f"latch={snap['latch']} v={snap['v']:.3f}")
    R.check(S, 'a fresh reset is required, and works', reset_emergency(probe, rig))
    stop_driving(probe, rig)


def s10_lift(probe, rig, R, esc):
    S = 'lift / tilt'
    drive(probe, rig, 0.3, 0.0, 2.5)
    with rig.lock:
        rig.ll.lifted_wheels = 2
        t0 = rig.now_ms()
    tz = rig.first_time_effective_zero(DRIVES, t0, 3000)
    R.check(S, "both wheels lifted: zero within the board's 100 ms lift period + chain (< 300 ms)",
            tz is not None and tz - t0 < 300, f'{(tz - t0) if tz else float("nan"):.0f} ms')
    em = probe.get('emergency')
    wait_for(lambda: probe.get('emergency').reason == 'Lift detected', 1)
    R.check(S, 'reason "Lift detected"', probe.get('emergency').reason == 'Lift detected')
    with rig.lock:
        rig.ll.lifted_wheels = 0
    time.sleep(0.3)
    reset_emergency(probe, rig)
    # One wheel: the board's own 2500 ms tilt period must be in force.
    with rig.lock:
        rig.ll.lifted_wheels = 1
        t1 = rig.now_ms()
    ok = wait_for(lambda: rig.snapshot()['emergency_bitmask'] & 4, 4)
    dt = rig.now_ms() - t1
    R.check(S, 'one wheel lifted trips after ~2500 ms (board default, not the 2000 ms template)',
            ok and 2400 < dt < 2800, f'{dt:.0f} ms')
    with rig.lock:
        rig.ll.lifted_wheels = 0
    time.sleep(0.3)
    reset_emergency(probe, rig)
    stop_driving(probe, rig)


def s11_software_estop(probe, rig, R, esc):
    S = 'software e-stop'
    drive(probe, rig, 0.3, 0.0, 2.5)
    t0 = rig.now_ms()
    estop(probe, 1)
    tz = rig.first_time_effective_zero(DRIVES, t0, 1000)
    R.check(S, 'drives at zero within 100 ms of the service call',
            tz is not None and tz - t0 < 100, f'{(tz - t0) if tz else float("nan"):.0f} ms')
    R.check(S, 'board latched through the heartbeat request', wait_for(lambda: rig.snapshot()['latch'], 1))
    em = probe.get('emergency')
    wait_for(lambda: probe.get('emergency').latched_emergency, 1)
    R.check(S, 'Emergency latched', probe.get('emergency').latched_emergency)
    R.check(S, 'reset clears it', reset_emergency(probe, rig))
    stop_driving(probe, rig)


def s12_blade(probe, rig, R, esc):
    S = 'blade'
    probe.hl_state = AUTONOMOUS
    time.sleep(1.0)
    mower(probe, 1, 1)
    ok = wait_for(lambda: rig.snapshot()['duty']['mow'] > 0.9, 2)
    R.check(S, 'mower_control(on, dir 1) spins the blade at +duty (OpenMower convention)', ok,
            f"mow duty={rig.snapshot()['duty']['mow']:+.2f}")
    wait_for(lambda: probe.get('status').blade_requested_direction == 'reverse', 1)
    st = probe.get('status')
    R.check(S, 'Status.mow_enabled + blade_requested_direction "reverse"',
            st.mow_enabled and st.blade_requested_direction == 'reverse', st.blade_requested_direction)
    wait_for(lambda: rig.snapshot()['blade_rpm'] > 2000, 3)
    wait_for(lambda: probe.get('status').mower_motor_rpm > 1000 or esc == 'xesc_2040', 2)
    mower(probe, 1, 0)
    R.check(S, 'direction 0 reverses the duty', wait_for(lambda: rig.snapshot()['duty']['mow'] < -0.9, 2))
    t0 = rig.now_ms()
    probe.hl_state = IDLE
    tz = rig.first_time_effective_zero(('mow',), t0, 2000)
    R.check(S, 'IDLE stops the blade', tz is not None and tz - t0 < 800, f'{(tz - t0) if tz else float("nan"):.0f} ms')
    probe.hl_state = AUTONOMOUS
    R.check(S, 'blade back in AUTONOMOUS', wait_for(lambda: abs(rig.snapshot()['duty']['mow']) > 0.9, 2))
    probe.hl_state = None  # behavior_tree_node dies
    t1 = rig.now_ms()
    tz = rig.first_time_effective_zero(('mow',), t1, 8000)
    R.check(S, 'behaviour tree silent 5 s -> blade off (watchdog on the last reported mode)',
            tz is not None and 4500 < tz - t1 < 6000, f'{(tz - t1) if tz else float("nan"):.0f} ms')
    probe.hl_state = AUTONOMOUS
    wait_for(lambda: abs(rig.snapshot()['duty']['mow']) > 0.9, 3)
    t2 = rig.now_ms()
    estop(probe, 1)
    tz = rig.first_time_effective_zero(('mow',), t2, 1000)
    R.check(S, 'e-stop stops the blade within 100 ms', tz is not None and tz - t2 < 100,
            f'{(tz - t2) if tz else float("nan"):.0f} ms')
    reset_emergency(probe, rig)
    mower(probe, 0, 0)
    ok = wait_for(lambda: abs(rig.snapshot()['duty']['mow']) < 1e-9, 2) and wait_for(
        lambda: probe.get('status').blade_requested_direction == 'off', 2)
    R.check(S, 'mower_control(off) -> duty 0, direction "off"', ok)


def s13_lowlevel_loss(probe, rig, R, esc):
    S = 'LowLevel link loss'
    drive(probe, rig, 0.3, 0.0, 2.5)
    n_cfg = len(rig.ll.config_requests)
    with rig.lock:
        rig.ll.paused = True
        t0 = rig.now_ms()
    tz = rig.first_time_effective_zero(DRIVES, t0, 3000)
    R.check(S, 'drives at zero within the 0.5 s status timeout (was 2 s)',
            tz is not None and tz - t0 < 700, f'{(tz - t0) if tz else float("nan"):.0f} ms')
    time.sleep(1.5)
    with rig.lock:
        rig.ll.paused = False
    R.check(S, 'board latched itself on the missing heartbeat', rig.snapshot()['latch'])
    R.check(S, 'config re-sent after the gap', wait_for(lambda: len(rig.ll.config_requests) > n_cfg, 3))
    R.check(S, 'reset after reconnection', reset_emergency(probe, rig))
    R.check(S, 'motion resumes', wait_for(lambda: rig.snapshot()['v'] > 0.1, 3))
    stop_driving(probe, rig)


def s14_esc_loss(probe, rig, R, esc):
    S = 'drive controller link loss'
    drive(probe, rig, 0.3, 0.0, 2.5)
    with rig.lock:
        rig.escs['right'].paused = True
        t0 = rig.now_ms()
    tz = rig.first_time_effective_zero(DRIVES, t0, 3000)
    R.check(S, 'BOTH drives zeroed when one controller goes silent (< 1.3 s)',
            tz is not None and tz - t0 < 1300, f'{(tz - t0) if tz else float("nan"):.0f} ms')
    ok = wait_for(lambda: not probe.get('status').firmware_compatible, 2)
    R.check(S, 'Status.firmware_compatible false', ok, probe.get('status').firmware_version)
    with rig.lock:
        rig.escs['right'].paused = False
    R.check(S, 'recovers when it answers again',
            wait_for(lambda: probe.get('status').firmware_compatible, 3) and
            wait_for(lambda: rig.snapshot()['v'] > 0.1, 3))
    stop_driving(probe, rig)


def s15_esc_reboot(probe, rig, R, esc):
    S = 'controller reboot mid-drive'
    drive(probe, rig, 0.3, 0.0, 3.0)
    t0 = time.monotonic()
    with rig.lock:
        rig.escs['left'].reboot()   # counters back to 0
    time.sleep(2.0)
    with probe.lock:
        vx = [v for (t, v, *_) in probe.odom if t >= t0]
    worst = max((abs(v) for v in vx), default=float('nan'))
    # The bug published the counter jump itself (2.2e7 m/s). A starved runner
    # also produces honest 50 ms windows at 2-3x (ticks landing one window
    # late, compensated by a low neighbour), so the bound is the bridge's own
    # physical plausibility limit, not a tight speed band.
    ok = bool(vx) and worst < MAX_PLAUSIBLE_WHEEL_SPEED_MPS
    if not ok:
        print('    diag: vx series ' + ' '.join(f'{v:.2f}' for v in vx), flush=True)
    R.check(S, f'no odometry spike from the counter reset (|vx| < {MAX_PLAUSIBLE_WHEEL_SPEED_MPS} m/s)',
            ok, f'max |vx|={worst:.2f} m/s over {len(vx)} msgs')
    if esc == 'xesc_2040':
        R.check(S, 'settings re-sent after the reboot', wait_for(lambda: rig.escs['left'].settings_received, 2))
    R.check(S, 'still drives straight after it', wait_for(lambda: abs(rig.snapshot()['v'] - 0.3) < 0.05, 4),
            f"v={rig.snapshot()['v']:.3f}")
    stop_driving(probe, rig)


def s16_docking(probe, rig, R, esc):
    S = 'docking (v1-fw)'
    probe.hl_state = IDLE
    with rig.lock:
        rig.ll.gyro_bias_z = 0.03
        rig.ll.v_battery = 27.0
        rig.ll.v_charge = 29.6
    ok = wait_for(lambda: probe.get('status').is_charging, 3)
    R.check(S, 'on the dock -> Status.is_charging', ok)
    ok = wait_for(lambda: probe.get('battery') is not None and
                  probe.get('battery').power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING, 15)
    b = probe.get('battery')
    R.check(S, 'BatteryState CHARGING, current = |charge current|',
            ok and abs(b.current - 1.0) < 0.05, f'current={b.current:.2f}' if b else '')
    t_dock = time.monotonic()

    def trailing_gz():
        return probe.mean_since('imu', time.monotonic() - 0.5, 1)

    # The injected bias must first SHOW on /imu/data, then be calibrated away.
    seen = wait_for(lambda: trailing_gz()[1] > 10 and abs(trailing_gz()[0] - 0.03) < 0.006, 3)
    R.check(S, 'the 0.03 rad/s bias shows on /imu/data before calibration', seen,
            f'gz={trailing_gz()[0]:+.4f}')
    ok = wait_for(lambda: trailing_gz()[1] > 10 and abs(trailing_gz()[0]) < 0.006, 12)
    took = time.monotonic() - t_dock
    R.check(S, 'docked IMU calibration removes the 0.03 rad/s gyro bias', ok,
            f'gz={trailing_gz()[0]:+.4f}, {took:.1f} s after contact')
    R.check(S, '...not before the wheels settled (1 s + 200 samples at 50 Hz)', took > 4.5,
            f'{took:.1f} s')
    time.sleep(1.5)  # crossing the threshold is not removal: check steady state
    steady, n = probe.mean_since('imu', time.monotonic() - 1.0, 1)
    # n only proves the stream is alive (a loaded runner delivers fewer of the
    # 50 Hz samples); the residual is the check.
    R.check(S, 'steady-state residual after calibration < 0.002 rad/s', n > 20 and abs(steady) < 0.002,
            f'gz={steady:+.5f} over {n}')
    t = time.monotonic() - 2.0
    vx, n = probe.mean_since('odom', t, 1)
    R.check(S, '/wheel_odom held at zero on the dock (Invariant 11)', n > 5 and abs(vx) < 1e-9)
    with rig.lock:
        rig.ll.v_battery = 29.3  # above the board's 29 V cutoff -> relay opens
    # /battery_state publishes slower than /power: wait for the whole picture.
    ok = wait_for(lambda: not probe.get('power').charger_enabled
                  and probe.get('battery').power_supply_status == BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING, 5)
    st, pw, b = probe.get('status'), probe.get('power'), probe.get('battery')
    R.check(S, 'battery full: relay open but STILL docked',
            ok and st.is_charging and pw.charger_status == 'docked, not charging'
            and b.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING,
            f'is_charging={st.is_charging} relay={pw.charger_enabled} "{pw.charger_status}"')
    with rig.lock:
        rig.ll.v_charge = 0.2
        rig.ll.v_battery = 27.0
        rig.ll.gyro_bias_z = 0.0
    R.check(S, 'undocked -> is_charging false', wait_for(lambda: not probe.get('status').is_charging, 3))


def s17_old_firmware(probe, rig, R, esc):
    S = 'v0.13 firmware charging bit'
    with rig.lock:
        rig.ll.generation = 'v0.13'
    # The relay last opened in the docking scenario; the firmware only retries
    # after CHARGING_RETRY_MILLIS (10 s).
    ok = wait_for(lambda: probe.get('power').charger_enabled, 13)
    R.check(S, 'old firmware reports the relay ON off the dock (after its 10 s retry)', ok)
    R.check(S, '...yet the bridge does NOT call it docked', not probe.get('status').is_charging)
    drive(probe, rig, 0.3, 0.0, 4.0)
    vx, n = probe.mean_since('odom', time.monotonic() - 1.5, 1)
    # Zeroed (Invariant 11 misapplied) would read 0.0; the speed itself is
    # checked in the straight-line scenario.
    R.check(S, '/wheel_odom is NOT zeroed while mowing', n > 5 and vx > 0.15, f'vx={vx:.3f}')
    stop_driving(probe, rig)
    with rig.lock:
        rig.ll.generation = 'v1-fw'


def s18_buttons(probe, rig, R, esc):
    S = 'panel buttons'
    for bid, dur, expected, label in [(3, 0, HighLevelControl.Request.COMMAND_START, 'PLAY'),
                                      (2, 0, HighLevelControl.Request.COMMAND_HOME, 'HOME'),
                                      (6, 2, HighLevelControl.Request.COMMAND_RESET_EMERGENCY, 'LOCK very long')]:
        t0 = time.monotonic()
        with rig.lock:
            rig.ll.send_ui_event(bid, dur)
        ok = wait_for(lambda: any(t >= t0 and c == expected for t, c in probe.hlc_commands), 2)
        R.check(S, f'{label} -> high_level_control({expected})', ok)
    t0 = time.monotonic()
    with rig.lock:
        rig.ll.send_ui_event(6, 0)
    time.sleep(1.0)
    R.check(S, 'LOCK short press is ignored', not any(t >= t0 for t, _ in probe.hlc_commands))


def s19_contract(probe, rig, R, esc):
    S = 'contract odds and ends'
    r = probe.call(probe.clear_dig, Trigger.Request())
    R.check(S, 'clear_dig_escalation answers success=false with a reason', r is not None and not r.success
            and 'OpenMower' in r.message, r.message if r else 'no answer')
    r = probe.call(probe.reboot, Trigger.Request())
    R.check(S, 'reboot_board answers success=false', r is not None and not r.success)
    req = SetBool.Request()
    req.data = True
    r = probe.call(probe.fw_debug, req)
    R.check(S, 'set_firmware_debug answers success=false', r is not None and not r.success)
    d = probe.get('dig')
    R.check(S, 'dig_escalated latched false', d is not None and d.data is False)
    probe.hl_state = AUTONOMOUS
    time.sleep(1.5)
    modes = [m for (t, m, _) in rig.ll.hl_states if t > rig.now_ms() - 1000]
    R.check(S, 'high-level mode forwarded to the board', bool(modes) and modes[-1] == AUTONOMOUS,
            f'last mode {modes[-1] if modes else None}')


def s19b_starved(probe, rig, R, esc, bridge):
    """The bridge process is starved for 350 ms mid-drive (a loaded Pi, a GC
    pause in a neighbour): under every 500 ms watchdog, so the robot keeps
    going. /wheel_odom must still report the true speed afterwards — the 1.x
    bridge divided three periods of ticks by one clamped control period."""
    S = 'bridge starved 350 ms'
    drive(probe, rig, 0.3, 0.0, 3.0)
    t0, s0 = time.monotonic(), rig.snapshot()
    os.killpg(bridge.pid, signal.SIGSTOP)
    time.sleep(0.35)
    os.killpg(bridge.pid, signal.SIGCONT)
    time.sleep(1.5)
    snap = rig.snapshot()
    with probe.lock:
        vx = [v for (t, v, *_) in probe.odom if t >= t0]
    plant_v = forward_speed(s0, snap)
    odom_v, _ = probe.odom_distance_mean(t0, 1)
    worst = max((abs(v) for v in vx), default=float('nan'))
    # The exact arithmetic (a 350 ms stall divided by 350 ms, not by a clamped
    # control period) is pinned by WheelTickSampler.AStalledBridgeMeasures-
    # TheWholeStall; here, what a consumer of the running bridge can see.
    R.check(S, 'the robot kept driving through the stall', abs(plant_v - 0.3) < 0.05,
            f'plant mean v={plant_v:.3f} m/s')
    R.check(S, '/wheel_odom agrees with the ground on average', abs(odom_v - plant_v) < 0.05,
            f'odom mean {odom_v:.3f} over {len(vx)} msgs')
    if not worst < MAX_PLAUSIBLE_WHEEL_SPEED_MPS:
        print('    diag: vx series ' + ' '.join(f'{v:.2f}' for v in vx), flush=True)
    R.check(S, f'no physically impossible window (< {MAX_PLAUSIBLE_WHEEL_SPEED_MPS} m/s)',
            worst < MAX_PLAUSIBLE_WHEEL_SPEED_MPS, f'max |vx|={worst:.2f} m/s')
    R.check(S, 'board did not latch (stall shorter than its 500 ms heartbeat timeout)',
            not snap['latch'])
    stop_driving(probe, rig)


def s20_crash(probe, rig, R, esc, bridge):
    S = 'bridge crash'
    drive(probe, rig, 0.3, 0.0, 2.5)
    mower(probe, 1, 1)
    wait_for(lambda: abs(rig.snapshot()['duty']['mow']) > 0.9, 2)
    t0 = rig.now_ms()
    os.killpg(bridge.pid, signal.SIGKILL)  # no destructor, no last zero duty
    wd = 500 if esc == 'xesc_2040' else 1000
    tz = rig.first_time_effective_zero(('left', 'right', 'mow'), t0, 3000)
    R.check(S, f'controllers stop on their own watchdog (~{wd} ms) — wheels AND blade',
            tz is not None and tz - t0 < wd + 150, f'{(tz - t0) if tz else float("nan"):.0f} ms')
    wait_for(lambda: rig.snapshot()['latch'], 1.5)
    with rig.lock:
        latched = [t for t in rig.ll.latch_events if t >= t0]
    dt = (latched[0] - t0) if latched else float('nan')
    R.check(S, 'board latched on the missing heartbeat (~500 ms)', bool(latched) and 450 < dt < 650,
            f'{dt:.0f} ms')
    R.check(S, 'robot physically stopped', wait_for(lambda: abs(rig.snapshot()['v']) < 0.01, 2))


SCENARIOS = [s01_boot, s02_boot_latch, s03_straight, s04_pivot, s05_slew, s06_cmd_timeout,
             s07_idle, s08_stop_button, s09_release_refused, s10_lift, s11_software_estop,
             s12_blade, s13_lowlevel_loss, s14_esc_loss, s15_esc_reboot, s16_docking,
             s17_old_firmware, s18_buttons, s19_contract]


def run_one(esc: str, R: Report, logdir: str, only=None):
    print(f'\n========== {esc} ==========', flush=True)
    workdir = tempfile.mkdtemp(prefix=f'omsim-{esc}-')
    rig = OpenMowerV1Rig(workdir, esc)
    rig.start()
    bridge = start_bridge(rig, esc, workdir, os.path.join(logdir, f'bridge-{esc}.log'))
    probe = Probe()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(probe)
    spin_in_background(ex)
    try:
        for sc in SCENARIOS:
            if only and sc.__name__ not in only:
                continue
            print(f'-- {esc} / {sc.__name__}', flush=True)
            try:
                sc(probe, rig, R, esc)
            except Exception as exc:  # a crashed scenario is a failure, not an abort
                R.check(sc.__name__, 'scenario raised', False, f'{exc!r}')
                traceback.print_exc()
            probe.cmd = None
        if not only or 's19b_starved' in only:
            print(f'-- {esc} / s19b_starved', flush=True)
            s19b_starved(probe, rig, R, esc, bridge)
        if not only or 's20_crash' in only:
            print(f'-- {esc} / s20_crash', flush=True)
            s20_crash(probe, rig, R, esc, bridge)
    finally:
        try:
            os.killpg(bridge.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        ex.shutdown()
        probe.destroy_node()
        rig.close()


def run_gui_config(R: Report, logdir: str):
    """Settings saved in the GUI (mowgli_robot.yaml) win over the installer's
    docker/.env: here .env names the wrong ESC protocol and dead ports, the yaml
    the real ones, plus one deliberate LowLevel override."""
    S = 'gui config'
    print(f'\n========== {S} ==========', flush=True)
    workdir = tempfile.mkdtemp(prefix='omsim-gui-')
    rig = OpenMowerV1Rig(workdir, 'xesc_2040')
    rig.start()
    yaml_keys = (f"    openmower_ll_port: \"{rig.paths['ll']}\"\n"
                 '    openmower_xesc_type: "xesc_2040"\n'
                 f"    openmower_xesc_left_port: \"{rig.paths['left']}\"\n"
                 f"    openmower_xesc_right_port: \"{rig.paths['right']}\"\n"
                 f"    openmower_xesc_mow_port: \"{rig.paths['mow']}\"\n"
                 '    both_wheels_lift_emergency_ms: 200\n'
                 '    one_wheel_lift_emergency_ms: 0\n')
    stale_env = {'OPENMOWER_LL_PORT': '/dev/null-ll', 'OPENMOWER_XESC_LEFT_PORT': '/dev/null-l',
                 'OPENMOWER_XESC_RIGHT_PORT': '/dev/null-r', 'OPENMOWER_XESC_MOW_PORT': '/dev/null-m',
                 'OPENMOWER_XESC_TYPE': 'xesc_mini'}
    bridge = start_bridge(rig, 'xesc_2040', workdir, os.path.join(logdir, 'bridge-gui-config.log'),
                          extra_yaml=yaml_keys, env_override=stale_env)
    probe = Probe()
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(probe)
    spin_in_background(ex)
    try:
        ok = wait_for(lambda: probe.get('status') is not None and probe.get('status').firmware_compatible, 20)
        R.check(S, 'connects through the yaml ports + ESC type, not the stale .env', ok,
                probe.get('status').firmware_version if probe.get('status') else 'no Status')
        ok = wait_for(lambda: rig.ll.config['lift_period'] == 200, 5)
        c = rig.ll.config
        R.check(S, 'the operator\'s both_wheels_lift_emergency_ms reached the board', ok,
                f"lift={c['lift_period']}")
        R.check(S, 'a 0 ms one-wheel period (= disabled on the Pico) is refused; the board keeps 2500',
                c['tilt_period'] == 2500, f"tilt={c['tilt_period']}")
    finally:
        try:
            os.killpg(bridge.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        ex.shutdown()
        probe.destroy_node()
        rig.close()


def main():
    # The emulator thread stands in for microcontrollers that answer within a
    # millisecond, but it shares the GIL with the probe's executor threads.
    # With Python's default 5 ms switch interval a starved runner made it
    # answer tens of ms late, and the bridge's (correct) timing then measured
    # the EMULATOR's latency as speed noise. Switch every 0.5 ms instead.
    sys.setswitchinterval(0.0005)
    ap = argparse.ArgumentParser()
    ap.add_argument('--esc', choices=['xesc_mini', 'xesc_2040', 'both'], default='both')
    ap.add_argument('--only', nargs='*', help='scenario function names')
    ap.add_argument('--logdir', default=tempfile.gettempdir())
    args = ap.parse_args()
    rclpy.init()
    R = Report()
    try:
        for esc in (['xesc_mini', 'xesc_2040'] if args.esc == 'both' else [args.esc]):
            # the R rows carry the scenario name; prefix with the controller
            before = len(R.rows)
            run_one(esc, R, args.logdir, args.only)
            R.rows[before:] = [(f'{esc} / {s}', n, ok, d) for (s, n, ok, d) in R.rows[before:]]
        if not args.only or 'gui_config' in args.only:
            run_gui_config(R, args.logdir)
    finally:
        rclpy.shutdown()
    total, bad = len(R.rows), R.failed
    print(f'\n===== {total - len(bad)}/{total} checks passed =====')
    for s, n, _, d in bad:
        print(f'  FAIL {s}: {n}' + (f' — {d}' if d else ''))
    sys.exit(1 if bad or total == 0 else 0)


if __name__ == '__main__':
    main()
