#!/usr/bin/env python3
# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
"""The seam between mowgli-ros2 and the OpenMower sidecar, as on a robot.

run_simulation.py exercises the bridge alone. This runs the REAL
mowgli_bringup/launch/mowgli.launch.py next to it, the way the two containers
share one DDS domain on a robot, and checks the three things that only exist
at that seam:

  1. HARDWARE_BACKEND=openmower: mowgli.launch.py starts robot_state_publisher
     and twist_mux but NOT the STM32 hardware_bridge_node, so exactly one
     /hardware_bridge exists (the sidecar's);
  2. a command entering a twist_mux INPUT lane (/cmd_vel_teleop) comes out on
     /cmd_vel and moves the emulated wheels;
  3. HARDWARE_BACKEND=mowgli still starts the STM32 bridge, and an unknown
     backend aborts the launch.

Needs mowgli_bringup, mowgli_hardware, mowgli_interfaces and
mowgli_openmower_bridge in the sourced workspace.
"""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import tempfile
import threading
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from openmower_v1_emulator import OpenMowerV1Rig  # noqa: E402
from run_simulation import (  # noqa: E402
    AUTONOMOUS, Probe, Report, estop, spin_in_background, start_bridge, wait_for)


def launch_mowgli(backend: str, log_path: str):
    env = dict(os.environ, HARDWARE_BACKEND=backend)
    log = open(log_path, 'w', encoding='utf-8')
    return subprocess.Popen(['ros2', 'launch', 'mowgli_bringup', 'mowgli.launch.py'], env=env,
                            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)


def kill(p):
    try:
        os.killpg(p.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    p.wait(timeout=10)


def node_names(probe):
    return [n for n, _ns in probe.get_node_names_and_namespaces()]


def main():
    sys.setswitchinterval(0.0005)  # see run_simulation.main
    logdir = sys.argv[1] if len(sys.argv) > 1 else tempfile.gettempdir()
    rclpy.init()
    R = Report()
    probe = Probe()
    teleop = probe.create_publisher(TwistStamped, '/cmd_vel_teleop',
                                    QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
    desc = {}
    probe.create_subscription(String, '/robot_description', lambda m: desc.setdefault('urdf', m.data),
                              QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                         durability=DurabilityPolicy.TRANSIENT_LOCAL))
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(probe)
    spinner = spin_in_background(ex)
    teleop_cmd: dict = {'v': None}

    def pump_teleop():
        while True:
            v = teleop_cmd['v']
            if v is not None:
                m = TwistStamped()
                m.header.stamp = probe.get_clock().now().to_msg()
                m.twist.linear.x = v
                teleop.publish(m)
            time.sleep(0.05)

    threading.Thread(target=pump_teleop, daemon=True).start()

    # ---- 1 + 2: openmower backend, both containers ------------------------
    S = 'HARDWARE_BACKEND=openmower'
    print(f'\n== {S} ==', flush=True)
    workdir = tempfile.mkdtemp(prefix='omstack-')
    rig = OpenMowerV1Rig(workdir, 'xesc_2040')
    rig.start()
    stack = launch_mowgli('openmower', os.path.join(logdir, 'stack-openmower.log'))
    bridge = start_bridge(rig, 'xesc_2040', workdir, os.path.join(logdir, 'stack-bridge.log'))
    try:
        ok = wait_for(lambda: {'twist_mux', 'robot_state_publisher', 'hardware_bridge'} <= set(node_names(probe)), 30)
        R.check(S, 'robot_state_publisher, twist_mux and the sidecar bridge are up', ok,
                ', '.join(sorted(n for n in node_names(probe) if n != 'om_sim_probe')))
        time.sleep(3.0)  # give a wrongly-launched STM32 bridge every chance to appear
        count = node_names(probe).count('hardware_bridge')
        R.check(S, 'exactly ONE /hardware_bridge (mowgli.launch.py did not start the STM32 one)',
                count == 1, f'{count} found')
        R.check(S, '/robot_description published', wait_for(lambda: 'urdf' in desc, 10))
        R.check(S, 'sidecar ready (firmware_compatible)',
                wait_for(lambda: probe.get('status') is not None and probe.get('status').firmware_compatible, 20))
        probe.hl_state = AUTONOMOUS
        estop(probe, 0)
        R.check(S, 'boot latch released', wait_for(lambda: not rig.snapshot()['latch'], 3))
        x0 = rig.snapshot()['x']
        teleop_cmd['v'] = 0.3
        ok = wait_for(lambda: abs(rig.snapshot()['v'] - 0.3) < 0.03, 8)
        R.check(S, '/cmd_vel_teleop -> twist_mux -> /cmd_vel -> bridge -> wheels at 0.3 m/s', ok,
                f"v={rig.snapshot()['v']:.3f}")
        time.sleep(2.0)
        R.check(S, 'the robot moved forward', rig.snapshot()['x'] - x0 > 0.5, f"dx={rig.snapshot()['x'] - x0:.2f} m")
        teleop_cmd['v'] = None  # operator lets go
        t0 = rig.now_ms()
        tz = rig.first_time_effective_zero(('left', 'right'), t0, 3000)
        R.check(S, 'operator lets go -> wheels stop (mux 0.5 s + bridge 1.0 s timeouts)',
                tz is not None and tz - t0 < 1700, f'{(tz - t0) if tz else float("nan"):.0f} ms')
    finally:
        kill(bridge)
        kill(stack)
        rig.close()

    # ---- 3: mowgli backend keeps its STM32 bridge -------------------------
    S = 'HARDWARE_BACKEND=mowgli'
    print(f'\n== {S} ==', flush=True)
    wait_for(lambda: 'hardware_bridge' not in node_names(probe), 15)
    stack = launch_mowgli('mowgli', os.path.join(logdir, 'stack-mowgli.log'))
    try:
        ok = wait_for(lambda: 'hardware_bridge' in node_names(probe), 30)
        R.check(S, 'the STM32 hardware_bridge_node is launched (no sidecar running)', ok)
    finally:
        kill(stack)

    S = 'HARDWARE_BACKEND=bogus'
    print(f'\n== {S} ==', flush=True)
    log = os.path.join(logdir, 'stack-bogus.log')
    stack = launch_mowgli('bogus', log)
    try:
        rc = stack.wait(timeout=60)
    except subprocess.TimeoutExpired:
        rc = None
        kill(stack)
    text = open(log, encoding='utf-8').read()
    R.check(S, 'unknown backend aborts the launch', rc not in (None, 0) and 'Invalid hardware_backend' in text,
            f'exit={rc}')

    ex.shutdown()
    # Join before rclpy.shutdown/exit: a daemon spinner still inside rclpy
    # at interpreter teardown segfaults the run (exit 139 after a green run).
    spinner.join(timeout=5.0)
    probe.destroy_node()
    rclpy.shutdown()
    total, bad = len(R.rows), R.failed
    print(f'\n===== {total - len(bad)}/{total} checks passed =====')
    for s, n, _, d in bad:
        print(f'  FAIL {s}: {n}' + (f' — {d}' if d else ''))
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
