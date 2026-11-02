# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Native body truth must remain independent of wheel/control history.

Replaces the wheel-integrator oracle with completed native Solid acquisitions.
The original command independence, braking, identity and recovery regressions
remain; wheel odometry still has its separate real upstream interval test.
"""

import importlib.util
import math
import os
from pathlib import Path
import time
from types import SimpleNamespace
import xml.etree.ElementTree as ET

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TwistStamped
import pytest
import yaml


def _load_module():
    source = Path(os.environ.get(
        'KINEMATIC_DRIVE_SOURCE',
        Path(__file__).resolve().parents[1]
        / 'mowgli_simulation' / 'kinematic_drive.py',
    ))
    spec = importlib.util.spec_from_file_location('kinematic_feedback_under_test', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def rotation(roll=0.0, pitch=0.0, yaw=0.0):
    # Independent Rz(yaw) Ry(pitch) Rx(roll) fixture.
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr,
            sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr, -sp, cp*sr, cp*cr]


class Field:

    def __init__(self, values, writes):
        self.values, self.writes = values, writes

    def getSFVec3f(self):
        return self.values.copy()

    def getSFRotation(self):
        return self.values.copy()

    def setSFVec3f(self, values):
        self.writes.append(('translation', values))

    def setSFRotation(self, values):
        self.writes.append(('rotation', values))


class Sensor:
    ROTATIONAL = 0

    def __init__(self):
        self.value = 0.0

    def enable(self, timestep):
        pass

    def getValue(self):
        return self.value

    def getType(self):
        return self.ROTATIONAL


@pytest.fixture
def plant(monkeypatch):
    module = _load_module()
    writes, devices, callbacks, pubs = [], [], [], {}
    sensors = {'left_wheel_sensor': Sensor(), 'right_wheel_sensor': Sensor()}
    clock = SimpleNamespace(native=1.0, wall=5.0)
    native = SimpleNamespace(position=[0.0, 0.0, 0.1], rotation=rotation())
    fields = {'translation': Field([0.0, 0.0, 0.1], writes),
              'rotation': Field([0.0, 0.0, 1.0, 0.0], writes)}
    robot = SimpleNamespace(
        _ref=1, getField=fields.get,
        getPosition=lambda: native.position.copy(),
        getOrientation=lambda: native.rotation.copy(),
        # This is deliberately absurd COM velocity, never origin truth.
        getVelocity=lambda: [99.0] * 6,
        setVelocity=lambda v: writes.append(('velocity', v)),
        resetPhysics=lambda: writes.append(('reset', None)),
    )

    def device(name):
        devices.append(name)
        return sensors[name]

    supervisor = SimpleNamespace(
        getBasicTimeStep=lambda: 20, getSelf=lambda: robot,
        getName=lambda: 'MowgliMower', getTime=lambda: clock.native,
        getDevice=device,
    )

    def publisher(_type, topic, _depth):
        pubs[topic] = []
        return SimpleNamespace(publish=pubs[topic].append)

    node = SimpleNamespace(
        create_subscription=lambda *args: callbacks.append(args[2]), create_publisher=publisher,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            to_msg=lambda: Time(sec=1_793_000_000))),
        get_logger=lambda: SimpleNamespace(info=lambda msg: None, warning=lambda msg: None),
    )
    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(module.rclpy, 'create_node', lambda name: node)
    monkeypatch.setattr(time, 'monotonic', lambda: clock.wall)
    drive = module.KinematicDrive()
    drive.init(SimpleNamespace(robot=supervisor), {})

    def command(vx, wz=0.0):
        msg = TwistStamped()
        msg.twist.linear.x, msg.twist.angular.z = vx, wz
        for callback in callbacks:
            callback(msg)

    def step(x=0.0, y=0.0, yaw=0.0, epoch=None, z=0.1, roll=0.0, pitch=0.0):
        native.position = [x, y, z]
        native.rotation = rotation(roll, pitch, yaw)
        if epoch is not None:
            clock.native = epoch
        drive.step()

    return SimpleNamespace(
        drive=drive, step=step, command=command, sensors=sensors, native=native,
        devices=devices, writes=writes, clock=clock,
        poses=pubs['/sim/ground_truth_pose'], twists=pubs['/sim/applied_twist'],
    )


def test_native_pose_overrides_control_and_corrupt_wheel_history(plant):
    plant.step()
    plant.command(0.215, 0.6)
    plant.sensors['left_wheel_sensor'].value = 198.0
    plant.sensors['right_wheel_sensor'].value = math.nan
    plant.step(-0.000372, epoch=1.02)
    pose, applied = plant.poses[-1], plant.twists[-1]
    assert pose.pose.position.x == -0.000372
    assert pose.pose.position.y == 0.0
    assert applied.twist.linear.x == pytest.approx(-0.0186, rel=0.0, abs=1e-15)
    assert applied.twist.angular.z == 0.0
    assert pose.header.frame_id == 'map'
    assert applied.header.frame_id == 'base_footprint'
    assert pose.header.stamp == Time(sec=1, nanosec=20_000_000)
    assert applied.header.stamp == pose.header.stamp
    assert plant.devices == []  # Neither encoders nor motors define body truth.
    assert plant.writes == []


def test_initial_pose_is_baseline_not_travel_and_stationary_is_exact_zero(plant):
    plant.step(12.0, -9.0, 1.2)
    assert plant.poses == plant.twists == []
    plant.command(0.215)
    plant.step(12.0, -9.0, 1.2, epoch=1.02)
    assert plant.poses[-1].pose.position.x == 12.0
    assert plant.twists[-1].twist.linear.x == plant.twists[-1].twist.angular.z == 0.0
    assert plant.writes == []


@pytest.mark.parametrize('speed,rate', [(0.2, 0.0), (-0.2, 0.0), (0.0, -0.8),
                                        (0.3, 0.8), (-0.3, -0.8), (0.3, 1e-8)])
def test_signed_straight_pivot_and_curved_intervals(plant, speed, rate):
    # Analytic circular motion about an independent fixed world center.
    start, dt = 0.7, 0.02
    plant.step(0.0, 0.0, start)
    end = start + rate * dt
    if abs(rate) > 1e-6:
        x = speed / rate * (math.sin(end) - math.sin(start))
        y = speed / rate * (math.cos(start) - math.cos(end))
    else:
        x, y = speed * dt * math.cos(start + rate*dt/2), speed * dt * math.sin(
            start + rate*dt/2)
    plant.step(x, y, end, epoch=1.02)
    applied = plant.twists[-1].twist
    assert plant.poses[-1].pose.position.x == x
    assert plant.poses[-1].pose.position.y == y
    assert applied.linear.x == pytest.approx(speed, rel=0.0, abs=1e-14)
    assert applied.linear.y == pytest.approx(0.0, rel=0.0, abs=1e-14)
    assert applied.angular.z == pytest.approx(rate, rel=0.0, abs=1e-14)


def test_native_side_slip_is_not_suppressed(plant):
    plant.step(yaw=math.pi/2)
    plant.step(-0.002, 0.004, math.pi/2, epoch=1.02)
    assert plant.twists[-1].twist.linear.x == pytest.approx(0.2, rel=0.0, abs=1e-15)
    assert plant.twists[-1].twist.linear.y == pytest.approx(0.1, rel=0.0, abs=1e-15)


def test_same_epoch_cannot_replace_pose_or_twist_on_wall_timeout(plant):
    plant.step()
    plant.step(0.000372, epoch=1.02)
    pose, twist = plant.poses[-1], plant.twists[-1]
    plant.clock.wall += 1.0
    plant.step(8.0, 9.0, 2.0, epoch=1.02)
    assert plant.poses == [pose]
    assert plant.twists == [twist]
    plant.step(0.000372, epoch=1.04)
    assert plant.twists[-1].twist.linear.x == plant.twists[-1].twist.angular.z == 0.0
    assert plant.writes == []


def test_expired_command_preserves_native_braking_then_stationary_stop(plant):
    plant.step()
    plant.step(0.000372, epoch=1.02)
    plant.clock.wall += 1.0
    plant.step(0.000558, epoch=1.04)
    assert plant.poses[-1].pose.position.x == 0.000558
    assert plant.twists[-1].twist.linear.x == pytest.approx(0.0093, rel=0.0, abs=1e-15)
    assert abs(plant.twists[-1].twist.linear.x - plant.twists[-2].twist.linear.x) / 0.02 < 1.0
    stopped_pose = plant.poses[-1].pose
    plant.step(0.000558, epoch=1.06)
    assert plant.twists[-1].twist.linear.x == plant.twists[-1].twist.angular.z == 0.0
    assert plant.poses[-1].pose == stopped_pose
    assert plant.writes == []


@pytest.mark.parametrize('bad', ['position', 'rotation', 'zero_rotation', 'reflection',
                                 'time', 'infinite_time', 'negative_time'])
def test_invalid_native_acquisition_emits_no_packet_then_rebaselines(plant, bad):
    plant.step()
    plant.step(0.000372, epoch=1.02)
    plant.clock.native = 1.04
    if bad == 'position':
        plant.native.position[2] = math.nan
    elif bad == 'rotation':
        plant.native.rotation[3] = math.inf
    elif bad == 'zero_rotation':
        plant.native.rotation = [0.0] * 9
    elif bad == 'reflection':
        plant.native.rotation[8] = -1.0
    elif bad == 'time':
        plant.clock.native = math.nan
    elif bad == 'infinite_time':
        plant.clock.native = math.inf
    else:
        plant.clock.native = -1.0
    plant.drive.step()
    assert len(plant.poses) == len(plant.twists) == 1
    plant.step(100.0, -100.0, epoch=1.06)
    assert len(plant.poses) == len(plant.twists) == 1
    plant.step(100.0, -100.0, epoch=1.08)
    assert plant.poses[-1].pose.position.x == 100.0
    assert plant.twists[-1].twist.linear.x == 0.0
    assert plant.writes == []


def test_native_rewind_rebaselines_at_new_native_location(plant):
    plant.step()
    plant.step(0.000372, epoch=1.02)
    plant.step(100.0, -100.0, epoch=0.5)
    assert len(plant.poses) == len(plant.twists) == 1
    plant.step(100.0, -100.0, epoch=0.52)
    assert plant.poses[-1].pose.position.x == 100.0
    assert plant.twists[-1].twist.linear.x == 0.0
    assert plant.twists[-1].header.stamp == Time(sec=0, nanosec=520_000_000)


@pytest.mark.parametrize('roll,pitch,yaw', [
    (0.3, -0.4, 0.7), (0.0, 0.0, math.pi),
    (math.pi, 0.2, 0.0), (0.0, math.pi, 0.0), (0.2, 2.0, 0.4),
])
def test_full_native_orientation_reconstructs_sensor_axes(plant, roll, pitch, yaw):
    plant.step()
    plant.step(0.0, 0.0, yaw, epoch=1.02, z=0.23, roll=roll, pitch=pitch)
    pose = plant.poses[-1].pose
    q = pose.orientation
    # Rotate the three coordinate basis vectors by the published quaternion.
    actual = [1-2*(q.y*q.y+q.z*q.z), 2*(q.x*q.y-q.z*q.w), 2*(q.x*q.z+q.y*q.w),
              2*(q.x*q.y+q.z*q.w), 1-2*(q.x*q.x+q.z*q.z), 2*(q.y*q.z-q.x*q.w),
              2*(q.x*q.z-q.y*q.w), 2*(q.y*q.z+q.x*q.w), 1-2*(q.x*q.x+q.y*q.y)]
    assert actual == pytest.approx(rotation(roll, pitch, yaw), rel=0.0, abs=1e-15)
    assert pose.position.z == 0.23


def test_yaw_wrap_is_small_native_turn(plant):
    plant.step(yaw=math.pi-0.001)
    plant.step(yaw=-math.pi+0.003, epoch=1.02)
    assert plant.twists[-1].twist.angular.z == pytest.approx(0.2, rel=0.0, abs=1e-13)


def test_physical_axes_match_positive_forward_ros_joints():
    package = Path(__file__).resolve().parents[1]
    params = yaml.safe_load((package / 'config_webots' / 'ros2_control.yaml').read_text())
    params = params['diffdrive_controller']['ros__parameters']
    assert params['wheel_radius'] == 0.093
    assert params['wheel_separation'] == 0.325
    proto = (package / 'protos' / 'MowgliMower.proto').read_text()
    assert proto.count('          axis 0 1 0\n') == 2
    urdf = ET.parse(package / 'urdf_webots' / 'mowgli_webots.urdf')
    for name in ('left_wheel_motor', 'right_wheel_motor'):
        assert urdf.find(f"./joint[@name='{name}']/axis").attrib['xyz'] == '0 1 0'
    assert proto.count('resolution 0.001') == 2


def test_initial_invalid_pose_requires_baseline_and_new_interval(plant):
    plant.step(x=math.nan)
    assert plant.poses == plant.twists == []
    plant.step(12.0, -9.0, epoch=1.02)
    assert plant.poses == plant.twists == []
    plant.step(12.0, -9.0, epoch=1.04)
    assert plant.poses[-1].pose.position.x == 12.0
    assert plant.twists[-1].twist.linear.x == 0.0


def test_curved_slip_twist_reconstructs_native_interval(plant):
    plant.step(0.25, -0.5, 0.7)
    plant.step(0.256, -0.497, 0.73, epoch=1.02)
    twist = plant.twists[-1].twist
    # Numerical midpoint quadrature of the published moving-frame velocity,
    # independent of the implementation's inverse SE(2) Jacobian.
    n, dt = 10000, 0.02
    dx = dy = 0.0
    for i in range(n):
        heading = 0.7 + twist.angular.z * (i+0.5)*dt/n
        dx += (twist.linear.x*math.cos(heading)-twist.linear.y*math.sin(heading))*dt/n
        dy += (twist.linear.x*math.sin(heading)+twist.linear.y*math.cos(heading))*dt/n
    assert 0.25+dx == pytest.approx(0.256, rel=0.0, abs=1e-14)
    assert -0.5+dy == pytest.approx(-0.497, rel=0.0, abs=1e-14)
    assert 0.7+twist.angular.z*dt == pytest.approx(0.73, rel=0.0, abs=1e-15)


def test_finite_pose_overflow_cannot_publish_nonfinite_twist(plant):
    plant.step(-1e308)
    plant.step(1e308, epoch=1.02)
    assert plant.poses == plant.twists == []
    plant.step(1.0, epoch=1.04)
    assert plant.poses == plant.twists == []
    plant.step(1.0, epoch=1.06)
    assert plant.twists[-1].twist.linear.x == 0.0
    assert plant.writes == []
