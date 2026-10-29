# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later

"""Exercise the real plugin against native-step supervisor/device fakes."""

import importlib.util
import math
import os
import time
from pathlib import Path
from types import SimpleNamespace

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


class Field:
    def __init__(self, values):
        self.values = values

    def getSFVec3f(self):
        return self.values.copy()

    def getSFRotation(self):
        return self.values.copy()

    def setSFVec3f(self, values):
        self.values = values

    def setSFRotation(self, values):
        self.values = values


class Sensor:
    ROTATIONAL = 0

    def __init__(self):
        self.value = 0.0
        self.enabled = []

    def enable(self, timestep):
        self.enabled.append(timestep)

    def getValue(self):
        return self.value

    def getType(self):
        return self.ROTATIONAL


@pytest.fixture
def plant(monkeypatch):
    module = _load_module()
    fields = {
        'translation': Field([0.0, 0.0, 0.1]),
        'rotation': Field([0.0, 0.0, 1.0, 0.0]),
    }
    sensors = {'left_wheel_sensor': Sensor(), 'right_wheel_sensor': Sensor()}
    pubs, devices, velocities, resets, callbacks = {}, [], [], [], []
    clock = SimpleNamespace(native=1.0, wall=5.0)
    robot = SimpleNamespace(
        _ref=1, getField=fields.get, setVelocity=velocities.append,
        resetPhysics=lambda: resets.append(clock.native),
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
        get_logger=lambda: SimpleNamespace(info=lambda msg: None, warning=lambda msg: None),
    )
    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(module.rclpy, 'create_node', lambda name: node)
    monkeypatch.setattr(module.rclpy, 'spin_once', lambda *args, **kwargs: None)
    monkeypatch.setattr(time, 'monotonic', lambda: clock.wall)
    drive = module.KinematicDrive()
    drive.init(SimpleNamespace(robot=supervisor), {
        'cmdVelTopic': '/cmd_vel_wheels', 'applyFirmwareModel': 'false',
    })

    def command(vx, wz=0.0):
        msg = TwistStamped()
        msg.twist.linear.x, msg.twist.angular.z = vx, wz
        for callback in callbacks:
            callback(msg)

    command(0.215)

    def step(left, right, native=None):
        sensors['left_wheel_sensor'].value = left
        sensors['right_wheel_sensor'].value = right
        if native is not None:
            clock.native = native
        drive.step()

    return SimpleNamespace(
        drive=drive, step=step, command=command, sensors=sensors, pubs=pubs, fields=fields,
        devices=devices, velocities=velocities, resets=resets, clock=clock,
        poses=pubs['/sim/ground_truth_pose'], twists=pubs['/sim/applied_twist'],
    )


def test_post_model_step_follows_measured_limited_wheels(plant):
    plant.command(0.0)
    plant.step(0.0, 0.0)
    plant.command(0.215)
    plant.step(0.004, 0.004, 1.02)
    pose, applied = plant.poses[-1], plant.twists[-1]
    # Firmware model's 0.15 target initially yields 0.215 m/s; the actual
    # limited, quantized wheel acquisition is authoritative, not that command.
    assert pose.pose.position.x == 0.000372
    assert pose.pose.position.y == 0.0
    assert applied.twist.linear.x == pytest.approx(0.0186, rel=0.0, abs=1e-15)
    assert applied.twist.angular.z == 0.0
    assert pose.header.stamp.sec == 1
    assert pose.header.stamp.nanosec == 20_000_000
    assert applied.header.stamp == pose.header.stamp
    assert plant.devices == ['left_wheel_sensor', 'right_wheel_sensor']
    assert all(sensor.enabled == [20] for sensor in plant.sensors.values())
    assert plant.resets == [1.0]
    assert plant.velocities[-1][2:5] == [0.0, 0.0, 0.0]
    assert plant.fields['translation'].values[2] == 0.1


def test_unchanged_wheels_do_not_replay_pending_command(plant):
    plant.step(0.0, 0.0)
    plant.step(0.004, 0.004, 1.02)
    previous = plant.poses[-1].pose
    plant.step(0.004, 0.004, 1.04)
    assert plant.poses[-1].pose == previous
    assert plant.twists[-1].twist.linear.x == 0.0
    assert plant.twists[-1].twist.angular.z == 0.0


def test_initial_wheel_angle_is_a_baseline_not_travel(plant):
    plant.step(12.0, -9.0)
    assert plant.fields['translation'].values == [0.0, 0.0, 0.1]
    plant.step(12.004, -8.996, 1.02)
    assert plant.poses[-1].pose.position.x == pytest.approx(0.000372, rel=0.0, abs=1e-15)
    assert plant.poses[-1].pose.position.y == 0.0


@pytest.mark.parametrize('left,right', [(-0.2, 0.2), (0.1, 0.4), (1e-7, 1.1e-7)])
def test_wheel_increment_uses_exact_se2_arc(plant, left, right):
    plant.step(0.0, 0.0)
    plant.step(left, right, 1.02)
    distance = 0.093 * (left + right) / 2.0
    turn = 0.093 * (right - left) / 0.325
    if abs(turn) > 1e-6:
        # Independent circular-arc oracle (radius and change of heading).
        expected_x = (distance / turn) * math.sin(turn)
        expected_y = (distance / turn) * (1.0 - math.cos(turn))
    else:
        # Analytic small-angle limit avoids cancellation in 1-cos(turn).
        expected_x, expected_y = distance, distance * turn / 2.0
    assert plant.poses[-1].pose.position.x == pytest.approx(
        expected_x, rel=0.0, abs=1e-15,
    )
    assert plant.poses[-1].pose.position.y == pytest.approx(
        expected_y, rel=0.0, abs=1e-15,
    )
    assert plant.twists[-1].twist.angular.z == pytest.approx(turn / 0.02, rel=0.0, abs=1e-14)


def test_continuous_position_sensor_crossing_pi_is_not_wrapped(plant):
    plant.step(math.pi - 0.001, math.pi - 0.001)
    plant.step(math.pi + 0.003, math.pi + 0.003, 1.02)
    assert plant.poses[-1].pose.position.x == pytest.approx(0.000372, rel=0.0, abs=1e-15)


def test_same_native_epoch_cannot_replace_observation_on_wall_timeout(plant):
    plant.step(0.0, 0.0)
    plant.step(0.004, 0.004, 1.02)
    pose, twist = plant.poses[-1], plant.twists[-1]
    velocity = plant.velocities[-1]
    plant.clock.wall += 1.0  # The old plugin watchdog would now stop its body.
    plant.step(0.004, 0.004, 1.02)
    assert plant.poses == [pose]
    assert plant.twists == [twist]
    assert plant.velocities[-1] == velocity
    plant.step(0.004, 0.004, 1.04)
    assert plant.poses[-1].pose == pose.pose
    assert plant.twists[-1].twist.linear.x == 0.0


def test_expired_command_preserves_measured_braking_then_stationary_stop(plant):
    plant.step(0.0, 0.0)
    plant.step(0.004, 0.004, 1.02)
    plant.clock.wall += 1.0
    # A stale command does not make measured motion disappear. These actual
    # quantized increments decelerate within the unchanged 1 m/s² limit.
    plant.step(0.006, 0.006, 1.04)
    assert plant.poses[-1].pose.position.x == pytest.approx(0.000558, rel=0.0, abs=1e-15)
    assert plant.twists[-1].twist.linear.x == pytest.approx(0.0093, rel=0.0, abs=1e-15)
    assert abs(plant.twists[-1].twist.linear.x - plant.twists[-2].twist.linear.x) / 0.02 < 1.0
    stopped_pose = plant.poses[-1].pose
    plant.step(0.006, 0.006, 1.06)
    assert plant.twists[-1].twist.linear.x == plant.twists[-1].twist.angular.z == 0.0
    assert plant.poses[-1].pose == stopped_pose
    assert plant.velocities[-1] == [0.0] * 6


@pytest.mark.parametrize('bad_left,bad_right,bad_time', [
    (math.nan, 0.004, 1.04), (0.004, math.inf, 1.04),
    (0.004, 0.004, math.nan), (0.004, 0.004, -1.0),
])
def test_invalid_acquisition_holds_without_packet_then_rebaselines(
    plant, bad_left, bad_right, bad_time,
):
    plant.step(0.0, 0.0)
    plant.step(0.004, 0.004, 1.02)
    previous_pose = plant.poses[-1].pose
    plant.step(bad_left, bad_right, bad_time)
    assert len(plant.poses) == len(plant.twists) == 1
    assert plant.velocities[-1] == [0.0] * 6
    assert plant.fields['translation'].values == [0.000372, 0.0, 0.1]
    plant.step(100.0, -100.0, 1.06)
    assert len(plant.poses) == len(plant.twists) == 1
    plant.step(100.004, -99.996, 1.08)
    assert plant.poses[-1].pose.position.x == pytest.approx(
        previous_pose.position.x + 0.000372, rel=0.0, abs=1e-15,
    )
    assert plant.poses[-1].pose.position.y == 0.0
    assert plant.resets == [1.0]


def test_native_rewind_rebaselines_without_replaying_wheel_offset(plant):
    plant.step(0.0, 0.0)
    plant.step(0.004, 0.004, 1.02)
    plant.step(100.0, -100.0, 0.5)
    assert len(plant.poses) == len(plant.twists) == 1
    plant.step(100.004, -99.996, 0.52)
    assert plant.poses[-1].pose.position.x == pytest.approx(0.000744, rel=0.0, abs=1e-15)
    assert plant.twists[-1].header.stamp.sec == 0
    assert plant.twists[-1].header.stamp.nanosec == 520_000_000


def test_sensor_geometry_agrees_with_shipped_controller_and_proto():
    package = Path(__file__).resolve().parents[1]
    params = yaml.safe_load((package / 'config_webots' / 'ros2_control.yaml').read_text())
    params = params['diffdrive_controller']['ros__parameters']
    module = _load_module()
    assert module.KinematicDrive.WHEEL_RADIUS == params['wheel_radius'] == 0.093
    assert module.KinematicDrive.WHEEL_SEPARATION == params['wheel_separation'] == 0.325
    proto = (package / 'protos' / 'MowgliMower.proto').read_text()
    assert proto.count('axis 0 -1 0') == 2
    assert proto.count('resolution 0.001') == 2


def test_translation_after_pivot_uses_current_world_heading(plant):
    plant.step(0.0, 0.0)
    plant.step(-0.2, 0.2, 1.02)
    heading = 0.4 * 0.093 / 0.325
    plant.step(-0.196, 0.204, 1.04)
    assert plant.poses[-1].pose.position.x == pytest.approx(
        0.000372 * math.cos(heading), rel=0.0, abs=1e-15,
    )
    assert plant.poses[-1].pose.position.y == pytest.approx(
        0.000372 * math.sin(heading), rel=0.0, abs=1e-15,
    )
    assert plant.velocities[-1][0] == pytest.approx(
        0.0186 * math.cos(heading), rel=0.0, abs=1e-15,
    )
    assert plant.velocities[-1][1] == pytest.approx(
        0.0186 * math.sin(heading), rel=0.0, abs=1e-15,
    )


def test_initial_nonfinite_sensor_needs_baseline_and_next_interval(plant):
    plant.step(math.nan, 0.0)
    assert plant.poses == plant.twists == []
    plant.step(12.0, -9.0, 1.02)
    assert plant.poses == plant.twists == []
    plant.step(12.004, -8.996, 1.04)
    assert plant.poses[-1].pose.position.x == pytest.approx(0.000372, rel=0.0, abs=1e-15)
