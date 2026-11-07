# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later

"""Exercise producer shutdown races with real ROS contexts and publishers."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped
import pytest
import rclpy
from rclpy.exceptions import InvalidHandle
from rclpy.publisher import Publisher


@pytest.fixture
def producer(monkeypatch):
    """Use real ROS publishers with only the native acquisition API replaced."""
    path = Path(__file__).resolve().parents[1] / 'mowgli_simulation/kinematic_drive.py'
    spec = importlib.util.spec_from_file_location('native_truth_shutdown', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    rclpy.init(args=[])
    state = SimpleNamespace(time=1.0, x=0.0, position_reads=0, on_position=lambda: None)

    def position():
        state.position_reads += 1
        state.on_position()
        return [state.x, 0.0, 0.1]

    body = SimpleNamespace(
        _ref=1, getPosition=position,
        getOrientation=lambda: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
    )
    robot = SimpleNamespace(getSelf=lambda: body, getName=lambda: 'NativeTruthShutdown',
                            getTime=lambda: state.time)
    drive = module.KinematicDrive()
    drive.init(SimpleNamespace(robot=robot), {})
    node = drive._KinematicDrive__node
    packets = []
    after_publish = SimpleNamespace(callback=lambda: None)
    real_publish = Publisher.publish

    def publish(publisher, message):
        result = real_publish(publisher, message)
        packets.append(message)
        after_publish.callback()
        return result

    monkeypatch.setattr(Publisher, 'publish', publish)
    drive.step()
    assert not packets
    state.time, state.x = 1.02, 0.004
    yield SimpleNamespace(drive=drive, node=node, state=state, packets=packets,
                          after_publish=after_publish)
    if node.context.ok():
        node.destroy_node()
    rclpy.try_shutdown()


@pytest.mark.parametrize('stage', ['before_step', 'during_acquisition', 'between_publishes'])
def test_shutdown_does_not_escape_the_native_step(producer, stage):
    """A shutdown can happen after the entry check or first successful publish."""
    p = producer
    context = p.node.context
    if stage == 'before_step':
        context.shutdown()
    elif stage == 'during_acquisition':
        p.state.on_position = context.shutdown
    else:
        p.after_publish.callback = context.shutdown

    p.drive.step()

    assert not context.ok()
    assert len(p.packets) == (1 if stage == 'between_publishes' else 0)
    if p.packets:
        assert isinstance(p.packets[0], PoseStamped)
        assert p.packets[0].pose.position.x == 0.004
        assert p.packets[0].header.stamp.nanosec == 20_000_000
    if stage == 'before_step':
        assert p.state.position_reads == 1
    # No pair atomicity is claimed across context shutdown. No later acquisition
    # or packet is attempted once this context has ended.
    previous_reads = p.state.position_reads
    p.drive.step()
    assert p.state.position_reads == previous_reads


def test_live_destroyed_publisher_is_still_an_error(producer):
    """A broken live publisher must not be misclassified as context shutdown."""
    producer.drive._KinematicDrive__gt_pose_pub.destroy()
    with pytest.raises(InvalidHandle):
        producer.drive.step()
    assert producer.node.context.ok()


def test_live_publish_runtime_error_is_preserved(producer, monkeypatch):
    """Only an ended ROS context justifies swallowing a publish RuntimeError."""
    error = RuntimeError('live publisher failure')

    def fail(_publisher, _message):
        raise error

    monkeypatch.setattr(Publisher, 'publish', fail)
    with pytest.raises(RuntimeError) as caught:
        producer.drive.step()
    assert caught.value is error
    assert producer.node.context.ok()


def test_native_acquisition_error_is_never_a_publish_shutdown(producer):
    """An acquisition fault propagates even if it coincides with shutdown."""
    error = RuntimeError('native acquisition failure')

    def fail():
        producer.node.context.shutdown()
        raise error

    producer.state.on_position = fail
    with pytest.raises(RuntimeError) as caught:
        producer.drive.step()
    assert caught.value is error


def test_normal_packets_keep_their_epoch_and_interval(producer):
    """The lifecycle fix does not change normal source identity or values."""
    producer.drive.step()
    pose, twist = producer.packets
    assert pose.header.stamp == twist.header.stamp
    assert (pose.header.stamp.sec, pose.header.stamp.nanosec) == (1, 20_000_000)
    assert pose.pose.position.x == 0.004
    assert twist.twist.linear.x == 0.2
    assert twist.twist.linear.y == twist.twist.angular.z == 0.0
    producer.drive.step()
    assert len(producer.packets) == 2
