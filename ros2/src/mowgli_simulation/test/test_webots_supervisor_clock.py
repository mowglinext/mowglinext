# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0

"""
Exercise the installed upstream clock callback, without starting Webots.

The simulator image applies the clock patch before installing the driver.
A ROS-only build can collect these tests but needs that optional driver to run.
"""

from types import SimpleNamespace

import pytest


@pytest.fixture
def clock_callback():
    module = pytest.importorskip('webots_ros2_driver.ros2_supervisor')
    # Avoid Node/Supervisor construction and native physics. The actual method
    # uses only step/getTime and the clock publisher on its successful path.
    node = object.__new__(module.Ros2Supervisor)
    state = SimpleNamespace(seconds=0.0, result=0, last=None, count=0)

    def publish(message):
        state.last = message.clock
        state.count += 1

    node._Ros2Supervisor__robot = SimpleNamespace(
        step=lambda timestep: state.result, getTime=lambda: state.seconds)
    node._Ros2Supervisor__timestep = 20
    node._Ros2Supervisor__clock_publisher = SimpleNamespace(publish=publish)
    node.get_logger = lambda: SimpleNamespace(info=lambda message: None)
    return node._Ros2Supervisor__supervisor_step_callback, state


@pytest.mark.parametrize('epoch_ns', [
    0, 1, 20_000_000, 1_234_567_890,
    65_740_000_000, 66_600_000_000, 130_260_000_000, 264_900_000_000,
])
def test_clock_preserves_representable_source_epoch(clock_callback, epoch_ns):
    callback, state = clock_callback
    state.seconds = epoch_ns / 1_000_000_000
    callback()
    assert state.count == 1
    assert state.last.sec * 1_000_000_000 + state.last.nanosec == epoch_ns


def test_every_twenty_millisecond_tick_through_native_mission_cap(clock_callback):
    callback, state = clock_callback
    for tick in range(30_001):
        state.seconds = tick * 20 / 1000
        callback()
        assert state.last.sec * 1_000_000_000 + state.last.nanosec == tick * 20_000_000


def test_pause_rewind_and_shutdown_keep_source_time_semantics(clock_callback):
    callback, state = clock_callback
    for epoch_ns in [66_600_000_000, 66_600_000_000, 0, 20_000_000]:
        state.seconds = epoch_ns / 1_000_000_000
        callback()
        assert state.last.sec * 1_000_000_000 + state.last.nanosec == epoch_ns
    before = state.count
    state.result = -1
    callback()
    assert state.count == before
