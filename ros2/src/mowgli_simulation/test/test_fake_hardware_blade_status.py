# Copyright (C) 2026 MowgliNext contributors
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

"""Exercise the installed bridge's service and blade telemetry, without Webots."""

import os
from pathlib import Path
import subprocess
import time
import uuid

from ament_index_python.packages import get_package_prefix
from mowgli_interfaces.msg import Status
from mowgli_interfaces.srv import MowerControl
import rclpy
from rclpy.executors import SingleThreadedExecutor


def test_blade_status_follows_off_on_off_requests(tmp_path):
    executable = (
        Path(get_package_prefix('mowgli_simulation'))
        / 'lib/mowgli_simulation/fake_hardware_bridge_node'
    )
    context = rclpy.Context()
    domain_id = int(os.environ.get('ROS_DOMAIN_ID', '186'))
    rclpy.init(context=context, domain_id=domain_id)
    node = rclpy.create_node('fake_blade_status_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    endpoint_prefix = f'/fake_blade_status_test_{uuid.uuid4().hex}'
    status_topic = f'{endpoint_prefix}/status'
    mower_service = f'{endpoint_prefix}/mower_control'
    samples = []
    subscription = node.create_subscription(
        Status, status_topic, samples.append, 10
    )
    client = node.create_client(MowerControl, mower_service)
    environment = dict(os.environ, ROS_DOMAIN_ID=str(domain_id))
    with (tmp_path / 'bridge.log').open('w') as log:
        process = subprocess.Popen(
            [
                str(executable), '--ros-args',
                '-r', f'/hardware_bridge/status:={status_topic}',
                '-r', f'/hardware_bridge/mower_control:={mower_service}',
            ],
            env=environment, stdout=log, stderr=subprocess.STDOUT
        )
        try:
            assert client.wait_for_service(timeout_sec=5.0)
            for enabled in (False, True, False):
                request = MowerControl.Request()
                request.mow_enabled = int(enabled)
                future = client.call_async(request)
                executor.spin_until_future_complete(future, timeout_sec=3.0)
                assert future.done() and future.result().success
                samples.clear()
                deadline = time.monotonic() + 3.0
                while time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=0.1)
                    if samples and samples[-1].mow_enabled == enabled:
                        break
                assert samples, 'No bridge status after mower_control request'
                status = samples[-1]
                assert status.mow_enabled == enabled
                # MapServer requires fresh telemetry AND an active controller;
                # command intent or positive RPM alone must not enable stamping.
                assert (status.mower_esc_status != 0) == enabled
                if enabled:
                    assert status.mower_motor_rpm >= 1000.0
                else:
                    assert status.mower_motor_rpm == 0.0
                stamp_ns = (
                    status.blade_status_stamp.sec * 1_000_000_000
                    + status.blade_status_stamp.nanosec
                )
                assert stamp_ns > 0
                assert 0 <= node.get_clock().now().nanoseconds - stamp_ns < 1_000_000_000
                assert process.poll() is None
        finally:
            process.terminate()
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3.0)
            node.destroy_subscription(subscription)
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown(context=context)
