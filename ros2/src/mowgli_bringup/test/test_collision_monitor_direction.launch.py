# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
"""Real Collision Monitor command tests on private, non-actuation topics."""

import importlib.util
import math
from pathlib import Path
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from geometry_msgs.msg import Point32, PolygonStamped, TransformStamped, TwistStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster
import yaml

PACKAGE = Path(__file__).resolve().parent.parent
SPEC = importlib.util.spec_from_file_location(
    "collision_test_config", PACKAGE / "launch" / "robot_config_util.py")
UTIL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(UTIL)
GEOMETRIES = {"normal": (.53, -.17, .275, .01),
              "custom": (.8344, -.2544, .3544, .027)}
BASE_FRAME = "collision_test_base"


def parameters(name):
    base = yaml.safe_load((PACKAGE / "config" / "nav2_params_base.yaml").read_text())
    overlay = yaml.safe_load((PACKAGE / "config" / "nav2_params_lidar.yaml").read_text())
    params = UTIL.deep_merge(base, overlay)["collision_monitor"]["ros__parameters"]
    narrow = params["PolygonStopNarrow"]
    narrow.pop("points", None)
    narrow.update(UTIL.collision_monitor_velocity_polygons(*GEOMETRIES[name]))
    params.update(use_sim_time=False, base_frame_id=BASE_FRAME,
                  odom_frame_id="collision_test_odom",
                  cmd_vel_in_topic=f"/collision_test/{name}/input",
                  cmd_vel_out_topic=f"/collision_test/{name}/output")
    params["scan"]["topic"] = f"/collision_test/{name}/scan"
    params["FootprintApproach"]["footprint_topic"] = f"/collision_test/{name}/footprint"
    return params


@pytest.mark.launch_test
def generate_test_description():
    monitors = [launch_ros.actions.Node(
        package="nav2_collision_monitor", executable="collision_monitor",
        name=f"collision_test_{name}", parameters=[parameters(name)], output="screen")
        for name in GEOMETRIES]
    return launch.LaunchDescription(
        monitors + [launch_testing.actions.ReadyToTest()]), {"monitors": monitors}


class TestCollisionDirection(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("collision_direction_test_driver")
        cls.outputs = {name: [] for name in GEOMETRIES}
        cls.publishers = {}
        for name in GEOMETRIES:
            prefix = f"/collision_test/{name}"
            cls.publishers[name] = (
                cls.node.create_publisher(LaserScan, prefix + "/scan", 10),
                cls.node.create_publisher(PolygonStamped, prefix + "/footprint", 10),
                cls.node.create_publisher(TwistStamped, prefix + "/input", 10))
            cls.node.create_subscription(TwistStamped, prefix + "/output",
                lambda msg, key=name: cls.outputs[key].append(msg), 100)
        cls.broadcaster = StaticTransformBroadcaster(cls.node)
        transform = TransformStamped()
        transform.header.stamp = cls.node.get_clock().now().to_msg()
        transform.header.frame_id = "collision_test_odom"
        transform.child_frame_id = BASE_FRAME
        transform.transform.rotation.w = 1.0
        cls.broadcaster.sendTransform(transform)
        for name in GEOMETRIES:
            client = cls.node.create_client(ChangeState, f"/collision_test_{name}/change_state")
            if not client.wait_for_service(timeout_sec=10.0):
                raise RuntimeError(f"missing lifecycle service for {name}")
            for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
                request = ChangeState.Request()
                request.transition.id = transition
                future = client.call_async(request)
                rclpy.spin_until_future_complete(cls.node, future, timeout_sec=10.0)
                if not future.done() or not future.result().success:
                    raise RuntimeError(f"lifecycle transition failed for {name}")
            cls.node.destroy_client(client)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def publish_for(self, name, points, vx, wz, duration, scan_enabled=True, frame=BASE_FRAME):
        scan = LaserScan()
        scan.header.frame_id = frame
        scan.angle_min, scan.angle_max = -math.pi, math.pi
        scan.angle_increment = math.pi / 360
        scan.range_min, scan.range_max = .1, 12.0
        scan.ranges = [float("inf")] * 721
        for x, y in points:
            index = round((math.atan2(y, x) - scan.angle_min) / scan.angle_increment)
            scan.ranges[index] = math.hypot(x, y)
        front, rear, half, padding = GEOMETRIES[name]
        f, r, h = (float(f"{value:.3f}") for value in (front, rear, half))
        footprint = PolygonStamped()
        footprint.header.frame_id = BASE_FRAME
        footprint.polygon.points = [Point32(x=x, y=y, z=0.) for x, y in
            [(f + padding, h + padding), (f + padding, -h - padding),
             (r - padding, -h - padding), (r - padding, h + padding)]]
        deadline = time.monotonic() + duration
        input_stamps = set()
        scan_pub, footprint_pub, input_pub = self.publishers[name]
        while time.monotonic() < deadline:
            stamp = self.node.get_clock().now().to_msg()
            scan.header.stamp = footprint.header.stamp = stamp
            if scan_enabled:
                scan_pub.publish(scan)
            footprint_pub.publish(footprint)
            # Deliver current evidence before asking for a motion decision.
            rclpy.spin_once(self.node, timeout_sec=.01)
            command = TwistStamped()
            command.header.stamp, command.header.frame_id = stamp, BASE_FRAME
            command.twist.linear.x, command.twist.angular.z = vx, wz
            input_stamps.add(stamp.sec * 10**9 + stamp.nanosec)
            input_pub.publish(command)
            rclpy.spin_once(self.node, timeout_sec=.04)
        return input_stamps

    def check_motion(self, name, points, vx, wz, expected, **options):
        # Reset any prior stop/slowdown with fresh CLEAR evidence, then require
        # exact input-stamp-matched outputs; old zeros cannot satisfy a case.
        self.publish_for(name, [], .15, 0., .5)
        self.outputs[name].clear()
        stamps = self.publish_for(name, points, vx, wz, options.pop("duration", 1.0), **options)
        outputs = [msg for msg in self.outputs[name]
                   if msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec in stamps]
        self.assertGreaterEqual(len(outputs), 5, "no real monitored command evidence")
        # Ignore acquisition/transport settling at the beginning of the case.
        for msg in outputs[-5:]:
            self.assertAlmostEqual(msg.twist.linear.x, expected[0], delta=1e-15)
            self.assertAlmostEqual(msg.twist.angular.z, expected[1], delta=1e-15)

    def test_front_post_allows_reverse_but_stops_forward(self):
        front = [(.60, -.05), (.60, .05)]
        self.check_motion("normal", front, -.15, 0., (-.15, 0.))
        self.check_motion("normal", front, .15, 0., (0., 0.))

    def test_two_points_inside_padding_stop_reverse_and_rotation(self):
        points = [(.535, -.05), (.535, .05)]
        self.check_motion("normal", points, -.15, 0., (0., 0.))
        self.check_motion("normal", points, 0., .5, (0., 0.))

    def test_rear_thin_obstacle_stops_reverse(self):
        self.check_motion("normal", [(-.19, -.05), (-.19, .05)], -.15, 0., (0., 0.))

    def test_clear_front_post_allows_both_pure_turns(self):
        points = [(.64, -.05), (.64, .05)]
        self.check_motion("normal", points, 0., .5, (0., .5))
        self.check_motion("normal", points, 0., -.5, (0., -.5))

    def test_tiny_signed_commands_select_current_direction(self):
        points = [(.60, -.05), (.60, .05)]
        self.check_motion("normal", points, -1e-12, 0., (-1e-12, 0.))
        self.check_motion("normal", points, 1e-12, 0., (0., 0.))

    def test_nondefault_rounded_geometry_and_padding(self):
        points = [(.88, -.05), (.88, .05)]
        self.check_motion("custom", points, -.15, 0., (-.15, 0.))
        self.check_motion("custom", points, .15, 0., (0., 0.))
        self.check_motion("custom", [(.85, -.05), (.85, .05)], -.15, 0., (0., 0.))

    def test_corner_and_curved_sweeps_stop(self):
        self.check_motion("normal", [(.50, .295), (.51, .295)], 0., .5, (0., 0.))
        self.check_motion("normal", [(-.19, -.24), (-.19, -.25)], 0., -.5, (0., 0.))
        self.check_motion("normal", [(.50, .295), (.505, .29)], -.15, .5, (0., 0.))
        self.check_motion("normal", [(.655, -.05), (.655, .05)], .15, .5, (0., 0.))

    def test_missing_scan_transform_stops(self):
        self.check_motion("normal", [(.60, -.05), (.60, .05)], -.15, 0., (0., 0.),
                          duration=2.2, frame="collision_test_unconnected_scan")

    def test_stale_scan_stops(self):
        self.check_motion("normal", [], -.15, 0., (0., 0.), duration=2.2, scan_enabled=False)


@launch_testing.post_shutdown_test()
class TestCollisionShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info, monitors):
        for monitor in monitors:
            launch_testing.asserts.assertExitCodes(proc_info, process=monitor,
                                                  allowable_exit_codes=[0])
