# Copyright 2026 Mowgli Project
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

"""Restart the real behavior process with fake map/Nav2, without hardware."""

import os
from pathlib import Path
import subprocess
import tempfile
import threading
import time
import unittest

from geometry_msgs.msg import Point32, PoseStamped, TransformStamped
import launch
import launch_testing.actions
import launch_testing.markers
from mowgli_interfaces.action import PlanCoverage
from mowgli_interfaces.msg import GnssFusionUpdate, GnssStatus
from mowgli_interfaces.srv import GetMowingArea, MowerControl, StartInArea
from nav2_msgs.action import FollowPath, NavigateToPose
from nav_msgs.msg import Path as NavPath
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool
from tf2_ros import StaticTransformBroadcaster


@launch_testing.markers.keep_alive
def generate_test_description():
    # Own both subprocesses so the crash occurs at the checkpoint boundary.
    return launch.LaunchDescription([launch_testing.actions.ReadyToTest()])


def straight_unit(start, end):
    path = NavPath()
    path.header.frame_id = 'map'
    for index in range(201):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = start + (end - start) * index / 200
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    return path


class TestCoverageProcessRestart(unittest.TestCase):
    """Cover checkpoint reload and next-unit dispatch across separate processes."""

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('coverage_restart_fakes')
        self.work = tempfile.TemporaryDirectory(prefix='coverage_restart_')
        self.checkpoint = Path(self.work.name) / 'coverage_resume.txt'
        self.tree = Path(self.work.name) / 'test_tree.xml'
        # Exercise production coverage nodes, not physical guards/actuators.
        # The default production tree is untouched; all endpoints here are fake.
        self.tree.write_text("""<root BTCPP_format="4"><BehaviorTree ID="Test">
          <Sequence>
            <IsCommand command="1"/>
            <GetNextUnmowedArea area_index="{area}"/>
            <PlanCoverageArea area_index="{area}"/>
            <FollowStrip/>
          </Sequence>
        </BehaviorTree></root>""")
        self.units = [straight_unit(0.0, 10.0), straight_unit(10.0, 20.0)]
        self.processes = []
        self.follow_goals = []
        self.transit_goals = []
        self.resume_values = []
        self.stopping = threading.Event()
        self.precision_enabled = threading.Event()
        self.observation_sequence = 0
        self.lock = threading.Lock()
        group = ReentrantCallbackGroup()
        self.map_service = self.node.create_service(
            GetMowingArea,
            '/map_server_node/get_mowing_area',
            self.map_area,
            callback_group=group,
        )
        self.blade_service = self.node.create_service(
            MowerControl,
            '/hardware_bridge/mower_control',
            self.blade_control,
            callback_group=group,
        )
        self.planner = ActionServer(
            self.node,
            PlanCoverage,
            '/plan_coverage',
            self.plan,
            callback_group=group,
        )
        self.follow = ActionServer(
            self.node,
            FollowPath,
            '/follow_path',
            self.execute_follow,
            callback_group=group,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )
        self.transit = ActionServer(
            self.node,
            NavigateToPose,
            '/navigate_to_pose',
            self.execute_transit,
            callback_group=group,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )
        self.start_client = self.node.create_client(
            StartInArea,
            '/mowgli_behavior_node/start_in_area',
            callback_group=group,
        )
        self.tf = StaticTransformBroadcaster(self.node)
        transform = TransformStamped()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_footprint'
        transform.transform.rotation.w = 1.0
        self.tf.sendTransform(transform)
        self.fix_pub = self.node.create_publisher(NavSatFix, '/gps/fix', 10)
        self.status_pub = self.node.create_publisher(GnssStatus, '/gps/status', 10)
        self.support_pub = self.node.create_publisher(
            GnssFusionUpdate,
            '/fusion_graph/gnss_update',
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.precision_timer = self.node.create_timer(0.1, self.publish_precision)
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self.spinner = threading.Thread(target=self.executor.spin)
        self.spinner.start()

    def tearDown(self):
        for process, output in self.processes:
            if process.poll() is None:
                process.kill()
            process.wait(timeout=10)
            output.close()
        self.stopping.set()
        self.executor.shutdown(timeout_sec=10)
        self.spinner.join(timeout=10)
        self.planner.destroy()
        self.follow.destroy()
        self.transit.destroy()
        self.node.destroy_node()
        rclpy.shutdown()
        self.work.cleanup()

    def publish_precision(self):
        # Supply the real gate's inputs; no production bypass is installed.
        # Keep source time behind receipt time without refreshing old evidence.
        if not self.precision_enabled.is_set():
            return
        self.observation_sequence += 1
        stamp = (self.node.get_clock().now() - Duration(nanoseconds=100_000_000)).to_msg()
        fix = NavSatFix()
        fix.header.stamp = stamp
        fix.header.frame_id = 'gps_link'
        fix.status.status = NavSatStatus.STATUS_GBAS_FIX
        fix.latitude = 48.0
        fix.longitude = 11.0
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        fix.position_covariance[0] = fix.position_covariance[4] = 0.000009
        status = GnssStatus()
        status.header = fix.header
        status.fix_valid = True
        status.fix_type = GnssStatus.FIX_TYPE_RTK_FIXED
        status.position_observation_sequence = self.observation_sequence
        status.source_id = 'restart-fixture'
        status.source_incarnation = 'receiver-1'
        support = GnssFusionUpdate()
        support.header.stamp = stamp
        support.header.frame_id = 'map'
        support.producer_incarnation = 'restart-fixture-fusion'
        support.graph_epoch = 1
        support.estimate_revision = self.observation_sequence
        support.applied_revision = self.observation_sequence
        support.applied_fix = fix
        support.has_applied_fix = True
        support.pose_published = True
        support.tf_support_stamp = stamp  # the fixture's static TF covers this epoch
        self.fix_pub.publish(fix)
        self.status_pub.publish(status)
        self.support_pub.publish(support)

    def map_area(self, request, response):
        response.success = request.index == 0
        response.area.id = 502
        response.area.name = 'test_lawn'
        response.area.area.points = [
            Point32(x=x, y=y)
            for x, y in ((-1.0, -1.0), (21.0, -1.0), (21.0, 1.0), (-1.0, 1.0))
        ]
        return response

    @staticmethod
    def blade_control(request, response):
        response.success = True
        return response

    def plan(self, handle):
        result = PlanCoverage.Result()
        result.success = True
        result.segments = self.units
        result.drivable_subpaths = self.units
        result.full_path.header.frame_id = 'map'
        result.full_path.poses = self.units[0].poses + self.units[1].poses
        result.swath_count = 2
        handle.succeed()
        return result

    def execute_goal(self, handle, goals, result_type):
        complete = threading.Event()
        with self.lock:
            goals.append((handle.request, complete))
        while not complete.wait(0.02):
            if handle.is_cancel_requested:
                handle.canceled()
                return result_type()
            if self.stopping.is_set():
                handle.abort()
                return result_type()
        handle.succeed()
        return result_type()

    def execute_follow(self, handle):
        return self.execute_goal(handle, self.follow_goals, FollowPath.Result)

    def execute_transit(self, handle):
        return self.execute_goal(handle, self.transit_goals, NavigateToPose.Result)

    def start_process(self):
        log = Path(self.work.name) / f'process_{len(self.processes)}.log'
        output = log.open('w')

        env = os.environ.copy()
        env['RCUTILS_LOGGING_BUFFERED_STREAM'] = '0'

        process = subprocess.Popen(
            [
                os.environ['MOWGLI_BEHAVIOR_EXECUTABLE'],
                '--ros-args',
                '-p',
                f'tree_file:={self.tree}',
                '-p',
                f'coverage_resume_path:={self.checkpoint}',
                '-p',
                'tick_rate:=20.0',
            ],
            stdout=output,
            stderr=subprocess.STDOUT,
            env=env,
        )
        self.processes.append((process, output))
        return process, log

    def wait_until(self, predicate, message, timeout=15):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.02)
        logs = '\n'.join(p.read_text() for p in Path(self.work.name).glob('*.log'))
        self.fail(f'{message}\n{logs}')

    def test_restart_between_completed_subpaths(self):
        first, _ = self.start_process()
        self.wait_until(self.start_client.service_is_ready, 'START service unavailable')
        request = StartInArea.Request()
        request.area = 0
        started = self.start_client.call_async(request)
        self.wait_until(started.done, 'START request did not finish')
        self.assertTrue(started.result().success)
        time.sleep(0.3)
        self.assertEqual(len(self.follow_goals), 0)  # missing precision still denies dispatch
        self.precision_enabled.set()
        self.wait_until(
            lambda: len(self.follow_goals) == 1, 'First subpath not dispatched'
        )
        self.assertEqual(self.follow_goals[0][0].path.poses, self.units[0].poses)
        self.follow_goals[0][1].set()
        self.wait_until(
            lambda: len(self.transit_goals) == 1, 'Transition not dispatched'
        )

        # Crash after completion is written and the cursor erased, while the
        # blade-off transit is held before the next unit starts.
        first.kill()
        first.wait(timeout=10)
        checkpoint = self.checkpoint.read_text()
        row = next(
            line.split()
            for line in checkpoint.splitlines()
            if line.startswith('area 0 ')
        )
        self.assertEqual(row[2], '502')
        self.assertEqual(row[5:], ['-1', 'completed', '0'])
        self.assertIn('current_command 1\n', checkpoint)
        self.assertIn('single_area_target 0\n', checkpoint)
        self.assertIn('single_area_target_id 502\n', checkpoint)

        # Subscribe only after the old process exits: the assertion must use
        # the restarted publisher, not a latched value received before the crash.
        self.resume_sub = self.node.create_subscription(
            Bool,
            '/mowgli_behavior_node/coverage_resume_available',
            lambda msg: self.resume_values.append(msg.data),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        second, log = self.start_process()  # no second START; test boot restoration
        self.assertNotEqual(first.pid, second.pid)
        self.wait_until(
            lambda: len(self.transit_goals) == 2, 'Restart did not resume next subpath'
        )
        self.assertEqual(len(self.follow_goals), 1)  # completed work was not repeated
        self.assertEqual(self.transit_goals[1][0].pose.pose.position.x, 10.0)
        self.wait_until(
            lambda: True in self.resume_values, 'GUI resume signal was false'
        )
        self.wait_until(
            lambda: (
                'resume cursor, 0 completed areas, 1 completed subpaths, auto_continue=true'
                in log.read_text()
            ),
            'Restart resume log was not emitted',
        )
        self.transit_goals[1][1].set()
        self.wait_until(
            lambda: len(self.follow_goals) == 2, 'Next subpath not followed'
        )
        self.assertEqual(self.follow_goals[1][0].path.poses, self.units[1].poses)
        self.assertIsNone(second.poll())
