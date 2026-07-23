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


"""
test_nodes_startup.launch.py

launch_testing integration test that verifies all Mowgli nodes start
and advertise their expected topics.

Nodes under test (no hardware required, use_sim_time=false):
  - wheel_odometry_node       (mowgli_localization)
  - localization_monitor_node (mowgli_localization)
  - map_server_node           (mowgli_map)
  - diagnostics_node          (mowgli_monitoring)
  - behavior_tree_node        (mowgli_behavior)

Test cases:
  - test_all_nodes_alive     – every process is still running after 5 s
  - test_expected_topics     – key topics are advertised
  - test_expected_services   – key services are reachable
"""

import time
import unittest

import launch
import launch.events.process
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy


# ---------------------------------------------------------------------------
# Launch description
# ---------------------------------------------------------------------------

@pytest.mark.launch_test
def generate_test_description():
    wheel_odometry = launch_ros.actions.Node(
        package="mowgli_localization",
        executable="wheel_odometry_node",
        name="wheel_odometry",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    localization_monitor = launch_ros.actions.Node(
        package="mowgli_localization",
        executable="localization_monitor_node",
        name="localization_monitor",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    map_server = launch_ros.actions.Node(
        package="mowgli_map",
        executable="map_server_node",
        name="map_server",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    diagnostics = launch_ros.actions.Node(
        package="mowgli_monitoring",
        executable="diagnostics_node",
        name="diagnostics_node",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    behavior_tree = launch_ros.actions.Node(
        package="mowgli_behavior",
        executable="behavior_tree_node",
        name="mowgli_behavior_node",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    return (
        launch.LaunchDescription(
            [
                wheel_odometry,
                localization_monitor,
                map_server,
                diagnostics,
                behavior_tree,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {
            "wheel_odometry": wheel_odometry,

            "localization_monitor": localization_monitor,
            "map_server": map_server,
            "diagnostics": diagnostics,
            "behavior_tree": behavior_tree,
        },
    )


# ---------------------------------------------------------------------------
# Active-state tests (run while all processes are alive)
# ---------------------------------------------------------------------------

class TestNodesStartup(unittest.TestCase):
    """Verify that every Mowgli node launches and exposes its public API."""

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()
        cls.node = rclpy.create_node("test_nodes_startup_helper")

    @classmethod
    def tearDownClass(cls) -> None:
        cls.node.destroy_node()
        rclpy.shutdown()

    # ------------------------------------------------------------------
    # test_all_nodes_alive
    # ------------------------------------------------------------------

    def test_all_nodes_alive(
        self,
        wheel_odometry,

        localization_monitor,
        map_server,
        diagnostics,
        behavior_tree,
        proc_info,
    ) -> None:
        """Every launched process must still be running after a 5-second soak."""
        time.sleep(5.0)

        for label, proc in [
            ("wheel_odometry_node", wheel_odometry),

            ("localization_monitor_node", localization_monitor),
            ("map_server_node", map_server),
            ("diagnostics_node", diagnostics),
            ("behavior_tree_node", behavior_tree),
        ]:
            with self.subTest(node=label):
                info = proc_info[proc]
                is_exited = isinstance(
                    info, launch.events.process.ProcessExited
                )
                if is_exited:
                    self.fail(
                        f"{label} has already exited (returncode="
                        f"{info.returncode})"
                    )

    # ------------------------------------------------------------------
    # test_expected_topics
    # ------------------------------------------------------------------

    def test_expected_topics(self) -> None:
        """Key topics must be advertised within 10 seconds of startup."""
        required_topics = {
            "/wheel_odom",
            "/diagnostics",
            "/mowgli_behavior_node/high_level_status",
        }

        deadline = time.monotonic() + 10.0
        found: set[str] = set()

        while time.monotonic() < deadline and found != required_topics:
            topic_names_and_types = self.node.get_topic_names_and_types()
            found = {name for name, _ in topic_names_and_types} & required_topics
            if found != required_topics:
                time.sleep(0.25)

        missing = required_topics - found
        self.assertFalse(
            missing,
            msg=f"The following topics were not advertised within 10 s: {missing}",
        )

    # ------------------------------------------------------------------
    # test_expected_services
    # ------------------------------------------------------------------

    def test_expected_services(self) -> None:
        """Key services must be discoverable within 10 seconds of startup."""
        required_services = {
            "/map_server/save_map",
            "/map_server/clear_map",
            "/mowgli_behavior_node/high_level_control",
            "/map_server/add_area",
        }

        deadline = time.monotonic() + 10.0
        found: set[str] = set()

        while time.monotonic() < deadline and found != required_services:
            service_names_and_types = self.node.get_service_names_and_types()
            found = {name for name, _ in service_names_and_types} & required_services
            if found != required_services:
                time.sleep(0.25)

        missing = required_services - found
        self.assertFalse(
            missing,
            msg=f"The following services were not advertised within 10 s: {missing}",
        )


# ---------------------------------------------------------------------------
# Post-shutdown tests (run after all processes have been asked to exit)
# ---------------------------------------------------------------------------

@launch_testing.post_shutdown_test()
class TestNodesShutdown(unittest.TestCase):
    """Verify that every Mowgli process exited cleanly (exit code 0)."""

    def test_exit_codes(
        self,
        proc_info,
        wheel_odometry,

        localization_monitor,
        map_server,
        diagnostics,
        behavior_tree,
    ) -> None:
        for label, proc in [
            ("wheel_odometry_node", wheel_odometry),

            ("localization_monitor_node", localization_monitor),
            ("map_server_node", map_server),
            ("diagnostics_node", diagnostics),
            ("behavior_tree_node", behavior_tree),
        ]:
            with self.subTest(node=label):
                launch_testing.asserts.assertExitCodes(
                    proc_info,
                    process=proc,
                    allowable_exit_codes=[0],
                )
