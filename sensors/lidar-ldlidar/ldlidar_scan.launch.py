# Custom launch for Myzhar ldrobot-lidar-ros2 in MowgliNext.
#
# Wraps the lidar component in a ComposableNodeContainer so we can remap
# the internal ~/scan topic to /scan (which all downstream nodes —
# slam_toolbox, Nav2 costmaps — expect). The bundled ldlidar_with_mgr
# launch publishes on /<node_name>/scan, which would force us to remap
# every consumer.
#
# Auto-configures and activates the lifecycle node via
# nav2_lifecycle_manager so the container can just be `docker run`.
#
# Also starts mowgli_lidar_pwm (issue #569) — the motor spin-down/up
# controller — gated by the lidar_pwm_enabled launch argument that
# start_lidar.sh resolves from mowgli_robot.yaml. With it false (the
# default), lidar_pwm_node still starts but does nothing (see the node's
# own enabled_ check) — kept unconditional rather than IfCondition-gated so
# the container's process set doesn't change shape based on a setting, which
# would complicate log-following and health checks for no real benefit.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description() -> LaunchDescription:
    params_file = "/ldlidar.yaml"
    if not os.path.isfile(params_file):
        params_file = os.path.join(
            get_package_share_directory("ldlidar_node"),
            "params",
            "ldlidar.yaml",
        )

    lidar_pwm_enabled = DeclareLaunchArgument(
        "lidar_pwm_enabled",
        default_value="false",
        description="Enable LD19 motor PWM spin-down control (issue #569)",
    )
    lidar_pwm_gpio_pin = DeclareLaunchArgument(
        "lidar_pwm_gpio_pin",
        default_value="12",
        description="GPIO pin (BCM numbering) driving the LD19 PWM control line",
    )

    lidar_component = ComposableNode(
        package="ldlidar_component",
        plugin="ldlidar::LdLidarComponent",
        name="ldlidar_node",
        namespace="",
        # bond_heartbeat_period: the driver's side of its lifecycle bond. Left at
        # bondcpp's 0.1 s default it publishes 10 Hz on the shared /bond topic —
        # which EVERY bonded node in the system receives and deserialises. The
        # Nav2 group already sets 0.5 s on its managed nodes; this matches it.
        # (The manager's own side is hardcoded to 0.10 s in nav2's
        # lifecycle_manager.cpp and cannot be configured.)
        parameters=[params_file, {"bond_heartbeat_period": 0.5}],
        remappings=[
            ("~/scan", "/scan"),
        ],
    )

    container = ComposableNodeContainer(
        name="ldlidar_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_isolated",
        composable_node_descriptions=[lidar_component],
        output="screen",
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="ldlidar_lifecycle_manager",
        output="screen",
        parameters=[{
            "autostart": True,
            "node_names": ["ldlidar_node"],
            "bond_timeout": 10.0,
        }],
    )

    lidar_pwm = Node(
        package="mowgli_lidar_pwm",
        executable="lidar_pwm_node",
        name="lidar_pwm_node",
        output="screen",
        parameters=[{
            "lidar_pwm_enabled": LaunchConfiguration("lidar_pwm_enabled"),
            "lidar_pwm_gpio_pin": LaunchConfiguration("lidar_pwm_gpio_pin"),
        }],
        remappings=[
            ("~/status", "/lidar_pwm/status"),
        ],
    )

    return LaunchDescription([
        lidar_pwm_enabled,
        lidar_pwm_gpio_pin,
        container,
        lifecycle_manager,
        lidar_pwm,
    ])
