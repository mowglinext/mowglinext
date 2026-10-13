"""Launch the UM960 GNSS/RTK node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_config = PathJoinSubstitution(
        [FindPackageShare("um960_gps_driver"), "config", "um960.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="YAML file with the um960_node parameters.",
            ),
            DeclareLaunchArgument("port", default_value="/dev/serial_rtk"),
            DeclareLaunchArgument("baud", default_value="115200"),
            DeclareLaunchArgument("frame_id", default_value="gps"),
            Node(
                package="um960_gps_driver",
                executable="um960_node",
                name="um960_node",
                output="screen",
                respawn=True,
                parameters=[
                    LaunchConfiguration("config_file"),
                    {
                        "port": LaunchConfiguration("port"),
                        "baud": LaunchConfiguration("baud"),
                        "frame_id": LaunchConfiguration("frame_id"),
                    },
                ],
            ),
        ]
    )
