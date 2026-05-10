"""Minimal u-blox GNSS backend bringup for Mowgli V1."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_dir = get_package_share_directory("mowgli_bringup")
    params_file = os.path.join(bringup_dir, "config", "ublox_gnss.yaml")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock when true.",
    )
    device_family_arg = DeclareLaunchArgument(
        "ublox_device_family",
        default_value="F9P",
        description="u-blox device family: F9P, F9R, or X20P.",
    )
    device_serial_string_arg = DeclareLaunchArgument(
        "ublox_device_serial_string",
        default_value="",
        description="Optional USB serial string to select a specific receiver.",
    )
    frame_id_arg = DeclareLaunchArgument(
        "ublox_frame_id",
        default_value="gps_link",
        description="Frame ID reported by the vendor backend.",
    )
    config_file_arg = DeclareLaunchArgument(
        "ublox_config_file",
        default_value="",
        description="Optional explicit u-blox TOML config file path.",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    device_family = LaunchConfiguration("ublox_device_family")
    device_serial_string = LaunchConfiguration("ublox_device_serial_string")
    frame_id = LaunchConfiguration("ublox_frame_id")
    config_file = LaunchConfiguration("ublox_config_file")

    ublox_backend = Node(
        package="ublox_dgnss_node",
        executable="ublox_dgnss_node",
        name="ublox_dgnss",
        namespace="ublox",
        output="screen",
        parameters=[
            params_file,
            {
                "DEVICE_FAMILY": device_family,
                "DEVICE_SERIAL_STRING": device_serial_string,
                "FRAME_ID": frame_id,
                "UBX_CONFIG_FILE": config_file,
                "use_sim_time": use_sim_time,
            },
        ],
    )

    ublox_fix_source = Node(
        package="ublox_nav_sat_fix_hp_node",
        executable="ublox_nav_sat_fix_hp",
        name="ublox_nav_sat_fix_hp",
        namespace="ublox",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    gnss_adapter = Node(
        package="mowgli_localization",
        executable="ublox_gnss_adapter_node",
        name="ublox_gnss_adapter",
        output="screen",
        parameters=[
            params_file,
            {
                "device_family": device_family,
                "frame_id_hint": frame_id,
                "use_sim_time": use_sim_time,
            },
        ],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            device_family_arg,
            device_serial_string_arg,
            frame_id_arg,
            config_file_arg,
            ublox_backend,
            ublox_fix_source,
            gnss_adapter,
        ]
    )
