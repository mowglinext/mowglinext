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
universal_gnss.launch.py

MowgliNext wrapper around Universal GNSS ROS 2 nodes.

This launch file maps the existing Mowgli GNSS runtime contract onto
`universal_gnss_ros2`:

  - GNSS backend / protocol / port / baud from docker/.env
  - NTRIP credentials from /ros2_ws/config/mowgli_robot.yaml
  - topic remaps onto the Mowgli GNSS graph

The wrapper intentionally keeps the receiver transport on the serial path.
USB-backed receivers are still represented as serial devices via
/dev/serial/by-id/... or the installer-managed /dev/gps symlink.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _bool_string(value) -> str:
    return "true" if bool(value) else "false"


def _normalize_text(value, default: str) -> str:
    if value is None:
        return default
    text = str(value).strip()
    return text if text else default


def _load_robot_params(bringup_dir: str) -> dict:
    runtime_config = "/ros2_ws/config/mowgli_robot.yaml"
    template_config = os.path.join(bringup_dir, "config", "mowgli_robot.yaml")
    config_path = runtime_config if os.path.isfile(runtime_config) else template_config

    if not os.path.isfile(config_path):
        return {}

    try:
        with open(config_path, "r") as config_file:
            config = yaml.safe_load(config_file) or {}
    except (OSError, yaml.YAMLError):
        return {}

    return config.get("mowgli", {}).get("ros__parameters", {})


def _normalized_status_source() -> str:
    return os.environ.get("GNSS_STATUS_SOURCE", "mowgli_local").strip().lower()


def _default_receiver_family(robot_params: dict) -> str:
    backend = _normalize_text(os.environ.get("GNSS_BACKEND"), "gps").lower()
    protocol = _normalize_text(
        os.environ.get("GPS_PROTOCOL"), str(robot_params.get("gps_protocol", "UBX"))
    ).lower()

    if backend == "unicore":
        return "unicore"
    if backend == "ublox":
        return "ublox"
    if protocol == "nmea":
        return "nmea"
    return "auto"


def _default_serial_device(robot_params: dict) -> str:
    gps_connection = _normalize_text(os.environ.get("GPS_CONNECTION"), "uart").lower()
    gps_by_id = _normalize_text(os.environ.get("GPS_BY_ID"), "")
    gps_port = _normalize_text(
        os.environ.get("GPS_PORT"), str(robot_params.get("gps_port", "/dev/gps"))
    )

    if gps_connection == "usb" and gps_by_id:
        return gps_by_id
    return gps_port


def _default_serial_baud(robot_params: dict) -> str:
    return _normalize_text(
        os.environ.get("GPS_BAUD"), str(robot_params.get("gps_baudrate", 921600))
    )


def _default_ntrip_enabled(robot_params: dict) -> str:
    return _bool_string(bool(robot_params.get("ntrip_enabled", False)))


def _default_ntrip_gga_enabled(robot_params: dict) -> str:
    mountpoint = _normalize_text(robot_params.get("ntrip_mountpoint"), "").upper()
    return _bool_string(mountpoint.startswith("NEAR"))


def _default_status_topic() -> str:
    return "/gps/status" if _normalized_status_source() == "universal" else "/status"


def generate_launch_description() -> LaunchDescription:
    bringup_dir = get_package_share_directory("mowgli_bringup")
    robot_params = _load_robot_params(bringup_dir)

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock when true.",
    )
    receiver_family_arg = DeclareLaunchArgument(
        "receiver_family",
        default_value=_default_receiver_family(robot_params),
        description="Universal GNSS receiver family: auto, nmea, ublox, unicore.",
    )
    transport_arg = DeclareLaunchArgument(
        "transport",
        default_value="serial",
        description="Universal GNSS transport type. Mowgli uses serial for UART and USB receivers.",
    )
    serial_device_arg = DeclareLaunchArgument(
        "serial_device",
        default_value=_default_serial_device(robot_params),
        description="Receiver device path for Universal GNSS.",
    )
    serial_baud_arg = DeclareLaunchArgument(
        "serial_baud",
        default_value=_default_serial_baud(robot_params),
        description="Receiver baud rate for Universal GNSS.",
    )
    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="gps_link",
        description="Frame id attached to Universal GNSS fix/status output.",
    )
    fix_topic_arg = DeclareLaunchArgument(
        "fix_topic",
        default_value="/gps/fix",
        description="Target NavSatFix topic for the Universal GNSS receiver.",
    )
    status_topic_arg = DeclareLaunchArgument(
        "status_topic",
        default_value=_default_status_topic(),
        description="Target typed status topic for the Universal GNSS receiver and NTRIP wrapper.",
    )
    diagnostics_topic_arg = DeclareLaunchArgument(
        "diagnostics_topic",
        default_value="/diagnostics",
        description="Target diagnostics topic for the Universal GNSS nodes.",
    )
    rtcm_topic_arg = DeclareLaunchArgument(
        "rtcm_topic",
        default_value="/rtcm",
        description="Target RTCM topic shared between the Universal GNSS nodes.",
    )
    ntrip_enabled_arg = DeclareLaunchArgument(
        "ntrip_enabled",
        default_value=_default_ntrip_enabled(robot_params),
        description="Launch the Universal GNSS NTRIP node when true.",
    )
    caster_host_arg = DeclareLaunchArgument(
        "caster_host",
        default_value=_normalize_text(robot_params.get("ntrip_host"), ""),
        description="NTRIP caster hostname.",
    )
    caster_port_arg = DeclareLaunchArgument(
        "caster_port",
        default_value=str(robot_params.get("ntrip_port", 2101)),
        description="NTRIP caster port.",
    )
    mountpoint_arg = DeclareLaunchArgument(
        "mountpoint",
        default_value=_normalize_text(robot_params.get("ntrip_mountpoint"), ""),
        description="NTRIP mountpoint.",
    )
    username_arg = DeclareLaunchArgument(
        "username",
        default_value=_normalize_text(robot_params.get("ntrip_user"), ""),
        description="NTRIP username.",
    )
    password_arg = DeclareLaunchArgument(
        "password",
        default_value=_normalize_text(robot_params.get("ntrip_password"), ""),
        description="NTRIP password.",
    )
    gga_enabled_arg = DeclareLaunchArgument(
        "gga_enabled",
        default_value=_default_ntrip_gga_enabled(robot_params),
        description="Enable GGA injection toward the NTRIP caster when true.",
    )
    gga_interval_s_arg = DeclareLaunchArgument(
        "gga_interval_s",
        default_value="10",
        description="GGA injection period in seconds when enabled.",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    receiver_family = LaunchConfiguration("receiver_family")
    transport = LaunchConfiguration("transport")
    serial_device = LaunchConfiguration("serial_device")
    serial_baud = LaunchConfiguration("serial_baud")
    frame_id = LaunchConfiguration("frame_id")
    fix_topic = LaunchConfiguration("fix_topic")
    status_topic = LaunchConfiguration("status_topic")
    diagnostics_topic = LaunchConfiguration("diagnostics_topic")
    rtcm_topic = LaunchConfiguration("rtcm_topic")
    ntrip_enabled = LaunchConfiguration("ntrip_enabled")
    caster_host = LaunchConfiguration("caster_host")
    caster_port = LaunchConfiguration("caster_port")
    mountpoint = LaunchConfiguration("mountpoint")
    username = LaunchConfiguration("username")
    password = LaunchConfiguration("password")
    gga_enabled = LaunchConfiguration("gga_enabled")
    gga_interval_s = LaunchConfiguration("gga_interval_s")

    receiver_node = Node(
        package="universal_gnss_ros2",
        executable="receiver_node",
        name="universal_gnss_receiver",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "receiver_family": receiver_family,
                "transport": transport,
                "serial_device": serial_device,
                "serial_baud": serial_baud,
                "publish_rate_hz": 5.0,
                "frame_id": frame_id,
            }
        ],
        remappings=[
            ("fix", fix_topic),
            ("status", status_topic),
            ("diagnostics", diagnostics_topic),
            ("rtcm", rtcm_topic),
        ],
    )

    ntrip_node = Node(
        condition=IfCondition(ntrip_enabled),
        package="universal_gnss_ros2",
        executable="ntrip_node",
        name="universal_gnss_ntrip",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "caster_host": caster_host,
                "caster_port": caster_port,
                "mountpoint": mountpoint,
                "username": username,
                "password": password,
                "gga_enabled": gga_enabled,
                "gga_interval_s": gga_interval_s,
                "tls_enabled": False,
            }
        ],
        remappings=[
            ("status", status_topic),
            ("diagnostics", diagnostics_topic),
            ("rtcm", rtcm_topic),
        ],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            receiver_family_arg,
            transport_arg,
            serial_device_arg,
            serial_baud_arg,
            frame_id_arg,
            fix_topic_arg,
            status_topic_arg,
            diagnostics_topic_arg,
            rtcm_topic_arg,
            ntrip_enabled_arg,
            caster_host_arg,
            caster_port_arg,
            mountpoint_arg,
            username_arg,
            password_arg,
            gga_enabled_arg,
            gga_interval_s_arg,
            receiver_node,
            ntrip_node,
        ]
    )
