# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0

import importlib.util
from pathlib import Path


def _load_module(filename: str, module_name: str):
    here = Path(__file__).resolve().parent
    path = here.parent / "launch" / filename
    spec = importlib.util.spec_from_file_location(module_name, path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_universal_wrapper_uses_gps_status_topic_in_universal_mode(monkeypatch) -> None:
    launch_module = _load_module("universal_gnss.launch.py", "universal_gnss_launch")
    monkeypatch.setenv("GNSS_STATUS_SOURCE", "universal")
    assert launch_module._default_status_topic() == "/gps/status"


def test_universal_wrapper_keeps_legacy_status_topic_outside_universal_mode(monkeypatch) -> None:
    launch_module = _load_module("universal_gnss.launch.py", "universal_gnss_launch_legacy")
    monkeypatch.setenv("GNSS_STATUS_SOURCE", "mowgli_local")
    assert launch_module._default_status_topic() == "/status"


def test_universal_wrapper_prefers_new_env_contract(monkeypatch) -> None:
    launch_module = _load_module("universal_gnss.launch.py", "universal_gnss_launch_env")
    monkeypatch.setenv("GNSS_RECEIVER_FAMILY", "unicore")
    monkeypatch.setenv("GNSS_SERIAL_DEVICE", "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")
    monkeypatch.setenv("GNSS_SERIAL_BAUD", "921600")
    monkeypatch.setenv("GNSS_TRANSPORT", "serial")
    monkeypatch.setenv("GNSS_NTRIP_ENABLED", "true")
    monkeypatch.setenv("GNSS_NTRIP_HOST", "rtk.local")
    monkeypatch.setenv("GNSS_NTRIP_PORT", "2102")
    monkeypatch.setenv("GNSS_NTRIP_MOUNTPOINT", "FIELD1")
    monkeypatch.setenv("GNSS_NTRIP_USERNAME", "operator")
    monkeypatch.setenv("GNSS_NTRIP_PASSWORD", "secret")

    robot_params = {
        "gps_protocol": "UBX",
        "gps_port": "/dev/gps",
        "gps_baudrate": 460800,
        "ntrip_enabled": False,
        "ntrip_host": "yaml-host",
        "ntrip_port": 2101,
        "ntrip_mountpoint": "YAML",
        "ntrip_user": "yaml-user",
        "ntrip_password": "yaml-password",
    }

    assert launch_module._default_receiver_family(robot_params) == "unicore"
    assert (
        launch_module._default_serial_device(robot_params)
        == "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
    )
    assert launch_module._default_serial_baud(robot_params) == "921600"
    assert launch_module._default_transport() == "serial"
    assert launch_module._default_ntrip_enabled(robot_params) == "true"
    assert launch_module._default_ntrip_host(robot_params) == "rtk.local"
    assert launch_module._default_ntrip_port(robot_params) == "2102"
    assert launch_module._default_ntrip_mountpoint(robot_params) == "FIELD1"
    assert launch_module._default_ntrip_username(robot_params) == "operator"
    assert launch_module._default_ntrip_password(robot_params) == "secret"


def test_universal_wrapper_prefers_gps_by_id_for_usb_fallback(monkeypatch) -> None:
    launch_module = _load_module("universal_gnss.launch.py", "universal_gnss_launch_usb_fallback")
    monkeypatch.delenv("GNSS_SERIAL_DEVICE", raising=False)
    monkeypatch.setenv("GPS_CONNECTION", "usb")
    monkeypatch.setenv("GPS_BY_ID", "/dev/serial/by-id/usb-u-blox_GNSS_receiver-if00")
    monkeypatch.setenv("GPS_PORT", "/dev/gps")

    robot_params = {
        "gps_port": "/dev/gps",
    }

    assert launch_module._default_serial_device(robot_params) == "/dev/serial/by-id/usb-u-blox_GNSS_receiver-if00"


def test_full_system_disables_local_status_when_universal_selected() -> None:
    launch_module = _load_module("full_system.launch.py", "full_system_launch")
    assert launch_module._local_gnss_status_enabled("mowgli_local") is True
    assert launch_module._local_gnss_status_enabled("universal") is False
    assert launch_module._local_gnss_status_enabled("external") is False
    assert launch_module._local_gnss_status_enabled("off") is False


def test_sim_full_system_matches_runtime_status_switch() -> None:
    launch_module = _load_module("sim_full_system.launch.py", "sim_full_system_launch")
    assert launch_module._local_gnss_status_enabled("gps") is True
    assert launch_module._local_gnss_status_enabled("universal") is False
