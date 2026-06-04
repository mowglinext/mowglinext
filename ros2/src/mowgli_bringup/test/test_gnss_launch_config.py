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
