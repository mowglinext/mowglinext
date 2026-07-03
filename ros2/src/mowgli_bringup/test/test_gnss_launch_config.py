# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0

import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription


def _load_module(filename: str, module_name: str):
    here = Path(__file__).resolve().parent
    path = here.parent / "launch" / filename
    spec = importlib.util.spec_from_file_location(module_name, path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _read_launch_source(filename: str) -> str:
    here = Path(__file__).resolve().parent
    return (here.parent / "launch" / filename).read_text()


def test_full_system_show_args_no_longer_exposes_internal_universal_toggle() -> None:
    launch_module = _load_module("full_system.launch.py", "full_system_launch_args")
    launch_description = launch_module.generate_launch_description()

    declared_args = [
        entity.name
        for entity in launch_description.entities
        if isinstance(entity, DeclareLaunchArgument)
    ]

    assert "use_universal_gnss" not in declared_args


def test_full_system_no_longer_includes_internal_universal_launch() -> None:
    launch_module = _load_module("full_system.launch.py", "full_system_launch_includes")
    launch_description = launch_module.generate_launch_description()

    included_locations = [
        entity.launch_description_source.location
        for entity in launch_description.entities
        if isinstance(entity, IncludeLaunchDescription)
    ]

    assert all(not location.endswith("universal_gnss.launch.py") for location in included_locations)


def test_full_system_no_longer_passes_legacy_gnss_status_params() -> None:
    launch_source = _read_launch_source("full_system.launch.py")
    assert "publish_" "gnss_status" not in launch_source
    assert "gnss_" "backend" not in launch_source
    assert "gps_" "protocol" not in launch_source


def test_sim_full_system_no_longer_passes_legacy_gnss_status_params() -> None:
    launch_source = _read_launch_source("sim_full_system.launch.py")
    assert "publish_" "gnss_status" not in launch_source
