# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0

import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node


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
    assert all(
        not (
            isinstance(entity, Node) and entity.node_package == "universal_gnss_ros2"
        )
        for entity in launch_description.entities
    )


def _bridge_nodes(launch_description):
    return [
        entity
        for entity in launch_description.entities
        if isinstance(entity, Node)
        and entity.node_package == "mowgli_gnss_bridge"
        and entity.node_executable == "universal_gnss_topic_bridge"
    ]


def _load_full_system_with(monkeypatch, module_name: str, **overrides):
    """Load full_system.launch.py with robot-config keys overridden.

    The GNSS stack is chosen by the merged robot config, never by the process
    environment, so the tests drive it the same way: through load_robot_params.
    """
    launch_module = _load_module("full_system.launch.py", module_name)
    real_load = launch_module.load_robot_params

    def _patched(*args, **kwargs):
        return {**real_load(*args, **kwargs), **overrides}

    monkeypatch.setattr(launch_module, "load_robot_params", _patched)
    return launch_module


def test_full_system_launches_contract_bridge_only_for_universal_stack(monkeypatch) -> None:
    launch_module = _load_full_system_with(monkeypatch, "full_system_default_bridge")
    bridge_nodes = _bridge_nodes(launch_module.generate_launch_description())

    assert len(bridge_nodes) == 1

    launch_source = _read_launch_source("full_system.launch.py")
    assert '"backend": "universal"' in launch_source
    assert '"input_status_topic": "/universal_gnss_receiver/status"' in launch_source
    assert '"output_status_topic": "/gps/status"' in launch_source
    assert '"input_rtcm_topic": "/universal_gnss_receiver/rtcm"' in launch_source
    assert '"output_rtcm_topic": "/rtcm"' in launch_source

    launch_module = _load_full_system_with(
        monkeypatch, "full_system_disabled_bridge", gnss_stack="disabled"
    )
    assert not _bridge_nodes(launch_module.generate_launch_description())


def test_gnss_stack_is_not_read_from_the_environment(monkeypatch) -> None:
    # Same rule as lidar_enabled (root CLAUDE.md Invariant 15): an env var that
    # outranks the yaml silently overrides what the operator set in the GUI.
    monkeypatch.setenv("GNSS_STACK", "disabled")
    launch_module = _load_full_system_with(monkeypatch, "full_system_env_ignored")

    assert len(_bridge_nodes(launch_module.generate_launch_description())) == 1
    assert "os.environ" not in _read_launch_source("full_system.launch.py")


def test_universal_bridge_does_not_depend_on_hardware_backend(monkeypatch) -> None:
    monkeypatch.setenv("HARDWARE_BACKEND", "mavros")
    launch_module = _load_full_system_with(
        monkeypatch, "full_system_mavros_universal_bridge", gnss_stack="universal"
    )

    assert len(_bridge_nodes(launch_module.generate_launch_description())) == 1


def test_full_system_no_longer_passes_legacy_gnss_status_params() -> None:
    launch_source = _read_launch_source("full_system.launch.py")
    assert "publish_" "gnss_status" not in launch_source
    assert "gnss_" "backend" not in launch_source
    assert "gps_" "protocol" not in launch_source


def test_sim_full_system_no_longer_passes_legacy_gnss_status_params() -> None:
    launch_source = _read_launch_source("sim_full_system.launch.py")
    assert "publish_" "gnss_status" not in launch_source


def test_sim_uses_hardware_factor_graph_cadence() -> None:
    launch_source = _read_launch_source("sim_full_system.launch.py")
    navigation_args = launch_source.split("navigation_launch =", 1)[1].split(
        "behavior_tree_node =", 1
    )[0]

    assert '"fusion_graph_node_period_s": "0.04"' in navigation_args
