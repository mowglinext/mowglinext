# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later
"""Evaluate the real launch builders and ROS parameter substitutions, without nodes."""

import builtins
import importlib.util
import json
import math
import os
from pathlib import Path
import tempfile

import ament_index_python.packages as packages
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
import launch.launch_description_sources
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
import launch_ros.actions
from launch_ros.utilities import evaluate_parameters, normalize_parameters
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
FIELDS = ("datum_lat", "datum_lon", "lever_arm_x", "lever_arm_y")
GARDEN = dict(zip(FIELDS, (48.137154, 11.576124, 0.30, 0.0)))
SITE = dict(datum_lat=59.3, datum_lon=18.0, gps_x=0.17, gps_y=-0.12)


@pytest.fixture
def builders(monkeypatch, tmp_path):
    """Use source package shares and redirect only the installed site config."""
    runtime = tmp_path / "mowgli_robot.yaml"
    installed = "/ros2_ws/config/mowgli_robot.yaml"
    real_open, real_isfile, real_exists = builtins.open, os.path.isfile, os.path.exists
    monkeypatch.setattr(
        builtins,
        "open",
        lambda path, *a, **kw: real_open(
            runtime if str(path) == installed else path, *a, **kw
        ),
    )
    monkeypatch.setattr(
        os.path,
        "isfile",
        lambda path: real_isfile(runtime if str(path) == installed else path),
    )
    monkeypatch.setattr(
        os.path,
        "exists",
        lambda path: real_exists(runtime if str(path) == installed else path),
    )
    monkeypatch.setattr(
        packages, "get_package_share_directory", lambda name: str(ROOT / name)
    )
    monkeypatch.setattr(
        packages, "get_package_prefix", lambda name: str(tmp_path / name)
    )
    monkeypatch.setattr(tempfile, "tempdir", str(tmp_path))
    monkeypatch.setenv("HARDWARE_BACKEND", "mowgli")
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))

    class Node(launch_ros.actions.Node):
        def __init__(self, **kwargs):
            self.test_options = kwargs
            super().__init__(**kwargs)

    monkeypatch.setattr(launch_ros.actions, "Node", Node)

    class Source(launch.launch_description_sources.PythonLaunchDescriptionSource):
        def __init__(self, path):
            self.test_path = path
            super().__init__(path)

    monkeypatch.setattr(
        launch.launch_description_sources, "PythonLaunchDescriptionSource", Source
    )

    def build(relative, arguments=None, site=None, scenario=None):
        if site is not None:
            runtime.write_text(yaml.safe_dump({"mowgli": {"ros__parameters": site}}))
        if scenario is not None:
            # Redirect the scenario read without altering the source fixture.
            scenario_path = ROOT / "mowgli_simulation/config_webots/mowgli_garden.yaml"
            replacement = tmp_path / "scenario.yaml"
            replacement.write_text(yaml.safe_dump(scenario))
            monkeypatch.setattr(
                builtins,
                "open",
                lambda path, *a, **kw: real_open(
                    replacement
                    if str(path) == str(scenario_path)
                    else runtime
                    if str(path) == installed
                    else path,
                    *a,
                    **kw,
                ),
            )
        path = ROOT / relative
        spec = importlib.util.spec_from_file_location("geometry_launch_test", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        description = module.generate_launch_description()
        context = LaunchContext()
        context.launch_configurations.update(arguments or {})
        for entity in description.entities:
            if isinstance(entity, DeclareLaunchArgument):
                entity.execute(context)
        return description.entities, context

    return build


def _include(entities, context, filename):
    include = next(
        entity
        for entity in entities
        if isinstance(entity, IncludeLaunchDescription)
        and filename in entity.launch_description_source.test_path
    )
    return {
        key: perform_substitutions(context, normalize_to_list_of_substitutions(value))
        for key, value in include.launch_arguments
    }


def _parameters(entities, context, executable):
    nodes = [
        entity
        for entity in entities
        if isinstance(entity, launch_ros.actions.Node)
        and entity.test_options.get("executable") == executable
    ]
    assert nodes
    results = []
    for node in nodes:
        result = {}
        for layer in evaluate_parameters(
            context, normalize_parameters(node.test_options["parameters"])
        ):
            if isinstance(layer, dict):
                result.update(layer)
        results.append(result)
    return results


@pytest.mark.parametrize("use_lidar", ["false", "true"])
@pytest.mark.parametrize(
    "geometry", [GARDEN, dict(zip(FIELDS, (60.1, -12.3, -0.21, 0.14)))]
)
@pytest.mark.parametrize("site", [None, SITE])
def test_scenario_reaches_every_localization_consumer(
    builders, use_lidar, geometry, site
):
    scenario = {
        "gps_reference": dict(
            latitude=geometry["datum_lat"], longitude=geometry["datum_lon"]
        ),
        "antenna_lever_arm": dict(x=geometry["lever_arm_x"], y=geometry["lever_arm_y"]),
    }
    sim, context = builders(
        "mowgli_bringup/launch/sim_full_system.launch.py",
        {"use_lidar": use_lidar},
        scenario=scenario,
        site=site,
    )
    nav_args = _include(sim, context, "navigation.launch.py")
    nav, nav_context = builders("mowgli_bringup/launch/navigation.launch.py", nav_args)
    fusion_args = _include(nav, nav_context, "fusion_graph.launch.py")
    fusion, fusion_context = builders(
        "fusion_graph/launch/fusion_graph.launch.py", fusion_args
    )
    consumers = _parameters(nav, nav_context, "cog_to_imu")
    consumers += _parameters(fusion, fusion_context, "fusion_graph_node")
    assert len(consumers) == 3  # COG plus primary and observer graph actions.
    for parameters in consumers:
        assert {key: parameters[key] for key in FIELDS} == geometry
        assert all(isinstance(parameters[key], float) for key in FIELDS)
        # Project a known antenna position back to chassis coordinates at
        # several headings. Both lever components rotate in the map frame.
        for yaw in (0.0, math.pi / 2, -math.pi / 2, math.pi):
            c, s = math.cos(yaw), math.sin(yaw)
            x, y = 4.49, 6.69
            antenna_x = x + c * geometry["lever_arm_x"] - s * geometry["lever_arm_y"]
            antenna_y = y + s * geometry["lever_arm_x"] + c * geometry["lever_arm_y"]
            scale = 6378137 * math.pi / 180
            latitude = geometry["datum_lat"] + antenna_y / scale
            longitude = geometry["datum_lon"] + antenna_x / (
                scale * math.cos(math.radians(geometry["datum_lat"]))
            )
            map_x = (
                (longitude - parameters["datum_lon"])
                * scale
                * math.cos(math.radians(parameters["datum_lat"]))
            )
            map_y = (latitude - parameters["datum_lat"]) * scale
            assert map_x - c * parameters["lever_arm_x"] + s * parameters[
                "lever_arm_y"
            ] == (pytest.approx(x, abs=1e-8))
            assert map_y - s * parameters["lever_arm_x"] - c * parameters[
                "lever_arm_y"
            ] == (pytest.approx(y, abs=1e-8))
    parameters = _parameters(sim, context, "map_server_node")[0]
    assert parameters["datum_lat"] == geometry["datum_lat"]
    assert parameters["datum_lon"] == geometry["datum_lon"]
    for executable in ("navsat_to_absolute_pose_node", "sim_navsat_rtk_fix.py"):
        parameters = _parameters(sim, context, executable)[0]
        assert parameters["datum_lat"] == geometry["datum_lat"]
        assert parameters["datum_lon"] == geometry["datum_lon"]
    assert {key: parameters[key] for key in FIELDS} == geometry


@pytest.mark.parametrize(
    "relative,executable",
    [
        ("mowgli_bringup/launch/navigation.launch.py", "cog_to_imu"),
        ("fusion_graph/launch/fusion_graph.launch.py", "fusion_graph_node"),
    ],
)
@pytest.mark.parametrize("use_sim_time", ["false", "true"])
@pytest.mark.parametrize("site", [None, SITE])
def test_empty_default_preserves_physical_geometry(
    builders, relative, executable, use_sim_time, site
):
    entities, context = builders(relative, {"use_sim_time": use_sim_time}, site=site)
    expected = (
        dict(zip(FIELDS, (0.0, 0.0, 0.30, 0.0)))
        if site is None
        else dict(
            zip(
                FIELDS,
                (site["datum_lat"], site["datum_lon"], site["gps_x"], site["gps_y"]),
            )
        )
    )
    for parameters in _parameters(entities, context, executable):
        assert {key: parameters[key] for key in FIELDS} == expected


@pytest.mark.parametrize(
    "relative,executable",
    [
        ("mowgli_bringup/launch/navigation.launch.py", "cog_to_imu"),
        ("fusion_graph/launch/fusion_graph.launch.py", "fusion_graph_node"),
    ],
)
@pytest.mark.parametrize(
    "bundle",
    [
        {},
        {"datum_lat": 48.0},
        {**GARDEN, "lever_arm_x": True},
        {**GARDEN, "lever_arm_x": "0.3"},
        {**GARDEN, "datum_lat": 91.0},
        {**GARDEN, "datum_lat": 0.0, "datum_lon": 0.0},
        {**GARDEN, "extra": 1.0},
        [],
        "invalid JSON",
    ]
    + [
        {**GARDEN, field: value}
        for field in FIELDS
        for value in (float("nan"), float("inf"), -float("inf"))
    ],
)
def test_invalid_bundle_fails(builders, relative, executable, bundle):
    entities, context = builders(
        relative,
        {
            "use_sim_time": "true",
            "simulation_geometry": bundle
            if isinstance(bundle, str)
            else json.dumps(bundle),
        },
    )
    with pytest.raises(ValueError):
        _parameters(entities, context, executable)


@pytest.mark.parametrize(
    "relative,executable",
    [
        ("mowgli_bringup/launch/navigation.launch.py", "cog_to_imu"),
        ("fusion_graph/launch/fusion_graph.launch.py", "fusion_graph_node"),
    ],
)
def test_explicit_bundle_requires_simulation_time(builders, relative, executable):
    entities, context = builders(relative, {"simulation_geometry": json.dumps(GARDEN)})
    with pytest.raises(ValueError, match="requires use_sim_time=true"):
        _parameters(entities, context, executable)
