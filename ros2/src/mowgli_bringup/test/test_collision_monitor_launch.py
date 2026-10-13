# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
"""Execute production navigation parameter generation without launching ROS."""

import ast
import importlib.util
import os
from pathlib import Path
import tempfile

import pytest
import yaml

PACKAGE = Path(__file__).resolve().parent.parent


class Action:
    def __init__(self, *args, **kwargs):
        self.args, self.kwargs = args, kwargs


@pytest.mark.parametrize("overrides", [{},
    {"chassis_length": .917, "chassis_width": .623, "chassis_center_x": .267}])
def test_production_navigation_injects_signed_catcher_from_same_costmap_geometry(
        overrides, tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location(
        "launch_collision_config", PACKAGE / "launch" / "robot_config_util.py")
    util = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(util)
    params = util.load_robot_params(str(PACKAGE), str(tmp_path / "absent.yaml"))
    params.update(overrides)
    namespace = {name: getattr(util, name) for name in dir(util) if not name.startswith("__")}
    namespace.update(os=os, yaml=yaml,
        get_package_share_directory=lambda package: str(PACKAGE.parent / package),
        get_package_prefix=lambda package: str(tmp_path / package),
        load_robot_params=lambda *args: params)
    for name in ("LaunchDescription", "DeclareLaunchArgument", "ExecuteProcess",
                 "GroupAction", "IncludeLaunchDescription", "RegisterEventHandler",
                 "IfCondition", "UnlessCondition", "OnProcessExit",
                 "PythonLaunchDescriptionSource", "LaunchConfiguration",
                 "PythonExpression", "Node", "SetParameter", "RewrittenYaml"):
        namespace[name] = Action
    rewrites = []
    def capture_rewrite(**kwargs):
        rewrites.append(kwargs)
        return Action(**kwargs)
    namespace["RewrittenYaml"] = capture_rewrite
    function = next(node for node in ast.parse(
        (PACKAGE / "launch" / "navigation.launch.py").read_text()).body
        if isinstance(node, ast.FunctionDef) and node.name == "generate_launch_description")
    monkeypatch.setattr(tempfile, "tempdir", str(tmp_path))
    exec(compile(ast.Module(body=[function], type_ignores=[]), "navigation.launch.py", "exec"),
         namespace)
    namespace["generate_launch_description"]()
    front, rear, half = util.chassis_footprint(params)
    assert len(rewrites) == 1
    assert yaml.safe_load(rewrites[0]["param_rewrites"]["footprint"]) == [
        [float(f"{front:.3f}"), float(f"{half:.3f}")],
        [float(f"{front:.3f}"), float(f"{-half:.3f}")],
        [float(f"{rear:.3f}"), float(f"{-half:.3f}")],
        [float(f"{rear:.3f}"), float(f"{half:.3f}")]]
    # Inspect precisely the two generated files selected by the launch's
    # RewrittenYaml source expression, not the earlier raw overlay merge.
    expression = rewrites[0]["source_file"].args[0]
    generated = [yaml.safe_load(Path(expression[index]).read_text()) for index in (1, 5)]
    lidar = [doc for doc in generated if "PolygonStopNarrow" in
             doc.get("collision_monitor", {}).get("ros__parameters", {})]
    assert len(lidar) == 1, "production launch did not select its LiDAR parameter document"
    for doc in lidar:
        cm = doc["collision_monitor"]["ros__parameters"]
        narrow = cm["PolygonStopNarrow"]
        front, rear, half = util.chassis_footprint(params)
        padding = doc["local_costmap"]["local_costmap"]["ros__parameters"]["footprint_padding"]
        expected = util.collision_monitor_velocity_polygons(front, rear, half, padding)
        assert all(narrow[key] == value for key, value in expected.items())
        assert "points" not in narrow
        assert cm["FootprintApproach"]["min_points"] == 3
        assert narrow["min_points"] == 2
    no_lidar = [doc for doc in generated if "PolygonStopNarrow" not in
                doc.get("collision_monitor", {}).get("ros__parameters", {})]
    assert len(no_lidar) == 1
    for doc in no_lidar:
        cm = doc["collision_monitor"]["ros__parameters"]
        assert cm["scan"]["enabled"] is False
        assert all(cm[name]["enabled"] is False for name in cm["polygons"])
