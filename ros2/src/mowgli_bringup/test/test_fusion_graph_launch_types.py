# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
"""Guards the numeric TYPES fusion_graph.launch.py forwards to the localizer.

fusion_graph_node declares lidar_map_resolution_m and lidar_map_tile_size_m as
``double`` and lidar_map_radius_tiles as ``int``
(fusion_graph/src/fusion_graph_node_setup_params.cpp). rclcpp does not widen an
int parameter to a double, so a config holding ``lidar_map_tile_size_m: 5``
instead of ``5.0`` makes declare_parameter<double> raise
InvalidParameterTypeException: the localizer aborts, nothing publishes
map->odom, the Nav2 costmaps never activate and the robot cannot mow. That is
the 2026-09-15 field incident, caused by the GUI settings writer dropping the
decimal point from every integral float when it re-marshalled the installed
mowgli_robot.yaml.

The launch file imports ``launch``, ``launch_ros`` and ``ament_index_python``,
none of which exist outside a sourced ROS2 install, so the module cannot be
imported here (same constraint as test_launch_injection.py). Instead of
regexing the text, this extracts the pure helper's own AST from the file and
executes it, so the assertions run the REAL launch-time code path.
"""
import ast
import os
from typing import Any, Callable, Dict

import pytest

LAUNCH_FILE = "fusion_graph.launch.py"
HELPER = "_map_geometry_params"


def _launch_path() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.abspath(
        os.path.join(here, "..", "..", "fusion_graph", "launch", LAUNCH_FILE)
    )


def _parse() -> ast.Module:
    with open(_launch_path(), "r", encoding="utf-8") as fh:
        return ast.parse(fh.read(), filename=LAUNCH_FILE)


def _load_helper() -> Callable[[Dict[str, Any]], Dict[str, Any]]:
    """Compile and return the launch file's own _map_geometry_params.

    The helper is pure (builtins only), so it runs without the ROS2 launch
    packages the rest of the module needs.
    """
    for node in _parse().body:
        if isinstance(node, ast.FunctionDef) and node.name == HELPER:
            module = ast.Module(body=[node], type_ignores=[])
            namespace: Dict[str, Any] = {}
            exec(compile(module, _launch_path(), "exec"), namespace)  # noqa: S102
            return namespace[HELPER]
    raise AssertionError(
        f"{LAUNCH_FILE} no longer defines {HELPER}() — the LiDAR map geometry "
        "must stay behind a typed helper (declare_parameter<double> aborts the "
        "localizer on an int)."
    )


def test_integral_metre_values_are_forwarded_as_floats() -> None:
    """The exact field case: a config demoted to ints must still launch."""
    params = _load_helper()({
        "lidar_map_resolution_m": 1,
        "lidar_map_tile_size_m": 5,
        "lidar_map_radius_tiles": 2,
    })

    assert isinstance(params["lidar_map_tile_size_m"], float)
    assert params["lidar_map_tile_size_m"] == 5.0
    assert isinstance(params["lidar_map_resolution_m"], float)
    assert params["lidar_map_resolution_m"] == 1.0


def test_tile_radius_is_forwarded_as_int() -> None:
    """lidar_map_radius_tiles is declare_parameter<int> — a float aborts too."""
    params = _load_helper()({
        "lidar_map_tile_size_m": 10.0,
        "lidar_map_radius_tiles": 2.0,
    })

    assert isinstance(params["lidar_map_radius_tiles"], int)
    assert not isinstance(params["lidar_map_radius_tiles"], bool)
    assert params["lidar_map_radius_tiles"] == 2


def test_float_values_are_preserved() -> None:
    params = _load_helper()({
        "lidar_map_resolution_m": 0.10,
        "lidar_map_tile_size_m": 10.0,
        "lidar_map_radius_tiles": 2,
    })

    assert params == {
        "lidar_map_resolution_m": 0.10,
        "lidar_map_tile_size_m": 10.0,
        "lidar_map_radius_tiles": 2,
    }


def test_absent_keys_fall_through_to_package_defaults() -> None:
    """An omitted key must NOT be injected, or fusion_graph.yaml is overridden."""
    assert _load_helper()({}) == {}
    assert _load_helper()({"lidar_map_tile_size_m": None}) == {}
    assert "lidar_map_radius_tiles" not in _load_helper()(
        {"lidar_map_resolution_m": 0.05}
    )


def test_launch_description_uses_the_typed_helper() -> None:
    """The raw ``{key: cfg[key] ...}`` comprehension must not come back."""
    tree = _parse()
    calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == HELPER
    ]
    assert calls, (
        f"{LAUNCH_FILE} defines {HELPER}() but never calls it — the LiDAR map "
        "geometry would reach the node untyped again."
    )

    for node in ast.walk(tree):
        if not isinstance(node, ast.DictComp):
            continue
        keys = {
            sub.value
            for sub in ast.walk(node)
            if isinstance(sub, ast.Constant) and isinstance(sub.value, str)
        }
        assert not (keys & {
            "lidar_map_resolution_m",
            "lidar_map_tile_size_m",
            "lidar_map_radius_tiles",
        }), (
            "LiDAR map geometry is being built by a dict comprehension again; "
            f"route it through {HELPER}() so the values keep their declared types."
        )


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
