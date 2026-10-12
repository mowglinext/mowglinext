# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
"""Execute endpoint injection expressions for the three production consumers."""

import ast
from pathlib import Path

import pytest
import yaml
from test_launch_injection import _find_node_call, _node_parameter_values, _parse


@pytest.mark.parametrize(
    "robot_params", [{}, {"battery_empty_voltage": 21, "battery_full_voltage": 28.5}]
)
def test_monitoring_and_bt_receive_same_typed_pack_endpoints(robot_params):
    tree = _parse("full_system.launch.py")
    for key, default in (
        ("battery_empty_voltage", 24.0),
        ("battery_full_voltage", 28.0),
    ):
        template = yaml.safe_load(
            (
                Path(__file__).resolve().parents[1] / "config" / "mowgli_robot.yaml"
            ).read_text()
        )["mowgli"]["ros__parameters"]
        assert template[key] == default
        values = []
        for executable in (
            "behavior_tree_node",
            "diagnostics_node",
            "mqtt_bridge_node",
        ):
            node = _find_node_call(tree, executable)
            expressions = _node_parameter_values(node, key)
            assert len(expressions) == 1, f"{executable} must receive configured {key}"
            value = eval(
                compile(ast.Expression(expressions[0]), "launch-endpoint", "eval"),
                {"robot_params": robot_params},
            )
            assert isinstance(value, float)
            values.append(value)
        assert values == [float(robot_params.get(key, default))] * 3
