# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
"""Execute the production launch builders with lightweight launch actions.

Like test_fusion_graph_launch_types, compile the actual function AST to avoid
requiring ROS on contributor machines. The adapters capture declarations,
includes and node options; they do not simulate a running bridge or respawn.
"""

import ast
import importlib.util
import os
import re
from pathlib import Path

import pytest
from test_launch_injection import _find_node_call, _parse

BRINGUP = Path(__file__).resolve().parents[1]


class Action:
    def __init__(self, *args, **kwargs):
        self.args = args
        self.options = kwargs


class Description:
    def __init__(self, entities):
        self.entities = entities


class Configuration:
    def __init__(self, name):
        self.name = name


class Parameter:
    def __init__(self, value, value_type):
        self.value = value
        self.value_type = value_type


class Declaration(Action):
    pass


class Include(Action):
    pass


class Node(Action):
    pass


def _resolve(value, context):
    if isinstance(value, Configuration):
        return context[value.name]
    if isinstance(value, Parameter):
        return value.value_type(_resolve(value.value, context))
    if isinstance(value, list):
        return [_resolve(item, context) for item in value]
    if isinstance(value, dict):
        return {key: _resolve(item, context) for key, item in value.items()}
    return value


def _build(filename, **overrides):
    spec = importlib.util.spec_from_file_location(
        "foxglove_test_robot_config", BRINGUP / "launch" / "robot_config_util.py"
    )
    config = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(config)
    namespace = {
        name: getattr(config, name)
        for name in (
            "DEFAULT_TOOL_WIDTH_M",
            "chassis_circumscribed_radius",
            "dig_proposal_radius",
            "dig_skip_radius",
            "keepout_obstacle_margin",
            "resolve_lidar_enabled",
            "warn_lidar_key_absent",
        )
    }
    # Use the real template/deep merge, with no machine-specific runtime file.
    namespace.update(
        {
            "os": os,
            "LaunchDescription": Description,
            "DeclareLaunchArgument": Declaration,
            "IncludeLaunchDescription": Include,
            "PythonLaunchDescriptionSource": Action,
            "IfCondition": Action,
            "LaunchConfiguration": Configuration,
            "PythonExpression": Action,
            "EnvironmentVariable": Action,
            "ParameterValue": Parameter,
            "Node": Node,
            "get_package_share_directory": lambda package: str(
                BRINGUP.parent / package
            ),
            "load_robot_params": lambda share, runtime: config.load_robot_params(
                share, str(BRINGUP / "test" / "absent-runtime-config.yaml")
            ),
        }
    )
    function = next(
        item
        for item in _parse(filename).body
        if isinstance(item, ast.FunctionDef)
        and item.name == "generate_launch_description"
    )
    exec(  # noqa: S102
        compile(ast.Module(body=[function], type_ignores=[]), filename, "exec"),
        namespace,
    )
    description = namespace["generate_launch_description"]()
    context = dict(overrides)
    for entity in description.entities:
        if isinstance(entity, Declaration):
            context.setdefault(entity.args[0], entity.options["default_value"])
    return description, context


def _bridge(description):
    nodes = [
        entity
        for entity in description.entities
        if isinstance(entity, Node)
        and entity.options.get("package") == "foxglove_bridge"
    ]
    assert len(nodes) == 1
    return nodes[0]


def _production_bridge(**overrides):
    full, context = _build("full_system.launch.py", **overrides)
    includes = [
        entity
        for entity in full.entities
        if isinstance(entity, Include)
        and entity.args[0].args[0].endswith("foxglove_bridge.launch.py")
    ]
    assert len(includes) == 1, "full_system must include the shared Foxglove launch"
    include = includes[0]
    arguments = {
        key: _resolve(value, context)
        for key, value in include.options["launch_arguments"]
    }
    shared, child_context = _build("foxglove_bridge.launch.py", **arguments)
    enabled = _resolve(include.options["condition"].args[0], context) == "true"
    return _bridge(shared), child_context, enabled


def _parameters(node, context):
    parameters = {}
    for layer in node.options["parameters"]:
        parameters.update(_resolve(layer, context))
    return parameters


def test_full_system_reuses_shared_node_with_buffer_and_respawn():
    node, context, enabled = _production_bridge()
    assert enabled
    assert _find_node_call(_parse("full_system.launch.py"), "foxglove_bridge") is None
    standalone, standalone_context = _build("foxglove_bridge.launch.py")
    expected = _parameters(_bridge(standalone), standalone_context)
    actual = _parameters(node, context)
    assert int(actual.pop("num_threads")) == 2  # existing production setting
    assert int(expected.pop("num_threads")) == 0  # existing standalone setting
    assert actual == expected
    assert int(actual["send_buffer_limit"]) == 1000000
    assert node.options["respawn"] is True
    assert node.options["respawn_delay"] == 2.0


@pytest.mark.parametrize("enabled", ["true", "false"])
def test_full_system_preserves_enable_switch_and_custom_port(enabled):
    node, context, should_launch = _production_bridge(
        enable_foxglove=enabled, foxglove_port="9876"
    )
    assert should_launch == (enabled == "true")
    assert int(_parameters(node, context)["port"]) == 9876


def test_standalone_preserves_buffer_and_port_overrides():
    standalone, context = _build(
        "foxglove_bridge.launch.py", port="9877", send_buffer_limit="2048"
    )
    parameters = _parameters(_bridge(standalone), context)
    assert int(parameters["port"]) == 9877
    assert int(parameters["send_buffer_limit"]) == 2048


def test_shared_definition_preserves_production_capabilities_and_gnss_filter():
    node, context, _ = _production_bridge()
    parameters = _parameters(node, context)
    assert parameters["address"] == "0.0.0.0"
    assert parameters["capabilities"] == [
        "clientPublish",
        "services",
        "connectionGraph",
        "parameters",
        "parametersSubscribe",
    ]
    assert parameters["topic_whitelist"] == parameters["client_topic_whitelist"]
    (pattern,) = parameters["topic_whitelist"]
    for prefix in ("/_gps_internal", "/gps_internal", "/universal_gnss"):
        assert re.match(pattern, prefix) is None
        assert re.match(pattern, prefix + "/status") is None
    for public in ("/gps/fix", "/gps/status", "/imu/data", "/universal_gnss_other"):
        assert re.match(pattern, public)
