# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
"""Source-level guards for hardware-backend launch selection."""

import ast
from pathlib import Path
from typing import Optional

import pytest


LAUNCH_DIR = Path(__file__).resolve().parent.parent / "launch"


def _parse(name: str) -> ast.Module:
    path = LAUNCH_DIR / name
    return ast.parse(path.read_text(encoding="utf-8"), filename=name)


def _call_name(call: ast.Call) -> Optional[str]:
    if isinstance(call.func, ast.Name):
        return call.func.id
    return None


def _keyword(call: ast.Call, name: str) -> Optional[ast.expr]:
    return next((kw.value for kw in call.keywords if kw.arg == name), None)


def _node(tree: ast.Module, executable: str) -> ast.Call:
    for candidate in ast.walk(tree):
        if not isinstance(candidate, ast.Call) or _call_name(candidate) != "Node":
            continue
        value = _keyword(candidate, "executable")
        if isinstance(value, ast.Constant) and value.value == executable:
            return candidate
    raise AssertionError(f"Node executable={executable!r} was not found")


def _declares_environment_backend(tree: ast.Module) -> bool:
    for call in ast.walk(tree):
        if not isinstance(call, ast.Call) or _call_name(call) != "DeclareLaunchArgument":
            continue
        if not call.args or not isinstance(call.args[0], ast.Constant):
            continue
        if call.args[0].value != "hardware_backend":
            continue
        default = _keyword(call, "default_value")
        if not isinstance(default, ast.Call) or _call_name(default) != "EnvironmentVariable":
            continue
        fallback = _keyword(default, "default_value")
        return (
            bool(default.args)
            and isinstance(default.args[0], ast.Constant)
            and default.args[0].value == "HARDWARE_BACKEND"
            and isinstance(fallback, ast.Constant)
            and fallback.value == "mowgli"
        )
    return False


def test_launch_stack_reads_and_forwards_hardware_backend() -> None:
    """Both launch layers must preserve the environment-selected backend."""
    mowgli_tree = _parse("mowgli.launch.py")
    full_system_tree = _parse("full_system.launch.py")
    assert _declares_environment_backend(mowgli_tree)
    assert _declares_environment_backend(full_system_tree)

    source = (LAUNCH_DIR / "full_system.launch.py").read_text(encoding="utf-8")
    assert '"hardware_backend": hardware_backend' in source


def test_only_native_bridge_is_conditioned_on_mowgli_backend() -> None:
    """MAVROS suppresses only the native bridge, not RSP or twist_mux."""
    tree = _parse("mowgli.launch.py")
    hardware_bridge = _node(tree, "hardware_bridge_node")
    condition = _keyword(hardware_bridge, "condition")
    assert isinstance(condition, ast.Call)
    rendered = ast.dump(condition)
    assert "IfCondition" in rendered
    assert "EqualsSubstitution" in rendered
    assert "hardware_backend" in rendered
    assert "mowgli" in rendered

    for executable in ("robot_state_publisher", "twist_mux"):
        assert _keyword(_node(tree, executable), "condition") is None


def test_invalid_backend_is_rejected_before_nodes_are_launched() -> None:
    """Unknown values must abort launch before the first Node action."""
    tree = _parse("mowgli.launch.py")
    assignments = {
        target.id: ast.literal_eval(node.value)
        for node in tree.body
        if isinstance(node, ast.Assign)
        for target in node.targets
        if isinstance(target, ast.Name)
        and target.id == "SUPPORTED_HARDWARE_BACKENDS"
    }
    assert assignments["SUPPORTED_HARDWARE_BACKENDS"] == ("mowgli", "mavros")

    validator = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "_validate_hardware_backend"
    )
    assert any(isinstance(node, ast.Raise) for node in ast.walk(validator))

    class FakeLaunchConfiguration:
        def __init__(self, _name: str) -> None:
            pass

        def perform(self, context: str) -> str:
            return context

    namespace = {"LaunchConfiguration": FakeLaunchConfiguration}
    executable = ast.Module(
        body=[
            ast.Assign(
                targets=[
                    ast.Name(id="SUPPORTED_HARDWARE_BACKENDS", ctx=ast.Store())
                ],
                value=ast.Constant(value=("mowgli", "mavros")),
            ),
            validator,
        ],
        type_ignores=[],
    )
    ast.fix_missing_locations(executable)
    exec(compile(executable, "mowgli.launch.py", "exec"), namespace)
    validate = namespace["_validate_hardware_backend"]
    assert validate("mowgli") == []
    assert validate("mavros") == []
    with pytest.raises(RuntimeError, match="Invalid hardware_backend"):
        validate("unexpected")

    launch_return = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Return)
        and isinstance(node.value, ast.Call)
        and _call_name(node.value) == "LaunchDescription"
    )
    entities = launch_return.value.args[0]
    assert isinstance(entities, ast.List)
    validator_index = next(
        index
        for index, entity in enumerate(entities.elts)
        if isinstance(entity, ast.Call) and _call_name(entity) == "OpaqueFunction"
    )
    first_node_index = next(
        index
        for index, entity in enumerate(entities.elts)
        if isinstance(entity, ast.Name) and entity.id.endswith("_node")
    )
    assert validator_index < first_node_index
