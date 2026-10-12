# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
"""Source-level guards for HARDWARE_BACKEND launch selection.

Only the ``mowgli`` backend may launch the in-tree ``hardware_bridge_node``.
The ``mavros`` and ``openmower`` backends run their bridge in a sidecar
container that publishes the same ``/hardware_bridge`` contract; launching the
STM32 bridge next to it would put two ``/cmd_vel`` consumers on the wire and
two publishers on every hardware topic. AST-based like test_launch_injection.py
(the launch modules cannot be imported without a sourced ROS2 install).
"""

import ast
from pathlib import Path
from typing import Optional

import pytest


LAUNCH_DIR = Path(__file__).resolve().parent.parent / "launch"
EXPECTED_BACKENDS = ("mowgli", "mavros", "openmower")


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
    """Both launch layers read HARDWARE_BACKEND (default mowgli) and pass it down."""
    assert _declares_environment_backend(_parse("mowgli.launch.py"))
    assert _declares_environment_backend(_parse("full_system.launch.py"))
    source = (LAUNCH_DIR / "full_system.launch.py").read_text(encoding="utf-8")
    assert '"hardware_backend": hardware_backend' in source


def test_only_native_bridge_is_conditioned_on_mowgli_backend() -> None:
    """The sidecar backends suppress ONLY hardware_bridge_node; RSP and twist_mux stay."""
    tree = _parse("mowgli.launch.py")
    hardware_bridge = _node(tree, "hardware_bridge_node")
    condition = _keyword(hardware_bridge, "condition")
    assert isinstance(condition, ast.Call)
    rendered = ast.dump(condition)
    assert "IfCondition" in rendered
    assert "EqualsSubstitution" in rendered
    assert "hardware_backend" in rendered
    assert "NATIVE_BRIDGE_BACKEND" in rendered

    for executable in ("robot_state_publisher", "twist_mux"):
        assert _keyword(_node(tree, executable), "condition") is None


def test_backend_constants_name_every_installer_backend() -> None:
    """The launch allowlist must match install/lib/backend_choice.sh's menu."""
    tree = _parse("mowgli.launch.py")
    assignments = {
        target.id: ast.literal_eval(node.value)
        for node in tree.body
        if isinstance(node, ast.Assign)
        for target in node.targets
        if isinstance(target, ast.Name)
        and target.id in ("SUPPORTED_HARDWARE_BACKENDS", "NATIVE_BRIDGE_BACKEND")
    }
    assert assignments["SUPPORTED_HARDWARE_BACKENDS"] == EXPECTED_BACKENDS
    assert assignments["NATIVE_BRIDGE_BACKEND"] == "mowgli"


def test_invalid_backend_is_rejected_before_nodes_are_launched() -> None:
    """Unknown values abort the launch before the first Node action."""
    tree = _parse("mowgli.launch.py")
    validator = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "_validate_hardware_backend"
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
                targets=[ast.Name(id="SUPPORTED_HARDWARE_BACKENDS", ctx=ast.Store())],
                value=ast.Constant(value=EXPECTED_BACKENDS),
            ),
            validator,
        ],
        type_ignores=[],
    )
    ast.fix_missing_locations(executable)
    exec(compile(executable, "mowgli.launch.py", "exec"), namespace)  # noqa: S102
    validate = namespace["_validate_hardware_backend"]
    for backend in EXPECTED_BACKENDS:
        assert validate(backend) == []
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


def test_openmower_sidecar_remaps_match_the_native_bridge() -> None:
    """The sidecar launch must apply the same remaps as mowgli.launch.py."""
    sidecar = (
        Path(__file__).resolve().parents[4]
        / "sensors/openmower/mowgli_openmower_bridge/launch/openmower_bridge.launch.py"
    )
    if not sidecar.exists():
        pytest.skip("sensors/ tree not present in this workspace")
    native = (LAUNCH_DIR / "mowgli.launch.py").read_text(encoding="utf-8")
    side = sidecar.read_text(encoding="utf-8")
    for remap in (
        '("~/imu/data_raw", "/imu/data")',
        '("~/wheel_odom", "/wheel_odom")',
        '("~/wheel_ticks", "/wheel_ticks")',
        '("~/emergency", "/hardware_bridge/emergency")',
        '("~/power", "/hardware_bridge/power")',
        '("~/status", "/hardware_bridge/status")',
        '("~/cmd_vel", "/cmd_vel")',
    ):
        assert remap in native, remap
        assert remap in side, remap
    assert 'name="hardware_bridge"' in side
