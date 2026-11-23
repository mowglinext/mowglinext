# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
"""The native launch uses a stable per-launch marker for BT process recovery."""
import ast

from test_launch_injection import _find_node_call, _parse


def _keyword(call: ast.Call, name: str):
    for kw in call.keywords:
        if kw.arg == name:
            return kw.value
    return None


def test_native_launch_scopes_respawn_and_retains_marker_across_child_restarts():
    tree = _parse("full_system.launch.py")
    node = _find_node_call(tree, "behavior_tree_node")
    assert node is not None
    respawn = _keyword(node, "respawn")
    assert isinstance(respawn, ast.Name)
    predicate = next(
        assignment.value for assignment in ast.walk(tree)
        if isinstance(assignment, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == respawn.id
                for target in assignment.targets)
    )
    assert ast.unparse(predicate) == "EqualsSubstitution(hardware_backend, 'mowgli')"
    delay = _keyword(node, "respawn_delay")
    assert isinstance(delay, ast.Constant) and delay.value >= 1.0
    params = _keyword(node, "parameters")
    marker = next(
        value for element in params.elts if isinstance(element, ast.Dict)
        for key, value in zip(element.keys, element.values)
        if isinstance(key, ast.Constant) and key.value == "boot_hold_marker"
    )
    assert isinstance(marker, ast.Call)
    expected = (f"IfElseSubstitution({respawn.id}, "
                "if_value=os.path.join(bt_launch_state.name, 'started'))")
    assert ast.unparse(marker.args[0]) == expected
