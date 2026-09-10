# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
"""Guards for the load-bearing SINGLE LINES that wire mowgli_robot.yaml values
into node parameters at launch time.

Why these exist: each of the injections below is one line in a launch file. Delete
any of them and every other test in the repo still passes — the ROS2 node keeps
its own hardcoded default, the GUI keeps showing the operator's setting, and the
setting silently does nothing on the robot. That is exactly the class of bug the
tool_width / operation_width split caused (54 % coverage, 2026-05-12).

SOURCE-LEVEL GUARDS, deliberately. The launch files import `launch`,
`launch_ros` and `ament_index_python`, none of which exist outside a sourced
ROS2 install, so they cannot be imported here (or on a contributor's laptop) —
`test_robot_config_util.py` and `test_tf_ownership.py` are the precedent for
asserting on launch-file CONTENT without a ROS2 runtime. Rather than regex the
text, these parse the file with `ast` and assert on the syntax tree, so an
assertion fails on a semantic change (a clamp wrapped around the value, the
injection moved to a different node) and not on reformatting.
"""
import ast
import os
from typing import List, Optional

import pytest


def _launch_path(name: str) -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(here, "..", "launch", name)


def _parse(name: str) -> ast.Module:
    with open(_launch_path(name), "r", encoding="utf-8") as fh:
        return ast.parse(fh.read(), filename=name)


def _subscript_assign_values(tree: ast.Module, target: str, key: str) -> List[ast.expr]:
    """RHS expressions of every `target["key"] = <rhs>` assignment in `tree`."""
    out: List[ast.expr] = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign):
            continue
        for tgt in node.targets:
            if (
                isinstance(tgt, ast.Subscript)
                and isinstance(tgt.value, ast.Name)
                and tgt.value.id == target
                and isinstance(tgt.slice, ast.Constant)
                and tgt.slice.value == key
            ):
                out.append(node.value)
    return out


def _reads_robot_param(tree: ast.Module, var: str, key: str) -> bool:
    """True iff some `var = ...rt_rp.get("key", ...)...` assignment exists."""
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign):
            continue
        if not any(isinstance(t, ast.Name) and t.id == var for t in node.targets):
            continue
        for sub in ast.walk(node.value):
            if (
                isinstance(sub, ast.Call)
                and isinstance(sub.func, ast.Attribute)
                and sub.func.attr == "get"
                and isinstance(sub.func.value, ast.Name)
                and sub.func.value.id == "rt_rp"
                and sub.args
                and isinstance(sub.args[0], ast.Constant)
                and sub.args[0].value == key
            ):
                return True
    return False


def _find_node_call(tree: ast.Module, executable: str) -> Optional[ast.Call]:
    """The `Node(...)` call whose `executable=` keyword is `executable`."""
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not (isinstance(node.func, ast.Name) and node.func.id == "Node"):
            continue
        for kw in node.keywords:
            if (
                kw.arg == "executable"
                and isinstance(kw.value, ast.Constant)
                and kw.value.value == executable
            ):
                return node
    return None


def _node_parameter_keys(call: ast.Call) -> List[str]:
    """String keys of every dict literal in a Node's `parameters=[...]` list."""
    keys: List[str] = []
    for kw in call.keywords:
        if kw.arg != "parameters":
            continue
        if not isinstance(kw.value, ast.List):
            continue
        for elt in kw.value.elts:
            if isinstance(elt, ast.Dict):
                for k in elt.keys:
                    if isinstance(k, ast.Constant) and isinstance(k.value, str):
                        keys.append(k.value)
    return keys


# ---------------------------------------------------------------------------
# (a) num_headland_passes must reach coverage_server UNCLAMPED (issue #429).
# ---------------------------------------------------------------------------


def test_navigation_launch_injects_num_headland_passes() -> None:
    """navigation.launch.py must read num_headland_passes from mowgli_robot.yaml
    and write it into coverage_server's parameter dict. Without the write the
    operator's Headland Passes setting does nothing — coverage_server keeps its
    own declare_int default of 0 (AUTO) forever.
    """
    tree = _parse("navigation.launch.py")
    assert _reads_robot_param(tree, "num_headland_passes", "num_headland_passes"), (
        "navigation.launch.py no longer reads num_headland_passes from the robot "
        "config (rt_rp.get) — the GUI setting is orphaned."
    )
    values = _subscript_assign_values(tree, "cov_params", "num_headland_passes")
    assert values, (
        'navigation.launch.py must assign cov_params["num_headland_passes"] — '
        "without it coverage_server keeps its own default and the setting is dead."
    )


def test_navigation_launch_does_not_clamp_num_headland_passes() -> None:
    """The value is a THREE-WAY sentinel (< 0 NONE, == 0 AUTO, > 0 FORCED,
    issue #429), so the negative branch MUST survive the trip to the node. A
    defensive `max(0, ...)` anywhere on this path silently turns "no perimeter
    rings" back into AUTO — the setting would still appear to apply, and nothing
    else in the suite would notice.
    """
    tree = _parse("navigation.launch.py")
    for value in _subscript_assign_values(tree, "cov_params", "num_headland_passes"):
        assert isinstance(value, ast.Name) and value.id == "num_headland_passes", (
            'cov_params["num_headland_passes"] must be assigned the bare variable, '
            f"not {ast.dump(value)} — any wrapper risks clamping away the negative "
            "NONE sentinel (#429)."
        )
    # And nothing upstream clamps it either.
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign):
            continue
        if not any(
            isinstance(t, ast.Name) and t.id == "num_headland_passes" for t in node.targets
        ):
            continue
        for sub in ast.walk(node.value):
            if isinstance(sub, ast.Call) and isinstance(sub.func, ast.Name):
                assert sub.func.id not in ("max", "min", "abs"), (
                    f"num_headland_passes is passed through {sub.func.id}() in "
                    "navigation.launch.py — that destroys the negative NONE sentinel."
                )


# ---------------------------------------------------------------------------
# (b) mowing_enabled must reach hardware_bridge_node (issue #195).
# ---------------------------------------------------------------------------


def test_mowgli_launch_passes_mowing_enabled_to_hardware_bridge() -> None:
    """mowgli.launch.py must hand `mowing_enabled` to hardware_bridge_node.

    hardware_bridge is the merged chokepoint every blade ENABLE goes through, so
    it is the only node where the dry-run inhibit can take effect. Drop the
    parameter and the node falls back to its own default (true): an operator who
    set mowing_enabled:false for a dry run gets a spinning blade instead.

    NOTE this is a plumbing guard, not a safety interlock — the STM32 firmware
    remains the sole blade safety authority.
    """
    tree = _parse("mowgli.launch.py")
    call = _find_node_call(tree, "hardware_bridge_node")
    assert call is not None, "mowgli.launch.py no longer launches hardware_bridge_node"
    keys = _node_parameter_keys(call)
    assert "mowing_enabled" in keys, (
        "hardware_bridge_node's parameters= list no longer carries mowing_enabled "
        f"(has: {sorted(set(keys))}) — the dry-run inhibit is orphaned (#195)."
    )


def test_mowing_enabled_is_not_wired_to_some_other_node() -> None:
    """Companion guard: moving the parameter onto a node that cannot act on it
    (e.g. the BT or coverage server) would keep this file's first assertion
    honest only if it is still on hardware_bridge. Assert every Node that
    receives mowing_enabled is hardware_bridge_node.
    """
    tree = _parse("mowgli.launch.py")
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not (isinstance(node.func, ast.Name) and node.func.id == "Node"):
            continue
        if "mowing_enabled" not in _node_parameter_keys(node):
            continue
        execs = [
            kw.value.value
            for kw in node.keywords
            if kw.arg == "executable" and isinstance(kw.value, ast.Constant)
        ]
        assert execs == ["hardware_bridge_node"], (
            f"mowing_enabled is passed to {execs} — only hardware_bridge_node "
            "gates the merged blade-command path (#195)."
        )


# ---------------------------------------------------------------------------
# (c) dock_max_retries / dock_use_charger_detection must reach docking_server
#     (issue #195). test_nav2_params.py only checks the keys exist in the static
#     YAML — which was already true BEFORE they were wired up, so that test
#     cannot fail if the injection is deleted. These can.
# ---------------------------------------------------------------------------


def test_navigation_launch_injects_dock_max_retries() -> None:
    tree = _parse("navigation.launch.py")
    assert _reads_robot_param(tree, "dock_max_retries", "dock_max_retries"), (
        "navigation.launch.py no longer reads dock_max_retries from the robot config."
    )
    values = _subscript_assign_values(tree, "ds", "max_retries")
    assert values, (
        'navigation.launch.py must assign ds["max_retries"] into the merged docking '
        "params — otherwise docking_server keeps nav2_params_base.yaml's static "
        "value and the mowgli_robot.yaml key is wired to nothing (#195)."
    )
    for value in values:
        names = {n.id for n in ast.walk(value) if isinstance(n, ast.Name)}
        assert "dock_max_retries" in names, (
            'ds["max_retries"] is assigned something other than dock_max_retries: '
            f"{ast.dump(value)}"
        )


def test_navigation_launch_injects_dock_use_charger_detection() -> None:
    tree = _parse("navigation.launch.py")
    assert _reads_robot_param(tree, "dock_use_charger_detection", "dock_use_charger_detection"), (
        "navigation.launch.py no longer reads dock_use_charger_detection from the robot config."
    )
    values = _subscript_assign_values(tree, "scd", "use_battery_status")
    assert values, (
        'navigation.launch.py must assign scd["use_battery_status"] into the merged '
        "simple_charging_dock params — otherwise the operator's charger-detection "
        "choice never reaches the plugin (#195)."
    )
    for value in values:
        names = {n.id for n in ast.walk(value) if isinstance(n, ast.Name)}
        assert "dock_use_charger_detection" in names, (
            'scd["use_battery_status"] is assigned something other than '
            f"dock_use_charger_detection: {ast.dump(value)}"
        )


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
