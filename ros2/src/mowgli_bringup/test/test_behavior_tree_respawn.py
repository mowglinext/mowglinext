# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
"""behavior_tree_node is respawned, so a dead mission brain does not stay dead.

hardware_bridge holds the firmware stopped while the tree's status is silent, but nothing
brought the tree back (issue 1171). A restarted tree starts with blade OFF and its
predecessor's Nav2 goals cancelled (boot_hold_timeout_s), so respawning is safe.

Source-level guard on the launch file, like test_launch_injection.py: the launch modules
cannot be imported outside a sourced ROS 2 install.
"""
import ast

from test_launch_injection import _find_node_call, _parse


def _keyword(call: ast.Call, name: str):
    for kw in call.keywords:
        if kw.arg == name:
            return kw.value
    return None


def test_full_system_respawns_the_behavior_tree_with_a_delay():
    node = _find_node_call(_parse("full_system.launch.py"), "behavior_tree_node")
    assert node is not None
    respawn = _keyword(node, "respawn")
    assert isinstance(respawn, ast.Constant) and respawn.value is True
    delay = _keyword(node, "respawn_delay")
    assert isinstance(delay, ast.Constant) and delay.value >= 1.0, (
        "an immediate respawn would turn a crash on start-up into a busy loop"
    )
