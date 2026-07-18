# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
#
# Static guard for CLAUDE.md Architecture Invariant 2: map→odom AND
# odom→base_footprint may be published ONLY by fusion_graph_node — it is
# the sole, default, unconditional localizer (the removed ekf_map_node /
# ekf_odom_node no longer exist, and there is no use_fusion_graph launch
# arg). wheel_odometry_node is the one other node in the repo capable of
# broadcasting odom→base_footprint; it is kept for twist-only consumers
# and its publish_tf parameter defaults to false specifically so its pose
# branch never competes with the localizer (wheel_odometry_node.cpp).
#
# No ROS node is spun up here — this is a source/config grep, the same
# "cheap, no nodes" static-guard style as test_nav2_params.py.
"""Regression tests for TF sole-ownership (Inv 2)."""
import os
import re

import yaml


def _ros2_src_root() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.abspath(os.path.join(here, "..", ".."))


def _walk_source_files(suffixes):
    root = _ros2_src_root()
    skip_segments = {"external", "build", "install", "log"}
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [d for d in dirnames if d not in skip_segments]
        for name in filenames:
            if name.endswith(suffixes):
                yield os.path.join(dirpath, name)


def _grep_files(pattern: str, suffixes):
    rx = re.compile(pattern)
    root = _ros2_src_root()
    hits = set()
    for path in _walk_source_files(suffixes):
        with open(path, "r", encoding="utf-8", errors="ignore") as fh:
            if rx.search(fh.read()):
                hits.add(os.path.relpath(path, root))
    return hits


# The only two nodes in the repo that construct a tf2_ros::TransformBroadcaster.
# fusion_graph_node is the sole, default localizer and owns both REP-105 edges
# (Inv 1/2). wheel_odometry_node's broadcast is gated behind publish_tf
# (pinned false below) and exists only for standalone/debug use.
EXPECTED_TF_BROADCASTER_FILES = {
    "fusion_graph/include/fusion_graph/fusion_graph_node.hpp",
    "fusion_graph/src/fusion_graph_node_setup_comms.cpp",
    "mowgli_localization/src/wheel_odometry_node.cpp",
    "mowgli_localization/include/mowgli_localization/wheel_odometry_node.hpp",
}


def test_only_known_nodes_construct_a_tf_broadcaster() -> None:
    """A new node quietly adding tf2_ros::TransformBroadcaster is exactly how
    Inv 2 gets violated — two nodes fighting over map→odom or
    odom→base_footprint. Pin the known set so a new broadcaster construction
    fails this test until it's reviewed and added here deliberately.
    """
    found = _grep_files(r"TransformBroadcaster\b", (".cpp", ".hpp"))
    assert found == EXPECTED_TF_BROADCASTER_FILES, (
        f"tf2_ros::TransformBroadcaster construction in unexpected files: "
        f"{found - EXPECTED_TF_BROADCASTER_FILES}. Missing expected files: "
        f"{EXPECTED_TF_BROADCASTER_FILES - found}. CLAUDE.md Inv 2: only "
        "fusion_graph_node may publish map→odom or odom→base_footprint."
    )


def test_wheel_odometry_shipped_config_disables_publish_tf() -> None:
    """wheel_odometry_node's publish_tf defaults to false in code
    (declare_parameter<bool>("publish_tf", false) in wheel_odometry_node.cpp),
    but a shipped config could still flip it on and put two nodes on
    odom→base_footprint. Pin the installed default explicitly.
    """
    path = os.path.join(_ros2_src_root(), "mowgli_localization", "config", "wheel_odometry.yaml")
    with open(path, "r", encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    publish_tf = cfg["wheel_odometry"]["ros__parameters"]["publish_tf"]
    assert publish_tf is False, (
        f"wheel_odometry.yaml ships publish_tf: {publish_tf} — this would make "
        "wheel_odometry_node broadcast odom→base_footprint alongside "
        "fusion_graph_node, violating Inv 2's single-writer rule."
    )


def test_wheel_odometry_publish_tf_not_overridden_by_any_launch_file() -> None:
    """Guard the other place publish_tf could be flipped on: a launch-time
    parameter override. No launch file in the repo should reference it.
    """
    hits = _grep_files(r"publish_tf", (".launch.py",))
    assert not hits, (
        f"publish_tf referenced in launch file(s) {hits} — verify it is not "
        "being set to true for wheel_odometry_node (would violate Inv 2)."
    )
