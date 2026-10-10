# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
"""Execute both production launch builders with configured chassis dimensions.

The existing launch-action adapters capture parameters; they do not start nodes
or claim simulation/hardware evidence.
"""

import pytest

from test_foxglove_launch_config import Node, _build


@pytest.mark.parametrize(
    "filename", ["full_system.launch.py", "sim_full_system.launch.py"]
)
@pytest.mark.parametrize(
    "length,width,center,front,rear,half",
    # Preserve the existing Nav2 chassis clearance of 0.05 m on each edge.
    [(0.8, 0.6, 0.15, 0.60, -0.30, 0.35),
     (0.94, 0.42, 0.25, 0.77, -0.27, 0.26)],
)
def test_blocked_start_escape_receives_configured_complete_chassis(
    filename, length, width, center, front, rear, half
):
    description, _ = _build(
        filename,
        robot_params_override={
            "chassis_length": length,
            "chassis_width": width,
            "chassis_center_x": center,
        },
    )
    behavior = [
        node for node in description.entities
        if isinstance(node, Node)
        and node.options.get("package") == "mowgli_behavior"
    ]
    assert len(behavior) == 1
    parameters = {}
    for entry in behavior[0].options["parameters"]:
        if isinstance(entry, dict):
            parameters.update(entry)
    assert parameters["motion_footprint"] == pytest.approx(
        [front, half, front, -half, rear, -half, rear, half]
    )
