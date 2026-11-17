# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later
"""Resolve a complete simulator datum/antenna snapshot at parameter evaluation."""

import json
import math

from launch import Substitution
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def resolve_simulation_geometry(spec, use_sim_time):
    """An empty bundle preserves robot configuration; overrides require sim time."""
    if not spec:
        return {}
    if str(use_sim_time).lower() not in ("true", "1"):
        raise ValueError("simulation_geometry requires use_sim_time=true")
    fields = {"datum_lat", "datum_lon", "lever_arm_x", "lever_arm_y"}
    geometry = json.loads(spec)
    if not isinstance(geometry, dict) or set(geometry) != fields:
        raise ValueError(
            "simulation_geometry must contain the complete datum and antenna geometry"
        )
    if any(
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        for value in geometry.values()
    ):
        raise ValueError("simulation_geometry values must be finite numbers")
    if not -90 < geometry["datum_lat"] < 90 or not -180 <= geometry["datum_lon"] <= 180:
        raise ValueError("simulation_geometry datum is outside WGS84 bounds")
    if geometry["datum_lat"] == 0 and geometry["datum_lon"] == 0:
        raise ValueError("simulation_geometry must specify a datum, not self-seeding")
    return {key: float(value) for key, value in geometry.items()}


class _GeometryValue(Substitution):
    """Defer selection until include arguments are available in the launch context."""

    def __init__(self, field, default):
        super().__init__()
        self.field = field
        self.default = default

    def describe(self):
        return f"simulation_geometry[{self.field}]"

    def perform(self, context):
        geometry = resolve_simulation_geometry(
            LaunchConfiguration("simulation_geometry", default="").perform(context),
            LaunchConfiguration("use_sim_time", default="false").perform(context),
        )
        return str(geometry.get(self.field, self.default))


def simulation_geometry_parameters(datum_lat, datum_lon, lever_x, lever_y):
    """Keep ROS double types for both scenario overrides and physical defaults."""
    return {
        key: ParameterValue(_GeometryValue(key, value), value_type=float)
        for key, value in zip(
            ("datum_lat", "datum_lon", "lever_arm_x", "lever_arm_y"),
            (datum_lat, datum_lon, lever_x, lever_y),
        )
    }
