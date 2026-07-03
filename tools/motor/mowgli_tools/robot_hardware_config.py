# Copyright 2026 Mowgli Project
#
# SPDX-License-Identifier: GPL-3.0

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .drive_pid_math import finite_or_none


@dataclass(frozen=True)
class RobotHardwareConfig:
    source_path: str | None = None
    chassis_mass_kg: float | None = None
    wheel_radius_m: float | None = None
    ticks_per_revolution: float | None = None


def extract_robot_parameter_mapping(payload: Any) -> dict[str, Any]:
    if not isinstance(payload, dict):
        return {}
    mowgli = payload.get("mowgli")
    if isinstance(mowgli, dict):
        ros_parameters = mowgli.get("ros__parameters")
        if isinstance(ros_parameters, dict):
            return ros_parameters
    ros_parameters = payload.get("ros__parameters")
    if isinstance(ros_parameters, dict):
        return ros_parameters
    return payload


def extract_robot_hardware_config(payload: Any, source_path: str | None) -> RobotHardwareConfig:
    params = extract_robot_parameter_mapping(payload)
    if not isinstance(params, dict):
        return RobotHardwareConfig(source_path=source_path)
    chassis_mass_kg = finite_or_none(params.get("chassis_mass_kg"), positive=True)
    if chassis_mass_kg is None:
        chassis_mass_kg = finite_or_none(params.get("robot_mass_kg"), positive=True)
    if chassis_mass_kg is None:
        chassis_mass_kg = finite_or_none(params.get("mass_kg"), positive=True)
    if chassis_mass_kg is None:
        chassis_mass_kg = finite_or_none(params.get("mass"), positive=True)
    return RobotHardwareConfig(
        source_path=source_path,
        chassis_mass_kg=chassis_mass_kg,
        wheel_radius_m=finite_or_none(params.get("wheel_radius"), positive=True),
        ticks_per_revolution=finite_or_none(params.get("ticks_per_revolution"), positive=True),
    )
