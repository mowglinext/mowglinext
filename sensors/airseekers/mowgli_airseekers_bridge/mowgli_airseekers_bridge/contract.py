# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
"""MCU state -> /hardware_bridge contract fields (pure, no ROS).

Kept apart from the node so the projections are unit-tested
(test/test_contract.py) without rclpy.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional

from .protocol import BatteryInfo, MotorInfo, VersionInfo, version_string

# sensor_msgs/BatteryState.POWER_SUPPLY_STATUS_*
PSS_CHARGING = 1
PSS_DISCHARGING = 2
PSS_NOT_CHARGING = 3


@dataclass
class PowerFields:
    v_charge: float
    v_battery: float
    charge_current: float
    charger_enabled: bool
    charger_status: str
    battery_current: float      # BatteryState.current (docking convention)
    battery_percentage: float   # 0..1
    power_supply_status: int


def charge_current_a(b: BatteryInfo) -> float:
    """Current INTO the battery in A (>0 while charging)."""
    return -b.discharge_a


def project_power(b: BatteryInfo, charging_current_min_a: float = 0.2) -> PowerFields:
    """Power + BatteryState semantics of the contract.

    ``charger_enabled`` / ``Status.is_charging`` mean ON THE DOCK (charger
    contacts live, the MCU's ``dock_ok``), as on the STM32 and OpenMower
    backends -- the BT reports it as HighLevelStatus.is_charging. Whether
    current actually flows is in ``charger_status``.

    ``BatteryState.current`` follows Invariant 12: abs(charge current) while
    docked, 0.0 otherwise. The Tron has no separate charge-contact voltage
    measurement: ``v_charge`` is 0.0 (not measured).
    """
    into = charge_current_a(b)
    docked = b.dock_ok
    flowing = docked and into >= charging_current_min_a
    if not docked:
        status, pss = 'idle', PSS_DISCHARGING
    elif flowing:
        status, pss = 'charging', PSS_CHARGING
    else:
        status, pss = 'docked, not charging', PSS_NOT_CHARGING
    current = abs(into) if docked else 0.0
    return PowerFields(
        v_charge=0.0,
        v_battery=b.voltage_v,
        charge_current=into if flowing else 0.0,
        charger_enabled=docked,
        charger_status=status,
        battery_current=current,
        battery_percentage=max(0.0, min(100.0, float(b.percentage))) / 100.0,
        power_supply_status=pss,
    )


def firmware_version_string(v: Optional[VersionInfo]) -> str:
    if v is None:
        return 'airseekers (no version reported yet)'
    return 'airseekers cutter %s / chassis %s' % (version_string(v.cutter),
                                                 version_string(v.chassis))


def blade_rpm(motors: Dict[str, MotorInfo]) -> float:
    m = motors.get('cutter')
    return float(abs(m.speed_rpm)) if m is not None else 0.0


def firmware_compatible(link_alive: bool, versions: Optional[VersionInfo], imu_alive: bool) -> bool:
    """What PreFlightCheck sees: the MCU stream is live and identified, and
    the IMU (which the MCU needs forwarded, or it reports a lift) is live."""
    return bool(link_alive and versions is not None and imu_alive)
