# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
from mowgli_airseekers_bridge.contract import (PSS_CHARGING, PSS_DISCHARGING, PSS_NOT_CHARGING,
                                               firmware_compatible, firmware_version_string,
                                               project_power)
from mowgli_airseekers_bridge.protocol import BatteryInfo, VersionInfo


def batt(docked, discharge):
    return BatteryInfo(24.8, discharge, 99, docked, 25, 0)


def test_off_dock():
    f = project_power(batt(False, 0.7))
    assert not f.charger_enabled and f.battery_current == 0.0 and f.charge_current == 0.0
    assert f.power_supply_status == PSS_DISCHARGING and f.charger_status == 'idle'
    assert f.battery_percentage == 0.99


def test_docked_charging_reports_abs_current():
    f = project_power(batt(True, -4.1))
    assert f.charger_enabled and f.power_supply_status == PSS_CHARGING
    assert f.battery_current == 4.1 and f.charge_current == 4.1


def test_docked_full_battery_is_still_docked():
    f = project_power(batt(True, 0.05))
    assert f.charger_enabled                        # is_charging means ON THE DOCK
    assert f.power_supply_status == PSS_NOT_CHARGING
    assert f.charge_current == 0.0 and f.charger_status == 'docked, not charging'


def test_firmware_compatible_needs_link_versions_and_imu():
    v = VersionInfo((0, 6, 36), (0, 6, 34), (0, 0, 0))
    assert firmware_compatible(True, v, True)
    assert not firmware_compatible(False, v, True)
    assert not firmware_compatible(True, None, True)
    assert not firmware_compatible(True, v, False)
    assert '0.6.36' in firmware_version_string(v)


def test_leaving_the_pads_drops_docked_and_charging():
    """dock_ok is the whole docked signal (Status.is_charging, Power,
    BatteryState): when the forward undock lifts the robot off the contacts,
    every docked field drops with it, so the normal driving rules take over."""
    on = project_power(batt(True, -4.1))
    off = project_power(batt(False, 0.7))
    assert on.charger_enabled and on.battery_current > 1.0
    assert not off.charger_enabled
    assert off.battery_current == 0.0 and off.charge_current == 0.0
