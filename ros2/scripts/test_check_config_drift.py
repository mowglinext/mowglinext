#!/usr/bin/env python3
# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later

"""Tests for check_config_drift.py (run with pytest).

Covers the three checks the script enforces over the sparse-config model
(Architecture Invariant 15):
  * structural drift (both files, differing values)
  * structural key without a template default
  * USER_OVERRIDE key padded with the template default (issue #381)
plus a regression test that the actual committed yaml pair passes.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import check_config_drift as ccd  # noqa: E402


# ── find_padded_defaults (issue #381) ────────────────────────────────


def test_flags_key_equal_to_template_default():
    # Arrange — swath_overlap is the issue's example of a padded default.
    tpl = {"swath_overlap": 0.02, "mowing_speed": 0.2}
    rt = {"swath_overlap": 0.02}

    # Act
    padded = ccd.find_padded_defaults(tpl, rt)

    # Assert
    assert padded == [("swath_overlap", 0.02)]


def test_flags_structural_key_equal_to_template_default():
    # tool_width padded into the sparse file is the issue's motivating
    # example — the historical 54%-coverage bug class.
    tpl = {"tool_width": 0.18}
    rt = {"tool_width": 0.18}

    assert ccd.find_padded_defaults(tpl, rt) == [("tool_width", 0.18)]


def test_ignores_genuinely_overridden_value():
    tpl = {"mowing_speed": 0.2}
    rt = {"mowing_speed": 0.35}

    assert ccd.find_padded_defaults(tpl, rt) == []


def test_ignores_key_absent_from_installed_file():
    tpl = {"mowing_speed": 0.2, "swath_overlap": 0.02}
    rt = {}

    assert ccd.find_padded_defaults(tpl, rt) == []


def test_ignores_key_absent_from_template():
    # lidar_enabled-style keys exist only in the installed file — there is
    # no template default to duplicate.
    tpl = {}
    rt = {"lidar_enabled": True}

    assert ccd.find_padded_defaults(tpl, rt) == []


def test_ignores_calibration_output_matching_template_seed():
    # A freshly-uncalibrated robot legitimately carries template-equal
    # calibration seeds (dock pose 0/0/0, imu_yaw 0.0).
    tpl = {"dock_pose_x": 0.0, "imu_yaw": 0.0, "ticks_per_meter": 399.0}
    rt = {"dock_pose_x": 0.0, "imu_yaw": 0.0, "ticks_per_meter": 399.0}

    assert ccd.find_padded_defaults(tpl, rt) == []


def test_ignores_install_seed_placeholders():
    # Bucket A placeholders are deliberately committed equal to the template
    # defaults so the installer / GUI can line-splice real values in.
    tpl = {"datum_lat": 0.0, "ntrip_enabled": False, "ntrip_port": 2101}
    rt = {"datum_lat": 0.0, "ntrip_enabled": False, "ntrip_port": 2101}

    assert ccd.find_padded_defaults(tpl, rt) == []


def test_flags_uncategorized_keys_too():
    # A tunable added to the template AFTER this script was last touched
    # must still be covered — no maintained key list to fall out of date.
    tpl = {"some_future_key": 1}
    rt = {"some_future_key": 1}

    assert ccd.find_padded_defaults(tpl, rt) == [("some_future_key", 1)]


def test_reports_multiple_padded_keys_sorted():
    tpl = {"transit_speed": 0.2, "mowing_speed": 0.2, "rain_mode": 2}
    rt = {"transit_speed": 0.2, "mowing_speed": 0.2, "rain_mode": 2}

    padded = ccd.find_padded_defaults(tpl, rt)

    assert padded == [
        ("mowing_speed", 0.2),
        ("rain_mode", 2),
        ("transit_speed", 0.2),
    ]


def test_install_seed_keys_are_categorized():
    # Guard against typos: an INSTALL_SEED entry outside the categorized sets
    # exempts nothing (find_padded_defaults only iterates USER_OVERRIDE).
    assert ccd.INSTALL_SEED <= ccd.STRUCTURAL | ccd.USER_OVERRIDE


# ── find_structural_issues (pre-existing checks, now extracted) ──────


def test_structural_drift_on_differing_values():
    tpl = {"chassis_width": 0.40}
    rt = {"chassis_width": 0.54}

    drift, no_default = ccd.find_structural_issues(tpl, rt)

    assert drift == [("chassis_width", 0.40, 0.54)]
    assert no_default == []


def test_structural_key_only_in_template_is_healthy():
    tpl = {"chassis_width": 0.40}
    rt = {}

    drift, no_default = ccd.find_structural_issues(tpl, rt)

    assert drift == []
    assert no_default == []


def test_structural_key_without_template_default_is_flagged():
    tpl = {}
    rt = {"chassis_width": 0.40}

    drift, no_default = ccd.find_structural_issues(tpl, rt)

    assert drift == []
    assert no_default == [("chassis_width", 0.40)]


def test_structural_calibration_output_is_exempt():
    # ticks_per_meter is STRUCTURAL and a calibration output — per-robot
    # divergence from the template seed is expected, never drift.
    tpl = {"ticks_per_meter": 399.0}
    rt = {"ticks_per_meter": 412.3}

    drift, no_default = ccd.find_structural_issues(tpl, rt)

    assert drift == []
    assert no_default == []


# ── regression: the committed yaml pair must pass every check ────────


def test_committed_repo_configs_are_clean():
    tpl = ccd.load(ccd.TEMPLATE)
    rt = ccd.load(ccd.RUNTIME)

    drift, no_default = ccd.find_structural_issues(tpl, rt)
    padded = ccd.find_padded_defaults(tpl, rt)

    assert drift == []
    assert no_default == []
    assert padded == []
