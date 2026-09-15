# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
"""Lockstep guard for the firmware wheel-PI defaults (CLAUDE.md Invariant 15).

The wheel_pid_* defaults exist in three places that must agree: the in-package
TEMPLATE (the value every robot without an override actually runs), the
fallback literal in `mowgli.launch.py` (used only if the template lost the
key), and the `declare_parameter` default in `hardware_bridge_node.cpp` (used
only if the launch file stopped injecting the key). On 2026-09-15 the robot was
found running kp 0.2 / ki 0.092 / integral_limit 15 — in the firmware's
UNSCALED PWM units that caps the wheel loop at ~24 PWM, below the ~40 PWM
static-friction deadband, so the slow inner wheel of every coverage arc simply
never turned. The field-validated set is kp 10 / ki 2000 / kd 0 /
integral_limit 45; this test pins it in all three places and additionally
checks the set can bridge the template's `deadband_pwm` — the same arithmetic
`hardware_bridge_node` applies before pushing the gains to the firmware.

Source-level, no ROS2 runtime (same approach as test_launch_injection.py): the
launch file is parsed with `ast`, the C++ with a regex over the
`bounded_double("wheel_pid_*", <default>, ...)` declarations.
"""
import ast
import re
from pathlib import Path
from typing import Dict

import pytest
import yaml

_PKG_DIR = Path(__file__).resolve().parent.parent
_TEMPLATE_PATH = _PKG_DIR / "config" / "mowgli_robot.yaml"
_LAUNCH_PATH = _PKG_DIR / "launch" / "mowgli.launch.py"
_BRIDGE_PATH = (
    _PKG_DIR.parent / "mowgli_hardware" / "src" / "hardware_bridge_node.cpp"
)

WHEEL_PID_KEYS = (
    "wheel_pid_kp",
    "wheel_pid_ki",
    "wheel_pid_kd",
    "wheel_pid_integral_limit",
    "wheel_pid_pwm_per_mps",
)

# The field-validated set (2026-09-15). Changing it is a deliberate retune:
# update the template, launch, bridge, GUI schema AND this table together.
EXPECTED_DEFAULTS = {
    "wheel_pid_kp": 10.0,
    "wheel_pid_ki": 2000.0,
    "wheel_pid_kd": 0.0,
    "wheel_pid_integral_limit": 45.0,
    "wheel_pid_pwm_per_mps": 282.135,
}
EXPECTED_DEADBAND_PWM = 40.0

# Mirrors mowgli_hardware/drive_gain_sanity.hpp: the slowest wheel the loop
# must still turn (inner wheel of a 0.20 m arc).
_PROBE_SPEED_MPS = 0.03


def _template_params() -> Dict[str, object]:
    with open(_TEMPLATE_PATH, "r", encoding="utf-8") as handle:
        doc = yaml.safe_load(handle) or {}
    return doc.get("mowgli", {}).get("ros__parameters", {})


def _launch_fallbacks() -> Dict[str, float]:
    """`robot_params.get("<key>", <literal>)` fallbacks in mowgli.launch.py."""
    with open(_LAUNCH_PATH, "r", encoding="utf-8") as handle:
        tree = ast.parse(handle.read(), filename=str(_LAUNCH_PATH))
    found: Dict[str, float] = {}
    for node in ast.walk(tree):
        if not (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "get"
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "robot_params"
            and len(node.args) == 2
            and isinstance(node.args[0], ast.Constant)
            and isinstance(node.args[1], ast.Constant)
        ):
            continue
        key = node.args[0].value
        if key in WHEEL_PID_KEYS or key == "deadband_pwm":
            assert key not in found, f"{key} injected twice in mowgli.launch.py"
            found[key] = float(node.args[1].value)
    return found


_BRIDGE_DECL_RE = re.compile(
    r'bounded_double\(\s*"(?P<key>wheel_pid_[a-z_]+)"\s*,\s*(?P<default>[0-9.eE+-]+)\s*,',
    re.MULTILINE,
)
_BRIDGE_DEADBAND_RE = re.compile(
    r'startup_double\(\s*"deadband_pwm"\s*,\s*(?P<default>[0-9.eE+-]+)\s*,',
    re.MULTILINE,
)


def _bridge_declared_defaults() -> Dict[str, float]:
    source = _BRIDGE_PATH.read_text(encoding="utf-8")
    found: Dict[str, float] = {}
    for match in _BRIDGE_DECL_RE.finditer(source):
        key = match.group("key")
        assert key not in found, f"{key} declared twice in hardware_bridge_node.cpp"
        found[key] = float(match.group("default"))
    deadband = _BRIDGE_DEADBAND_RE.search(source)
    assert deadband is not None, "deadband_pwm is no longer declared via startup_double"
    found["deadband_pwm"] = float(deadband.group("default"))
    return found


@pytest.mark.parametrize("key", WHEEL_PID_KEYS)
def test_template_carries_the_field_validated_default(key: str) -> None:
    params = _template_params()
    assert key in params, f"{key} missing from the template (Invariant 15: defaults live there)"
    assert params[key] == pytest.approx(EXPECTED_DEFAULTS[key]), (
        f"template {key}={params[key]} != field-validated {EXPECTED_DEFAULTS[key]}"
    )


def test_template_carries_deadband_pwm() -> None:
    params = _template_params()
    assert params.get("deadband_pwm") == pytest.approx(EXPECTED_DEADBAND_PWM)


@pytest.mark.parametrize("key", WHEEL_PID_KEYS + ("deadband_pwm",))
def test_launch_fallback_matches_template(key: str) -> None:
    fallbacks = _launch_fallbacks()
    template = _template_params()
    assert key in fallbacks, f"mowgli.launch.py no longer injects {key} into hardware_bridge"
    assert fallbacks[key] == pytest.approx(template[key]), (
        f"mowgli.launch.py fallback {key}={fallbacks[key]} drifted from template {template[key]}"
    )


@pytest.mark.parametrize("key", WHEEL_PID_KEYS + ("deadband_pwm",))
def test_bridge_declared_default_matches_template(key: str) -> None:
    declared = _bridge_declared_defaults()
    template = _template_params()
    assert key in declared, f"hardware_bridge_node.cpp no longer declares {key} (regex found none)"
    assert declared[key] == pytest.approx(template[key]), (
        f"hardware_bridge_node.cpp default {key}={declared[key]} drifted from template {template[key]}"
    )


def test_template_gains_bridge_the_deadband() -> None:
    """Same arithmetic as DriveGainsBridgeDeadband(): the template set must be
    able to push a wheel through the stiction deadband at 0.03 m/s."""
    params = _template_params()
    ceiling = (
        float(params["wheel_pid_integral_limit"])
        + float(params["wheel_pid_kp"]) * _PROBE_SPEED_MPS
        + float(params["wheel_pid_pwm_per_mps"]) * _PROBE_SPEED_MPS
    )
    assert ceiling >= float(params["deadband_pwm"]), (
        f"template wheel_pid_* reach only {ceiling:.1f} PWM at {_PROBE_SPEED_MPS} m/s, "
        f"below deadband_pwm {params['deadband_pwm']} — hardware_bridge would substitute the template gains"
    )


def test_old_stiction_locked_set_would_have_failed_the_gate() -> None:
    """Regression pin for the 2026-09-15 finding: the previous template values
    did not clear the deadband, which is why the guard exists."""
    ceiling = 15.0 + 0.2 * _PROBE_SPEED_MPS + 282.135 * _PROBE_SPEED_MPS
    assert ceiling < EXPECTED_DEADBAND_PWM
