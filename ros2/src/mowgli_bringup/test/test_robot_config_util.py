import math
# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
#
# Unit tests for _util.load_robot_params — the deep-merge that
# lets the INSTALLED mowgli_robot.yaml be sparse (install choices + calibration
# outputs only) while every other default falls through to the in-package
# template. These run without any ROS deps — only PyYAML is required.

import importlib.util
import json
import os
import sys
import tempfile
from pathlib import Path

import pytest
import yaml

# mowgli_bringup package root: test/ -> package dir; the template lives at
# <pkg>/config/mowgli_robot.yaml and the helper at <pkg>/launch/.
_PKG_DIR = Path(__file__).resolve().parent.parent
_LAUNCH_DIR = _PKG_DIR / "launch"
_TEMPLATE_PATH = _PKG_DIR / "config" / "mowgli_robot.yaml"


def _load_helper():
    """Import robot_config_util.py directly from the launch dir (no ROS)."""
    sys.path.insert(0, str(_LAUNCH_DIR))
    spec = importlib.util.spec_from_file_location(
        "robot_config_util", str(_LAUNCH_DIR / "robot_config_util.py"))
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_util = _load_helper()
load_robot_params = _util.load_robot_params
# Pure turn-geometry / turn-speed helpers (issue #499) — see the tests at the
# bottom of this file. Kept in robot_config_util because navigation.launch.py
# imports launch/launch_ros and cannot be imported outside a sourced ROS2
# install, so the arithmetic would otherwise be untestable.
derive_turn_speed = _util.derive_turn_speed
check_turn_geometry = _util.check_turn_geometry

# Launch files that must derive their tool_width fallback from the one shared
# constant instead of each hardcoding their own literal (task #17 — that
# duplication, mower_width=0.18 vs a separately-hardcoded operation_width=0.20,
# caused the 54% coverage regression: map_server's mark_cells_mowed stamp
# radius went narrower than F2C's swath spacing, leaving an un-mowed strip
# between every pair of adjacent swaths).
_MAP_SERVER_LAUNCH = _LAUNCH_DIR / "full_system.launch.py"
_COVERAGE_LAUNCH = _LAUNCH_DIR / "navigation.launch.py"


def _template_params() -> dict:
    with open(_TEMPLATE_PATH, "r") as handle:
        doc = yaml.safe_load(handle) or {}
    return doc.get("mowgli", {}).get("ros__parameters", {})


def _write_sparse(params: dict) -> str:
    """Write a sparse runtime config to a temp file, return its path."""
    tmp = tempfile.NamedTemporaryFile(
        mode="w", prefix="test_robot_cfg_", suffix=".yaml", delete=False)
    yaml.safe_dump({"mowgli": {"ros__parameters": params}}, tmp)
    tmp.close()
    return tmp.name


def test_template_only_returns_full_defaults():
    """A nonexistent runtime path yields the pure template defaults."""
    template = _template_params()
    assert template, "template must be non-empty for this test to mean anything"

    merged = load_robot_params(
        str(_PKG_DIR), runtime_path="/nonexistent/mowgli_robot.yaml")
    assert merged == template


def test_sparse_overrides_and_falls_through():
    """Sparse runtime overrides its own keys; absent keys use the template."""
    template = _template_params()
    # Pick a key present in the template to override.
    assert "ticks_per_meter" in template
    original = template["ticks_per_meter"]
    override_val = float(original) + 111.0

    path = _write_sparse({
        "ticks_per_meter": override_val,
        "datum_lat": 48.123456,
        # An install-decided key that is ABSENT from the template — it must
        # still surface (and its presence is what launch files test for).
        "lidar_enabled": False,
    })
    try:
        merged = load_robot_params(str(_PKG_DIR), runtime_path=path)
    finally:
        os.unlink(path)

    # Overridden keys win.
    assert merged["ticks_per_meter"] == override_val
    assert merged["datum_lat"] == 48.123456
    # Install-decided key surfaces even though the template lacks it.
    assert "lidar_enabled" in merged
    assert merged["lidar_enabled"] is False
    # Untouched keys fall through to the template default.
    for key, value in template.items():
        if key in ("ticks_per_meter", "datum_lat"):
            continue
        assert merged[key] == value


def test_removing_key_reverts_to_template_default():
    """Reset-to-default: dropping a key from the sparse runtime reverts it."""
    template = _template_params()
    assert "ticks_per_meter" in template
    default_val = template["ticks_per_meter"]

    # Runtime that DOES override the key.
    with_key = _write_sparse({"ticks_per_meter": float(default_val) + 55.0})
    # Runtime that omits it entirely (simulates the GUI deleting the key).
    without_key = _write_sparse({"datum_lat": 1.0})
    try:
        merged_with = load_robot_params(str(_PKG_DIR), runtime_path=with_key)
        merged_without = load_robot_params(
            str(_PKG_DIR), runtime_path=without_key)
    finally:
        os.unlink(with_key)
        os.unlink(without_key)

    assert merged_with["ticks_per_meter"] == float(default_val) + 55.0
    # Key absent from runtime -> template default restored.
    assert merged_without["ticks_per_meter"] == default_val


def test_full_runtime_merges_to_itself():
    """Backward compat: a FULL runtime config (every template key) is a no-op
    merge — the merged result equals that full runtime for every template key."""
    template = _template_params()
    # Build a full runtime = template with every value bumped, so we can prove
    # each one wins over the identical-key template default.
    full = {}
    for key, value in template.items():
        if isinstance(value, bool):
            full[key] = not value
        elif isinstance(value, (int, float)):
            full[key] = value + 1
        else:
            full[key] = value  # strings/lists kept as-is

    path = _write_sparse(full)
    try:
        merged = load_robot_params(str(_PKG_DIR), runtime_path=path)
    finally:
        os.unlink(path)

    for key in template:
        assert merged[key] == full[key]


def test_default_tool_width_matches_template():
    """The last-resort DEFAULT_TOOL_WIDTH_M fallback must agree with the
    template's tool_width — it is a floor beneath the real (template) single
    source of truth, not a second one. If these ever diverge, a robot whose
    installed config AND the template both somehow lack the key would get a
    silently different value in map_server vs coverage_server again."""
    template = _template_params()
    assert "tool_width" in template
    assert _util.DEFAULT_TOOL_WIDTH_M == template["tool_width"]


def test_map_server_and_coverage_launch_share_tool_width_default():
    """Regression guard: full_system.launch.py (map_server.tool_width) and
    navigation.launch.py (feeds coverage_server.operation_width) must both
    fall back to _util.DEFAULT_TOOL_WIDTH_M — not each hardcode
    their own literal. This is a source-text check (launch files pull in the
    full `launch`/`launch_ros`/ament ROS2 packages via generate_launch_description,
    so they cannot be imported directly in a plain-Python test); it fails
    loudly if either file stops importing/using the shared constant, or if a
    stray hardcoded tool_width fallback literal (e.g. "0.18") reappears.
    """
    map_server_src = _MAP_SERVER_LAUNCH.read_text()
    coverage_src = _COVERAGE_LAUNCH.read_text()

    assert "DEFAULT_TOOL_WIDTH_M" in map_server_src, (
        f"{_MAP_SERVER_LAUNCH.name} must import DEFAULT_TOOL_WIDTH_M from robot_config_util"
    )
    assert "DEFAULT_TOOL_WIDTH_M" in coverage_src, (
        f"{_COVERAGE_LAUNCH.name} must import DEFAULT_TOOL_WIDTH_M from robot_config_util"
    )
    # The fallback for the tool_width param itself must be the shared
    # constant, not a bare numeric literal re-hardcoded at the call site.
    assert 'robot_params.get("tool_width", DEFAULT_TOOL_WIDTH_M)' in map_server_src
    assert "tool_width = DEFAULT_TOOL_WIDTH_M" in coverage_src


# ── Coverage turn geometry / turn speed (issue #499) ─────────────────────────
#
# The violent swath-end turns that dig the lawn were traced to two numbers that
# must be consistent but live in different files with nothing relating them:
# coverage_server's turn radii (mowgli_robot.yaml) and FollowCoveragePath's
# speed/angular clamps (nav2_params_base.yaml). These exercise the ARITHMETIC —
# navigation.launch.py only prints what these return.


class TestDeriveTurnSpeed:
    """FollowCoveragePath.speed_slow derived from the operator's mowing_speed."""

    def test_scales_mowing_speed_by_the_ratio(self):
        # Arrange / Act
        speed, warnings = derive_turn_speed(0.20, 0.8, 0.10)

        # Assert — 0.8 x 0.20 reproduces the historical static 0.16 exactly, so a
        # robot on the default mowing_speed sees no behaviour change.
        assert speed == pytest.approx(0.16)
        assert warnings == []

    def test_turn_is_never_faster_than_the_straight(self):
        """THE defect: with a static speed_slow=0.16 an operator on
        mowing_speed=0.15 drove swath-end turns 7 % FASTER than the straights."""
        # Arrange — the exact live robot config from the 2026-08-24 logs.
        mowing_speed = 0.15

        # Act
        speed, _ = derive_turn_speed(mowing_speed, 0.8, 0.10)

        # Assert
        assert speed <= mowing_speed, (
            f"turn speed {speed} exceeds mowing_speed {mowing_speed} — the issue "
            "#499 defect is back")

    def test_ratio_above_one_is_clamped_and_warned(self):
        # Arrange / Act — a ratio > 1 IS the defect, expressed as config.
        speed, warnings = derive_turn_speed(0.20, 1.5, 0.10)

        # Assert
        assert speed == pytest.approx(0.20), "clamped ratio must be exactly 1.0"
        assert any("outside (0, 1.0]" in w for w in warnings)

    def test_ratio_at_or_below_zero_is_clamped(self):
        """A zero/negative ratio would stop the robot dead in every bend."""
        # Arrange / Act
        speed, warnings = derive_turn_speed(0.20, 0.0, 0.01)

        # Assert
        assert speed > 0.0
        assert any("outside (0, 1.0]" in w for w in warnings)

    def test_floored_at_min_speed_with_a_warning(self):
        """Below FTC's own min_speed_mps the target is a fiction — FTC floors its
        output there regardless, and the wheels stall under the firmware
        deadband. Say so rather than inject a value that cannot happen."""
        # Arrange — 0.6 x 0.15 = 0.09, under min_speed_mps.
        # Act
        speed, warnings = derive_turn_speed(0.15, 0.6, 0.15)

        # Assert
        assert speed == pytest.approx(0.15)
        assert any("min_speed_mps" in w for w in warnings)

    def test_ceiling_wins_over_floor_when_they_conflict(self):
        """min_speed_mps above mowing_speed: the turn must still never exceed the
        straight, so the mowing_speed ceiling is the binding constraint."""
        # Arrange / Act
        speed, _ = derive_turn_speed(0.12, 0.8, 0.20)

        # Assert
        assert speed <= 0.12


class TestCheckTurnGeometry:
    """Undrivable planned turn radii — reported, never raised."""

    # The deployed pair as of issue #499: floor below the half-track.
    LIVE_TRACK = 0.325
    LIVE_MIN_R = 0.15
    LIVE_CONN_R = 0.18
    LIVE_TURN_SPEED = 0.16
    LIVE_WZ_MAX = 0.8

    def test_flags_the_live_config_inner_wheel_reversal(self):
        """min_turning_radius 0.15 <= half-track 0.1625: the inner wheel must
        REVERSE to trace the arc. This is the shipped state, and it must warn."""
        # Act
        warnings = check_turn_geometry(self.LIVE_MIN_R, self.LIVE_CONN_R,
                                       self.LIVE_TRACK, self.LIVE_TURN_SPEED,
                                       self.LIVE_WZ_MAX)

        # Assert
        assert any("half-track" in w for w in warnings), (
            "the deployed 0.15 m floor against a 0.325 m track is exactly the "
            "geometry that carves the lawn and must be reported")

    def test_flags_the_planner_controller_mismatch(self):
        """The tightest arc FTC can command is speed_slow/max_cmd_vel_ang =
        0.16/0.8 = 0.20 m, but coverage plans down to 0.15 m."""
        # Act
        warnings = check_turn_geometry(self.LIVE_MIN_R, self.LIVE_CONN_R,
                                       self.LIVE_TRACK, self.LIVE_TURN_SPEED,
                                       self.LIVE_WZ_MAX)

        # Assert
        assert any("max_cmd_vel_ang" in w for w in warnings)

    def test_silent_when_the_geometry_is_drivable(self):
        """Radius comfortably above the half-track AND above the commandable
        floor — nothing to say."""
        # Arrange — 0.40 m arcs: inner/outer ratio 0.42, needs wz = 0.16/0.40 = 0.4.
        # Act
        warnings = check_turn_geometry(0.40, 0.45, self.LIVE_TRACK,
                                       self.LIVE_TURN_SPEED, self.LIVE_WZ_MAX)

        # Assert
        assert warnings == []

    def test_reports_the_actual_wheel_speeds(self):
        """The warning has to carry NUMBERS — the whole failure was that the two
        offending values lived in different files and nobody related them."""
        # Act
        warnings = check_turn_geometry(self.LIVE_MIN_R, self.LIVE_CONN_R,
                                       self.LIVE_TRACK, self.LIVE_TURN_SPEED,
                                       self.LIVE_WZ_MAX)

        # Assert — v_inner = v(1 - b/R) = 0.16 * (1 - 0.1625/0.15) = -0.013 m/s.
        carve = next(w for w in warnings if "half-track" in w)
        assert "-0.013" in carve, f"inner-wheel speed missing from: {carve}"

    def test_boundary_radius_equal_to_half_track_still_warns(self):
        """At R == half-track the inner wheel is at exactly zero — the wheel is
        dragged, not rolled. Still carving; must not be treated as fine."""
        # Act
        warnings = check_turn_geometry(0.1625, 0.20, 0.325, 0.16, 0.8)

        # Assert
        assert any("half-track" in w for w in warnings)

    def test_never_raises_on_any_input(self):
        """WARN-only is load-bearing: the shipped defaults trip this, so raising
        would refuse to start navigation on every existing robot. Firmware stays
        the sole blade-safety authority either way."""
        # Arrange — including degenerate values the caller's clamps would contain.
        for args in [(0.0, 0.0, 0.325, 0.16, 0.8),
                     (0.15, 0.18, 0.0, 0.16, 0.8),
                     (0.15, 0.18, 0.325, 0.0, 0.0),
                     (5.0, 5.0, 0.325, 0.16, 0.8)]:
            # Act / Assert — must return a list, never throw.
            assert isinstance(check_turn_geometry(*args), list)


# ---------------------------------------------------------------------------
# LiDAR presence: config only, never the LIDAR_ENABLED env var
# ---------------------------------------------------------------------------
#
# The env var used to be a "fallback when the yaml is silent", and on a live
# robot that fallback WAS the bug: `lidar_enabled` absent from the installed
# config plus a stale `docker/.env` saying `LIDAR_ENABLED=false` ran the whole
# stack GPS-only while the operator toggled LiDAR on in the GUI and saw nothing
# change. These pin that the env var is inert, that yaml true/false resolve,
# and that an absent key resolves to the documented default AND says so loudly.

def _find_schema_property(node, name):
    """Depth-first search of a JSON schema for a named property definition.

    The GUI schema groups fields into sections (hardware_settings, ...), so the
    path to a given key is not fixed. Returns the property dict or None.
    """
    if not isinstance(node, dict):
        return None
    properties = node.get("properties")
    if isinstance(properties, dict):
        if name in properties:
            return properties[name]
        for child in properties.values():
            found = _find_schema_property(child, name)
            if found is not None:
                return found
    return None


class TestLidarEnabledResolution:
    """_util.resolve_lidar_enabled + its absent-key warning."""

    def setup_method(self):
        # warn_lidar_key_absent dedupes per process; clear between tests.
        _util._LIDAR_WARNED_PATHS.clear()

    def test_yaml_true_resolves_enabled(self):
        # Arrange / Act
        enabled, explicit = _util.resolve_lidar_enabled({"lidar_enabled": True})

        # Assert
        assert enabled is True
        assert explicit is True

    def test_yaml_false_resolves_disabled(self):
        # Arrange / Act
        enabled, explicit = _util.resolve_lidar_enabled({"lidar_enabled": False})

        # Assert
        assert enabled is False
        assert explicit is True

    def test_absent_key_resolves_to_default_and_is_not_explicit(self):
        """Absent means "no LiDAR was ever recorded" -> DEFAULT_LIDAR_ENABLED.

        The installer always writes the key, so absence is a hand-rolled
        deployment; a wrongly-OFF stack is coherent GPS-only operation, whereas
        a wrongly-ON one is the broken half-state (obstacle layer with no
        observation source, fusion_graph subscribed to a dead topic).
        """
        # Arrange / Act
        enabled, explicit = _util.resolve_lidar_enabled({"datum_lat": 48.0})

        # Assert
        assert enabled is _util.DEFAULT_LIDAR_ENABLED
        assert _util.DEFAULT_LIDAR_ENABLED is False
        assert explicit is False

    @pytest.mark.parametrize(
        "env_value", ["false", "0", "no", "true", "1", "yes", ""])
    def test_env_var_does_not_influence_resolution(self, monkeypatch, env_value):
        """LIDAR_ENABLED must be inert in BOTH directions, for every key state.

        That is the whole point: an ambient env var is not an operator decision.
        """
        # Arrange
        monkeypatch.setenv("LIDAR_ENABLED", env_value)

        # Act / Assert — explicit true, explicit false and absent all ignore it.
        assert _util.resolve_lidar_enabled({"lidar_enabled": True})[0] is True
        assert _util.resolve_lidar_enabled({"lidar_enabled": False})[0] is False
        assert _util.resolve_lidar_enabled({})[0] is _util.DEFAULT_LIDAR_ENABLED

    def test_launch_files_never_read_the_env_var(self):
        """Source-text guard: neither launch file may look up LIDAR_ENABLED.

        The launch files pull in the full launch/launch_ros/ament stack via
        generate_launch_description, so they cannot be imported in a plain
        Python test; this fails loudly if the fallback is ever re-added.
        """
        # Arrange / Act
        sources = {
            _MAP_SERVER_LAUNCH.name: _MAP_SERVER_LAUNCH.read_text(),
            _COVERAGE_LAUNCH.name: _COVERAGE_LAUNCH.read_text(),
        }

        # Assert — the name may appear in prose, but never in an env lookup.
        for name, src in sources.items():
            assert 'environ.get("LIDAR_ENABLED"' not in src, (
                f"{name} must not read the LIDAR_ENABLED environment variable")
            assert 'getenv("LIDAR_ENABLED"' not in src, (
                f"{name} must not read the LIDAR_ENABLED environment variable")
            assert "resolve_lidar_enabled" in src, (
                f"{name} must resolve lidar_enabled through robot_config_util")

    def test_absent_key_warning_names_file_key_and_mode(self):
        """Silence is what made the original diagnosis take an investigation."""
        # Arrange
        path = "/ros2_ws/config/mowgli_robot.yaml"

        # Act
        message = _util.lidar_absent_warning(path)

        # Assert
        assert path in message
        assert "lidar_enabled" in message
        assert "use_lidar=false" in message
        # States that the env var is no longer consulted.
        assert "LIDAR_ENABLED" in message

    def test_warn_lidar_key_absent_emits_once_per_path(self):
        """full_system + navigation both resolve in ONE process; a doubled
        multi-line warning teaches the operator to skim it."""
        # Arrange
        emitted = []

        class _Collector:
            def warning(self, message):
                emitted.append(message)

        logger = _Collector()

        # Act
        first = _util.warn_lidar_key_absent("/tmp/cfg.yaml", logger=logger)
        second = _util.warn_lidar_key_absent("/tmp/cfg.yaml", logger=logger)

        # Assert
        assert first is not None
        assert second is None
        assert len(emitted) == 1

    def test_schema_default_matches_resolution_default(self):
        """The GUI settings backend PRUNES any value equal to its schema default
        (sparsifyFlat, Invariant 15). If the schema said true while this resolves
        absent->false, an operator switching LiDAR ON would write `true`, have it
        pruned as "same as default", and the toggle would be inert in the ON
        direction forever."""
        # Arrange
        schema_path = (_PKG_DIR.parents[2] / "gui" / "asserts"
                       / "mower_config.schema.json")
        if not schema_path.is_file():
            pytest.skip("gui/asserts/mower_config.schema.json not in this tree")

        # Act — the schema groups fields into sections, so walk for the key.
        with open(schema_path, "r") as handle:
            schema = json.load(handle)
        found = _find_schema_property(schema, "lidar_enabled")

        # Assert
        assert found is not None, "lidar_enabled missing from the GUI schema"
        assert found["default"] is _util.DEFAULT_LIDAR_ENABLED


class TestScanFactorsFollowLidar:
    """use_scan_matching / use_loop_closure must follow use_lidar.

    The gating leaked: both default from the TEMPLATE (`true` for each), which
    has no relation to `lidar_enabled`, so a GPS-only stack still handed
    fusion_graph use_scan_matching=True. Observed live 2026-08-31 with
    use_lidar=false: use_scan_matching=True, use_loop_closure=True,
    /scan_deskewed publisher count 0, scans_received 0.

    navigation.launch.py ANDs each flag with use_lidar at SUBSTITUTION time (so
    a CLI `use_lidar:=` override is covered too). These tests evaluate the same
    expression the launch file builds, and pin that it is actually applied to
    both flags.
    """

    @staticmethod
    def _gate(lidar_text, flag_text):
        """Evaluate the launch file's LiDAR gate for two launch-arg texts.

        eval() is deliberate and safe here: it is the ONLY way to exercise the
        real semantics of launch.substitutions.PythonExpression, which itself
        evaluates this exact source string at launch time. The inputs are
        hard-coded literals from the parametrize lists below (never user or
        file data), and it runs with empty builtins.
        """
        tokens = str(_util.TRUE_TOKENS)
        expr = ("'true' if '" + lidar_text + "'.strip().lower() in " + tokens
                + " and '" + flag_text + "'.strip().lower() in " + tokens
                + " else 'false'")
        return eval(expr, {"__builtins__": {}})  # noqa: S307

    @pytest.mark.parametrize("flag", ["use_scan_matching", "use_loop_closure"])
    def test_lidar_off_forces_flag_off(self, flag):
        """No scanner -> no scan factors, whatever the yaml asked for."""
        # Arrange / Act / Assert
        assert self._gate("false", "true") == "false", flag
        assert self._gate("false", "false") == "false", flag

    @pytest.mark.parametrize("flag", ["use_scan_matching", "use_loop_closure"])
    def test_lidar_on_passes_the_operator_choice_through(self, flag):
        """The gate must not become an override: with a LiDAR present the
        operator's own use_scan_matching / use_loop_closure still decides."""
        # Arrange / Act / Assert
        assert self._gate("true", "true") == "true", flag
        assert self._gate("true", "false") == "false", flag

    def test_navigation_launch_gates_both_flags(self):
        """Source-text guard that the gate is wired to BOTH flags where they are
        handed to fusion_graph (a pure truth-table test cannot see that)."""
        # Arrange / Act
        src = _COVERAGE_LAUNCH.read_text()

        # Assert
        assert '"use_scan_matching": lidar_gated(use_scan_matching)' in src
        assert '"use_loop_closure": lidar_gated(use_loop_closure)' in src

    def test_scan_deskew_warns_when_lidar_on_but_no_scans(self):
        """The opposite mismatch: config says LiDAR on, the mowgli-lidar
        CONTAINER was never started (docker/.env still owns that), so no scan
        ever arrives. scan_deskew_node is itself use_lidar-gated, so it is the
        cheapest honest place to notice. Must WARN, never abort."""
        # Arrange
        node_src = (_PKG_DIR.parent / "mowgli_localization" / "src"
                    / "scan_deskew_node.cpp")
        assert node_src.is_file(), node_src

        # Act
        src = node_src.read_text()

        # Assert
        assert "scan_watchdog_period_s" in src
        assert "NO LiDAR SCANS" in src
        assert "LIDAR_ENABLED in docker/.env" in src
        assert "RCLCPP_WARN" in src
        # Non-fatal: the silent path must not shut the node down or throw.
        watchdog = src.split("void on_scan_watchdog()")[1].split("void on_scan(")[0]
        assert "rclcpp::shutdown()" not in watchdog
        assert "throw " not in watchdog


# ---------------------------------------------------------------------------
# Chassis footprint geometry
#
# The local-costmap inflation floor is DERIVED from these, because chassis
# dimensions are operator-editable in the GUI. Before 2026-09-05 the floor was
# a literal (0.58) that had drifted below the real circumscribed radius
# (0.586 m at the then-shipped chassis_width 0.40) without anyone noticing.
# These pin the derivation so that cannot recur.
# ---------------------------------------------------------------------------

def test_chassis_footprint_uses_template_defaults_when_params_empty():
    front, rear, half_width = _util.chassis_footprint({})
    assert front == pytest.approx(0.18 + 0.60 / 2 + 0.05)
    assert rear == pytest.approx(0.18 - 0.60 / 2 - 0.05)
    assert half_width == pytest.approx(0.45 / 2 + 0.05)


def test_chassis_footprint_follows_the_params():
    front, rear, half_width = _util.chassis_footprint(
        {"chassis_length": 0.50, "chassis_width": 0.30, "chassis_center_x": 0.10}
    )
    assert front == pytest.approx(0.10 + 0.25 + 0.05)
    assert rear == pytest.approx(0.10 - 0.25 - 0.05)
    assert half_width == pytest.approx(0.15 + 0.05)


def test_circumscribed_radius_at_shipped_dimensions():
    # sqrt(0.530^2 + 0.275^2); the shipped inflation floor must not sit below it.
    assert _util.chassis_circumscribed_radius({}) == pytest.approx(0.5971, abs=1e-4)


def test_circumscribed_radius_grows_with_a_wider_chassis():
    narrow = _util.chassis_circumscribed_radius({"chassis_width": 0.40})
    wide = _util.chassis_circumscribed_radius({"chassis_width": 0.45})
    assert wide > narrow
    # The regression this guards: 0.58 was shipped as the floor while the real
    # radius at chassis_width 0.40 was already 0.586 m.
    assert narrow == pytest.approx(0.5860, abs=1e-4)


def test_circumscribed_radius_encloses_every_footprint_corner():
    params = {"chassis_length": 0.72, "chassis_width": 0.51, "chassis_center_x": 0.22}
    front, rear, half_width = _util.chassis_footprint(params)
    radius = _util.chassis_circumscribed_radius(params)
    for x in (front, rear):
        for y in (half_width, -half_width):
            assert math.hypot(x, y) <= radius + 1e-9


# ---------------------------------------------------------------------------
# Blade-load slowdown (FollowCoveragePath.blade_load_*)
#
# The FTC decision itself is unit-tested in mowgli_nav2_plugins
# (test_ftc_blade_load.cpp); these cover the launch-side validation that turns
# the operator's four mowgli_robot.yaml keys into the injected FTC params.


class TestDeriveBladeLoadParams:
    """navigation.launch.py injection of the blade-load slowdown knobs."""

    def test_passes_a_valid_config_through_unchanged(self):
        # Arrange / Act
        params, warnings = _util.derive_blade_load_params(True, 2500.0, 1800.0, 0.4)

        # Assert
        assert warnings == []
        assert params == {
            "blade_load_slowdown_enabled": True,
            "blade_load_rpm_full": 2500.0,
            "blade_load_rpm_min": 1800.0,
            "blade_load_min_speed_ratio": 0.4,
        }

    def test_template_defaults_are_disabled_and_warning_free(self):
        params, warnings = _util.derive_blade_load_params(
            False,
            _util.DEFAULT_BLADE_LOAD_RPM_FULL,
            _util.DEFAULT_BLADE_LOAD_RPM_MIN,
            _util.DEFAULT_BLADE_LOAD_MIN_SPEED_RATIO,
        )
        assert warnings == []
        assert params["blade_load_slowdown_enabled"] is False
        # A disabled feature must still ship a usable ramp so flipping the GUI
        # toggle alone engages it.
        assert params["blade_load_rpm_full"] > params["blade_load_rpm_min"]

    def test_accepts_yaml_string_booleans(self):
        # The sparse installed file may carry the key as a string token.
        params, _ = _util.derive_blade_load_params("true", 2500.0, 1800.0, 0.4)
        assert params["blade_load_slowdown_enabled"] is True
        params, _ = _util.derive_blade_load_params("false", 2500.0, 1800.0, 0.4)
        assert params["blade_load_slowdown_enabled"] is False

    def test_empty_ramp_disables_with_a_warning(self):
        # rpm_full == rpm_min: FTC would fail open silently — say so and disable.
        params, warnings = _util.derive_blade_load_params(True, 2000.0, 2000.0, 0.4)
        assert params["blade_load_slowdown_enabled"] is False
        assert len(warnings) == 1
        assert "ramp is empty" in warnings[0]

    def test_inverted_ramp_disables_with_a_warning(self):
        params, warnings = _util.derive_blade_load_params(True, 1500.0, 2500.0, 0.4)
        assert params["blade_load_slowdown_enabled"] is False
        assert any("DISABLED" in w for w in warnings)

    def test_bad_ramp_is_silent_when_the_feature_is_off(self):
        # Nothing to warn about: the thresholds are inert while disabled.
        _, warnings = _util.derive_blade_load_params(False, 1500.0, 2500.0, 0.4)
        assert warnings == []

    def test_ratio_above_one_is_clamped(self):
        params, warnings = _util.derive_blade_load_params(True, 2500.0, 1800.0, 1.5)
        assert params["blade_load_min_speed_ratio"] == 1.0
        assert any("clamped" in w for w in warnings)

    def test_ratio_floor_prevents_parking_the_robot(self):
        params, warnings = _util.derive_blade_load_params(True, 2500.0, 1800.0, 0.0)
        assert params["blade_load_min_speed_ratio"] == pytest.approx(
            _util.BLADE_LOAD_MIN_SPEED_RATIO_FLOOR)
        assert any("clamped" in w for w in warnings)
