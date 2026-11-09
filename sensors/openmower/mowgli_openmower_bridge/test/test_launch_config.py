# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
"""Parameter layering of launch/openmower_bridge.launch.py.

Run from the repository checkout (it reads the ROS2 robot template in place):
    python3 -m pytest sensors/openmower/mowgli_openmower_bridge/test/test_launch_config.py
"""

import importlib.util
import os

import pytest
import yaml

PKG = os.path.normpath(os.path.join(os.path.dirname(__file__), '..'))
REPO = os.path.normpath(os.path.join(PKG, '..', '..', '..'))
TEMPLATE = os.path.join(REPO, 'ros2', 'src', 'mowgli_bringup', 'config', 'mowgli_robot.yaml')
BACKEND = os.path.join(os.path.dirname(TEMPLATE), 'backends', 'openmower.yaml')
DEFAULTS = os.path.join(PKG, 'config', 'openmower_bridge.yaml')
ENV_KEYS = ('OPENMOWER_LL_PORT', 'OPENMOWER_XESC_TYPE', 'OPENMOWER_XESC_LEFT_PORT',
            'OPENMOWER_XESC_RIGHT_PORT', 'OPENMOWER_XESC_MOW_PORT', 'OPENMOWER_MOW_XESC_ENABLED')


def _load_launch():
    spec = importlib.util.spec_from_file_location(
        'openmower_bridge_launch', os.path.join(PKG, 'launch', 'openmower_bridge.launch.py'))
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


launch = _load_launch()


def _params(path):
    with open(path, encoding='utf-8') as fh:
        return yaml.safe_load(fh)


def _effective(layers):
    """Fold the launch layers the way ROS does: later dicts win."""
    out = dict(_params(layers[0])['hardware_bridge']['ros__parameters'])
    for layer in layers[1:]:
        out.update(layer)
    return out


@pytest.fixture(autouse=True)
def _clean_env(monkeypatch):
    for key in ENV_KEYS:
        monkeypatch.delenv(key, raising=False)


def _write_config(tmp_path, body):
    path = tmp_path / 'mowgli_robot.yaml'
    path.write_text('mowgli:\n  ros__parameters:\n' + body, encoding='utf-8')
    return str(path)


def _layers(config_path, template=TEMPLATE, backend=BACKEND):
    return launch.layered_parameters(DEFAULTS, template, backend, config_path)


def test_template_openmower_keys_equal_the_package_defaults():
    # A robot whose config has no openmower_* key must behave exactly as the
    # package defaults say, whether or not the template is present.
    template = _params(TEMPLATE)['mowgli']['ros__parameters']
    package = _params(DEFAULTS)['hardware_bridge']['ros__parameters']
    om_keys = [k for k in template if k.startswith('openmower_')]
    assert om_keys, 'the template carries no openmower_* key'
    for key in om_keys:
        assert key in launch.ROBOT_CONFIG_KEYS, f'{key} is in the template but never forwarded'
        param, _ = launch.ROBOT_CONFIG_KEYS[key]
        assert template[key] == package[param], f'{key}: template {template[key]!r} != package {package[param]!r}'


def test_every_forwarded_key_has_a_template_default():
    template = _params(TEMPLATE)['mowgli']['ros__parameters']
    missing = [k for k in launch.ROBOT_CONFIG_KEYS if k not in template]
    assert not missing, f'no template default (GUI reset would be undefined): {missing}'


def test_no_openmower_duplicate_of_a_shared_setting():
    # A setting MowgliNext already has must be bridged, not duplicated.
    board_fields = ('ll_v_charge_cutoff', 'll_i_charge_cutoff', 'll_v_battery_cutoff',
                    'll_v_battery_empty', 'll_v_battery_full', 'll_lift_period_ms', 'll_tilt_period_ms')
    for key, (param, _) in launch.ROBOT_CONFIG_KEYS.items():
        if key.startswith('openmower_'):
            assert param not in board_fields, f'{key} duplicates a shared setting for {param}'
    targets = [param for param, _ in launch.ROBOT_CONFIG_KEYS.values()]
    assert len(targets) == len(set(targets)), 'two config keys feed the same node parameter'


def test_fresh_install_gets_openmowers_own_board_values(tmp_path):
    eff = _effective(_layers(_write_config(tmp_path, '')))
    assert eff['ll_lift_period_ms'] == 100
    assert eff['ll_tilt_period_ms'] == 2500
    assert eff['ll_v_battery_cutoff'] == 29.0
    assert eff['ll_i_charge_cutoff'] == 1.2
    assert eff['ll_v_charge_cutoff'] == -1  # no MowgliNext equivalent: the board's own
    assert eff['emergency_input_config'] == ''
    assert eff['ticks_per_meter'] == 1600.0


def test_backend_defaults_equal_what_mowgli_ros2_resolves():
    # The same overlay file reaches both containers.
    overlay = _params(BACKEND)['mowgli']['ros__parameters']
    eff = _effective(launch.layered_parameters(DEFAULTS, TEMPLATE, BACKEND, '/nonexistent'))
    for key, value in overlay.items():
        param, _ = launch.ROBOT_CONFIG_KEYS[key]
        assert eff[param] == value, key


def test_shared_keys_without_backend_override_follow_the_template(tmp_path):
    template = _params(TEMPLATE)['mowgli']['ros__parameters']
    eff = _effective(_layers(_write_config(tmp_path, '')))
    assert eff['wheel_track'] == template['wheel_track']
    assert eff['ll_v_battery_full'] == template['battery_full_voltage']


def test_env_beats_the_defaults_and_the_config_beats_env(tmp_path, monkeypatch):
    monkeypatch.setenv('OPENMOWER_LL_PORT', '/dev/ttyAMA1')
    monkeypatch.setenv('OPENMOWER_XESC_TYPE', 'xesc_2040')
    monkeypatch.setenv('OPENMOWER_XESC_LEFT_PORT', '/dev/ttyAMA9')
    cfg = _write_config(tmp_path, '    openmower_xesc_left_port: "/dev/ttyUSB0"\n')
    eff = _effective(_layers(cfg))
    assert eff['ll_serial_port'] == '/dev/ttyAMA1'
    assert eff['xesc_type'] == 'xesc_2040'
    assert eff['left_xesc_port'] == '/dev/ttyUSB0'


def test_invalid_values_are_ignored_not_forwarded(tmp_path, monkeypatch):
    monkeypatch.setenv('OPENMOWER_XESC_TYPE', 'xesc_2040')
    cfg = _write_config(tmp_path, '    openmower_xesc_type: "vesc"\n'
                                  '    openmower_ll_port: ""\n'
                                  '    openmower_wheel_kp: "fast"\n')
    eff = _effective(_layers(cfg))
    assert eff['xesc_type'] == 'xesc_2040'
    assert eff['ll_serial_port'] == '/dev/ttyAMA0'
    assert eff['wheel_kp'] == 0.5


def test_string_booleans_are_parsed_not_truthy(tmp_path):
    cfg = _write_config(tmp_path, '    mowing_enabled: "false"\n')
    eff = _effective(_layers(cfg))
    assert eff['mowing_enabled'] is False


def test_operator_setting_reaches_the_board(tmp_path):
    cfg = _write_config(tmp_path, '    both_wheels_lift_emergency_ms: 200\n')
    eff = _effective(_layers(cfg))
    assert eff['ll_lift_period_ms'] == 200
    assert eff['ll_tilt_period_ms'] == 2500


def test_missing_template_and_overlay_keep_the_board_undefined(tmp_path):
    eff = _effective(_layers(_write_config(tmp_path, ''), template=str(tmp_path / 'a.yaml'),
                             backend=str(tmp_path / 'b.yaml')))
    assert eff['ticks_per_meter'] == 1600.0
    assert eff['ll_lift_period_ms'] == -1
