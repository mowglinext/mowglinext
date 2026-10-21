# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
"""Parameter layering of launch/airseekers_bridge.launch.py (needs the ROS
launch packages; skipped without them).

    python3 -m pytest sensors/airseekers/mowgli_airseekers_bridge/test/test_launch_config.py
"""

import importlib.util
import os

import pytest
import yaml

pytest.importorskip('launch_ros')

PKG = os.path.normpath(os.path.join(os.path.dirname(__file__), '..'))
REPO = os.path.normpath(os.path.join(PKG, '..', '..', '..'))
TEMPLATE = os.path.join(REPO, 'ros2', 'src', 'mowgli_bringup', 'config', 'mowgli_robot.yaml')
BACKEND = os.path.join(os.path.dirname(TEMPLATE), 'backends', 'airseekers.yaml')
DEFAULTS = os.path.join(PKG, 'config', 'airseekers_bridge.yaml')


def _load_launch():
    spec = importlib.util.spec_from_file_location(
        'airseekers_bridge_launch', os.path.join(PKG, 'launch', 'airseekers_bridge.launch.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


launch = _load_launch()


def _defaults():
    with open(DEFAULTS, encoding='utf-8') as fh:
        return yaml.safe_load(fh)['hardware_bridge']['ros__parameters']


def _effective(layers):
    out = dict(_defaults())
    for layer in layers[1:]:
        out.update(layer)
    return out


def _cfg(tmp_path, body=''):
    p = tmp_path / 'mowgli_robot.yaml'
    p.write_text('mowgli:\n  ros__parameters:\n' + body, encoding='utf-8')
    return str(p)


def test_template_airseekers_keys_equal_package_defaults():
    tpl = launch.read_robot_params(TEMPLATE)
    d = _defaults()
    for key, (param, _cast) in launch.BRIDGE_KEYS.items():
        if key.startswith('airseekers_'):
            assert key in tpl, key
            assert tpl[key] == d[param], key


def test_backend_overrides_only_existing_keys():
    tpl = launch.read_robot_params(TEMPLATE)
    for key in launch.read_robot_params(BACKEND):
        assert key in tpl, 'backend yaml may not add keys: %s' % key


def test_fresh_install_gets_tron_defaults(tmp_path):
    eff = _effective(launch.bridge_parameters(DEFAULTS, TEMPLATE, BACKEND, _cfg(tmp_path), {}))
    assert eff['wheel_track'] == 0.48 and eff['max_mps'] == 0.3
    assert eff['max_angular_radps'] == 0.3 and eff['mcu_port'] == '/dev/serial_mower'
    assert eff['tilt_emergency_ms'] == 300


def test_layer_order_env_then_gui(tmp_path):
    env = {'AIRSEEKERS_MCU_PORT': '/dev/ttyS9'}
    eff = _effective(launch.bridge_parameters(DEFAULTS, TEMPLATE, BACKEND, _cfg(tmp_path), env))
    assert eff['mcu_port'] == '/dev/ttyS9'
    cfg = _cfg(tmp_path, '    airseekers_mcu_port: "/dev/gui"\n    wheel_track: 0.5\n'
                         '    airseekers_heading_hold: "true"\n')
    eff = _effective(launch.bridge_parameters(DEFAULTS, TEMPLATE, BACKEND, cfg, env))
    assert eff['mcu_port'] == '/dev/gui' and eff['wheel_track'] == 0.5 and eff['heading_hold'] is True


def test_gnss_owned_by_sidecar_by_default(tmp_path):
    cfg = launch.merged(TEMPLATE, BACKEND, _cfg(tmp_path, '    ntrip_enabled: true\n'
                                                          '    ntrip_host: "caster"\n'))
    assert launch.gnss_enabled(cfg)
    g = launch.gnss_parameters(cfg)
    assert g['port'] == '/dev/serial_rtk' and g['baud'] == 115200
    assert g['fix_topic'] == '/gps/fix' and g['status_topic'] == '/gps/status'
    assert g['correction_source'] == 'ntrip' and g['ntrip_host'] == 'caster'
    assert g['publish_rtk_heading'] is False and g['frame_id'] == 'gps_link'
    cfg = launch.merged(TEMPLATE, BACKEND, _cfg(tmp_path, '    gnss_stack: "universal"\n'))
    assert not launch.gnss_enabled(cfg)
    assert launch.gnss_parameters(launch.merged(TEMPLATE, BACKEND, _cfg(tmp_path)))[
        'correction_source'] == 'lora'
