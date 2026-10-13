# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
"""The safety model: every branch of the blade / wheel / dock gates, the
emergency latch and the SpeedData transmit policy."""
import itertools
import os

import pytest

from mowgli_airseekers_bridge.safety import (
    HL_AUTONOMOUS, HL_IDLE, HL_MANUAL_MOWING, HL_NULL, HL_RECORDING, BladeInputs, DockGuard,
    EmergencyInputs, EmergencyLatch, SpeedTx, TickIntegrator, blade_may_run, limit_slew,
    wheels_may_run)


def ok_blade(**kw):
    b = BladeInputs(requested=True, mowing_enabled=True, emergency=False, link_alive=True,
                    hl_mode=HL_AUTONOMOUS, hl_status_fresh=True, update_maintenance=False)
    for k, v in kw.items():
        setattr(b, k, v)
    return b


def test_blade_runs_only_in_mowing_modes():
    assert blade_may_run(ok_blade())
    assert blade_may_run(ok_blade(hl_mode=HL_MANUAL_MOWING))
    for mode in (HL_NULL, HL_IDLE, HL_RECORDING):
        assert not blade_may_run(ok_blade(hl_mode=mode))


@pytest.mark.parametrize('field,value', [('requested', False), ('mowing_enabled', False),
                                         ('emergency', True), ('link_alive', False),
                                         ('hl_status_fresh', False),
                                         ('update_maintenance', True)])
def test_each_blade_inhibit(field, value):
    assert not blade_may_run(ok_blade(**{field: value}))


def test_wheels_gate():
    assert wheels_may_run(False, True, HL_AUTONOMOUS, True)
    assert wheels_may_run(False, True, HL_NULL, True)        # driving before the BT
    assert not wheels_may_run(False, True, HL_IDLE, True)    # IDLE = hard stop
    for e, l, c in itertools.product((False, True), repeat=3):
        if e or not l or not c:
            assert not wheels_may_run(e, l, HL_AUTONOMOUS, c)


def test_dock_guard_holds_until_undock():
    g = DockGuard()
    assert g.filter(False, HL_IDLE, False, 0.2, 0.1)[:2] == (0.2, 0.1)
    assert g.filter(True, HL_IDLE, True, 0.2, 0.0)[:2] == (0.0, 0.0)
    assert g.filter(True, HL_NULL, True, 0.2, 0.0)[:2] == (0.0, 0.0)
    assert g.filter(True, HL_AUTONOMOUS, False, 0.2, 0.0)[:2] == (0.0, 0.0)   # stale BT
    assert g.filter(True, HL_AUTONOMOUS, True, 0.2, 0.1)[:2] == (0.2, 0.1)    # undock
    assert g.filter(True, HL_AUTONOMOUS, True, -0.1, 0.0)[:2] == (0.0, 0.0)   # into the charger
    assert DockGuard(allow_reverse=True).filter(True, HL_AUTONOMOUS, True, -0.1, 0.0)[:2] \
        == (-0.1, 0.0)


UNDOCK_SPEED = 0.16   # mowgli_robot.yaml undock_speed (template default)


def _wheel_gate(guard, docked, hl_mode, hl_fresh, lin, ang=0.0, emergency=False):
    """The bridge's _control_tick order: wheel gate, then the dock guard."""
    if not wheels_may_run(emergency, True, hl_mode, True):
        return 0.0, 0.0
    return guard.filter(docked, hl_mode, hl_fresh, lin, ang)[:2]


def test_forward_undock_sequence():
    """docked + IDLE: nothing; the BT leaves IDLE and drives forward at the
    undock speed (undock_forward): it passes; a reverse (a stock BackUp undock)
    is refused for as long as dock_ok is reported; once the contacts open the
    normal driving rules apply and reverse passes again."""
    g = DockGuard()
    # On the dock, BT idle: the forward undock command must not leak early.
    assert _wheel_gate(g, True, HL_IDLE, True, UNDOCK_SPEED) == (0.0, 0.0)
    assert _wheel_gate(g, True, HL_NULL, True, UNDOCK_SPEED) == (0.0, 0.0)
    # BT left IDLE but its status is stale: still held.
    assert _wheel_gate(g, True, HL_AUTONOMOUS, False, UNDOCK_SPEED) == (0.0, 0.0)
    # Undock issued: forward passes unchanged, reverse is refused.
    assert _wheel_gate(g, True, HL_AUTONOMOUS, True, UNDOCK_SPEED) == (UNDOCK_SPEED, 0.0)
    assert _wheel_gate(g, True, HL_AUTONOMOUS, True, -UNDOCK_SPEED) == (0.0, 0.0)
    assert _wheel_gate(g, True, HL_AUTONOMOUS, True, -1e-3) == (0.0, 0.0)
    # Also in manual mode (operator undock): forward only.
    assert _wheel_gate(g, True, HL_MANUAL_MOWING, True, UNDOCK_SPEED) == (UNDOCK_SPEED, 0.0)
    assert _wheel_gate(g, True, HL_MANUAL_MOWING, True, -UNDOCK_SPEED) == (0.0, 0.0)
    # An emergency still wins over the undock.
    assert _wheel_gate(g, True, HL_AUTONOMOUS, True, UNDOCK_SPEED, emergency=True) == (0.0, 0.0)
    # Off the pads (dock_ok dropped): the dock guard steps aside.
    assert _wheel_gate(g, False, HL_AUTONOMOUS, True, -UNDOCK_SPEED) == (-UNDOCK_SPEED, 0.0)
    assert _wheel_gate(g, False, HL_AUTONOMOUS, True, UNDOCK_SPEED, 0.2) == (UNDOCK_SPEED, 0.2)


def test_dock_guard_reason_names_the_refusal():
    g = DockGuard()
    assert 'reverse refused' in g.filter(True, HL_AUTONOMOUS, True, -0.1, 0.0)[2]
    assert 'waiting for an undock' in g.filter(True, HL_IDLE, True, 0.1, 0.0)[2]
    assert g.filter(True, HL_AUTONOMOUS, True, UNDOCK_SPEED, 0.0)[2] == ''


def test_speed_tx_streams_the_forward_undock_and_stops_cleanly():
    """The undock leg on the wire: slewed up to the undock speed, never a
    reverse frame, then explicit zeros once the leg ends."""
    tx = SpeedTx()
    frames = []
    for i in range(40):                                   # 2 s at 20 Hz
        frames += tx.tick(i * 0.05, UNDOCK_SPEED, 0.0)
    assert frames and all(lin >= 0.0 for lin, _ in frames)
    assert frames[-1][0] == pytest.approx(UNDOCK_SPEED)
    stop = []
    for i in range(40, 50):
        stop += tx.tick(i * 0.05, 0.0, 0.0)
    assert stop == [(0.0, 0.0)] * 3


def test_airseekers_backend_undocks_forward():
    """The guard refuses reverse on the dock, so the backend must make the BT
    undock forward (template key undock_forward, backends/airseekers.yaml)."""
    yaml = pytest.importorskip('yaml')
    cfg = os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'ros2', 'src',
                       'mowgli_bringup', 'config')
    with open(os.path.join(cfg, 'mowgli_robot.yaml'), encoding='utf-8') as fh:
        template = yaml.safe_load(fh)['mowgli']['ros__parameters']
    with open(os.path.join(cfg, 'backends', 'airseekers.yaml'), encoding='utf-8') as fh:
        backend = yaml.safe_load(fh)['mowgli']['ros__parameters']
    assert template['undock_forward'] is False      # upstream default: reverse off the dock
    assert backend['undock_forward'] is True
    assert DockGuard().allow_reverse is False


def test_latch_on_boot_and_release():
    l = EmergencyLatch()
    assert l.is_emergency and l.state().latched
    assert l.request_release()[0]
    assert not l.is_emergency


def test_trigger_latches_and_release_refused_while_held():
    l = EmergencyLatch(latch_on_boot=False)
    s = l.update(EmergencyInputs(stop=True))
    assert s.active and s.latched and 'STOP' in s.reason
    ok, why = l.request_release()
    assert not ok and 'STOP' in why
    s = l.update(EmergencyInputs())          # button let go
    assert s.latched and not s.active        # still latched: the refused release is forgotten
    assert l.is_emergency
    assert l.request_release()[0]
    assert not l.update(EmergencyInputs()).latched


@pytest.mark.parametrize('trig', ['lift', 'tilt', 'bumper'])
def test_every_trigger_latches(trig):
    l = EmergencyLatch(latch_on_boot=False)
    l.update(EmergencyInputs(**{trig: True}))
    l.update(EmergencyInputs())
    assert l.is_emergency


def test_host_request_latches_until_release():
    l = EmergencyLatch(latch_on_boot=False)
    l.request_emergency()
    assert l.update(EmergencyInputs()).latched
    assert l.request_release()[0]
    assert not l.is_emergency


def test_slew_matches_mowgli_hardware_semantics():
    assert limit_slew(0.0, 0.3, 0.05, 0.3, 0.6) == 0.0                 # zero = stop now
    assert limit_slew(0.3, 0.0, 0.1, 0.3, 0.6) == pytest.approx(0.03)
    assert limit_slew(0.1, 0.3, 0.1, 0.3, 0.6) == pytest.approx(0.24)  # decel limit
    assert limit_slew(-0.3, 0.2, 0.1, 0.3, 0.6) == pytest.approx(0.14)  # through zero first


def run(tx, seq, dt=0.05, t0=0.0):
    out = []
    t = t0
    for lin, ang in seq:
        out.append(tx.tick(t, lin, ang))
        t += dt
    return out, t


def test_speedtx_streams_while_moving_then_three_zeros_then_silence():
    tx = SpeedTx()
    frames, t = run(tx, [(0.3, 0.0)] * 30)
    sent = [f for fs in frames for f in fs]
    assert sent and all(lin > 0.0 for lin, _ in sent)
    assert sent[-1][0] == pytest.approx(0.3)            # reached after the ramp
    frames, t = run(tx, [(0.0, 0.0)] * 20, t0=t)
    zeros = [f for fs in frames for f in fs]
    assert zeros == [(0.0, 0.0)] * 3
    frames, _ = run(tx, [(0.0, 0.0)] * 50, t0=t)
    assert not any(frames)                               # idle = silence


def test_speedtx_silent_at_rest_from_boot():
    tx = SpeedTx()
    frames, _ = run(tx, [(0.0, 0.0)] * 100)
    assert not any(frames)


def test_speedtx_new_motion_cancels_stop_sequence():
    tx = SpeedTx()
    _, t = run(tx, [(0.2, 0.0)] * 10)
    f1, t = run(tx, [(0.0, 0.0)], t0=t)
    assert f1 == [[(0.0, 0.0)]]
    f2, _ = run(tx, [(0.2, 0.0)] * 5, t0=t)
    sent = [f for fs in f2 for f in fs]
    assert all(lin > 0.0 for lin, _ in sent)
    assert not tx.stopping


def test_speedtx_ramp_respects_accel():
    tx = SpeedTx(accel_lin=0.3)
    frames, _ = run(tx, [(0.3, 0.0)] * 11, dt=0.05)
    sent = [f[0] for fs in frames for f in fs]
    for a, b in zip(sent, sent[1:]):
        assert b - a <= 0.3 * 0.05 + 1e-9


def test_tick_integrator_derives_per_wheel_ticks():
    t = TickIntegrator(ticks_per_meter=1000.0, wheel_track=0.5)
    t.feed(0.2, 0.0, 1.0)
    assert t.ticks() == (200, 200)
    t.feed(0.0, 0.4, 1.0)            # pivot left: left wheel back, right forward
    assert t.ticks() == (300, 300) and t.direction == [0, 1]
