# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
# Ported from the Airseekers ROS 2 port (base_keys/keys_logic.py); same author, relicensed GPL-3.0.
"""Pure (ROS-free, evdev-free) logic for the top-panel buttons.

Everything here is unit-testable with a fake event source: the evdev decoding,
the press / long-press / debounce state machine, input-device discovery from
``/proc/bus/input/devices`` text, and the key -> action mapping.

Vendor reference (ROS 1 ``mower_base_node``, ``button.cpp`` ``Buttons::handleKey``,
decompile ``native_decompile/dec/mower_base_node/mower_base_node.c`` ~240378):

    device "/dev/keyboard"  (udev symlink -> /dev/input/event5, name "key input")
    only EV_KEY (type 1); value 0 (release) publishes key 0
    code 0x19 = 25 KEY_P -> 4  KEY_POWER_SHORT       "电源键短按"
    code 0x1f = 31 KEY_S -> 2  KEY_GO_DOCKING        "回充键"
    code 0x26 = 38 KEY_L -> 8  KEY_POWER_LONG        "电源键长按"
    code 0x2e = 46 KEY_C -> 1  KEY_WORKING_OR_PAUSE  "暂停键"
    code 0x32 = 50 KEY_M -> 16 KEY_DOCK_AND_PAUSE    "回充暂停组合键"
    anything else -> "MowerBase: Unknown key code: N"

There is no software long-press timer and no debounce in the vendor userspace: the
"key input" device is created by the vendor kernel module ``init_test.ko``, which
polls the power GPIO every 40 ms and does the timing itself (disassembled
2026-10-09): released after 2..18 ticks -> KEY_P (press+release
on release), 19..74 ticks -> nothing ("ignore short press"), 75 ticks (3.0 s,
still held) -> KEY_L, released after that -> KEY_R + /usr/bin/sys_close,
> 750 ticks (30 s) -> "stuck", nothing on release. We keep that as the primary
path; the optional software long-press on KEY_P (``long_press_ms``) never fires
on this hardware because KEY_P press and release arrive together.
"""
import re
import struct
from dataclasses import dataclass, field

# --- linux/input.h --------------------------------------------------------
INPUT_EVENT_FMT = 'llHHi'          # struct input_event on 64-bit: timeval, type, code, value
INPUT_EVENT_SIZE = struct.calcsize(INPUT_EVENT_FMT)   # 24 on aarch64 (vendor reads 0x18)
EV_SYN = 0
EV_KEY = 1
KEY_RELEASE, KEY_PRESS, KEY_AUTOREPEAT = 0, 1, 2

# --- MowerBaseButtonInfo / MowerSensorInfo.key_pressed bitmask -------------
KEY_NONE = 0
KEY_WORKING_OR_PAUSE = 1
KEY_GO_DOCKING = 2
KEY_POWER_SHORT = 4
KEY_POWER_LONG = 8
KEY_DOCK_AND_PAUSE = 16

BIT_NAMES = {
    KEY_WORKING_OR_PAUSE: 'work_pause',
    KEY_GO_DOCKING: 'go_docking',
    KEY_POWER_SHORT: 'power_short',
    KEY_POWER_LONG: 'power_long',
    KEY_DOCK_AND_PAUSE: 'dock_and_pause',
}

# --- vendor evdev codes on the "key input" device -------------------------
VENDOR_KEY_POWER = 25          # KEY_P
VENDOR_KEY_GO_DOCKING = 31     # KEY_S
VENDOR_KEY_POWER_LONG = 38     # KEY_L
VENDOR_KEY_WORK_PAUSE = 46     # KEY_C
VENDOR_KEY_DOCK_AND_PAUSE = 50  # KEY_M
# KEY_R (19): init_test.ko emits it when a >= 3 s power hold is RELEASED, right before it runs
# /usr/bin/sys_close; the vendor handler logs it as unknown. We only log it.
VENDOR_KEY_POWER_LONG_RELEASE = 19

VENDOR_DEVICE_NAME = 'key input'
VENDOR_DEVICE_PATH = '/dev/keyboard'


def decode_events(buf):
    """Split a raw read() buffer into (sec, type, code, value) tuples.
    Returns (events, leftover_bytes)."""
    out = []
    n = len(buf) // INPUT_EVENT_SIZE
    for i in range(n):
        sec, usec, etype, code, value = struct.unpack_from(INPUT_EVENT_FMT, buf, i * INPUT_EVENT_SIZE)
        out.append((sec + usec * 1e-6, etype, code, value))
    return out, buf[n * INPUT_EVENT_SIZE:]


# --- device discovery -----------------------------------------------------
@dataclass
class InputDevice:
    name: str
    handlers: list = field(default_factory=list)
    key_bits: int = 0

    @property
    def event_node(self):
        for h in self.handlers:
            if re.fullmatch(r'event\d+', h):
                return '/dev/input/' + h
        return None

    def has_keys(self, codes):
        return all((self.key_bits >> c) & 1 for c in codes)


def parse_proc_input_devices(text):
    """Parse /proc/bus/input/devices into InputDevice records."""
    devs = []
    for block in re.split(r'\n\s*\n', text.strip()):
        name, handlers, key_bits = None, [], 0
        for line in block.splitlines():
            if line.startswith('N: Name='):
                name = line[len('N: Name='):].strip().strip('"')
            elif line.startswith('H: Handlers='):
                handlers = line[len('H: Handlers='):].split()
            elif line.startswith('B: KEY='):
                # space-separated hex words, most significant first, each a C long (64 bit here)
                words = line[len('B: KEY='):].split()
                key_bits = 0
                for w in words:
                    key_bits = (key_bits << 64) | int(w, 16)
        if name is not None:
            devs.append(InputDevice(name, handlers, key_bits))
    return devs


def find_device(proc_text, name=VENDOR_DEVICE_NAME, required_codes=()):
    """Return (event_path, reason). First exact name match, then the first device
    exposing every code in ``required_codes``. (None, reason) if nothing fits."""
    devs = parse_proc_input_devices(proc_text)
    if name:
        for d in devs:
            if d.name == name and d.event_node:
                return d.event_node, 'name "%s"' % name
    if required_codes:
        for d in devs:
            if d.event_node and d.has_keys(required_codes):
                return d.event_node, 'key capabilities %s (device "%s")' % (list(required_codes), d.name)
    return None, 'no input device named "%s" or exposing keys %s' % (name, list(required_codes))


# --- press / long-press / debounce state machine ---------------------------
class KeyStateMachine:
    """Turns raw EV_KEY events into semantic button bits.

    ``code_map``: evdev code -> bit. Bits are emitted on the *press* edge
    (vendor behaviour) except for ``power_code`` when a software long press is
    enabled (``long_press_s > 0``): then a power press emits KEY_POWER_SHORT on
    release if held < long_press_s, or KEY_POWER_LONG as soon as the hold
    reaches long_press_s (via ``tick``), and nothing on the later release.

    ``debounce_s``: a press of the same code within this window of the
    previous accepted press is ignored (as is its release).

    ``feed``/``tick`` return a list of emitted bits (ints). ``held_bits`` is the
    bitmask of currently held semantic buttons (for MowerBaseButtonInfo).
    """

    def __init__(self, code_map, power_code=None, long_press_s=0.0, debounce_s=0.0):
        self.code_map = dict(code_map)
        self.power_code = power_code
        self.long_press_s = float(long_press_s)
        self.debounce_s = float(debounce_s)
        self._down = {}           # code -> press time (accepted presses only)
        self._last_press = {}     # code -> last accepted press time
        self._long_fired = False
        self.unknown_codes = []   # codes seen but unmapped (for logging)

    @property
    def held_bits(self):
        bits = 0
        for c in self._down:
            bits |= self.code_map.get(c, 0)
        return bits

    def _soft_long(self, code):
        return code == self.power_code and self.long_press_s > 0

    def feed(self, etype, code, value, t):
        if etype != EV_KEY or value == KEY_AUTOREPEAT:
            return []
        if code not in self.code_map:
            if value == KEY_PRESS:
                self.unknown_codes.append(code)
            return []
        bit = self.code_map[code]
        if value == KEY_PRESS:
            if code in self._down:
                return []       # duplicate press without release
            last = self._last_press.get(code)
            if last is not None and self.debounce_s > 0 and (t - last) < self.debounce_s:
                return []       # bounce
            self._last_press[code] = t
            self._down[code] = t
            if self._soft_long(code):
                self._long_fired = False
                return []       # decided on release / tick
            return [bit]
        if value == KEY_RELEASE:
            if code not in self._down:
                return []       # release of a debounced / unseen press
            t0 = self._down.pop(code)
            if self._soft_long(code):
                if self._long_fired:
                    self._long_fired = False
                    return []
                if (t - t0) >= self.long_press_s:
                    return [KEY_POWER_LONG]
                return [bit]
            return []
        return []

    def tick(self, t):
        """Call periodically; fires KEY_POWER_LONG while the power key is held."""
        if self.power_code is None or self.long_press_s <= 0 or self._long_fired:
            return []
        t0 = self._down.get(self.power_code)
        if t0 is not None and (t - t0) >= self.long_press_s:
            self._long_fired = True
            return [KEY_POWER_LONG]
        return []

    def reset(self):
        """Forget held keys (device closed / unplugged)."""
        self._down.clear()
        self._long_fired = False


# --- key -> action mapping --------------------------------------------------
HL_NULL, HL_IDLE, HL_AUTONOMOUS, HL_RECORDING, HL_MANUAL_MOWING = 0, 1, 2, 3, 4
CMD_START, CMD_HOME, CMD_STOP, CMD_RESET_EMERGENCY = 1, 2, 8, 254

POWER_LONG_ACTIONS = ('log_only', 'sequence', 'poweroff_service', 'shutdown')


@dataclass
class ActionConfig:
    on_power_short: bool = True                # call /clear_estop
    power_short_reset_emergency: bool = True   # + HighLevelControl RESET_EMERGENCY(254)
    on_work_pause: bool = True                 # HighLevelControl START/STOP toggle
    on_go_docking: bool = True                 # HighLevelControl HOME
    on_dock_and_pause: bool = False            # treat combo like go_docking
    power_long_action: str = 'log_only'


@dataclass(frozen=True)
class Action:
    kind: str        # 'clear_estop' | 'hlc' | 'power_sequence' | 'poweroff_service' | 'shutdown_hook' | 'log'
    arg: int = 0
    note: str = ''


def map_key(bit, hl_state, cfg):
    """Return the ordered list of Actions for an emitted key bit.

    ``hl_state`` is the last HighLevelStatus.state, or None if never received.
    """
    name = BIT_NAMES.get(bit, str(bit))
    if bit == KEY_POWER_SHORT:
        if not cfg.on_power_short:
            return [Action('log', note='power_short ignored (on_power_short=false)')]
        acts = [Action('clear_estop', note='power short -> clear e-stop (vendor clearEstopAlarm)')]
        if cfg.power_short_reset_emergency:
            acts.append(Action('hlc', CMD_RESET_EMERGENCY, 'power short -> RESET_EMERGENCY'))
        return acts
    if bit == KEY_WORKING_OR_PAUSE:
        if not cfg.on_work_pause:
            return [Action('log', note='work_pause ignored (on_work_pause=false)')]
        if hl_state is None:
            return [Action('log', note='work_pause ignored: no high_level_status received yet')]
        if hl_state in (HL_AUTONOMOUS, HL_MANUAL_MOWING):
            return [Action('hlc', CMD_STOP, 'work_pause while mowing -> STOP')]
        if hl_state == HL_IDLE:
            return [Action('hlc', CMD_START, 'work_pause while idle -> START')]
        return [Action('log', note='work_pause ignored in high-level state %d' % hl_state)]
    if bit == KEY_GO_DOCKING or (bit == KEY_DOCK_AND_PAUSE and cfg.on_dock_and_pause):
        enabled = cfg.on_go_docking
        if not enabled:
            return [Action('log', note='%s ignored (on_go_docking=false)' % name)]
        return [Action('hlc', CMD_HOME, '%s -> HOME' % name)]
    if bit == KEY_DOCK_AND_PAUSE:
        return [Action('log', note='dock_and_pause: no action (vendor mower_logic ignores it too)')]
    if bit == KEY_POWER_LONG:
        a = cfg.power_long_action
        if a == 'sequence':
            return [Action('power_sequence', note='power long -> power-off sequence')]
        if a == 'poweroff_service':
            return [Action('hlc', CMD_STOP, 'power long -> STOP before poweroff'),
                    Action('poweroff_service', note='power long -> /poweroff')]
        if a == 'shutdown':
            return [Action('hlc', CMD_STOP, 'power long -> STOP before shutdown'),
                    Action('shutdown_hook', note='power long -> host shutdown hook')]
        return [Action('log', note='power long: log only (power_long_action=%s)' % a)]
    return [Action('log', note='unmapped key bit %d' % bit)]
