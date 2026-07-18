#!/usr/bin/env python3
"""Guard: board.h and board.h.template can't drift from the single source.

board_defaults.h is the ONE place the operator-facing firmware safety defaults
(battery/charge envelope, emergency-sensor timeouts, onboard-IMU tilt threshold)
live. Both consumers must get the values FROM it, never by re-hardcoding their
own copy:
  * board.h        — what firmware CI compiles / what the prebuilt binaries ship.
  * board.h.template — what the GUI renders and flashes.
If either re-hardcodes one of these macros, the two can silently disagree on a
SAFETY value again — the exact bug this guard prevents.

This guard is purely STRUCTURAL and does not read the GUI form defaults: the GUI
`default={}` values (FlashBoardComponent.tsx) are intentionally allowed to lag
until Phase 2 syncs them (new users flash prebuilt binaries, not the compile
path). A separate Phase-2 check will bind the GUI defaults to board_defaults.h.

--check fails if:
  * board_defaults.h does not define every managed macro, OR
  * board.h re-#defines any managed macro (it must only #include the header), OR
  * board.h / board.h.template does not #include board_defaults.h, OR
  * board.h.template still hardcodes the inclination threshold instead of
    substituting it.
Mirrors the repo's other drift guards (protocol_version_guard.py,
schema_template_parity_test.go).
"""

import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
INCLUDE = REPO_ROOT / "firmware/stm32/ros_usbnode/include"
DEFAULTS_H = INCLUDE / "board_defaults.h"
BOARD_H = INCLUDE / "board.h"
TEMPLATE = INCLUDE / "board.h.template"

# The operator-facing safety macros single-sourced in board_defaults.h. board.h
# must NOT define any of these itself.
MANAGED_MACROS = [
    "MAX_CHARGE_CURRENT",
    "MAX_CHARGE_VOLTAGE",
    "LIMIT_VOLTAGE_150MA",
    "BAT_CHARGE_CUTOFF_VOLTAGE",
    "ONE_WHEEL_LIFT_EMERGENCY_MILLIS",
    "BOTH_WHEELS_LIFT_EMERGENCY_MILLIS",
    "TILT_EMERGENCY_MILLIS",
    "STOP_BUTTON_EMERGENCY_MILLIS",
    "PLAY_BUTTON_CLEAR_EMERGENCY_MILLIS",
    "IMU_ONBOARD_INCLINATION_THRESHOLD",
]

INCLUDE_LINE = '#include "board_defaults.h"'


def _defines(text, macro):
    """True if `text` has a real `#define <macro> ...` (not #ifndef/#undef)."""
    return re.search(rf"^\s*#\s*define\s+{macro}\b", text, re.MULTILINE) is not None


def main():
    errors = []

    defaults = DEFAULTS_H.read_text()
    for macro in MANAGED_MACROS:
        if not _defines(defaults, macro):
            errors.append(f"board_defaults.h is missing #define {macro}")

    board = BOARD_H.read_text()
    if INCLUDE_LINE not in board:
        errors.append(f"board.h must {INCLUDE_LINE}")
    for macro in MANAGED_MACROS:
        if _defines(board, macro):
            errors.append(
                f"board.h re-#defines {macro} — remove it; the value must come "
                f"from board_defaults.h (single source)."
            )

    template = TEMPLATE.read_text()
    if INCLUDE_LINE not in template:
        errors.append(f"board.h.template must {INCLUDE_LINE}")
    # The inclination threshold must be operator-configurable via substitution,
    # not a hardcoded literal.
    if re.search(
        r"#\s*define\s+IMU_ONBOARD_INCLINATION_THRESHOLD\s+0x[0-9A-Fa-f]",
        template,
    ):
        errors.append(
            "board.h.template hardcodes IMU_ONBOARD_INCLINATION_THRESHOLD — it "
            "must substitute {{.ImuOnboardInclinationThreshold}} (clamped in "
            "board_defaults.h)."
        )

    if errors:
        print("FAIL: board.h / template drifted from board_defaults.h:")
        for e in errors:
            print(f"  - {e}")
        return 1

    print(
        f"OK: board_defaults.h is the sole source of {len(MANAGED_MACROS)} "
        "safety defaults; board.h + template consume it without re-hardcoding."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
