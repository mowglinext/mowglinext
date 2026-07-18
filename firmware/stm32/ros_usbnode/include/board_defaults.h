#ifndef __BOARD_DEFAULTS_H
#define __BOARD_DEFAULTS_H

/*******************************************************************************
 * SINGLE SOURCE OF TRUTH for the board-independent, operator-facing firmware
 * defaults (battery/charge envelope, emergency-sensor timeouts, onboard-IMU
 * tilt threshold).
 *
 * WHY THIS FILE EXISTS
 *   These values used to be hand-copied in TWO places that silently drifted:
 *     - the committed board.h (what firmware CI compiles), and
 *     - board.h.template's {{.Field}} placeholders (what the GUI renders +
 *       flashes), whose fill values are the FlashBoardComponent.tsx form
 *       defaults.
 *   The two disagreed on 5 SAFETY values (charge cutoff, one-wheel-lift
 *   timeout, ...). This header is now the ONE place the blessed defaults live.
 *   Both consumers include it:
 *     - board.h  #includes it and defines NONE of these itself -> CI builds
 *       exactly these blessed values.
 *     - board.h.template renders `#define X {{.X}}` (the GUI value) and THEN
 *       includes this file: every default below is `#ifndef`-guarded, so a
 *       GUI-supplied value WINS and an absent one falls back to the blessed
 *       default. (This is the "baked #ifndef fallbacks the GUI overrides"
 *       model.)
 *   A CI drift guard (firmware/scripts/board_defaults_parity.py) asserts the
 *   FlashBoardComponent.tsx form defaults equal the values below, so the
 *   board.h-vs-GUI drift can never silently recur.
 *
 * CHANGING A VALUE
 *   Edit it HERE, then update the matching FlashBoardComponent.tsx `default={}`
 *   (the guard will fail until they agree). Do NOT re-hardcode any of these in
 *   board.h or inline in the template.
 *
 * SAFETY
 *   Every value here affects physical behaviour (charging envelope, e-stop
 *   response, tilt cutoff). Treat any change as safety-critical and
 *   robot-verify it. This file NEVER enables I_DONT_NEED_MY_FINGERS.
 ******************************************************************************/

/* --- Battery / charge envelope ---------------------------------------------
 * Blessed 2026-07-18 to the (higher) board.h values. NOTE: this RAISES the
 * charge envelope vs what shipped via the GUI form defaults before this change
 * — safety-critical, robot-verify required. */
#ifndef MAX_CHARGE_CURRENT
#define MAX_CHARGE_CURRENT 1.2f
#endif
#ifndef MAX_CHARGE_VOLTAGE
#define MAX_CHARGE_VOLTAGE 29.4f
#endif
// DEAD: no firmware code reads LIMIT_VOLTAGE_150MA (defined only here + in the
// template). NOT runtime-migrated (P4) — a runtime knob would do nothing. Left
// in place; removing the dead #define is separate cleanup.
#ifndef LIMIT_VOLTAGE_150MA
#define LIMIT_VOLTAGE_150MA 28.8f
#endif
// SEED-ONLY: this only initialises charger.c's charge_end_voltage; that variable
// is overwritten every charger tick (charger.c:173, to `v` clamped to
// MAX_CHARGE_VOLTAGE at charger.c:164). NOT runtime-migrated (P4) — a runtime
// value would not persist. The live charge ceiling is MAX_CHARGE_VOLTAGE.
#ifndef BAT_CHARGE_CUTOFF_VOLTAGE
#define BAT_CHARGE_CUTOFF_VOLTAGE 29.2f
#endif

/* Fixed battery constants (not operator-configurable, identical in both former
 * copies — centralised here to end the duplication). */
#ifndef CHARGE_END_LIMIT_CURRENT
#define CHARGE_END_LIMIT_CURRENT 0.08f
#endif
#ifndef MIN_DOCKED_VOLTAGE
#define MIN_DOCKED_VOLTAGE 20.0f
#endif
#ifndef MIN_BATTERY_VOLTAGE
#define MIN_BATTERY_VOLTAGE 5.0f
#endif
#ifndef MIN_CHARGE_CURRENT
#define MIN_CHARGE_CURRENT 0.1f
#endif
#ifndef LOW_BAT_THRESHOLD
#define LOW_BAT_THRESHOLD 25.2f /* near 20% SOC */
#endif
#ifndef LOW_CRI_THRESHOLD
#define LOW_CRI_THRESHOLD 23.5f /* near 0% SOC */
#endif

/* --- Emergency-sensor timeouts [ms] ----------------------------------------
 * ONE_WHEEL_LIFT blessed 2026-07-18 to 2000 (board.h value); the GUI form
 * default was 10000 and is being brought into line. Safety-relevant. */
#ifndef ONE_WHEEL_LIFT_EMERGENCY_MILLIS
#define ONE_WHEEL_LIFT_EMERGENCY_MILLIS 2000
#endif
#ifndef BOTH_WHEELS_LIFT_EMERGENCY_MILLIS
#define BOTH_WHEELS_LIFT_EMERGENCY_MILLIS 1000
#endif
#ifndef TILT_EMERGENCY_MILLIS
#define TILT_EMERGENCY_MILLIS 500 /* mechanical + accelerometer detection */
#endif
#ifndef STOP_BUTTON_EMERGENCY_MILLIS
#define STOP_BUTTON_EMERGENCY_MILLIS 100
#endif
#ifndef PLAY_BUTTON_CLEAR_EMERGENCY_MILLIS
#define PLAY_BUTTON_CLEAR_EMERGENCY_MILLIS 2000
#endif

/* --- Onboard-IMU (LIS3DH) tilt-interrupt threshold -------------------------
 * Feeds lis3dh_int1_gen_threshold_set() (src/i2c.c). Now operator-configurable
 * via the template ({{.ImuOnboardInclinationThreshold}}), so the firmware
 * CLAMPS it to a safe envelope: any value outside [0x2C, 0x40] is rejected and
 * forced back to the vetted default 0x38, so a bad/hostile config can never
 * weaken tilt detection. 0x38 is stricter than stock firmware's 0x2C (which
 * allows more inclination); 0x2C is the most-permissive value we still treat as
 * shipped-safe, hence the lower bound. Direction-agnostic: extremes on EITHER
 * side fall back to 0x38. SAFETY: robot-verify any change to the envelope.
 * Mirrors the runtime pkt_set_*_t clamp discipline, at compile time. */
#ifndef IMU_ONBOARD_INCLINATION_THRESHOLD
#define IMU_ONBOARD_INCLINATION_THRESHOLD 0x38
#endif
#if (IMU_ONBOARD_INCLINATION_THRESHOLD < 0x2C) || \
    (IMU_ONBOARD_INCLINATION_THRESHOLD > 0x40)
#undef IMU_ONBOARD_INCLINATION_THRESHOLD
#define IMU_ONBOARD_INCLINATION_THRESHOLD 0x38
#endif

#endif /* __BOARD_DEFAULTS_H */
