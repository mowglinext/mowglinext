// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include "charger_under_test.c"

static void reset(void) {
    chargerInputVoltage = 0.0f;
    ChargeController();
    charger_set_charge_limits(MAX_CHARGE_VOLTAGE, MAX_CHARGE_CURRENT);
    charger_set_end_voltage(BAT_CHARGE_CUTOFF_VOLTAGE);
    test_tick = 1000u;
    battery_voltage = charge_voltage = 26.0f;
    current = 0.05f;
}

static void connect(void) {
    chargerInputVoltage = 32.0f;
    ChargeController();
    assert(chargecontrol_is_charging == CHARGER_STATE_CONNECTED);
    test_tick += 101u;
    ChargeController();
    assert(chargecontrol_is_charging == CHARGER_STATE_CHARGING_CC);
    chargecontrol_pwm_val = 1000u;
}

static void no_increase(float battery, float charge, int must_decrease) {
    battery_voltage = battery;
    charge_voltage = charge;
    for (unsigned i = 0; i < 3; ++i) {
        const unsigned previous = chargecontrol_pwm_val;
        ChargeController();
        printf("PWM: %u -> %u (battery %.1f V, charge %.1f V)\n",
               previous, chargecontrol_pwm_val, battery, charge);
        fflush(stdout);
        assert(chargecontrol_pwm_val <= previous);
        if (must_decrease) assert(chargecontrol_pwm_val < previous);
        assert(TIM1->CCR1 == chargecontrol_pwm_val);
    }
}

int main(void) {
    /* Runtime ceiling lowered before connecting; issue #873's exact inputs. */
    reset();
    charger_set_charge_limits(27.0f, 1.2f);
    connect();
    current = 0.5f;
    no_increase(28.0f, 28.0f, 1);
    assert(chargecontrol_is_charging == CHARGER_STATE_CHARGING_CV);

    /* Lower it while already in CC, including each sensor independently
     * and equality at the ceiling (no PWM increase at the limit). */
    const float readings[][2] = {{28.0f, 28.0f}, {28.0f, 26.0f}, {26.0f, 28.0f},
                                {27.0f, 26.0f}, {26.0f, 27.0f}};
    for (unsigned i = 0; i < sizeof(readings) / sizeof(readings[0]); ++i) {
        reset();
        connect();
        charger_set_charge_limits(27.0f, 1.2f);
        no_increase(readings[i][0], readings[i][1], i < 3u);
    }

    /* Lower it in CV with battery voltage above it but charge voltage below.
     * Low current isolates voltage regulation from CV's current decrement. */
    reset();
    connect();
    charge_voltage = BAT_CHARGE_CUTOFF_VOLTAGE;
    ChargeController();
    assert(chargecontrol_is_charging == CHARGER_STATE_CHARGING_CV);
    charger_set_charge_limits(27.0f, 1.2f);
    no_increase(28.0f, 26.0f, 1);
    no_increase(26.0f, 28.0f, 1);

    /* Below the ceiling, charging still advances; a stricter end target
     * remains effective when the runtime ceiling is raised again. */
    reset();
    connect();
    charger_set_charge_limits(27.0f, 1.2f);
    ChargeController();
    assert(chargecontrol_pwm_val == 1001u);
    charger_set_charge_limits(MAX_CHARGE_VOLTAGE, MAX_CHARGE_CURRENT);
    charger_set_end_voltage(26.5f);
    no_increase(27.0f, 27.0f, 1);

    /* Raising a reduced ceiling resumes CC only if the effective target rises. */
    reset();
    connect();
    charger_set_charge_limits(27.0f, 1.2f);
    battery_voltage = charge_voltage = 27.0f;
    current = 0.5f;
    ChargeController();
    assert(chargecontrol_is_charging == CHARGER_STATE_CHARGING_CV);
    charger_set_charge_limits(27.0f, 1.2f);
    ChargeController();
    assert(chargecontrol_is_charging == CHARGER_STATE_CHARGING_CV);
    charger_set_charge_limits(MAX_CHARGE_VOLTAGE, MAX_CHARGE_CURRENT);
    const unsigned previous = chargecontrol_pwm_val;
    ChargeController();
    assert(chargecontrol_is_charging == CHARGER_STATE_CHARGING_CC);
    assert(chargecontrol_pwm_val == previous + 1u);

    charger_set_end_voltage(26.5f);
    ChargeController();
    assert(chargecontrol_is_charging == CHARGER_STATE_CHARGING_CV);
    charger_set_charge_limits(27.0f, 1.2f);
    charger_set_charge_limits(MAX_CHARGE_VOLTAGE, MAX_CHARGE_CURRENT);
    ChargeController();
    assert(chargecontrol_is_charging == CHARGER_STATE_CHARGING_CV);

    /* Default target/ceiling retain their existing CC/CV behavior. */
    reset();
    connect();
    battery_voltage = charge_voltage = 28.0f;
    ChargeController();
    assert(chargecontrol_is_charging == CHARGER_STATE_CHARGING_CC);
    assert(chargecontrol_pwm_val == 1001u);
    battery_voltage = charge_voltage = BAT_CHARGE_CUTOFF_VOLTAGE;
    ChargeController();
    assert(chargecontrol_is_charging == CHARGER_STATE_CHARGING_CV);
    assert(chargecontrol_pwm_val == 1001u);
    no_increase(29.3f, 29.3f, 1);

    puts("PASS: runtime voltage ceiling before charging, CC, CV, sensor boundaries and lower end target");
    return 0;
}
