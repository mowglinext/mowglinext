// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include "charger_under_test.c"

static void start(void) {
    chargerInputVoltage = 0.0f;
    ChargeController();
    test_tick = 1000u;
    tf4 = 1u;  // Normal battery-powerbus state at boot.
    offset_writes = 0u;
    charge_current_offset.f = 0.125f;
    current_without_offset = 0.25f;
    battery_voltage = charge_voltage = 26.0f;
    current = 0.0f;
    chargerInputVoltage = 32.0f;
    ChargeController();
    assert(chargecontrol_is_charging == CHARGER_STATE_CONNECTED);
    assert(tf4 == 0u);
    assert(chargecontrol_pwm_val == 0u && TIM1->CCR1 == 0u);
}

static void disconnected(void) {
    chargerInputVoltage = 0.0f;
    ChargeController();
    assert(chargecontrol_is_charging == CHARGER_STATE_IDLE);
    assert(chargecontrol_pwm_val == 0u && TIM1->CCR1 == 0u);
    printf("Disconnected at %u ms: TF4=%u\n", test_tick, tf4);
    fflush(stdout);
    assert(tf4 == 1u);
}

int main(void) {
    const unsigned delays[] = {0u, 50u, 99u, 100u, 101u};
    for (unsigned i = 0; i < sizeof(delays) / sizeof(delays[0]); ++i) {
        start();
        test_tick += delays[i];
        if (delays[i] > 100u) {
            ChargeController();
            assert(chargecontrol_is_charging == CHARGER_STATE_CHARGING_CC);
            assert(tf4 == 1u);
        }
        disconnected();
        for (unsigned j = 0; j < 10; ++j) {
            test_tick += 10u;
            disconnected();
        }
        assert(offset_writes == (delays[i] > 100u ? 2u : 0u));
        assert(charge_current_offset.f == (delays[i] > 100u ? 0.25f : 0.125f));

        /* Reconnection starts a fresh settling interval, then powers the
         * bus again and records the offset exactly once. */
        chargerInputVoltage = 32.0f;
        ChargeController();
        assert(chargecontrol_is_charging == CHARGER_STATE_CONNECTED);
        assert(tf4 == 0u);
        const unsigned writes_before = offset_writes;
        test_tick += 100u;
        ChargeController();
        assert(chargecontrol_is_charging == CHARGER_STATE_CONNECTED);
        assert(tf4 == 0u && offset_writes == writes_before);
        test_tick += 1u;
        ChargeController();
        assert(chargecontrol_is_charging == CHARGER_STATE_CHARGING_CC);
        assert(tf4 == 1u && offset_writes == writes_before + 2u);
        ChargeController();
        assert(tf4 == 1u && offset_writes == writes_before + 2u);
        disconnected();
    }
    puts("PASS: aborted/settled contacts restore powerbus; offset and reconnect timing preserved");
    return 0;
}
