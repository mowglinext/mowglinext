import {describe, expect, it} from 'vitest';
import {computeBatteryPercent, getBatteryLevel, hasBatteryReading} from './battery';
describe('battery display validity', () => {
    it.each([undefined, null, NaN, Infinity, -Infinity, '26', 0, -1])('has no pack reading for %j', voltage => {
        expect(hasBatteryReading(voltage)).toBe(false);
        expect(computeBatteryPercent(undefined, voltage, {})).toBeNaN();
    });
    it('retains valid empty/full pack percentages, legacy string thresholds and positive reported percentages', () => {
        expect(computeBatteryPercent(undefined, 24, {})).toBe(0);
        expect(computeBatteryPercent(undefined, 28, {})).toBe(100);
        expect(computeBatteryPercent(72, 27, {})).toBe(72);
        expect(computeBatteryPercent(0, 26, {})).toBe(50); // ROS default 0 has no validity bit.
        expect(computeBatteryPercent(undefined, 26, {battery_full_voltage: '28.0', battery_empty_voltage: 24})).toBe(50);
    });
    it.each([NaN, Infinity, -Infinity, '72', 101, -1])('ignores an invalid reported percentage %j and uses valid voltage', percent => {
        expect(computeBatteryPercent(percent, 26, {})).toBe(50);
    });
    it.each(['28 V', '', '0x1c', false, {}, NaN, Infinity, 24, 20])('does not calculate a percentage with invalid/inverted thresholds %j', full => {
        expect(computeBatteryPercent(undefined, 26, {battery_full_voltage: full})).toBeNaN();
    });
    it.each([NaN, Infinity, -1, 101])('does not label invalid percent %j healthy', percent => {
        expect(getBatteryLevel(percent)).toBe('unknown');
    });
});
