import {describe, expect, it} from 'vitest';
import {decodePower, decodeBladeMeasurements, decodeWheelTicks} from './physicalTelemetry';
import {bladeTelemetry, motorDisplayValue, wheelControllerFromDiagnostics, wheelRpmBetween} from './motorTelemetry';
const escDiagnostic = (role = 'left_wheel', fields: Record<string, string> = {}) => ({status: [{
    name: `mowgli_mavros_bridge/vesc_${role}`, message: 'online', hardware_id: '', level: 0, receivedAt: 10000, receivedMonotonic: 10000,
    values: Object.entries({online: 'true', stale: 'false', age_ms: '100', current_a: '2.5', ...fields})
        .map(([key, value]) => ({key, value})),
}]});

const stamp = {sec: 10, nanosec: 0};
describe('physical measurement boundaries', () => {
    it.each([null, [], '0', 0, false])('rejects a non-object %j', value => {
        expect(decodePower(value)).toBeUndefined();
        expect(decodeBladeMeasurements(value)).toBeUndefined();
        expect(decodeWheelTicks(value)).toBeUndefined();
    });
    it('preserves sparse power, backend extensions and measured zero current/voltage', () => {
        expect(decodePower({})).toEqual({});
        const power = {stamp, v_battery: 26, v_charge: 0, charge_current: 0, charger_enabled: false, charger_status: 'idle', extension: 'future'};
        expect(decodePower(power)).toEqual(power);
    });
    it.each([null, undefined, '0', '', '24V', NaN, Infinity, -Infinity])('does not coerce unavailable or invalid power channels %j', value => {
        expect(decodePower({stamp, v_battery: value, v_charge: value, charge_current: value}))
            .toEqual({stamp});
    });
    it('retains valid traction voltage when a MAVROS dock channel is unavailable', () => {
        expect(decodePower({stamp, v_battery: 24, v_charge: NaN, charge_current: NaN, charger_status: 'unavailable'}))
            .toEqual({stamp, v_battery: 24, charger_status: 'unavailable'});
    });
    it('rejects invalid identity and boolean fields without inventing charger state', () => {
        expect(decodePower({stamp: {sec: 1, nanosec: 1e9}, v_battery: 24})).toBeUndefined();
        expect(decodePower({v_battery: 24, charger_enabled: 'false'})).toBeUndefined();
        expect(decodePower({v_battery: -1, charge_current: -0.2})).toEqual({charge_current: -0.2});
    });
    it.each([NaN, Infinity, -Infinity, '0', null])('rejects one malformed blade channel without losing other valid measurements %j', value => {
        const decoded = decodeBladeMeasurements({blade_status_stamp: stamp, mower_motor_rpm: 0, mower_esc_current: 0, mower_esc_temperature: value});
        expect(decoded).toEqual({blade_status_stamp: stamp, mower_motor_rpm: 0, mower_esc_current: 0});
    });
    it.each(['mowgli', 'mavros', 'openmower'])('keeps measured zero RPM and current for %s', backend => {
        const value = bladeTelemetry({blade_status_stamp: stamp, mower_motor_rpm: 0, mower_esc_current: 0}, 1, 2, backend);
        expect(value.rpm).toBe(0); expect(value.current).toBe(0);
    });
    it('does not treat unpopulated native ESC temperature or MAVROS winding temperature as measured zero', () => {
        const sample = {blade_status_stamp: stamp, mower_esc_temperature: 0, mower_motor_temperature: 0};
        expect(bladeTelemetry(sample, 1, 2, 'mowgli').escTemp).toBeNaN();
        expect(bladeTelemetry(sample, 1, 2, 'mavros').motorTemp).toBeNaN();
        expect(bladeTelemetry(sample, 1, 2, 'openmower')).toMatchObject({escTemp: 0, motorTemp: 0});
        expect(bladeTelemetry(sample, 1, 2, 'future').motorTemp).toBeNaN();
    });
    it('accepts fractional encoder calibration, negative ROS time and sparse wheel frames', () => {
        expect(decodeWheelTicks({})).toEqual({});
        const ticks = {stamp: {sec: -1, nanosec: 0}, wheel_tick_factor: 319.305, valid_wheels: 12, wheel_ticks_rl: 0, wheel_direction_rl: 1};
        expect(decodeWheelTicks(ticks)).toEqual(ticks);
    });
    it.each([NaN, Infinity, '0', -1, 0.5, 0x100000000])('rejects invalid wheel counters %j', value => {
        expect(decodeWheelTicks({wheel_ticks_rl: value})).toBeUndefined();
    });
    it('keeps unknown wheel RPM and rejects malformed masks and stamps', () => {
        expect(decodeWheelTicks({valid_wheels: '12'})).toBeUndefined();
        expect(decodeWheelTicks({valid_wheels: 16})).toBeUndefined();
        expect(decodeWheelTicks({stamp: {sec: 1, nanosec: 0.5}})).toBeUndefined();
        expect(motorDisplayValue(NaN)).toBe('—');
        expect(wheelRpmBetween({stamp, wheel_tick_factor: 1000, valid_wheels: 12},
            {stamp: {sec: 11, nanosec: 0}, wheel_tick_factor: 1000, valid_wheels: 12, wheel_ticks_rl: 0.5}, 0.05).rl).toBeNaN();
    });
    it.each(['', '  ', '0x18', 'NaN', 'Infinity', '25 V', 'unavailable'])('does not coerce diagnostic numbers %j', value => {
        expect(wheelControllerFromDiagnostics(escDiagnostic('left_wheel', {current_a: value}), 'left_wheel', 10000)?.current).toBeNaN();
    });
    it.each([
        ['rpm_abs', 'motorRpmAbs', '-1'], ['voltage_v', 'voltage', '-1'],
        ['failure_flags', 'failureFlags', '0.5'], ['error_count', 'errorCount', '4294967296'],
    ] as const)('rejects invalid %s domain values', (key, field, invalid) => {
        const sample = escDiagnostic('left_wheel', {[key]: invalid, rpm_valid: 'true', failure_flags_valid: 'true', error_count_valid: 'true'});
        expect(wheelControllerFromDiagnostics(sample, 'left_wheel', 10000)?.[field]).toBeNaN();
    });
    it('guards malformed diagnostic structures instead of throwing or showing a reading', () => {
        const fixture = escDiagnostic();
        const entry = fixture.status?.[0];
        for (const values of [null, [], [{key: 'current_a', value: 0}], [{key: 'age_ms', value: '0'}, {key: 'age_ms', value: '0'}]]) {
            const result = wheelControllerFromDiagnostics({status: [{...entry, values}]}, 'left_wheel', 10000);
            if (Array.isArray(values) && values.length === 0) expect(result?.current).toBeNaN();
            else expect(result).toBeUndefined();
        }
        expect(wheelControllerFromDiagnostics({status: null}, 'left_wheel', 1)).toBeUndefined();
    });
});
