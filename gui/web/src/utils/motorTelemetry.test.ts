import {describe, expect, it} from 'vitest';
import type {Status, WheelTick} from '../types/ros';
import type {DiagnosticArray} from '../hooks/useDiagnostics';
import {wheelControllerFromDiagnostics, wheelControllerFields, showEncoderWheel, bladeTelemetry, motorDisplayValue, wheelRpmBetween} from './motorTelemetry';

const status: Status = {stamp: {sec: 100, nanosec: 0}, blade_status_stamp: {sec: 99, nanosec: 0},
    mower_motor_rpm: 3200, mower_esc_current: 1.25, mower_esc_temperature: 42,
    mower_motor_temperature: 36};
const ticks = (sec: number, count: number, factor = 1000): WheelTick => ({stamp: {sec, nanosec: 0},
    wheel_tick_factor: factor, valid_wheels: 12, wheel_ticks_rl: count, wheel_ticks_rr: count,
    wheel_direction_rl: 1, wheel_direction_rr: 0});

describe('existing motor telemetry contract', () => {
    it('projects native Status unchanged in RPM, A and degrees Celsius', () => {
        expect(bladeTelemetry(status, 1000, 1001, 'mowgli')).toEqual({fresh: true, rpm: 3200,
            current: 1.25, escTemp: 42, motorTemp: 36});
    });
    it('uses the same blade values for MAVROS and keeps unavailable winding temperature unknown', () => {
        const result = bladeTelemetry(status, 1000, 1001, 'mavros');
        expect(result).toMatchObject({rpm: 3200, current: 1.25, escTemp: 42});
        expect(result.motorTemp).toBeNaN();
    });
    it('preserves actual zero readings and does not infer zero from absent/null/nonfinite fields', () => {
        expect(bladeTelemetry({...status, mower_motor_rpm: 0, mower_esc_current: 0}, 1000, 1001, 'mavros'))
            .toMatchObject({rpm: 0, current: 0});
        const result = bladeTelemetry({...status, mower_esc_current: undefined, mower_esc_temperature: NaN}, 1000, 1001, 'mavros');
        expect(result.current).toBeNaN(); expect(result.escTemp).toBeNaN();
        expect(motorDisplayValue(result.current)).toBe('—');
    });
    it.each([
        {...status, blade_status_stamp: {sec: 0, nanosec: 0}},
        {...status, stamp: {sec: 106, nanosec: 0}},
    ])('requires a current direct blade report in the shared ROS clock', sample => {
        expect(bladeTelemetry(sample, 1000, 1001, 'mowgli').rpm).toBeNaN();
    });
    it('accepts a blade report newer than the independent MAVROS envelope', () => {
        expect(bladeTelemetry({...status, stamp: {sec: 98, nanosec: 0}}, 1000, 1001, 'mavros').rpm).toBe(3200);
    });
    it('independently rejects missing/expired receipt liveness, without comparing ROS time to wall time', () => {
        expect(bladeTelemetry(status, null, Date.now(), 'mowgli').rpm).toBeNaN();
        expect(bladeTelemetry(status, 1000, 6000, 'mowgli').rpm).toBeNaN();
        expect(bladeTelemetry(status, 1000, 1001, 'mowgli').rpm).toBe(3200);
    });
    it('derives the same calibrated wheel RPM from native and scaled MAVROS ticks, not raw motor RPM', () => {
        const radius = 1 / (2 * Math.PI);
        const native = wheelRpmBetween(ticks(1, 0), ticks(2, 100), radius);
        const mavros = wheelRpmBetween(ticks(1, 0, 1000000), ticks(2, 100000, 1000000), radius);
        expect(native).toEqual(mavros);
        expect(native.rl).toBe(6); expect(native.rr).toBe(-6);
        expect(native.fl).toBeNaN(); expect(native.fr).toBeNaN();
    });
    it('preserves valid stopped wheels and respects per-wheel invalidation', () => {
        const previous = ticks(1, 100); const current = ticks(2, 100);
        expect(wheelRpmBetween(previous, current, 0.04).rl).toBe(0);
        expect(wheelRpmBetween(previous, {...current, valid_wheels: 8}, 0.04).rl).toBeNaN();
    });
    it('invalidates calibration changes, uncalibrated data, resets and invalid timing', () => {
        for (const current of [ticks(2, 100, 0), ticks(2, 100, 2000), ticks(1, 100), ticks(0, 100), ticks(10, 100), ticks(2, 0)]) {
            expect(wheelRpmBetween(ticks(1, 10), current, 0.04).rl).toBeNaN();
        }
    });
    it('handles uint32 rollover without interpreting an ordinary counter reset as motion', () => {
        expect(wheelRpmBetween(ticks(1, 0xffffffff), ticks(2, 1), 1/(2*Math.PI)).rl).toBeCloseTo(0.12);
    });
});

export const escDiagnostic = (role = 'left_wheel', overrides: Record<string, string> = {}, level = 0): DiagnosticArray => ({status: [{
    name: `mowgli_mavros_bridge/vesc_${role}`, message: 'online', hardware_id: '', level, receivedAt: 10000,
    values: Object.entries({online: 'true', stale: 'false', age_ms: '100', rpm_abs: '9000', rpm_valid: 'true',
        temperature_c: '31.5', temperature_valid: 'true', current_a: '2.5', voltage_v: '24', totalcurrent_ah: '1.25',
        failure_flags: '0', failure_flags_valid: 'true', error_count: '0', error_count_valid: 'true', count: '123', esc_index: '47', ...overrides})
        .map(([key, value]) => ({key, value})),
}]});
describe('wheel ESC presentation from role diagnostics', () => {
    it('uses logical wheel roles regardless of hardware slots and never maps blade-only or absent diagnostics onto wheels', () => {
        const diagnostics = {status: [...escDiagnostic().status!, ...escDiagnostic('right_wheel', {esc_index: '3', temperature_c: '42'}).status!]};
        expect(wheelControllerFromDiagnostics(diagnostics, 'left_wheel', 10100)?.temperature).toBe(31.5);
        expect(wheelControllerFromDiagnostics(diagnostics, 'right_wheel', 10100)?.temperature).toBe(42);
        expect(wheelControllerFromDiagnostics({}, 'left_wheel', 10100)).toBeUndefined();
        expect(wheelControllerFromDiagnostics(escDiagnostic('mower'), 'left_wheel', 10100)).toBeUndefined();
    });
    it.each([
        [{online: 'false', stale: 'false', age_ms: '-1'}, 'absent'],
        [{online: 'false', stale: 'true', age_ms: '4000'}, 'stale'],
        [{online: 'true', stale: 'false', age_ms: '9000'}, 'stale'],
    ])('suppresses unavailable acquisitions even when diagnostics have just been republished', (values, state) => {
        const telemetry = wheelControllerFromDiagnostics(escDiagnostic('left_wheel', values), 'left_wheel', 10000)!;
        expect(telemetry.state).toBe(state);expect(telemetry.voltage).toBeNaN();expect(telemetry.motorRpmAbs).toBeNaN();
    });
    it('adds GUI elapsed time to acquisition age and rejects stale receipt/future clock stamps', () => {
        expect(wheelControllerFromDiagnostics(escDiagnostic('left_wheel', {age_ms: '4900'}), 'left_wheel', 10200)?.state).toBe('stale');
        expect(wheelControllerFromDiagnostics(escDiagnostic(), 'left_wheel', 16000)?.state).toBe('stale');
        expect(wheelControllerFromDiagnostics(escDiagnostic(), 'left_wheel', 9000)?.state).toBe('stale');
    });
    it('shows fault state and still exposes fresh measured electrical readings, never telemetry count as errors', () => {
        const telemetry = wheelControllerFromDiagnostics(escDiagnostic('left_wheel', {failure_flags: '4', error_count: '7'}, 1), 'left_wheel', 10000)!;
        expect(telemetry.state).toBe('fault');expect(telemetry.current).toBe(2.5);expect(telemetry.failureFlags).toBe(4);expect(telemetry.errorCount).toBe(7);
        expect(wheelControllerFromDiagnostics(escDiagnostic('left_wheel', {}, 2), 'left_wheel', 10000)?.state).toBe('fault');
    });
    it('honors per-field validity and rejects blank/unavailable/non-finite values while retaining real zeros', () => {
        const telemetry = wheelControllerFromDiagnostics(escDiagnostic('left_wheel', {temperature_valid: 'false', current_a: 'unavailable', voltage_v: 'NaN', totalcurrent_ah: '', rpm_abs: '0', error_count_valid: 'false'}), 'left_wheel', 10000)!;
        expect(telemetry.temperature).toBeNaN();expect(telemetry.current).toBeNaN();expect(telemetry.voltage).toBeNaN();expect(telemetry.ampHours).toBeNaN();expect(telemetry.errorCount).toBeNaN();expect(telemetry.motorRpmAbs).toBe(0);
        const zero = wheelControllerFromDiagnostics(escDiagnostic('left_wheel', {current_a: '0', voltage_v: '0', temperature_c: '0', totalcurrent_ah: '0', error_count: '0'}), 'left_wheel', 10000)!;
        expect([zero.current, zero.voltage, zero.temperature, zero.ampHours, zero.errorCount]).toEqual([0,0,0,0,0]);
    });
    it('keeps motor RPM distinct from calibrated WheelTick RPM and survives absent wheel calibration', () => {
        const ticks = (sec: number, count: number, factor = 1000): WheelTick => ({stamp: {sec, nanosec: 0}, wheel_tick_factor: factor, valid_wheels: 12,
            wheel_ticks_rl: count, wheel_ticks_rr: count, wheel_direction_rl: 1, wheel_direction_rr: 0});
        const physical = wheelRpmBetween(ticks(1, 0), ticks(2, 100), 1 / (2 * Math.PI));
        expect(physical.rl).toBe(6);expect(physical.rr).toBe(-6);
        const motor = wheelControllerFromDiagnostics(escDiagnostic(), 'left_wheel', 10000)!;
        expect(motor.motorRpmAbs).toBe(9000);
        expect(wheelRpmBetween(ticks(1, 0, 0), ticks(2, 100, 0), 0.1).rl).toBeNaN();
        expect(motor.voltage).toBe(24);
    });
    it('retains an existing rear slot independently of encoder mask or calibration', () => {
        const telemetry = wheelControllerFromDiagnostics(escDiagnostic(), 'left_wheel', 10000);
        expect(showEncoderWheel(8, 4, telemetry)).toBe(true);expect(showEncoderWheel(8, 4)).toBe(false);expect(showEncoderWheel(0, 4)).toBe(true);
    });
    it('uses one field presentation for every available source and leaves native wheel-only data unchanged', () => {
        const nativeWheelOnly = wheelRpmBetween(ticks(1, 0), ticks(2, 100), 1 / (2 * Math.PI));
        expect(nativeWheelOnly.rl).toBe(6);expect(wheelControllerFields()).toEqual([]);
        const controller = wheelControllerFromDiagnostics(escDiagnostic('left_wheel', {current_a: '0', temperature_c: '0'}), 'left_wheel', 10000);
        expect(wheelControllerFields(controller).find(field => field.key === 'current')?.value).toBe(0);
        expect(wheelControllerFields(controller).find(field => field.key === 'motorRpm')?.value).toBe(9000);
        expect(nativeWheelOnly.rl).toBe(6);
        expect(wheelControllerFields({...controller!, voltage: NaN}).find(field => field.key === 'voltage')?.value).toBeNaN();
    });

});
