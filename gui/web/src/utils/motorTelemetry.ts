import {isDiagnosticStale} from '../hooks/useDiagnostics';
import {isJsonObject} from './json';
import {isTimestampedDiagnosticStatus} from './diagnosticTelemetry';
import {isRosStamp, decodeWheelTicks} from './physicalTelemetry';
import type {Status, WheelTick} from '../types/ros';

/** Display liveness only; never participates in motor control or safety. */
export const MOTOR_DISPLAY_STALE_MS = 5000;
const finite = (v: unknown): number => typeof v === 'number' && Number.isFinite(v) ? v : NaN;
export const motorDisplayValue = (v: number, precision = 0): string => Number.isFinite(v) ? v.toFixed(precision) : '—';
const stampSeconds = (stamp: Status['stamp']): number =>
    isRosStamp(stamp)
        ? stamp.sec + stamp.nanosec * 1e-9 : NaN;

export function bladeTelemetry(status: Status, receivedAt: number | null, now: number, backend?: string) {
    // Compare observation and envelope only in their shared ROS clock. Receipt
    // liveness is independent, in the browser clock (including simulation).
    // MAVROS envelope freshness comes from other streams and may precede the
    // blade observation; that must not invalidate a newly acquired blade report.
    const blade = stampSeconds(status.blade_status_stamp);
    const envelope = stampSeconds(status.stamp);
    const age = envelope - blade;
    const fresh = receivedAt !== null && now - receivedAt >= 0 && now - receivedAt < MOTOR_DISPLAY_STALE_MS
        && blade > 0 && (!Number.isFinite(envelope) || envelope <= blade || age < MOTOR_DISPLAY_STALE_MS / 1000);
    return {
        fresh,
        rpm: fresh ? finite(status.mower_motor_rpm) : NaN,
        current: fresh ? finite(status.mower_esc_current) : NaN,
        // Native Status never populates ESC temperature; its default zero is not measured.
        escTemp: fresh && (backend === 'mavros' || backend === 'openmower') ? finite(status.mower_esc_temperature) : NaN,
        // MAVROS reports ESC temperature, not winding temperature. Its default
        // scalar zero cannot prove a measurement of the motor winding.
        motorTemp: fresh && (backend === 'mowgli' || backend === 'openmower') ? finite(status.mower_motor_temperature) : NaN,
    };
}

export const unknownWheelRpm = () => ({fl: NaN, fr: NaN, rl: NaN, rr: NaN, bodyOmega: NaN});

export function wheelRpmBetween(previous: WheelTick, current: WheelTick, radius: number) {
    const result = unknownWheelRpm();
    if (decodeWheelTicks(previous) === undefined || decodeWheelTicks(current) === undefined) return result;
    const factor = finite(current.wheel_tick_factor);
    const dt = stampSeconds(current.stamp) - stampSeconds(previous.stamp);
    if (!(factor > 0) || factor !== previous.wheel_tick_factor || !(radius > 0) || !Number.isFinite(radius)
        || !(dt > 0 && dt < MOTOR_DISPLAY_STALE_MS / 1000)) return result;
    const circumference = 2 * Math.PI * radius;
    for (const [wheel, mask] of [['fl', 1], ['fr', 2], ['rl', 4], ['rr', 8]] as const) {
        if (!((current.valid_wheels ?? 0) & (previous.valid_wheels ?? 0) & mask)) continue;
        const direction = current[`wheel_direction_${wheel}`];
        const counter = finite(current[`wheel_ticks_${wheel}`]);
        const old = finite(previous[`wheel_ticks_${wheel}`]);
        if ((direction !== 0 && direction !== 1) || !Number.isFinite(counter) || !Number.isFinite(old)) continue;
        let delta = counter - old;
        if (delta < -0x80000000) delta += 0x100000000; // uint32 rollover, not an ordinary reset
        if (delta < 0) continue;
        result[wheel] = delta * (direction === 1 ? 1 : -1) / factor / circumference * 60 / dt;
    }
    // Preserve the existing display estimate; no host-side motion control.
    result.bodyOmega = (result.rr - result.rl) * circumference / 60 / 0.32;
    return result;
}

export type WheelRole = 'left_wheel' | 'right_wheel';
export interface WheelControllerTelemetry {
    state: 'online' | 'absent' | 'stale' | 'fault'; ageMs: number;
    temperature: number; current: number; voltage: number; ampHours: number; motorRpmAbs: number;
    failureFlags: number; errorCount: number;
}
function numberValue(value: unknown) {
    return typeof value === 'string' && /^[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:e[+-]?\d+)?$/i.test(value.trim()) && Number.isFinite(Number(value)) ? Number(value) : NaN;
}
/** Role-based display only. No assumptions about physical ESC indices or calibrated wheel speed. */
export function wheelControllerFromDiagnostics(diagnostics: unknown, role: WheelRole, now: number): WheelControllerTelemetry | undefined {
    if (!isJsonObject(diagnostics) || !Array.isArray(diagnostics.status)) return undefined;
    const entries: unknown[] = diagnostics.status;
    const entry = entries.find(item => isJsonObject(item) && item.name === `mowgli_mavros_bridge/vesc_${role}`);
    if (!isTimestampedDiagnosticStatus(entry)) return undefined;
    const values = new Map(entry.values.map(value => [value.key, value.value]));
    const field = (key: string) => numberValue(values.get(key));
    const elapsed = now - entry.receivedAt;
    const age = field('age_ms');
    const ageMs = age >= 0 && Number.isFinite(elapsed) && elapsed >= 0 ? age + elapsed : NaN;
    const receiptFresh = Number.isFinite(entry.receivedAt) && elapsed >= 0 && !isDiagnosticStale(entry, now)
        && elapsed < MOTOR_DISPLAY_STALE_MS;
    const online = values.get('online') === 'true';
    const absent = !online && values.get('stale') === 'false' && age < 0;
    // A newly republished diagnostic preserves acquisition age; receipt is not acquisition.
    const fresh = receiptFresh && online && values.get('stale') === 'false' && [0, 1, 2].includes(entry.level)
        && Number.isFinite(ageMs) && ageMs < MOTOR_DISPLAY_STALE_MS;
    const valid = (key: string, flag?: string) => {
        if (!fresh || (flag && values.get(flag) !== 'true')) return NaN;
        const value = field(key);
        if ((key === 'rpm_abs' || key === 'voltage_v') && value < 0) return NaN;
        if ((key === 'failure_flags' || key === 'error_count')
            && (!Number.isInteger(value) || value < 0 || value > 0xffffffff)) return NaN;
        return value;
    };
    const failureFlags = valid('failure_flags', 'failure_flags_valid');
    const fault = entry.level === 1 || entry.level === 2 || (Number.isFinite(failureFlags) && failureFlags !== 0);
    return {state: !receiptFresh ? 'stale' : absent ? 'absent' : !fresh ? 'stale' : fault ? 'fault' : 'online',
        ageMs, temperature: valid('temperature_c', 'temperature_valid'), current: valid('current_a'), voltage: valid('voltage_v'),
        ampHours: valid('totalcurrent_ah'), motorRpmAbs: valid('rpm_abs', 'rpm_valid'), failureFlags,
        errorCount: valid('error_count', 'error_count_valid')};
}
/** Keep an existing rear-wheel slot visible when ESC data exists even without encoder calibration. */
export function showEncoderWheel(mask: number, bit: number, esc?: WheelControllerTelemetry): boolean {
    return mask === 0 || (mask & bit) !== 0 || esc !== undefined;
}

/** One optional field model for the existing wheel renderer, independent of hardware backend. */
export function wheelControllerFields(telemetry?: WheelControllerTelemetry) {
    if (!telemetry) return [];
    return [
        {key: 'temperature', value: telemetry.temperature, unit: '°C', precision: 1},
        {key: 'current', value: telemetry.current, unit: 'A', precision: 2},
        {key: 'voltage', value: telemetry.voltage, unit: 'V', precision: 2},
        {key: 'ampHours', value: telemetry.ampHours, unit: 'Ah', precision: 3},
        {key: 'motorRpm', value: telemetry.motorRpmAbs, unit: 'RPM', precision: 0},
        {key: 'failureFlags', value: telemetry.failureFlags, unit: '', precision: 0},
        {key: 'errorCount', value: telemetry.errorCount, unit: '', precision: 0},
        {key: 'age', value: telemetry.ageMs, unit: 'ms', precision: 0},
    ];
}
