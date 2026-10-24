import type {Power, Status, WheelTick} from '../types/ros';
import {isJsonObject} from './json';
import {decodeStatus} from './statusTelemetry';

export const finiteMeasurement = (value: unknown): value is number =>
    typeof value === 'number' && Number.isFinite(value);
export const powerDisplayValue = (value: unknown): number | '—' => finiteMeasurement(value) ? value : '—';
const uint = (value: unknown, max: number) => finiteMeasurement(value)
    && Number.isInteger(value) && value >= 0 && value <= max;

export function isRosStamp(value: unknown): value is NonNullable<Power['stamp']> {
    return isJsonObject(value) && finiteMeasurement(value.sec) && Number.isInteger(value.sec)
        && value.sec >= -2147483648 && value.sec <= 2147483647 && uint(value.nanosec, 999999999);
}

/** Partial channels are legitimate (notably MAVROS unavailable-channel NaN).
 * Invalid measurements are omitted, never coerced into zero or treated as observed.
 */
export function decodePower(raw: unknown): Power | undefined {
    if (!isJsonObject(raw)) return undefined;
    const {stamp, v_charge, v_battery, charge_current, charger_enabled, charger_status, ...extensions} = raw;
    if ((stamp !== undefined && !isRosStamp(stamp))
        || (charger_enabled !== undefined && typeof charger_enabled !== 'boolean')
        || (charger_status !== undefined && typeof charger_status !== 'string')) return undefined;
    return {
        ...extensions,
        ...(isRosStamp(stamp) ? {stamp} : {}),
        ...(finiteMeasurement(v_charge) && v_charge >= 0 ? {v_charge} : {}),
        ...(finiteMeasurement(v_battery) && v_battery >= 0 ? {v_battery} : {}),
        ...(finiteMeasurement(charge_current) ? {charge_current} : {}),
        ...(typeof charger_enabled === 'boolean' ? {charger_enabled} : {}),
        ...(typeof charger_status === 'string' ? {charger_status} : {}),
    };
}

/** Physical channel validation is independent of the unchanged safety-status decoder. */
export function decodeBladeMeasurements(raw: unknown): Status | undefined {
    if (!isJsonObject(raw)) return undefined;
    const {mower_motor_rpm, mower_esc_current, mower_esc_temperature, mower_motor_temperature, ...rest} = raw;
    const decoded = decodeStatus({
        ...rest,
        ...(finiteMeasurement(mower_motor_rpm) ? {mower_motor_rpm} : {}),
        ...(finiteMeasurement(mower_esc_current) ? {mower_esc_current} : {}),
        ...(finiteMeasurement(mower_esc_temperature) ? {mower_esc_temperature} : {}),
        ...(finiteMeasurement(mower_motor_temperature) ? {mower_motor_temperature} : {}),
    });
    return decoded.kind === 'valid' ? decoded.data : undefined;
}

export function decodeWheelTicks(raw: unknown): WheelTick | undefined {
    if (!isWheelTicks(raw)) return undefined;
    return raw;
}
function isWheelTicks(raw: unknown): raw is WheelTick {
    if (!isJsonObject(raw)) return false;
    if ('stamp' in raw && !isRosStamp(raw.stamp)) return false;
    if ('wheel_tick_factor' in raw && (!finiteMeasurement(raw.wheel_tick_factor) || raw.wheel_tick_factor < 0)) return false;
    if ('valid_wheels' in raw && !uint(raw.valid_wheels, 15)) return false;
    for (const wheel of ['fl', 'fr', 'rl', 'rr']) {
        if (`wheel_ticks_${wheel}` in raw && !uint(raw[`wheel_ticks_${wheel}`], 0xffffffff)) return false;
        if (`wheel_direction_${wheel}` in raw && !uint(raw[`wheel_direction_${wheel}`], 255)) return false;
    }
    return true;
}
