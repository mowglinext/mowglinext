import type {Emergency, HighLevelStatus, Status} from '../types/ros';
import {HighLevelStatusConstants, StatusConstants} from '../types/ros';
import {isJsonObject} from './json';

type FieldGuard = (value: unknown) => boolean;
type Fields<T> = {[K in keyof T]-?: FieldGuard};
const boolean: FieldGuard = value => typeof value === 'boolean';
const string: FieldGuard = value => typeof value === 'string';
const finite: FieldGuard = value => typeof value === 'number' && Number.isFinite(value);
const integer = (min: number, max: number): FieldGuard => value =>
    typeof value === 'number' && Number.isInteger(value) && value >= min && value <= max;
const uint8 = integer(0, 255);
const int16 = integer(-32768, 32767); // Negative sentinel indices are legitimate.
const nonnegative: FieldGuard = value => finite(value) && typeof value === 'number' && value >= 0;
const percent: FieldGuard = value => finite(value) && typeof value === 'number' && value >= 0 && value <= 100;
const stamp: FieldGuard = value => isJsonObject(value)
    && integer(-2147483648, 2147483647)(value.sec)
    && integer(0, 999999999)(value.nanosec);

// Validate every present known field. Preserve extensions and missing fields;
// present null/undefined is invalid rather than an invented default.
function matches(value: unknown, fields: Record<string, FieldGuard>): value is Record<string, unknown> {
    return isJsonObject(value) && Object.entries(fields).every(([key, guard]) =>
        !Object.prototype.hasOwnProperty.call(value, key) || guard(value[key]));
}
const emergencyFields: Fields<Emergency> = {
    stamp, active_emergency: boolean, latched_emergency: boolean,
    lift_warning: boolean, lift_duration_sec: nonnegative, reason: string,
};
const highLevelFields: Fields<HighLevelStatus> = {
    state: uint8, state_name: string, sub_state_name: string,
    current_area: int16, current_path: int16, current_path_index: int16,
    total_swaths: int16, completed_swaths: int16, skipped_swaths: int16,
    coverage_percent: percent, gps_quality_percent: percent, battery_percent: percent,
    is_charging: boolean, emergency: boolean,
};
const statusFields: Fields<Status> = {
    stamp, mower_status: uint8, reset_cause: uint8, reset_cause_name: string,
    raspberry_pi_power: boolean, is_charging: boolean, esc_power: boolean,
    rain_detected: boolean, sound_module_available: boolean, sound_module_busy: boolean,
    ui_board_available: boolean, mow_enabled: boolean, firmware_debug_enabled: boolean,
    dig_escalated: boolean, dig_escalated_distance_m: nonnegative,
    dig_escalated_required_distance_m: nonnegative, mower_esc_status: uint8,
    mower_esc_temperature: finite, mower_esc_current: finite,
    mower_motor_temperature: finite, mower_motor_rpm: finite, blade_status_stamp: stamp,
    blade_requested_direction: string, firmware_version: string,
    firmware_protocol_version: uint8, firmware_compatible: boolean,
};
function isEmergency(value: unknown): value is Emergency { return matches(value, emergencyFields); }
function isHighLevelStatus(value: unknown): value is HighLevelStatus { return matches(value, highLevelFields); }
function isStatus(value: unknown): value is Status { return matches(value, statusFields); }

export type TelemetryDecode<T> = {kind: 'valid'; data: T} | {kind: 'invalid' | 'unsupported'};
export function decodeEmergency(value: unknown): TelemetryDecode<Emergency> {
    return isEmergency(value) ? {kind: 'valid', data: value} : {kind: 'invalid'};
}
export function decodeHighLevelStatus(value: unknown): TelemetryDecode<HighLevelStatus> {
    if (!isHighLevelStatus(value)) return {kind: 'invalid'};
    if (value.state !== undefined && ![
        HighLevelStatusConstants.HIGH_LEVEL_STATE_NULL,
        HighLevelStatusConstants.HIGH_LEVEL_STATE_IDLE,
        HighLevelStatusConstants.HIGH_LEVEL_STATE_AUTONOMOUS,
        HighLevelStatusConstants.HIGH_LEVEL_STATE_RECORDING,
        HighLevelStatusConstants.HIGH_LEVEL_STATE_MANUAL_MOWING,
    ].includes(value.state)) return {kind: 'unsupported'};
    // Producer state/substate labels are extensible, not a closed enum.
    return {kind: 'valid', data: value};
}
export function decodeStatus(value: unknown): TelemetryDecode<Status> {
    if (!isStatus(value)) return {kind: 'invalid'};
    if (value.mower_status !== undefined && ![
        StatusConstants.MOWER_STATUS_INITIALIZING, StatusConstants.MOWER_STATUS_OK,
    ].includes(value.mower_status)) return {kind: 'unsupported'};
    if (value.reset_cause !== undefined && ![
        StatusConstants.RESET_CAUSE_UNKNOWN, StatusConstants.RESET_CAUSE_PIN,
        StatusConstants.RESET_CAUSE_POR_PDR, StatusConstants.RESET_CAUSE_BOR,
        StatusConstants.RESET_CAUSE_SFTRST, StatusConstants.RESET_CAUSE_IWDG,
        StatusConstants.RESET_CAUSE_WWDG, StatusConstants.RESET_CAUSE_LPWR,
    ].includes(value.reset_cause)) return {kind: 'unsupported'};
    if (value.blade_requested_direction !== undefined
        && !['', 'unknown', 'off', 'forward', 'reverse'].includes(value.blade_requested_direction)) return {kind: 'unsupported'};
    return {kind: 'valid', data: value};
}

/** Conservative display retention, not a firmware latch or command authority. */
export function retainEmergency(previous: Emergency, next: Emergency): Emergency {
    return {
        ...next,
        ...(previous.active_emergency === true && next.active_emergency === undefined ? {active_emergency: true} : {}),
        ...(previous.latched_emergency === true && next.latched_emergency === undefined ? {latched_emergency: true} : {}),
        ...(previous.lift_warning === true && next.lift_warning === undefined ? {lift_warning: true} : {}),
    };
}
export function retainHighLevelEmergency(previous: HighLevelStatus, next: HighLevelStatus): HighLevelStatus {
    return previous.emergency === true && next.emergency === undefined ? {...next, emergency: true} : next;
}
export function retainStatusWarning(previous: Status, next: Status): Status {
    return previous.dig_escalated === true && next.dig_escalated === undefined ? {...next, dig_escalated: true} : next;
}
/** None of these messages establishes a producer epoch/ordered acquisition.
 * Keep assertion history separate from the latest payload and command guards.
 * A structurally valid false bit cannot acknowledge that history as cleared.
 */
export function accumulateEmergencyAssertions(previous: Emergency, next: Emergency): Emergency {
    return {
        ...(previous.active_emergency === true || next.active_emergency === true ? {active_emergency: true} : {}),
        ...(previous.latched_emergency === true || next.latched_emergency === true ? {latched_emergency: true} : {}),
    };
}
export function accumulateHighLevelAssertions(previous: HighLevelStatus, next: HighLevelStatus): HighLevelStatus {
    return previous.emergency === true || next.emergency === true ? {emergency: true} : {};
}
export type EmergencyDisplay = Emergency & {clearanceUnverified?: boolean};
export type HighLevelDisplay = HighLevelStatus & {clearanceUnverified?: boolean};
/** Never mask a cross-topic assertion with another topic's false bit. */
export function hasEmergency(emergency: EmergencyDisplay, highLevel: HighLevelDisplay): boolean {
    return emergency.active_emergency === true || emergency.latched_emergency === true || highLevel.emergency === true
        || emergency.clearanceUnverified === true || highLevel.clearanceUnverified === true;
}
export function emergencyIsKnown(emergency: EmergencyDisplay, highLevel: HighLevelDisplay): boolean {
    return hasEmergency(emergency, highLevel)
        || (emergency.active_emergency === false && emergency.latched_emergency === false);
}
export function emergencyClearanceUnverified(emergency: EmergencyDisplay, highLevel: HighLevelDisplay): boolean {
    return emergency.clearanceUnverified === true || highLevel.clearanceUnverified === true;
}
/** Opaque producer-clock identity; never compare its value to a browser clock. */
export function telemetryStampIdentity(value: unknown): string | null {
    if (!isJsonObject(value) || !isJsonObject(value.stamp) || !stamp(value.stamp)) return null;
    if (value.stamp.sec === 0 && value.stamp.nanosec === 0) return null;
    return `${String(value.stamp.sec)}:${String(value.stamp.nanosec)}`;
}
