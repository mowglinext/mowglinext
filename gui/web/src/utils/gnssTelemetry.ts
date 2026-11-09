import type {GnssStatus} from '../types/ros';
import {GnssStatusConstants as C} from '../types/ros';
import {isJsonObject} from './json';
import {diagnosticPublicationIdentity} from './diagnosticTelemetry';
import {finiteMeasurement, isRosStamp} from './physicalTelemetry';
import type {TelemetryDecode} from './statusTelemetry';

const uint = (value: unknown, max: number): boolean => finiteMeasurement(value)
    && Number.isInteger(value) && value >= 0 && value <= max;
const boolean = (value: unknown): boolean => typeof value === 'boolean';
const string = (value: unknown): boolean => typeof value === 'string';
const nonnegative = (value: unknown): boolean => finiteMeasurement(value) && value >= 0;
const schema = {
    header: (v: unknown) => isJsonObject(v) && isRosStamp(v.stamp) && typeof v.frame_id === 'string',
    position_observation_sequence: (v: unknown) => uint(v, Number.MAX_SAFE_INTEGER),
    source_id: string, source_incarnation: string,
    backend: string, receiver_vendor: string, receiver_model: string, receiver_firmware: string,
    fix_type: (v: unknown) => uint(v, C.FIX_TYPE_DGPS),
    fix_valid: boolean, differential_corrections: boolean, corrections_active: boolean,
    dead_reckoning: boolean, rtk_mode: (v: unknown) => uint(v, C.RTK_MODE_FIXED),
    dual_antenna_heading: boolean, interference_detected: boolean, jamming_detected: boolean,
    capability_flags: (v: unknown) => uint(v, 0xffffffff),
    value_flags: (v: unknown) => uint(v, 0xffffffff),
    horizontal_accuracy_m: nonnegative, vertical_accuracy_m: nonnegative,
    satellites_used: (v: unknown) => uint(v, 65535),
    satellites_visible: (v: unknown) => uint(v, 65535),
    satellites_tracked: (v: unknown) => uint(v, 65535),
    mean_cn0_db_hz: nonnegative, max_cn0_db_hz: nonnegative,
    dual_antenna_baseline: boolean, baseline_azimuth_deg: finiteMeasurement,
    baseline_pitch_deg: finiteMeasurement, baseline_length_m: nonnegative,
    baseline_solution_status: (v: unknown) => uint(v, C.BASELINE_STATUS_NOT_CONFIGURED),
    correction_stream_status: (v: unknown) => uint(v, C.CORRECTION_STREAM_STATUS_ERROR),
    msm_summary_seen: boolean, msm_summary_decoded: boolean, msm_summary_valid: boolean,
    msm_summary_message_type: (v: unknown) => uint(v, 65535),
    msm_summary_station_id: (v: unknown) => uint(v, 65535),
    msm_summary_constellations_seen: string,
    msm_summary_satellite_count: (v: unknown) => uint(v, 65535),
    msm_summary_signal_count: (v: unknown) => uint(v, 65535),
    msm_summary_cell_count: (v: unknown) => uint(v, 65535),
    msm_summary_age_s: nonnegative,
} satisfies Partial<Record<keyof GnssStatus, (value: unknown) => boolean>>;

const measurementFlags: Record<string, number> = {
    horizontal_accuracy_m: C.CAP_HORIZONTAL_ACCURACY, vertical_accuracy_m: C.CAP_VERTICAL_ACCURACY,
    mean_cn0_db_hz: C.CAP_MEAN_CN0, max_cn0_db_hz: C.CAP_MAX_CN0,
    baseline_azimuth_deg: C.CAP_BASELINE_AZIMUTH, baseline_pitch_deg: C.CAP_BASELINE_PITCH,
    baseline_length_m: C.CAP_BASELINE_LENGTH, msm_summary_age_s: C.CAP_MSM_SUMMARY,
};

const currentFields: Record<string, number> = {
    ...measurementFlags, rtk_mode: C.CAP_RTK_MODE,
    satellites_used: C.CAP_SATELLITES_USED, satellites_visible: C.CAP_SATELLITES_VISIBLE,
    satellites_tracked: C.CAP_SATELLITES_TRACKED, dual_antenna_baseline: C.CAP_DUAL_ANTENNA_BASELINE,
    interference_detected: C.CAP_INTERFERENCE_STATUS, jamming_detected: C.CAP_JAMMING_STATUS,
    baseline_solution_status: C.CAP_BASELINE_SOLUTION_STATUS,
    correction_stream_status: C.CAP_CORRECTION_STREAM,
    msm_summary_seen: C.CAP_MSM_SUMMARY, msm_summary_decoded: C.CAP_MSM_SUMMARY,
    msm_summary_valid: C.CAP_MSM_SUMMARY,
};

/** All retained generated-model fields have been checked; extension fields are not consumed. */
function isGnssStatus(value: unknown): value is GnssStatus {
    return isJsonObject(value) && Object.entries(schema).every(([key, validate]) =>
        !Object.prototype.hasOwnProperty.call(value, key) || validate(value[key]));
}

/** Decoded MessagePack is unknown. Absent fields are not zero/false measurements. */
export function decodeGnssStatus(raw: unknown): TelemetryDecode<GnssStatus> {
    if (!isJsonObject(raw)) return {kind: 'invalid'};
    if (typeof raw.capability_flags === 'number' && typeof raw.value_flags === 'number'
        && uint(raw.capability_flags, 0xffffffff) && uint(raw.value_flags, 0xffffffff)
        && (raw.value_flags & ~raw.capability_flags) !== 0) return {kind: 'invalid'};
    if (typeof raw.value_flags === 'number' && uint(raw.value_flags, 0xffffffff)) {
        for (const [field, flag] of Object.entries(currentFields)) {
            if ((raw.value_flags & flag) !== 0 && !Object.prototype.hasOwnProperty.call(raw, field)) return {kind: 'invalid'};
        }
    }
    const decoded: Record<string, unknown> = {};
    for (const [key, validate] of Object.entries(schema)) {
        if (!Object.prototype.hasOwnProperty.call(raw, key)) continue;
        const value = raw[key];
        // MAVROS and native adapters can report NaN for unavailable float channels.
        // Foxglove JSON sanitizes that sentinel to null. Only an explicit absent-value bit
        // permits either representation; never accept it
        // as an available measurement, or permit strings as numeric sentinels.
        const flag = measurementFlags[key];
        if (flag !== undefined && (value === null || typeof value === 'number' && Number.isNaN(value))
            && uint(raw.value_flags, 0xffffffff) && typeof raw.value_flags === 'number'
            && (raw.value_flags & flag) === 0) continue;
        if (!validate(value)) return {kind: 'invalid'};
        decoded[key] = value;
    }
    if (!Object.keys(decoded).some(key => key !== 'header')) return {kind: 'unsupported'};
    if (!isGnssStatus(decoded)) return {kind: 'invalid'};
    return {kind: 'valid', data: decoded};
}

/** Position sequence is producer-owned. Header alone can advance on non-position updates.
 * There is no source-incarnation ID; this identity must not imply a global epoch. */
export function gnssObservationIdentity(raw: unknown): string | null {
    if (!isJsonObject(raw)) return null;
    const source = typeof raw.backend === 'string' ? `${raw.backend}:` : '';
    if (typeof raw.position_observation_sequence === 'number' && Number.isSafeInteger(raw.position_observation_sequence)
        && raw.position_observation_sequence > 0) return `${source}position:${raw.position_observation_sequence}`;
    const stamp = diagnosticPublicationIdentity(raw);
    return stamp === null ? null : `${source}${stamp}`;
}
