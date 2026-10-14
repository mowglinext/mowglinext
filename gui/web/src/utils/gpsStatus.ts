import {GnssStatus, GnssStatusConstants} from "../types/ros.ts";
import i18n from "../i18n";

export type GpsFixType = "RTK_FIX" | "RTK_FLOAT" | "DGPS" | "3D_FIX" | "2D_FIX" | "NO_FIX" | "UNKNOWN";
export type OptionalGnssBooleanState = "unsupported" | "unknown" | "false" | "true";

export interface GpsStatus {
    fixType: GpsFixType;
    label: string;
    percent: number;
}

export type GnssRtkModeLabel = "Unknown" | "None" | "Float" | "Fixed";

const CORRECTION_STREAM_DIAGNOSTIC_NAMES = [
    "universal_gnss_ntrip/rtcm_forwarding",
    "universal_gnss/rtcm_forwarding",
];

const MSM_SUMMARY_DIAGNOSTIC_NAMES = [
    "universal_gnss_ntrip/rtcm_semantic/msm_summary",
    "universal_gnss/rtcm_semantic/msm_summary",
];

export interface DiagnosticStatusLike {
    name?: string;
    hardware_id?: string;
    message?: string;
    values?: { key?: string; value?: string }[];
}

export interface DiagnosticArrayLike {
    status?: DiagnosticStatusLike[];
}

function fromFixType(fixType: number | undefined | null): GpsStatus | null {
    switch (fixType) {
        case GnssStatusConstants.FIX_TYPE_RTK_FIXED:
            return {fixType: "RTK_FIX", label: i18n.t("gpsStatus.compactRtkFixed"), percent: 100};
        case GnssStatusConstants.FIX_TYPE_RTK_FLOAT:
            return {fixType: "RTK_FLOAT", label: i18n.t("gpsStatus.compactRtkFloat"), percent: 80};
        case GnssStatusConstants.FIX_TYPE_DGPS:
            return {fixType: "DGPS", label: i18n.t("gpsStatus.compactDgps"), percent: 60};
        case GnssStatusConstants.FIX_TYPE_3D_FIX:
            return {fixType: "3D_FIX", label: i18n.t("gpsStatus.compact3d"), percent: 40};
        case GnssStatusConstants.FIX_TYPE_2D_FIX:
            return {fixType: "2D_FIX", label: i18n.t("gpsStatus.compact2d"), percent: 20};
        case GnssStatusConstants.FIX_TYPE_GPS_FIX:
            return {fixType: "UNKNOWN", label: i18n.t("gpsStatus.compactUnknown"), percent: 0};
        case GnssStatusConstants.FIX_TYPE_DEAD_RECKONING:
            return {fixType: "UNKNOWN", label: i18n.t("gpsStatus.compactUnknown"), percent: 0};
        case GnssStatusConstants.FIX_TYPE_NO_FIX:
            return {fixType: "NO_FIX", label: i18n.t("gpsStatus.compactUnknown"), percent: 0};
        default:
            return null;
    }
}

function unknownGpsStatus(): GpsStatus {
    return {fixType: "UNKNOWN", label: i18n.t("gpsStatus.compactUnknown"), percent: 0};
}

function optionalValueAvailable(status: GnssStatus | undefined | null, flag: number): boolean {
    return status != null && (status.capability_flags === undefined || hasGnssCapability(status, flag))
        && (status.value_flags === undefined || hasGnssValue(status, flag));
}

export function currentCorrectionStreamStatus(status: GnssStatus | undefined | null): number | undefined {
    return optionalValueAvailable(status, GnssStatusConstants.CAP_CORRECTION_STREAM)
        ? status?.correction_stream_status : undefined;
}

export function currentRtkMode(status: GnssStatus | undefined | null): number | undefined {
    if (!status) return undefined;
    if ((status.capability_flags !== undefined && !hasGnssCapability(status, GnssStatusConstants.CAP_RTK_MODE)) ||
        (status.value_flags !== undefined && !hasGnssValue(status, GnssStatusConstants.CAP_RTK_MODE))) return undefined;
    return status.rtk_mode;
}

// Source of truth: GnssStatus from /gps/status.
export function deriveGpsStatus(gnssStatus: GnssStatus | undefined | null): GpsStatus {
    if (gnssStatus?.fix_valid === false) {
        return {fixType: "NO_FIX", label: i18n.t("gpsStatus.compactUnknown"), percent: 0};
    }

    if (gnssStatus?.dead_reckoning === true) return unknownGpsStatus();
    if (gnssStatus?.fix_type !== undefined && fromFixType(gnssStatus.fix_type) === null) return unknownGpsStatus();
    if (gnssStatus?.fix_type === GnssStatusConstants.FIX_TYPE_NO_FIX) {
        return {fixType: "NO_FIX", label: i18n.t("gpsStatus.compactUnknown"), percent: 0};
    }
    if (gnssStatus?.fix_type === GnssStatusConstants.FIX_TYPE_DEAD_RECKONING) return unknownGpsStatus();
    if (gnssStatus?.fix_type === undefined && gnssStatus?.fix_valid !== true) return unknownGpsStatus();
    if (gnssStatus?.rtk_mode !== undefined && (!Number.isInteger(gnssStatus.rtk_mode)
        || gnssStatus.rtk_mode < 0 || gnssStatus.rtk_mode > Number(GnssStatusConstants.RTK_MODE_FIXED))) return unknownGpsStatus();
    const rtkMode = currentRtkMode(gnssStatus);
    switch (rtkMode) {
        case GnssStatusConstants.RTK_MODE_FIXED:
            return {fixType: "RTK_FIX", label: i18n.t("gpsStatus.compactRtkFixed"), percent: 100};
        case GnssStatusConstants.RTK_MODE_FLOAT:
            return {fixType: "RTK_FLOAT", label: i18n.t("gpsStatus.compactRtkFloat"), percent: 80};
        case GnssStatusConstants.RTK_MODE_NONE:
            if (gnssStatus?.fix_type === GnssStatusConstants.FIX_TYPE_RTK_FIXED ||
                gnssStatus?.fix_type === GnssStatusConstants.FIX_TYPE_RTK_FLOAT) {
                return {fixType: "UNKNOWN", label: i18n.t("gpsStatus.compactUnknown"), percent: 0};
            }
            break;
        case GnssStatusConstants.RTK_MODE_UNKNOWN:
        default:
            break;
    }

    const fromTypedStatus = fromFixType(gnssStatus?.fix_type);
    if (fromTypedStatus) {
        return fromTypedStatus;
    }

    return {fixType: "UNKNOWN", label: i18n.t("gpsStatus.compactUnknown"), percent: 0};
}

export function gnssRtkModeLabel(gnssStatus: GnssStatus | undefined | null): GnssRtkModeLabel | undefined {
    switch (currentRtkMode(gnssStatus)) {
        case GnssStatusConstants.RTK_MODE_NONE:
            return "None";
        case GnssStatusConstants.RTK_MODE_FLOAT:
            return "Float";
        case GnssStatusConstants.RTK_MODE_FIXED:
            return "Fixed";
        case GnssStatusConstants.RTK_MODE_UNKNOWN:
            return "Unknown";
        default:
            return undefined;
    }
}

export function gnssBaselineSolutionStatusLabel(gnssStatus: GnssStatus | undefined | null): string | undefined {
    if (!optionalValueAvailable(gnssStatus, GnssStatusConstants.CAP_BASELINE_SOLUTION_STATUS)) return undefined;
    switch (gnssStatus?.baseline_solution_status) {
        case GnssStatusConstants.BASELINE_STATUS_UNKNOWN:
            return i18n.t("gpsStatus.baselineUnknown");
        case GnssStatusConstants.BASELINE_STATUS_COMPUTED:
            return i18n.t("gpsStatus.baselineComputed");
        case GnssStatusConstants.BASELINE_STATUS_NOT_SOLVED:
            return i18n.t("gpsStatus.baselineNotSolved");
        case GnssStatusConstants.BASELINE_STATUS_INSUFFICIENT_OBSERVATIONS:
            return i18n.t("gpsStatus.baselineInsufficientObservations");
        case GnssStatusConstants.BASELINE_STATUS_NO_CONVERGENCE:
            return i18n.t("gpsStatus.baselineNoConvergence");
        case GnssStatusConstants.BASELINE_STATUS_OUT_OF_TOLERANCE:
            return i18n.t("gpsStatus.baselineOutOfTolerance");
        case GnssStatusConstants.BASELINE_STATUS_COVARIANCE_TRACE_EXCEEDED:
            return i18n.t("gpsStatus.baselineCovarianceTraceExceeded");
        case GnssStatusConstants.BASELINE_STATUS_NOT_CONFIGURED:
            return i18n.t("gpsStatus.baselineNotConfigured");
        default:
            return undefined;
    }
}

export function gnssCorrectionStreamStatusLabel(gnssStatus: GnssStatus | undefined | null): string | undefined {
    switch (currentCorrectionStreamStatus(gnssStatus)) {
        case GnssStatusConstants.CORRECTION_STREAM_STATUS_UNKNOWN:
            return i18n.t("gpsStatus.correctionStreamUnknown");
        case GnssStatusConstants.CORRECTION_STREAM_STATUS_IDLE:
            return i18n.t("gpsStatus.correctionStreamIdle");
        case GnssStatusConstants.CORRECTION_STREAM_STATUS_WAITING:
            return i18n.t("gpsStatus.correctionStreamWaiting");
        case GnssStatusConstants.CORRECTION_STREAM_STATUS_ACTIVE:
            return i18n.t("gpsStatus.correctionStreamActive");
        case GnssStatusConstants.CORRECTION_STREAM_STATUS_UNAVAILABLE:
            return i18n.t("gpsStatus.correctionStreamUnavailable");
        case GnssStatusConstants.CORRECTION_STREAM_STATUS_ERROR:
            return i18n.t("gpsStatus.correctionStreamError");
        default:
            return undefined;
    }
}

export function hasTypedGnssStatusSample(gnssStatus: GnssStatus | undefined | null): boolean {
    return gnssStatus != null && Object.keys(gnssStatus).some(key => key !== 'header');
}

export function diagnosticsValueMap(entry: DiagnosticStatusLike | undefined): Record<string, string> {
    const out: Record<string, string> = {};
    for (const item of entry?.values ?? []) {
        const key = item.key?.trim();
        if (!key) {
            continue;
        }
        out[key] = item.value?.trim() ?? "";
    }
    return out;
}

export function parseDiagnosticBool(value: string | undefined): boolean | undefined {
    if (!value) {
        return undefined;
    }
    const normalized = value.trim().toLowerCase();
    if (normalized === "true") {
        return true;
    }
    if (normalized === "false") {
        return false;
    }
    return undefined;
}

function parseDiagnosticInt(value: string | undefined): number | undefined {
    if (!value) {
        return undefined;
    }
    if (!/^-?\d+$/.test(value.trim())) return undefined;
    const parsed = Number(value);
    return Number.isSafeInteger(parsed) ? parsed : undefined;
}

function parseDiagnosticUint16(value: string | undefined): number | undefined {
    const parsed = parseDiagnosticInt(value);
    return parsed !== undefined && parsed >= 0 && parsed <= 65535 ? parsed : undefined;
}

function parseDiagnosticFloat(value: string | undefined): number | undefined {
    if (!value) {
        return undefined;
    }
    if (!/^[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?$/.test(value.trim())) return undefined;
    const parsed = Number(value);
    return Number.isFinite(parsed) && parsed >= 0 ? parsed : undefined;
}

function correctionStreamStatusFromMessage(message: string | undefined): number | undefined {
    const normalized = message?.trim().toLowerCase() ?? "";
    if (!normalized) {
        return undefined;
    }
    if (normalized.includes("write error") || normalized.endsWith("error")) {
        return GnssStatusConstants.CORRECTION_STREAM_STATUS_ERROR;
    }
    if (normalized.includes("unavailable")) {
        return GnssStatusConstants.CORRECTION_STREAM_STATUS_UNAVAILABLE;
    }
    if (/^rtcm forwarding waiting(?: for .*)?$/.test(normalized)) {
        return GnssStatusConstants.CORRECTION_STREAM_STATUS_WAITING;
    }
    if (/^rtcm forwarding active$/.test(normalized)) {
        return GnssStatusConstants.CORRECTION_STREAM_STATUS_ACTIVE;
    }
    if (/^rtcm forwarding (?:idle|stale)$/.test(normalized)) {
        return GnssStatusConstants.CORRECTION_STREAM_STATUS_IDLE;
    }
    return undefined;
}

/**
 * Fix-type codes arrive over the wire as plain numbers, while
 * `GnssStatusConstants` is a const enum — comparing the two directly is
 * (correctly) rejected, since nothing proves the number is a member. Funnel
 * every such comparison through this one documented widening.
 */
export function isGnssFixType(
    value: number | undefined,
    fixType: GnssStatusConstants,
): boolean {
    return value === (fixType as number);
}

function navSatFixStatusToGnssFixType(fixStatus: number | undefined): number | undefined {
    switch (fixStatus) {
        case -1:
            return GnssStatusConstants.FIX_TYPE_NO_FIX;
        case 2:
        case 1:
        case 0:
            return GnssStatusConstants.FIX_TYPE_GPS_FIX;
        default:
            return undefined;
    }
}

export function findDiagnosticStatusByName(
    diagnostics: DiagnosticArrayLike | undefined | null,
    name: string,
): DiagnosticStatusLike | undefined {
    return (diagnostics?.status ?? []).find((entry) => entry.name === name);
}

function findDiagnosticStatusByNames(
    diagnostics: DiagnosticArrayLike | undefined | null,
    names: string[],
): DiagnosticStatusLike | undefined {
    for (const name of names) {
        const match = findDiagnosticStatusByName(diagnostics, name);
        if (match) {
            return match;
        }
    }
    return undefined;
}

export function deriveGnssStatusFromDiagnostics(
    diagnostics: DiagnosticArrayLike | undefined | null,
): GnssStatus | undefined {
    const summary = findDiagnosticStatusByName(diagnostics, "universal_gnss/summary");
    const gps = findDiagnosticStatusByName(diagnostics, "GPS");
    const correctionStream = findDiagnosticStatusByNames(diagnostics, CORRECTION_STREAM_DIAGNOSTIC_NAMES);
    const msmSummary = findDiagnosticStatusByNames(diagnostics, MSM_SUMMARY_DIAGNOSTIC_NAMES);
    if (!summary && !gps && !correctionStream && !msmSummary) {
        return undefined;
    }

    const summaryValues = diagnosticsValueMap(summary);
    const gpsValues = diagnosticsValueMap(gps);
    const correctionStreamStatus = correctionStreamStatusFromMessage(correctionStream?.message);
    const msmSummaryValues = diagnosticsValueMap(msmSummary);
    const msmSummarySeen = parseDiagnosticBool(msmSummaryValues.seen);
    const msmSummaryDecoded = parseDiagnosticBool(msmSummaryValues.decoded);
    const msmSummaryValid = parseDiagnosticBool(msmSummaryValues.valid);
    const msmSummaryMessageType = parseDiagnosticUint16(msmSummaryValues.message_type);
    const msmSummaryStationId = parseDiagnosticUint16(msmSummaryValues.station_id);
    const msmSummarySatelliteCount = parseDiagnosticUint16(msmSummaryValues.satellite_count);
    const msmSummarySignalCount = parseDiagnosticUint16(msmSummaryValues.signal_count);
    const msmSummaryCellCount = parseDiagnosticUint16(msmSummaryValues.cell_count);
    const msmSummaryAgeS = parseDiagnosticFloat(msmSummaryValues.age_s);
    const msmSummaryConstellationsSeen = msmSummaryValues.constellations_seen;
    const fixType = navSatFixStatusToGnssFixType(parseDiagnosticInt(gpsValues.fix_status));
    const summaryFixValid = parseDiagnosticBool(summaryValues.fix_valid);
    const fixValid = summaryFixValid === false || fixType === GnssStatusConstants.FIX_TYPE_NO_FIX
        ? false : summaryFixValid ?? (fixType !== undefined ? true : undefined);
    const capabilityFlags =
        (correctionStream ? GnssStatusConstants.CAP_CORRECTION_STREAM : 0) |
        (msmSummary ? GnssStatusConstants.CAP_MSM_SUMMARY : 0);
    const valueFlags =
        (correctionStreamStatus !== undefined ? GnssStatusConstants.CAP_CORRECTION_STREAM : 0) |
        ((msmSummarySeen !== undefined && msmSummaryDecoded !== undefined && msmSummaryValid !== undefined)
            ? GnssStatusConstants.CAP_MSM_SUMMARY
            : 0);

    const projected: GnssStatus = {
        backend: summary ? "universal" : undefined,
        ...(fixType !== undefined ? {fix_type: fixType} : {}),
        ...(fixValid !== undefined ? {fix_valid: fixValid} : {}),
    };
    if (capabilityFlags) {
        projected.capability_flags = capabilityFlags;
    }
    if (valueFlags) {
        projected.value_flags = valueFlags;
    }
    if (correctionStream) {
        projected.correction_stream_status =
            correctionStreamStatus ?? GnssStatusConstants.CORRECTION_STREAM_STATUS_UNKNOWN;
    }
    if (msmSummary) {
        if (msmSummarySeen !== undefined) projected.msm_summary_seen = msmSummarySeen;
        if (msmSummaryDecoded !== undefined) projected.msm_summary_decoded = msmSummaryDecoded;
        if (msmSummaryValid !== undefined) projected.msm_summary_valid = msmSummaryValid;
        if (msmSummaryMessageType !== undefined) projected.msm_summary_message_type = msmSummaryMessageType;
        if (msmSummaryStationId !== undefined) projected.msm_summary_station_id = msmSummaryStationId;
        projected.msm_summary_constellations_seen = msmSummaryConstellationsSeen;
        if (msmSummarySatelliteCount !== undefined) projected.msm_summary_satellite_count = msmSummarySatelliteCount;
        if (msmSummarySignalCount !== undefined) projected.msm_summary_signal_count = msmSummarySignalCount;
        if (msmSummaryCellCount !== undefined) projected.msm_summary_cell_count = msmSummaryCellCount;
        if (msmSummaryAgeS !== undefined) projected.msm_summary_age_s = msmSummaryAgeS;
    }
    return projected;
}

const CAPABILITY_GROUP_FIELDS: Array<{flag: number; fields: (keyof GnssStatus)[]}> = [
    {
        flag: GnssStatusConstants.CAP_CORRECTION_STREAM,
        fields: ["correction_stream_status"],
    },
    {
        flag: GnssStatusConstants.CAP_MSM_SUMMARY,
        fields: [
            "msm_summary_seen",
            "msm_summary_decoded",
            "msm_summary_valid",
            "msm_summary_message_type",
            "msm_summary_station_id",
            "msm_summary_constellations_seen",
            "msm_summary_satellite_count",
            "msm_summary_signal_count",
            "msm_summary_cell_count",
            "msm_summary_age_s",
        ],
    },
];

function copyDiagnosticField<K extends keyof GnssStatus>(
    target: GnssStatus, source: GnssStatus, field: K,
): void {
    if (source[field] !== undefined) target[field] = source[field];
}

export function mergeGnssStatusDiagnosticProjection(
    gnssStatus: GnssStatus | undefined | null,
    diagnosticProjection: GnssStatus | undefined,
): GnssStatus {
    const base = gnssStatus ?? {};
    if (!diagnosticProjection) {
        return base;
    }
    if (!hasTypedGnssStatusSample(base)) {
        return diagnosticProjection;
    }

    const merged: GnssStatus = {...base};
    const coreFields = new Set(['header', 'backend', 'receiver_vendor', 'receiver_model', 'receiver_firmware',
        'fix_type', 'fix_valid', 'dead_reckoning']);
    // A single generated mask cannot express partial authority. Do not add a
    // diagnostic-only mask that would make legacy typed RTK/channels unsupported.
    if (Object.keys(base).some(field => !coreFields.has(field))) return merged;
    // Typed positioning and identity never come from an unrelated diagnostic.
    // Explicit masks define unsupported/unknown as well as available fields.
    if (base.capability_flags !== undefined || base.value_flags !== undefined) return merged;
    for (const group of CAPABILITY_GROUP_FIELDS) {
        if (group.fields.some(field => base[field] !== undefined) ||
            !hasGnssCapability(diagnosticProjection, group.flag)) continue;
        for (const field of group.fields) copyDiagnosticField(merged, diagnosticProjection, field);
        merged.capability_flags = (merged.capability_flags ?? 0) | group.flag;
        if (hasGnssValue(diagnosticProjection, group.flag)) {
            merged.value_flags = (merged.value_flags ?? 0) | group.flag;
        }
    }

    return merged;
}

export function hasGnssCapability(gnssStatus: GnssStatus | undefined | null, flag: number): boolean {
    return ((gnssStatus?.capability_flags ?? 0) & flag) !== 0;
}

export function hasGnssValue(gnssStatus: GnssStatus | undefined | null, flag: number): boolean {
    return ((gnssStatus?.value_flags ?? 0) & flag) !== 0;
}

export function readGnssNumber(
    gnssStatus: GnssStatus | undefined | null,
    flag: number,
    value: number | undefined | null,
): number | undefined {
    return hasGnssCapability(gnssStatus, flag) && hasGnssValue(gnssStatus, flag)
        && typeof value === 'number' && Number.isFinite(value) ? value : undefined;
}

export function displayHorizontalAccuracyM(
    gnssStatus: GnssStatus | undefined | null,
): number | undefined {
    return readGnssNumber(
        gnssStatus,
        GnssStatusConstants.CAP_HORIZONTAL_ACCURACY,
        gnssStatus?.horizontal_accuracy_m,
    );
}

export function readGnssBooleanState(
    gnssStatus: GnssStatus | undefined | null,
    flag: number,
    value: boolean | undefined | null,
): OptionalGnssBooleanState {
    if (!hasGnssCapability(gnssStatus, flag)) {
        return "unsupported";
    }
    if (!hasGnssValue(gnssStatus, flag) || value == null) {
        return "unknown";
    }
    return value ? "true" : "false";
}

export function gnssReceiverLabel(gnssStatus: GnssStatus | undefined | null): string {
    const vendor = gnssStatus?.receiver_vendor?.trim() ?? "";
    const model = gnssStatus?.receiver_model?.trim() ?? "";

    if (vendor && model) {
        return `${vendor} ${model}`;
    }
    if (model) {
        return model;
    }
    if (vendor) {
        return vendor;
    }

    return "GNSS";
}
