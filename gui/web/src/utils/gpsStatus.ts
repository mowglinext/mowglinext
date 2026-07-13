import {GnssStatus, GnssStatusConstants} from "../types/ros.ts";
import i18n from "../i18n";

export type GpsFixType = "RTK_FIX" | "RTK_FLOAT" | "GPS_FIX" | "NO_FIX";
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
            return {fixType: "RTK_FIX", label: i18n.t("gpsStatus.rtkFixed"), percent: 100};
        case GnssStatusConstants.FIX_TYPE_RTK_FLOAT:
            return {fixType: "RTK_FLOAT", label: i18n.t("gpsStatus.rtkFloat"), percent: 50};
        case GnssStatusConstants.FIX_TYPE_GPS_FIX:
            return {fixType: "GPS_FIX", label: i18n.t("gpsStatus.gpsFix"), percent: 25};
        case GnssStatusConstants.FIX_TYPE_DEAD_RECKONING:
            return {fixType: "NO_FIX", label: i18n.t("gpsStatus.deadReckoning"), percent: 10};
        case GnssStatusConstants.FIX_TYPE_NO_FIX:
            return {fixType: "NO_FIX", label: i18n.t("gpsStatus.noGps"), percent: 0};
        default:
            return null;
    }
}

// Source of truth: GnssStatus from /gps/status.
export function deriveGpsStatus(gnssStatus: GnssStatus | undefined | null): GpsStatus {
    if (gnssStatus?.fix_valid === false) {
        return {fixType: "NO_FIX", label: i18n.t("gpsStatus.noGps"), percent: 0};
    }

    if (gnssStatus?.rtk_mode === GnssStatusConstants.RTK_MODE_FIXED) {
        return {fixType: "RTK_FIX", label: i18n.t("gpsStatus.rtkFixed"), percent: 100};
    }

    if (gnssStatus?.rtk_mode === GnssStatusConstants.RTK_MODE_FLOAT) {
        return {fixType: "RTK_FLOAT", label: i18n.t("gpsStatus.rtkFloat"), percent: 50};
    }

    const fromTypedStatus = fromFixType(gnssStatus?.fix_type);
    if (fromTypedStatus && (fromTypedStatus.fixType !== "NO_FIX" || gnssStatus?.fix_valid !== true)) {
        return fromTypedStatus;
    }

    if (gnssStatus?.fix_valid === true) {
        return {fixType: "GPS_FIX", label: i18n.t("gpsStatus.gpsFix"), percent: 25};
    }

    return {fixType: "NO_FIX", label: i18n.t("gpsStatus.noGps"), percent: 0};
}

export function gnssRtkModeLabel(gnssStatus: GnssStatus | undefined | null): GnssRtkModeLabel | undefined {
    switch (gnssStatus?.rtk_mode) {
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
    switch (gnssStatus?.correction_stream_status) {
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
    return gnssStatus?.fix_type !== undefined ||
        gnssStatus?.fix_valid !== undefined ||
        (gnssStatus?.backend?.trim().length ?? 0) > 0 ||
        (gnssStatus?.receiver_vendor?.trim().length ?? 0) > 0 ||
        (gnssStatus?.receiver_model?.trim().length ?? 0) > 0;
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
    const parsed = Number.parseInt(value, 10);
    return Number.isFinite(parsed) ? parsed : undefined;
}

function parseDiagnosticFloat(value: string | undefined): number | undefined {
    if (!value) {
        return undefined;
    }
    const parsed = Number.parseFloat(value);
    return Number.isFinite(parsed) ? parsed : undefined;
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
    if (normalized.includes("waiting")) {
        return GnssStatusConstants.CORRECTION_STREAM_STATUS_WAITING;
    }
    if (normalized.includes("active")) {
        return GnssStatusConstants.CORRECTION_STREAM_STATUS_ACTIVE;
    }
    if (normalized.includes("idle")) {
        return GnssStatusConstants.CORRECTION_STREAM_STATUS_IDLE;
    }
    return undefined;
}

function navSatFixStatusToGnssFixType(fixStatus: number | undefined): number | undefined {
    switch (fixStatus) {
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
    const msmSummaryMessageType = parseDiagnosticInt(msmSummaryValues.message_type);
    const msmSummaryStationId = parseDiagnosticInt(msmSummaryValues.station_id);
    const msmSummarySatelliteCount = parseDiagnosticInt(msmSummaryValues.satellite_count);
    const msmSummarySignalCount = parseDiagnosticInt(msmSummaryValues.signal_count);
    const msmSummaryCellCount = parseDiagnosticInt(msmSummaryValues.cell_count);
    const msmSummaryAgeS = parseDiagnosticFloat(msmSummaryValues.age_s);
    const msmSummaryConstellationsSeen = msmSummaryValues.constellations_seen;
    const fixType = navSatFixStatusToGnssFixType(parseDiagnosticInt(gpsValues.fix_status));
    const fixValid = parseDiagnosticBool(summaryValues.fix_valid) ??
        (fixType !== undefined ? fixType !== GnssStatusConstants.FIX_TYPE_NO_FIX : undefined);
    const capabilityFlags =
        (correctionStream ? GnssStatusConstants.CAP_CORRECTION_STREAM : 0) |
        (msmSummary ? GnssStatusConstants.CAP_MSM_SUMMARY : 0);
    const valueFlags =
        (correctionStreamStatus !== undefined ? GnssStatusConstants.CAP_CORRECTION_STREAM : 0) |
        ((msmSummarySeen !== undefined ||
            msmSummaryDecoded !== undefined ||
            msmSummaryValid !== undefined ||
            msmSummaryMessageType !== undefined ||
            msmSummaryStationId !== undefined ||
            msmSummaryConstellationsSeen !== undefined ||
            msmSummarySatelliteCount !== undefined ||
            msmSummarySignalCount !== undefined ||
            msmSummaryCellCount !== undefined ||
            msmSummaryAgeS !== undefined)
            ? GnssStatusConstants.CAP_MSM_SUMMARY
            : 0);

    const projected: GnssStatus = {
        backend: summary ? "universal" : undefined,
        fix_type: fixType ?? GnssStatusConstants.FIX_TYPE_NO_FIX,
        fix_valid: fixValid ?? false,
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
        projected.msm_summary_seen = msmSummarySeen ?? false;
        projected.msm_summary_decoded = msmSummaryDecoded ?? false;
        projected.msm_summary_valid = msmSummaryValid ?? false;
        projected.msm_summary_message_type = msmSummaryMessageType ?? 0;
        projected.msm_summary_station_id = msmSummaryStationId ?? 0;
        projected.msm_summary_constellations_seen = msmSummaryConstellationsSeen;
        projected.msm_summary_satellite_count = msmSummarySatelliteCount ?? 0;
        projected.msm_summary_signal_count = msmSummarySignalCount ?? 0;
        projected.msm_summary_cell_count = msmSummaryCellCount ?? 0;
        projected.msm_summary_age_s = msmSummaryAgeS ?? 0;
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

    const merged: GnssStatus = {
        ...diagnosticProjection,
        ...base,
        capability_flags: (base.capability_flags ?? 0) | (diagnosticProjection.capability_flags ?? 0),
        value_flags: (base.value_flags ?? 0) | (diagnosticProjection.value_flags ?? 0),
    };

    for (const group of CAPABILITY_GROUP_FIELDS) {
        if (((base.capability_flags ?? 0) & group.flag) !== 0 ||
            ((diagnosticProjection.capability_flags ?? 0) & group.flag) === 0) {
            continue;
        }
        for (const field of group.fields) {
            (merged as Record<string, unknown>)[field] = diagnosticProjection[field];
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
    return hasGnssValue(gnssStatus, flag) && value != null ? value : undefined;
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
