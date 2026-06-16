import {GnssStatus, GnssStatusConstants} from "../types/ros.ts";

export type GpsFixType = "RTK_FIX" | "RTK_FLOAT" | "GPS_FIX" | "NO_FIX";
export type OptionalGnssBooleanState = "unsupported" | "unknown" | "false" | "true";

export interface GpsStatus {
    fixType: GpsFixType;
    label: string;
    percent: number;
}

export type GnssRtkModeLabel = "Unknown" | "None" | "Float" | "Fixed";

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
            return {fixType: "RTK_FIX", label: "RTK fixe", percent: 100};
        case GnssStatusConstants.FIX_TYPE_RTK_FLOAT:
            return {fixType: "RTK_FLOAT", label: "RTK flottant", percent: 50};
        case GnssStatusConstants.FIX_TYPE_GPS_FIX:
            return {fixType: "GPS_FIX", label: "GPS simple", percent: 25};
        case GnssStatusConstants.FIX_TYPE_DEAD_RECKONING:
            return {fixType: "NO_FIX", label: "Estime", percent: 10};
        case GnssStatusConstants.FIX_TYPE_NO_FIX:
            return {fixType: "NO_FIX", label: "Pas de GPS", percent: 0};
        default:
            return null;
    }
}

// Source of truth: GnssStatus from /gps/status.
export function deriveGpsStatus(gnssStatus: GnssStatus | undefined | null): GpsStatus {
    if (gnssStatus?.fix_valid === false) {
        return {fixType: "NO_FIX", label: "Pas de GPS", percent: 0};
    }

    if (gnssStatus?.rtk_mode === GnssStatusConstants.RTK_MODE_FIXED) {
        return {fixType: "RTK_FIX", label: "RTK fixe", percent: 100};
    }

    if (gnssStatus?.rtk_mode === GnssStatusConstants.RTK_MODE_FLOAT) {
        return {fixType: "RTK_FLOAT", label: "RTK flottant", percent: 50};
    }

    const fromTypedStatus = fromFixType(gnssStatus?.fix_type);
    if (fromTypedStatus && (fromTypedStatus.fixType !== "NO_FIX" || gnssStatus?.fix_valid !== true)) {
        return fromTypedStatus;
    }

    if (gnssStatus?.fix_valid === true) {
        return {fixType: "GPS_FIX", label: "GPS simple", percent: 25};
    }

    return {fixType: "NO_FIX", label: "Pas de GPS", percent: 0};
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

export function deriveGnssStatusFromDiagnostics(
    diagnostics: DiagnosticArrayLike | undefined | null,
): GnssStatus | undefined {
    const summary = findDiagnosticStatusByName(diagnostics, "universal_gnss/summary");
    const gps = findDiagnosticStatusByName(diagnostics, "GPS");
    if (!summary && !gps) {
        return undefined;
    }

    const summaryValues = diagnosticsValueMap(summary);
    const gpsValues = diagnosticsValueMap(gps);
    const fixType = navSatFixStatusToGnssFixType(parseDiagnosticInt(gpsValues.fix_status));
    const fixValid = parseDiagnosticBool(summaryValues.fix_valid) ??
        (fixType !== undefined ? fixType !== GnssStatusConstants.FIX_TYPE_NO_FIX : undefined);

    return {
        backend: summary ? "universal" : undefined,
        fix_type: fixType ?? GnssStatusConstants.FIX_TYPE_NO_FIX,
        fix_valid: fixValid ?? false,
    };
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
