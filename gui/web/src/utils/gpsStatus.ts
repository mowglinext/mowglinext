import {GnssStatus, GnssStatusConstants} from "../types/ros.ts";

export type GpsFixType = "RTK_FIX" | "RTK_FLOAT" | "GPS_FIX" | "NO_FIX";
export type OptionalGnssBooleanState = "unsupported" | "unknown" | "false" | "true";

export interface GpsStatus {
    fixType: GpsFixType;
    label: string;
    percent: number;
}

function fromFixType(fixType: number | undefined | null): GpsStatus | null {
    switch (fixType) {
        case GnssStatusConstants.FIX_TYPE_RTK_FIXED:
            return {fixType: "RTK_FIX", label: "RTK fixed", percent: 100};
        case GnssStatusConstants.FIX_TYPE_RTK_FLOAT:
            return {fixType: "RTK_FLOAT", label: "RTK float", percent: 50};
        case GnssStatusConstants.FIX_TYPE_GPS_FIX:
            return {fixType: "GPS_FIX", label: "GPS fix", percent: 25};
        case GnssStatusConstants.FIX_TYPE_DEAD_RECKONING:
            return {fixType: "NO_FIX", label: "Dead reckoning", percent: 10};
        case GnssStatusConstants.FIX_TYPE_NO_FIX:
            return {fixType: "NO_FIX", label: "No GPS", percent: 0};
        default:
            return null;
    }
}

// Source of truth: GnssStatus from /gps/status.
export function deriveGpsStatus(gnssStatus: GnssStatus | undefined | null): GpsStatus {
    const fromTypedStatus = fromFixType(gnssStatus?.fix_type);
    if (fromTypedStatus) {
        return fromTypedStatus;
    }
    return {fixType: "NO_FIX", label: "No GPS", percent: 0};
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
        return `${vendor} GNSS`;
    }

    return "GNSS";
}
