import type { GpsFixType } from "../../utils/gpsStatus.ts";
import { GnssStatusConstants } from "../../types/ros.ts";

export const GNSS_CN0_FULL_SCALE_DB_HZ = 45;

export function liveStatusTagColor(fixType: GpsFixType) {
    if (fixType === "RTK_FIX") {
        return "success";
    }
    if (fixType === "NO_FIX") {
        return "warning";
    }
    return "processing";
}

export function rtkModeTagColor(rtkMode: number | undefined) {
    if (rtkMode === GnssStatusConstants.RTK_MODE_FIXED) {
        return "success";
    }
    if (rtkMode === GnssStatusConstants.RTK_MODE_FLOAT) {
        return "processing";
    }
    if (rtkMode === GnssStatusConstants.RTK_MODE_NONE) {
        return "warning";
    }
    return undefined;
}

export function correctionStreamTagColor(status: number | undefined) {
    if (status === GnssStatusConstants.CORRECTION_STREAM_STATUS_ACTIVE) {
        return "success";
    }
    if (status === GnssStatusConstants.CORRECTION_STREAM_STATUS_WAITING) {
        return "processing";
    }
    if (
        status === GnssStatusConstants.CORRECTION_STREAM_STATUS_UNAVAILABLE ||
        status === GnssStatusConstants.CORRECTION_STREAM_STATUS_ERROR
    ) {
        return "error";
    }
    if (status === GnssStatusConstants.CORRECTION_STREAM_STATUS_IDLE) {
        return "warning";
    }
    return undefined;
}

export function clampRatio(numerator: number | undefined, denominator: number | undefined) {
    if (
        numerator === undefined ||
        denominator === undefined ||
        !Number.isFinite(numerator) ||
        !Number.isFinite(denominator) ||
        denominator <= 0
    ) {
        return undefined;
    }
    return Math.min(1, Math.max(0, numerator / denominator));
}
