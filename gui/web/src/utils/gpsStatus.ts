import {AbsolutePoseConstants} from "../types/ros.ts";

export type GpsFixType = "RTK_FIX" | "RTK_FLOAT" | "GPS_FIX" | "NO_FIX";

export interface GpsStatus {
    fixType: GpsFixType;
    label: string;
    percent: number;
}

// Source of truth: AbsolutePose flags from /gps/absolute_pose. The bitfield
// distinguishes RTK FIX (cm-level) from RTK FLOAT (sub-metre) from a plain
// GPS fix. highLevelStatus.gps_quality_percent has historically been
// unreliable (issue #154 — wrong state in Health Check / GPS tile), so we
// derive everything from the flags here and ignore the percent fallback.
export function deriveGpsStatus(flags: number | undefined | null): GpsStatus {
    const f = flags ?? 0;
    if (f & AbsolutePoseConstants.FLAG_GPS_RTK_FIXED) {
        return {fixType: "RTK_FIX", label: "RTK fixed", percent: 100};
    }
    if (f & AbsolutePoseConstants.FLAG_GPS_RTK_FLOAT) {
        return {fixType: "RTK_FLOAT", label: "RTK float", percent: 50};
    }
    if (f & AbsolutePoseConstants.FLAG_GPS_RTK) {
        return {fixType: "GPS_FIX", label: "GPS fix", percent: 25};
    }
    return {fixType: "NO_FIX", label: "No GPS", percent: 0};
}
