import {GnssStatus, GnssStatusConstants} from "../../types/ros.ts";
import {deriveGpsStatus, gnssCorrectionStreamStatusLabel} from "../../utils/gpsStatus.ts";
import type {FusionGraphStats} from "../../hooks/useFusionGraphDiagnostics.ts";
import type {CalibrationStatus} from "../../hooks/useCalibrationStatus.ts";

// ── Thresholds (named so they don't read as magic numbers) ──────────────
/** A fusion snapshot older than this (vs. now) reads as "still starting". */
export const FUSION_FRESHNESS_MS = 8000;
/** Position σ below this (m, per axis) counts as a confident localizer. */
export const LOCALIZER_CONFIDENT_SIGMA_M = 0.1;
/** |imu_yaw| above this (rad) means the mounting yaw has been solved. */
export const IMU_YAW_SOLVED_EPS_RAD = 1e-4;
/** A polygon needs at least this many points to be a real mowing area. */
const MIN_AREA_POINTS = 3;

export type ReadinessState = "pass" | "pending" | "fail";

/** Where a check's remediation CTA sends the user. Resolved to an action by
 *  the ReadinessStep (in-wizard jump vs. route navigation). */
export type ReadinessCtaTarget =
    | "gps"
    | "ntrip"
    | "datum"
    | "diagnostics"
    | "calibration"
    | "firmware"
    | "map";

export interface ReadinessCheck {
    id: string;
    required: boolean;
    state: ReadinessState;
    /** i18n key for the check's name. */
    labelKey: string;
    /** Already-localized live value (e.g. "RTK Float", "±8 cm"), if any. */
    valueText?: string;
    /** i18n key for the remediation CTA label, present only when actionable. */
    ctaKey?: string;
    ctaTarget?: ReadinessCtaTarget;
}

/** A working-area polygon, shaped like the /map working_area entries. */
export interface WorkingAreaLike {
    area?: {points?: Array<{x?: number; y?: number}>};
}

export interface ReadinessSnapshot {
    gnss: GnssStatus | null | undefined;
    fusion: FusionGraphStats | null;
    calibration: CalibrationStatus | null;
    firmwareCompatible: boolean | null;
    /** The wizard's live settings (datum_lat/lon, imu_yaw, use_magnetometer). */
    values: Record<string, unknown>;
    workingArea: WorkingAreaLike[] | undefined;
    /** Date.now() at render — passed in so the helper stays pure/testable. */
    nowMs: number;
}

function isNonZeroFinite(value: unknown): boolean {
    return typeof value === "number" && Number.isFinite(value) && value !== 0;
}

function parseIntField(value: string | undefined): number | undefined {
    if (value === undefined) return undefined;
    const parsed = Number.parseInt(value, 10);
    return Number.isFinite(parsed) ? parsed : undefined;
}

function parseFloatField(value: string | undefined): number | undefined {
    if (value === undefined) return undefined;
    const parsed = Number.parseFloat(value);
    return Number.isFinite(parsed) ? parsed : undefined;
}

function countValidAreas(workingArea: WorkingAreaLike[] | undefined): number {
    if (!workingArea) return 0;
    return workingArea.filter((a) => (a.area?.points?.length ?? 0) >= MIN_AREA_POINTS).length;
}

// ── Per-check evaluators (each pure, one snapshot slice → tri-state) ─────

function rtkCheck(snap: ReadinessSnapshot): ReadinessCheck {
    const gps = deriveGpsStatus(snap.gnss);
    const state: ReadinessState =
        gps.fixType === "RTK_FIX" ? "pass" : gps.fixType === "NO_FIX" ? "fail" : "pending";
    return {
        id: "rtk",
        required: true,
        state,
        labelKey: "onboardingPage.readinessCheckRtk",
        valueText: gps.label,
        ctaKey: state === "pass" ? undefined : "onboardingPage.readinessCtaFixGps",
        ctaTarget: "gps",
    };
}

function correctionsCheck(snap: ReadinessSnapshot): ReadinessCheck {
    const status = snap.gnss?.correction_stream_status;
    let state: ReadinessState;
    if (status === GnssStatusConstants.CORRECTION_STREAM_STATUS_ACTIVE) {
        state = "pass";
    } else if (
        status === GnssStatusConstants.CORRECTION_STREAM_STATUS_ERROR ||
        status === GnssStatusConstants.CORRECTION_STREAM_STATUS_UNAVAILABLE
    ) {
        state = "fail";
    } else {
        state = "pending";
    }
    return {
        id: "corrections",
        required: false,
        state,
        labelKey: "onboardingPage.readinessCheckCorrections",
        valueText: gnssCorrectionStreamStatusLabel(snap.gnss),
        ctaKey: state === "pass" ? undefined : "onboardingPage.readinessCtaFixNtrip",
        ctaTarget: "ntrip",
    };
}

function datumCheck(snap: ReadinessSnapshot): ReadinessCheck {
    const {datum_lat: lat, datum_lon: lon} = snap.values;
    let state: ReadinessState;
    if (lat === undefined || lon === undefined) {
        state = "pending";
    } else {
        state = isNonZeroFinite(lat) && isNonZeroFinite(lon) ? "pass" : "fail";
    }
    return {
        id: "datum",
        required: true,
        state,
        labelKey: "onboardingPage.readinessCheckDatum",
        ctaKey: state === "pass" ? undefined : "onboardingPage.readinessCtaSetDatum",
        ctaTarget: "datum",
    };
}

function localizerRunningCheck(snap: ReadinessSnapshot): ReadinessCheck {
    const nodes = parseIntField(snap.fusion?.values.total_nodes);
    const fresh =
        snap.fusion != null && snap.nowMs - snap.fusion.receivedAt <= FUSION_FRESHNESS_MS;
    const state: ReadinessState = nodes !== undefined && fresh ? "pass" : "pending";
    return {
        id: "localizer",
        required: true,
        state,
        labelKey: "onboardingPage.readinessCheckLocalizer",
        valueText: nodes !== undefined ? String(nodes) : undefined,
        ctaKey: state === "pass" ? undefined : "onboardingPage.readinessCtaDiagnostics",
        ctaTarget: "diagnostics",
    };
}

function localizerConfidenceCheck(snap: ReadinessSnapshot): ReadinessCheck {
    const covXx = parseFloatField(snap.fusion?.values.cov_xx);
    const covYy = parseFloatField(snap.fusion?.values.cov_yy);
    let state: ReadinessState;
    let valueText: string | undefined;
    if (covXx === undefined || covYy === undefined) {
        state = "pending";
    } else {
        const sigmaX = Math.sqrt(Math.max(covXx, 0));
        const sigmaY = Math.sqrt(Math.max(covYy, 0));
        state =
            sigmaX < LOCALIZER_CONFIDENT_SIGMA_M && sigmaY < LOCALIZER_CONFIDENT_SIGMA_M
                ? "pass"
                : "fail";
        valueText = `±${Math.round(Math.max(sigmaX, sigmaY) * 100)} cm`;
    }
    return {
        id: "localizerConfidence",
        required: false,
        state,
        labelKey: "onboardingPage.readinessCheckLocalizerConf",
        valueText,
        ctaKey: state === "pass" ? undefined : "onboardingPage.readinessCtaDiagnostics",
        ctaTarget: "diagnostics",
    };
}

function firmwareCheck(snap: ReadinessSnapshot): ReadinessCheck {
    const state: ReadinessState =
        snap.firmwareCompatible === true
            ? "pass"
            : snap.firmwareCompatible === false
                ? "fail"
                : "pending";
    return {
        id: "firmware",
        required: true,
        state,
        labelKey: "onboardingPage.readinessCheckFirmware",
        ctaKey: state === "pass" ? undefined : "onboardingPage.readinessCtaReflash",
        ctaTarget: "firmware",
    };
}

function presenceCheck(
    id: string,
    labelKey: string,
    present: boolean | undefined,
    ctaTarget: ReadinessCtaTarget,
    ctaKey: string,
): ReadinessCheck {
    const state: ReadinessState = present === undefined ? "pending" : present ? "pass" : "fail";
    return {
        id,
        required: true,
        state,
        labelKey,
        ctaKey: state === "pass" ? undefined : ctaKey,
        ctaTarget,
    };
}

function imuYawCheck(snap: ReadinessSnapshot): ReadinessCheck {
    const yaw = snap.values.imu_yaw;
    let state: ReadinessState;
    if (yaw === undefined) {
        state = "pending";
    } else {
        state =
            typeof yaw === "number" && Math.abs(yaw) > IMU_YAW_SOLVED_EPS_RAD ? "pass" : "fail";
    }
    return {
        id: "imuYaw",
        required: true,
        state,
        labelKey: "onboardingPage.readinessCheckImuYaw",
        ctaKey: state === "pass" ? undefined : "onboardingPage.readinessCtaCalibrate",
        ctaTarget: "calibration",
    };
}

function magCheck(snap: ReadinessSnapshot): ReadinessCheck {
    const present = snap.calibration?.mag?.present;
    const state: ReadinessState =
        present === undefined ? "pending" : present ? "pass" : "fail";
    return {
        id: "mag",
        required: false,
        state,
        labelKey: "onboardingPage.readinessCheckMag",
        ctaKey: state === "pass" ? undefined : "onboardingPage.readinessCtaDiagnostics",
        ctaTarget: "diagnostics",
    };
}

function areaCheck(snap: ReadinessSnapshot): ReadinessCheck {
    const count = countValidAreas(snap.workingArea);
    const state: ReadinessState = count >= 1 ? "pass" : "fail";
    return {
        id: "area",
        required: true,
        state,
        labelKey: "onboardingPage.readinessCheckArea",
        ctaKey: state === "pass" ? undefined : "onboardingPage.readinessCtaDrawArea",
        ctaTarget: "map",
    };
}

/**
 * Build the ordered readiness checklist from a live snapshot. Pure: every
 * input is passed in (including `nowMs`) so the result is fully determined by
 * its arguments and unit-testable without wall-clock or i18n side effects
 * beyond the deterministic label lookups.
 */
export function computeReadinessChecks(snap: ReadinessSnapshot): ReadinessCheck[] {
    const dock = snap.calibration?.dock?.present;
    const imuBias = snap.calibration?.imu?.present;
    const checks: ReadinessCheck[] = [
        rtkCheck(snap),
        correctionsCheck(snap),
        datumCheck(snap),
        localizerRunningCheck(snap),
        localizerConfidenceCheck(snap),
        firmwareCheck(snap),
        presenceCheck(
            "dock",
            "onboardingPage.readinessCheckDock",
            dock,
            "calibration",
            "onboardingPage.readinessCtaCalibrate",
        ),
        presenceCheck(
            "imuBias",
            "onboardingPage.readinessCheckImuBias",
            imuBias,
            "calibration",
            "onboardingPage.readinessCtaCalibrate",
        ),
        imuYawCheck(snap),
    ];
    // Magnetometer is only a check when the operator opted into using one.
    if (snap.values.use_magnetometer === true) {
        checks.push(magCheck(snap));
    }
    checks.push(areaCheck(snap));
    return checks;
}

/** Required checks that are not yet passing — these gate "Finish & apply". */
export function requiredFailingChecks(checks: ReadinessCheck[]): ReadinessCheck[] {
    return checks.filter((c) => c.required && c.state !== "pass");
}
