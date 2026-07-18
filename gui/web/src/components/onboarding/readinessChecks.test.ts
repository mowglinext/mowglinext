import {describe, expect, it} from "vitest";
import {GnssStatusConstants} from "../../types/ros.ts";
import type {FusionGraphStats} from "../../hooks/useFusionGraphDiagnostics.ts";
import type {CalibrationStatus} from "../../hooks/useCalibrationStatus.ts";
import {
    computeReadinessChecks,
    requiredFailingChecks,
    type ReadinessSnapshot,
    type ReadinessState,
} from "./readinessChecks.ts";

const NOW = 1_000_000;

function calibration(
    dock: boolean,
    imu: boolean,
    mag: boolean,
): CalibrationStatus {
    return {
        dock: {present: dock},
        imu: {present: imu},
        mag: {present: mag},
    };
}

function freshFusion(values: Record<string, string>): FusionGraphStats {
    return {receivedAt: NOW - 1000, level: 0, message: "", values};
}

/** A snapshot where every check passes; individual tests override slices. */
function passingSnapshot(overrides: Partial<ReadinessSnapshot> = {}): ReadinessSnapshot {
    return {
        gnss: {
            rtk_mode: GnssStatusConstants.RTK_MODE_FIXED,
            correction_stream_status: GnssStatusConstants.CORRECTION_STREAM_STATUS_ACTIVE,
        },
        fusion: freshFusion({total_nodes: "42", cov_xx: "0.0009", cov_yy: "0.0009"}),
        calibration: calibration(true, true, true),
        firmwareCompatible: true,
        values: {datum_lat: 48.87, datum_lon: 2.17, imu_yaw: 0.12},
        workingArea: [{area: {points: [{x: 0, y: 0}, {x: 1, y: 0}, {x: 1, y: 1}]}}],
        nowMs: NOW,
        ...overrides,
    };
}

function stateOf(checks: ReturnType<typeof computeReadinessChecks>, id: string): ReadinessState {
    const found = checks.find((c) => c.id === id);
    if (!found) throw new Error(`no check with id ${id}`);
    return found.state;
}

describe("computeReadinessChecks — all-pass baseline", () => {
    it("passes every required check for a fully ready robot", () => {
        const checks = computeReadinessChecks(passingSnapshot());
        expect(requiredFailingChecks(checks)).toHaveLength(0);
    });

    it("omits the magnetometer check unless use_magnetometer is on", () => {
        const withoutMag = computeReadinessChecks(passingSnapshot());
        expect(withoutMag.some((c) => c.id === "mag")).toBe(false);

        const withMag = computeReadinessChecks(
            passingSnapshot({values: {datum_lat: 48.87, datum_lon: 2.17, imu_yaw: 0.12, use_magnetometer: true}}),
        );
        expect(withMag.some((c) => c.id === "mag")).toBe(true);
    });
});

describe("RTK check", () => {
    it("passes on RTK fixed", () => {
        expect(stateOf(computeReadinessChecks(passingSnapshot()), "rtk")).toBe("pass");
    });

    it("is pending on RTK float", () => {
        const snap = passingSnapshot({gnss: {rtk_mode: GnssStatusConstants.RTK_MODE_FLOAT}});
        expect(stateOf(computeReadinessChecks(snap), "rtk")).toBe("pending");
    });

    it("fails on no fix", () => {
        const snap = passingSnapshot({gnss: {fix_valid: false}});
        expect(stateOf(computeReadinessChecks(snap), "rtk")).toBe("fail");
    });
});

describe("NTRIP corrections check (recommended)", () => {
    it("passes when the correction stream is active", () => {
        expect(stateOf(computeReadinessChecks(passingSnapshot()), "corrections")).toBe("pass");
    });

    it("is pending while waiting", () => {
        const snap = passingSnapshot({
            gnss: {
                rtk_mode: GnssStatusConstants.RTK_MODE_FIXED,
                correction_stream_status: GnssStatusConstants.CORRECTION_STREAM_STATUS_WAITING,
            },
        });
        expect(stateOf(computeReadinessChecks(snap), "corrections")).toBe("pending");
    });

    it("fails on a stream error", () => {
        const snap = passingSnapshot({
            gnss: {
                rtk_mode: GnssStatusConstants.RTK_MODE_FIXED,
                correction_stream_status: GnssStatusConstants.CORRECTION_STREAM_STATUS_ERROR,
            },
        });
        expect(stateOf(computeReadinessChecks(snap), "corrections")).toBe("fail");
    });

    it("does not gate finishing (recommended only)", () => {
        const snap = passingSnapshot({
            gnss: {
                rtk_mode: GnssStatusConstants.RTK_MODE_FIXED,
                correction_stream_status: GnssStatusConstants.CORRECTION_STREAM_STATUS_ERROR,
            },
        });
        const failing = requiredFailingChecks(computeReadinessChecks(snap));
        expect(failing.some((c) => c.id === "corrections")).toBe(false);
    });
});

describe("Datum check", () => {
    it("passes with finite non-zero coordinates", () => {
        expect(stateOf(computeReadinessChecks(passingSnapshot()), "datum")).toBe("pass");
    });

    it("fails at (0,0)", () => {
        const snap = passingSnapshot({values: {datum_lat: 0, datum_lon: 0, imu_yaw: 0.12}});
        expect(stateOf(computeReadinessChecks(snap), "datum")).toBe("fail");
    });

    it("is pending before values load", () => {
        const snap = passingSnapshot({values: {imu_yaw: 0.12}});
        expect(stateOf(computeReadinessChecks(snap), "datum")).toBe("pending");
    });
});

describe("Localizer running check", () => {
    it("passes with fresh total_nodes", () => {
        expect(stateOf(computeReadinessChecks(passingSnapshot()), "localizer")).toBe("pass");
    });

    it("is pending when no fusion snapshot yet", () => {
        const snap = passingSnapshot({fusion: null});
        expect(stateOf(computeReadinessChecks(snap), "localizer")).toBe("pending");
    });

    it("is pending when the snapshot is stale", () => {
        const stale: FusionGraphStats = {receivedAt: NOW - 60_000, level: 0, message: "", values: {total_nodes: "42"}};
        const snap = passingSnapshot({fusion: stale});
        expect(stateOf(computeReadinessChecks(snap), "localizer")).toBe("pending");
    });
});

describe("Localizer confidence check (recommended)", () => {
    it("passes when σ is below the threshold", () => {
        expect(stateOf(computeReadinessChecks(passingSnapshot()), "localizerConfidence")).toBe("pass");
    });

    it("fails when σ is too large", () => {
        const snap = passingSnapshot({
            fusion: freshFusion({total_nodes: "42", cov_xx: "0.25", cov_yy: "0.25"}),
        });
        expect(stateOf(computeReadinessChecks(snap), "localizerConfidence")).toBe("fail");
    });

    it("is pending when covariance is absent", () => {
        const snap = passingSnapshot({fusion: freshFusion({total_nodes: "42"})});
        expect(stateOf(computeReadinessChecks(snap), "localizerConfidence")).toBe("pending");
    });
});

describe("Firmware check", () => {
    it("passes when compatible", () => {
        expect(stateOf(computeReadinessChecks(passingSnapshot()), "firmware")).toBe("pass");
    });

    it("is pending before the handshake (null)", () => {
        const snap = passingSnapshot({firmwareCompatible: null});
        expect(stateOf(computeReadinessChecks(snap), "firmware")).toBe("pending");
    });

    it("fails when incompatible", () => {
        const snap = passingSnapshot({firmwareCompatible: false});
        expect(stateOf(computeReadinessChecks(snap), "firmware")).toBe("fail");
    });
});

describe("Dock / IMU-bias presence checks", () => {
    it("are pending when calibration status is unavailable", () => {
        const snap = passingSnapshot({calibration: null});
        expect(stateOf(computeReadinessChecks(snap), "dock")).toBe("pending");
        expect(stateOf(computeReadinessChecks(snap), "imuBias")).toBe("pending");
    });

    it("fail when not present", () => {
        const snap = passingSnapshot({calibration: calibration(false, false, false)});
        expect(stateOf(computeReadinessChecks(snap), "dock")).toBe("fail");
        expect(stateOf(computeReadinessChecks(snap), "imuBias")).toBe("fail");
    });
});

describe("IMU mounting yaw check", () => {
    it("passes on a non-zero solve", () => {
        expect(stateOf(computeReadinessChecks(passingSnapshot()), "imuYaw")).toBe("pass");
    });

    it("fails when still zero", () => {
        const snap = passingSnapshot({values: {datum_lat: 48.87, datum_lon: 2.17, imu_yaw: 0}});
        expect(stateOf(computeReadinessChecks(snap), "imuYaw")).toBe("fail");
    });

    it("is pending before the value loads", () => {
        const snap = passingSnapshot({values: {datum_lat: 48.87, datum_lon: 2.17}});
        expect(stateOf(computeReadinessChecks(snap), "imuYaw")).toBe("pending");
    });
});

describe("Mowing area check (required by product decision)", () => {
    it("passes with at least one 3+ point polygon", () => {
        expect(stateOf(computeReadinessChecks(passingSnapshot()), "area")).toBe("pass");
    });

    it("fails with no areas", () => {
        const snap = passingSnapshot({workingArea: []});
        const checks = computeReadinessChecks(snap);
        expect(stateOf(checks, "area")).toBe("fail");
        expect(requiredFailingChecks(checks).some((c) => c.id === "area")).toBe(true);
    });

    it("fails a degenerate polygon with fewer than 3 points", () => {
        const snap = passingSnapshot({workingArea: [{area: {points: [{x: 0, y: 0}, {x: 1, y: 1}]}}]});
        expect(stateOf(computeReadinessChecks(snap), "area")).toBe("fail");
    });
});

describe("requiredFailingChecks gating", () => {
    it("lists exactly the failing/pending required checks", () => {
        const snap = passingSnapshot({
            firmwareCompatible: false,
            values: {datum_lat: 0, datum_lon: 0, imu_yaw: 0.12},
        });
        const failing = requiredFailingChecks(computeReadinessChecks(snap)).map((c) => c.id);
        expect(failing).toEqual(expect.arrayContaining(["firmware", "datum"]));
        expect(failing).not.toContain("corrections");
        expect(failing).not.toContain("localizerConfidence");
    });
});
