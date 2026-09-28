import { render, screen } from "@testing-library/react";
import { beforeEach, describe, expect, it, vi } from "vitest";
import { FirmwareParamsCard } from "./FirmwareParamsCard.tsx";
import type { FirmwareParam, FirmwareParams } from "../../types/ros.generated.ts";

let report: FirmwareParams | null = null;

vi.mock("../../hooks/useFirmwareParams.ts", () => ({
    useFirmwareParams: () => report,
}));

const param = (overrides: Partial<FirmwareParam>): FirmwareParam => ({
    id: 42,
    name: "tilt_emergency_ms",
    requested_valid: true,
    requested: 500,
    reported: true,
    applied: 500,
    default_value: 500,
    min_value: 10,
    max_value: 1000,
    status: 0,
    persisted: true,
    is_volatile: false,
    ...overrides,
});

describe("FirmwareParamsCard", () => {
    beforeEach(() => {
        report = null;
    });

    it("says nothing was reported before the first message", () => {
        render(<FirmwareParamsCard />);
        expect(screen.getByText(/No report from the firmware yet/)).toBeInTheDocument();
    });

    it("shows the applied value, the envelope and the stored state", () => {
        report = { firmware_incompatible: false, boot_source: 1, last_commit: 2, records_left: 30, params: [param({})] };
        render(<FirmwareParamsCard />);
        expect(screen.getByText("Tilt trip")).toBeInTheDocument();
        expect(screen.getByText("10 – 1000")).toBeInTheDocument();
        expect(screen.getByText("Stored")).toBeInTheDocument();
        expect(screen.getByText(/started on the values stored in its flash/)).toBeInTheDocument();
        expect(screen.queryByText("Moved into range")).not.toBeInTheDocument();
    });

    it("flags a value the firmware moved into its envelope", () => {
        report = {
            firmware_incompatible: false,
            boot_source: 1,
            last_commit: 1,
            records_left: 29,
            params: [param({ requested: 60000, applied: 1000, status: 1, persisted: false })],
        };
        render(<FirmwareParamsCard />);
        expect(screen.getByText("Moved into range")).toBeInTheDocument();
        expect(screen.getByText("Not stored yet")).toBeInTheDocument();
        expect(screen.getByText("60000")).toBeInTheDocument();
    });

    it("marks the gyro bias as measured, never stored", () => {
        report = {
            firmware_incompatible: false,
            boot_source: 1,
            last_commit: 2,
            records_left: 29,
            params: [param({ id: 15, name: "imu_gyro_bias_z", requested: 0.01, applied: 0.01, is_volatile: true, persisted: false })],
        };
        render(<FirmwareParamsCard />);
        expect(screen.getByText("Not stored (measured)")).toBeInTheDocument();
    });

    it("warns when the board runs an incompatible firmware or cannot write its flash", () => {
        report = { firmware_incompatible: true, boot_source: 255, last_commit: 5, records_left: 0, params: [param({ reported: false })] };
        render(<FirmwareParamsCard />);
        expect(screen.getByText(/does not speak this protocol version/)).toBeInTheDocument();
        expect(screen.getByText(/could not write its flash/)).toBeInTheDocument();
        expect(screen.getByText("No report")).toBeInTheDocument();
    });
});

describe("FirmwareParamsCard names", () => {
    it("shows a label, the parameter name and a description for every parameter", () => {
        report = {
            firmware_incompatible: false,
            boot_source: 1,
            last_commit: 2,
            records_left: 29,
            params: [
                param({}),
                param({ id: 2, name: "wheel_pid_kp", requested: 10, applied: 10 }),
                param({ id: 15, name: "imu_gyro_bias_z", requested: 0.01, applied: 0.01, is_volatile: true }),
            ],
        };
        render(<FirmwareParamsCard />);
        // Editable setting: its settingsFields label + the raw name.
        expect(screen.getByText("Tilt trip")).toBeInTheDocument();
        expect(screen.getByText("tilt_emergency_ms")).toBeInTheDocument();
        // Tuned elsewhere / measured: labels from firmwareParams.params.
        expect(screen.getByText("Wheel PI proportional gain")).toBeInTheDocument();
        expect(screen.getByText("wheel_pid_kp")).toBeInTheDocument();
        expect(screen.getByText("Gyro bias (measured)")).toBeInTheDocument();
        // Each row carries its description (tooltip icon, labelled by it).
        expect(screen.getByLabelText(/must persist before the firmware latches/)).toBeInTheDocument();
        expect(screen.getByLabelText(/Proportional gain of the per-wheel velocity loop/)).toBeInTheDocument();
        expect(screen.getByLabelText(/At-rest gyro Z offset/)).toBeInTheDocument();
    });
});
