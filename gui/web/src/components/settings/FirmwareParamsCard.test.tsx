import { fireEvent, render, screen, waitFor } from "@testing-library/react";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { FirmwareParamsCard } from "./FirmwareParamsCard.tsx";
import type { FirmwareParam, FirmwareParams } from "../../types/ros.generated.ts";

const mocks = vi.hoisted(() => ({
    report: null as FirmwareParams | null,
    firmwareAt: null as number | null,
    storeSequence: 0,
    storeAt: null as number | null,
    topics: {} as Record<string, { data: unknown; lastMessageAt: number | null }>,
    socketStatus: "open" as "open" | "closed",
    callCreate: vi.fn(),
}));

vi.mock("../../hooks/useFirmwareParams.ts", () => ({
    useFirmwareParams: () => ({
        report: mocks.report,
        lastMessageAt: mocks.firmwareAt,
        storeStatusSequence: mocks.storeSequence,
        lastStoreStatusAt: mocks.storeAt,
    }),
}));

vi.mock("../../hooks/useTopic.ts", () => ({
    useTopic: (topic: string) => mocks.topics[topic] ?? { data: {}, lastMessageAt: null },
}));

vi.mock("../../hooks/useWS.ts", () => ({
    useMultiplexStatus: () => mocks.socketStatus,
}));

vi.mock("../../hooks/useApi.ts", () => ({
    useApi: () => ({ mowglinext: { callCreate: mocks.callCreate } }),
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

const setSafeTelemetry = (now: number) => {
    const stamp = { sec: Math.floor(now / 1000), nanosec: (now % 1000) * 1_000_000 };
    mocks.topics = {
        status: {
            lastMessageAt: now,
            data: {
                stamp,
                blade_status_stamp: stamp,
                mower_status: 255,
                firmware_protocol_version: 8,
                firmware_compatible: true,
                mow_enabled: false,
                mower_esc_status: 0,
                mower_motor_rpm: 0,
            },
        },
        highLevelStatus: { lastMessageAt: now, data: { state: 1 } },
        wheelOdom: {
            lastMessageAt: now,
            data: { twist: { twist: { linear: { x: 0 }, angular: { z: 0 } } } },
        },
    };
};

describe("FirmwareParamsCard", () => {
    beforeEach(() => {
        mocks.report = null;
        mocks.firmwareAt = null;
        mocks.storeSequence = 0;
        mocks.storeAt = null;
        mocks.topics = {};
        mocks.socketStatus = "open";
        mocks.callCreate.mockReset();
        mocks.callCreate.mockResolvedValue({ data: { message: "request_id=123; sent" } });
    });

    afterEach(() => {
        vi.restoreAllMocks();
    });

    it("says nothing was reported before the first message", () => {
        render(<FirmwareParamsCard />);
        expect(screen.getByText(/No report from the firmware yet/)).toBeInTheDocument();
    });

    it("shows the applied value, the envelope and the stored state", () => {
        mocks.report = { firmware_incompatible: false, boot_source: 1, last_commit: 2, records_left: 30, params: [param({})] };
        render(<FirmwareParamsCard />);
        expect(screen.getByText("Tilt trip")).toBeInTheDocument();
        expect(screen.getByText("10 – 1000")).toBeInTheDocument();
        expect(screen.getByText("Stored")).toBeInTheDocument();
        expect(screen.getByText(/started on the values stored in its flash/)).toBeInTheDocument();
        expect(screen.queryByText("Moved into range")).not.toBeInTheDocument();
    });

    it("flags a value the firmware moved into its envelope", () => {
        mocks.report = {
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
        mocks.report = {
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
        mocks.report = { firmware_incompatible: true, boot_source: 255, last_commit: 5, records_left: 0, params: [param({ reported: false })] };
        render(<FirmwareParamsCard />);
        expect(screen.getByText(/does not speak this protocol version/)).toBeInTheDocument();
        expect(screen.getByText(/parameter-store operation failed or a reset request was rejected/)).toBeInTheDocument();
        expect(screen.getByText("No report")).toBeInTheDocument();
    });
});

describe("FirmwareParamsCard names", () => {
    it("shows a label, the parameter name and a description for every parameter", () => {
        mocks.report = {
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

    it("keeps reset disabled until the board state and protocol are fresh and safe", () => {
        mocks.report = { firmware_incompatible: false, boot_source: 1, last_commit: 2, records_left: 20, params: [] };
        render(<FirmwareParamsCard />);
        expect(screen.getByRole("button", { name: /Reset stored parameters/ })).toBeDisabled();
    });

    it("evaluates new telemetry against the current clock before the next timer tick", () => {
        let now = 100_000;
        vi.spyOn(Date, "now").mockImplementation(() => now);
        mocks.report = {
            last_commit: 6,
            reset_request_id: 77,
            params: [],
        };
        mocks.firmwareAt = now;
        mocks.storeSequence = 21;
        mocks.storeAt = now;
        setSafeTelemetry(now);
        const view = render(<FirmwareParamsCard />);
        expect(screen.getByRole("button", { name: /Reboot board to apply/ })).toBeInTheDocument();

        now += 200;
        mocks.firmwareAt = now;
        mocks.storeSequence = 22;
        mocks.storeAt = now;
        setSafeTelemetry(now);
        view.rerender(<FirmwareParamsCard />);
        expect(screen.getByRole("button", { name: /Reboot board to apply/ })).toBeInTheDocument();
        expect(mocks.callCreate).not.toHaveBeenCalled();
    });

    it("requires a fresh matching reset status, then asks separately before reboot", async () => {
        const now = Date.now();
        vi.spyOn(Date, "now").mockReturnValue(now);
        mocks.report = { firmware_incompatible: false, boot_source: 1, last_commit: 4, records_left: 0, params: [] };
        mocks.firmwareAt = now;
        mocks.storeSequence = 10;
        mocks.storeAt = now;
        setSafeTelemetry(now);
        const view = render(<FirmwareParamsCard />);

        fireEvent.click(screen.getByRole("button", { name: /Reset stored parameters/ }));
        expect(screen.getByText(/This sends a one-time request/)).toBeInTheDocument();
        expect(screen.getByText(/compiled charge ceiling may be higher/)).toBeInTheDocument();
        fireEvent.click(screen.getByRole("button", { name: "Send reset request" }));
        await waitFor(() => expect(mocks.callCreate).toHaveBeenCalledWith("reset_firmware_param_store", {}));
        expect(screen.queryByRole("button", { name: /Reboot board to apply/ })).not.toBeInTheDocument();

        mocks.report = {
            firmware_incompatible: false,
            boot_source: 1,
            last_commit: 6,
            reset_request_id: 123,
            records_left: 0,
            params: [],
        };
        mocks.firmwareAt = Date.now();
        mocks.storeSequence = 11;
        mocks.storeAt = Date.now();
        view.rerender(<FirmwareParamsCard />);
        expect(await screen.findByRole("button", { name: /Reboot board to apply/ })).toBeInTheDocument();
        expect(screen.getByText(/armed or still unresolved/)).toBeInTheDocument();

        // A code-6 ACK from the old sequence cannot be reused after a new
        // actual store-status packet reports an error or a different owner ID.
        mocks.report = { ...mocks.report, last_commit: 5 };
        mocks.storeSequence = 12;
        mocks.storeAt = Date.now();
        view.rerender(<FirmwareParamsCard />);
        expect(screen.queryByRole("button", { name: /Reboot board to apply/ })).not.toBeInTheDocument();

        mocks.report = { ...mocks.report, last_commit: 6, reset_request_id: 456 };
        mocks.storeSequence = 13;
        mocks.storeAt = Date.now();
        view.rerender(<FirmwareParamsCard />);
        expect(screen.queryByRole("button", { name: /Reboot board to apply/ })).not.toBeInTheDocument();

        mocks.report = { ...mocks.report, last_commit: 6, reset_request_id: 123 };
        mocks.storeSequence = 14;
        mocks.storeAt = Date.now();
        view.rerender(<FirmwareParamsCard />);
        const rebootButton = await screen.findByRole("button", { name: /Reboot board to apply/ });
        fireEvent.click(rebootButton);
        expect(screen.getByText(/separate, explicit reboot/)).toBeInTheDocument();
        fireEvent.click(screen.getByRole("button", { name: "Reboot board" }));
        await waitFor(() => expect(mocks.callCreate).toHaveBeenLastCalledWith("reboot_board", {}));
        expect(mocks.callCreate).toHaveBeenCalledTimes(2);
    });

    it("hides the reboot action when the live stream closes", () => {
        const now = Date.now();
        mocks.report = {
            firmware_incompatible: false,
            boot_source: 1,
            last_commit: 6,
            reset_request_id: 77,
            records_left: 0,
            params: [],
        };
        mocks.firmwareAt = now;
        mocks.storeSequence = 21;
        mocks.storeAt = now;
        setSafeTelemetry(now);
        const view = render(<FirmwareParamsCard />);
        expect(screen.getByRole("button", { name: /Reboot board to apply/ })).toBeInTheDocument();

        mocks.socketStatus = "closed";
        view.rerender(<FirmwareParamsCard />);
        expect(screen.queryByRole("button", { name: /Reboot board to apply/ })).not.toBeInTheDocument();
    });
});
