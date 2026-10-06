import { beforeEach, describe, expect, it, vi } from "vitest";
import { fireEvent, render, screen, waitFor } from "@testing-library/react";
import { MavrosCalibrationCard } from "./MavrosCalibrationCard.tsx";

const mock = vi.hoisted(() => ({ request: vi.fn() }));
vi.mock("../../hooks/useApi.ts", () => ({ useApi: () => mock }));

describe("MAVROS RTK calibration", () => {
    beforeEach(() => { mock.request.mockReset(); });
    it("records, calculates and applies only ticks_per_meter", async () => {
        let state = "idle";
        mock.request.mockImplementation(({ path }: { path: string }) => {
            if (path.endsWith("start")) state = "recording";
            if (path.endsWith("finish")) state = "ready";
            if (path.endsWith("apply")) state = "applied";
            return Promise.resolve({ data: {
            state,
            samples: 5, distance_m: 4, ticks_per_meter: 300,
        } }); });
        const accept = vi.fn(); render(<MavrosCalibrationCard acceptPersistedValues={accept} />);
        fireEvent.click(screen.getByRole("button", { name: "Start recording" }));
        await waitFor(() => expect(screen.getByRole("button", { name: "Finish recording" })).toBeEnabled());
        fireEvent.click(screen.getByRole("button", { name: "Finish recording" }));
        await waitFor(() => expect(screen.getByRole("button", { name: "Apply ticks_per_meter" })).toBeEnabled());
        expect(screen.getByText("300.000")).toBeInTheDocument();
        fireEvent.click(screen.getByRole("button", { name: "Apply ticks_per_meter" }));
        await waitFor(() => expect(accept).toHaveBeenCalledWith({ ticks_per_meter: 300 }));
        expect(mock.request.mock.calls.every(([request]) => request.path.startsWith("/tools/drive/mavros-calibration"))).toBe(true);
    });
    it("keeps a failed live application visible and does not claim persistence", async () => {
        mock.request.mockImplementation(({ path }: { path: string }) => Promise.resolve(path.endsWith("apply")
            ? { error: { error: "sidecar unavailable" } }
            : { data: { state: "ready", samples: 5, distance_m: 4, ticks_per_meter: 300 } }));
        const accept = vi.fn(); render(<MavrosCalibrationCard acceptPersistedValues={accept} />);
        await waitFor(() => expect(screen.getByRole("button", { name: "Apply ticks_per_meter" })).toBeEnabled());
        fireEvent.click(screen.getByRole("button", { name: "Apply ticks_per_meter" }));
        expect(await screen.findByText("sidecar unavailable")).toBeInTheDocument(); expect(accept).not.toHaveBeenCalled();
    });
});
