import {describe, expect, it, vi, beforeEach} from "vitest";
import {renderHook, waitFor, screen, fireEvent} from "@testing-library/react";
import {App} from "antd";
import React from "react";
import {useDigObstructionAck} from "./useDigObstructionAck.ts";

const mockStatus = vi.fn<() => Record<string, unknown>>();
vi.mock("./useStatus.ts", () => ({useStatus: () => mockStatus()}));

const callCreateMock = vi.fn();
vi.mock("./useApi.ts", () => ({
    useApi: () => ({mowglinext: {callCreate: callCreateMock}}),
}));

const wrapper = ({children}: {children: React.ReactNode}) => <App>{children}</App>;

describe("useDigObstructionAck", () => {
    beforeEach(() => {
        callCreateMock.mockReset();
        callCreateMock.mockResolvedValue({error: undefined});
    });

    it("runs the start action unchanged when no escalation is latched", async () => {
        mockStatus.mockReturnValue({dig_escalated: false});
        const start = vi.fn().mockResolvedValue(undefined);
        const {result} = renderHook(() => useDigObstructionAck("IDLE"), {wrapper});

        await result.current(start)();

        expect(start).toHaveBeenCalledTimes(1);
        expect(callCreateMock).not.toHaveBeenCalled();
    });

    it("treats the DIG_OBSTRUCTION state as held even before the status arrives", async () => {
        mockStatus.mockReturnValue({});
        const start = vi.fn().mockResolvedValue(undefined);
        const {result} = renderHook(() => useDigObstructionAck("DIG_OBSTRUCTION"), {wrapper});

        const pending = result.current(start)();
        fireEvent.click((await screen.findByText(/resume mowing/i)).closest("button")!);
        await pending;

        expect(callCreateMock).toHaveBeenCalledWith("clear_dig_escalation", {});
        expect(start).toHaveBeenCalledTimes(1);
    });

    it("does not start when releasing the hold fails", async () => {
        mockStatus.mockReturnValue({dig_escalated: true, dig_escalated_distance_m: 2, dig_escalated_required_distance_m: 0.5});
        callCreateMock.mockResolvedValue({error: {error: "bridge unavailable"}});
        const start = vi.fn().mockResolvedValue(undefined);
        const {result} = renderHook(() => useDigObstructionAck(), {wrapper});

        await expect(result.current(start)()).rejects.toThrow("bridge unavailable");
        await waitFor(() => expect(start).not.toHaveBeenCalled());
    });
});
