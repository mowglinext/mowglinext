import {describe, expect, it, vi, beforeEach} from "vitest";
import {act, renderHook, waitFor, screen, fireEvent} from "@testing-library/react";
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

        let pending: Promise<void>;
        await act(async () => {
            pending = result.current(start)();
            await Promise.resolve(); // Yield without waiting for the user-dependent confirmation.
        });
        const dialog = await screen.findByRole("dialog");
        expect(dialog).toHaveTextContent(/resume from the blocked spot/i);
        // jsdom does not run CSS animations or emit their completion events.
        // rc-motion selects the WebKit event when AnimationEvent is unavailable.
        const animationEnd = "AnimationEvent" in window ? "animationend" : "webkitAnimationEnd";
        await waitFor(() => expect(dialog).toHaveClass("ant-zoom-appear-active"));
        fireEvent(dialog, new Event(animationEnd, {bubbles: true}));
        await waitFor(() => expect(dialog).toBeVisible());
        const confirmButton = (await screen.findByText(/resume mowing/i)).closest("button")!;
        await act(async () => {
            fireEvent.click(confirmButton);
            await pending;
        });
        await waitFor(() => expect(dialog).toHaveClass("ant-zoom-leave-active"));
        fireEvent(dialog, new Event(animationEnd, {bubbles: true}));
        await waitFor(() => expect(screen.queryByRole("dialog", {hidden: true})).not.toBeInTheDocument());

        expect(callCreateMock).toHaveBeenCalledWith("clear_dig_escalation", {});
        expect(callCreateMock).toHaveBeenCalledTimes(1);
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
