import {act, cleanup, renderHook} from "@testing-library/react";
import {afterEach, beforeEach, describe, expect, it, vi} from "vitest";
import {useBlackbox, type BlackboxStatus} from "./useBlackbox";

const request = vi.hoisted(() => vi.fn());
const api = {request, baseUrl: "/api"};
vi.mock("./useApi", () => ({useApi: () => api}));

const status: BlackboxStatus = {
    config: {enabled: true, pre_seconds: 60, post_seconds: 15, memory_bytes: 16 << 20,
        max_message_bytes: 8192, max_snapshots: 20, max_disk_bytes: 256 << 20,
        cooldown_seconds: 60, min_free_disk_bytes: 32 << 20},
    phase: "ready", buffered_seconds: 4, buffered_bytes: 1024, effective_memory_bytes: 16 << 20,
    dropped_messages: 0, memory_pressure: false, topics: [], recordings: [],
};

describe("blackbox status isolation", () => {
    beforeEach(() => { vi.useFakeTimers(); request.mockReset(); });
    afterEach(() => { cleanup(); vi.useRealTimers(); });

    it.each([{}, null, {...status, config: {}}, {...status, topics: [{}]}, {...status, recordings: [{}]}])(
        "rejects malformed success responses and recovers on the next poll: %j", async data => {
            request.mockResolvedValueOnce({data}).mockResolvedValue({data: status});
            const {result} = renderHook(() => useBlackbox());
            await act(async () => { await vi.advanceTimersByTimeAsync(0); });
            expect(result.current.status).toBeNull();
            expect(result.current.error).toBe("Invalid blackbox status response");
            await act(async () => { await vi.advanceTimersByTimeAsync(4000); });
            expect(result.current.status).toEqual(status);
            expect(result.current.error).toBeNull();
        },
    );

    it("keeps the last valid recordings during an API failure and stops polling on unmount", async () => {
        request.mockResolvedValueOnce({data: status}).mockRejectedValue(new Error("Disconnected"));
        const {result, unmount} = renderHook(() => useBlackbox());
        await act(async () => { await vi.advanceTimersByTimeAsync(0); });
        await act(async () => { await vi.advanceTimersByTimeAsync(4000); });
        expect(result.current.status).toEqual(status);
        expect(result.current.error).toBe("Disconnected");
        unmount();
        await act(async () => { await vi.advanceTimersByTimeAsync(8000); });
        expect(request).toHaveBeenCalledTimes(2);
    });
});
