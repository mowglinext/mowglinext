import { act, renderHook } from "@testing-library/react";
import { beforeEach, describe, expect, it, vi } from "vitest";
import { useFirmwareParams } from "./useFirmwareParams.ts";

const mocks = vi.hoisted(() => ({
    onError: undefined as ((error: Error) => void) | undefined,
    onData: undefined as ((data: unknown) => void) | undefined,
    start: vi.fn(),
    stop: vi.fn(),
}));

vi.mock("./useWS.ts", () => ({
    useWS: (
        onError: (error: Error) => void,
        _onInfo: (message: string) => void,
        onData: (data: unknown) => void,
    ) => {
        mocks.onError = onError;
        mocks.onData = onData;
        return { start: mocks.start, stop: mocks.stop };
    },
}));

describe("useFirmwareParams store-status freshness", () => {
    beforeEach(() => {
        vi.useFakeTimers();
        vi.setSystemTime(100_000);
        mocks.start.mockReset();
        mocks.stop.mockReset();
        mocks.onError = undefined;
        mocks.onData = undefined;
    });

    it("treats the initial latched sequence as a baseline and only timestamps a changed identity", () => {
        const { result } = renderHook(() => useFirmwareParams());
        const onData = mocks.onData!;

        act(() => onData({ store_status_sequence: 5, last_commit: 6 }));
        expect(result.current.storeStatusSequence).toBe(5);
        expect(result.current.lastStoreStatusAt).toBeNull();

        vi.setSystemTime(101_000);
        act(() => onData({ store_status_sequence: 5, last_commit: 6 }));
        expect(result.current.lastStoreStatusAt).toBeNull();

        vi.setSystemTime(102_000);
        act(() => onData({ store_status_sequence: 6, last_commit: 6 }));
        expect(result.current.storeStatusSequence).toBe(6);
        expect(result.current.lastStoreStatusAt).toBe(102_000);

        vi.setSystemTime(103_000);
        act(() => onData({ store_status_sequence: 6, last_commit: 6, params: [{ id: 1 }] }));
        expect(result.current.lastMessageAt).toBe(103_000);
        expect(result.current.lastStoreStatusAt).toBe(102_000);
    });

    it("requires a changed sequence after reconnect instead of refreshing from a cached replay", () => {
        const { result } = renderHook(() => useFirmwareParams());
        const onData = mocks.onData!;
        act(() => onData({ store_status_sequence: 5, last_commit: 6 }));
        act(() => onData({ store_status_sequence: 6, last_commit: 6 }));
        expect(result.current.lastStoreStatusAt).toBe(100_000);

        vi.setSystemTime(110_000);
        act(() => mocks.onError!(new Error("Stream closed")));
        expect(result.current.storeStatusSequence).toBe(0);
        expect(result.current.lastStoreStatusAt).toBeNull();

        vi.setSystemTime(111_000);
        act(() => onData({ store_status_sequence: 7, last_commit: 6 }));
        expect(result.current.storeStatusSequence).toBe(7);
        expect(result.current.lastStoreStatusAt).toBeNull();

        vi.setSystemTime(112_000);
        act(() => onData({ store_status_sequence: 8, last_commit: 6 }));
        expect(result.current.storeStatusSequence).toBe(8);
        expect(result.current.lastStoreStatusAt).toBe(112_000);
    });
});
