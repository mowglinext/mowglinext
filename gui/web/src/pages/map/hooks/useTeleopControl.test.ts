import {afterEach, beforeEach, describe, expect, it, vi} from "vitest";
import {act, renderHook} from "@testing-library/react";

const mockSocket = vi.hoisted(() => ({
    readyState: 0,
    options: undefined as {onClose?: () => void; onError?: () => void; onMessage?: (event: MessageEvent) => void} | undefined,
    sendJsonMessage: vi.fn(),
}));

vi.mock("react-use-websocket", () => ({
    ReadyState: {CONNECTING: 0, OPEN: 1, CLOSING: 2, CLOSED: 3},
    default: (_url: string | null, options: typeof mockSocket.options) => {
        mockSocket.options = options;
        return {readyState: mockSocket.readyState, sendJsonMessage: mockSocket.sendJsonMessage};
    },
}));

import {useTeleopControl} from "./useTeleopControl.ts";

function deliver(state: "available" | "busy" | "owner" | "stopped", revision: number) {
    act(() => mockSocket.options?.onMessage?.({
        data: JSON.stringify({type: "teleop_state", state, revision}),
    } as MessageEvent));
}

describe("useTeleopControl", () => {
    beforeEach(() => {
        mockSocket.readyState = 1;
        mockSocket.sendJsonMessage.mockClear();
        mockSocket.options = undefined;
    });

    afterEach(() => vi.clearAllMocks());

    it("acquires immediately when Manual Mow opens an available session", () => {
        const {result} = renderHook(() => useTeleopControl());
        act(() => result.current.start("/api/mowglinext/publish/joy"));

        // A session opened by this operator is already available by the time
        // the successful Manual Mow request resolves.
        deliver("available", 1);
        act(() => result.current.requestControl());
        expect(mockSocket.sendJsonMessage).toHaveBeenCalledWith({type: "acquire"});
    });

    it("waits through the initial stopped snapshot after a successful Manual Mow request", () => {
        const {result} = renderHook(() => useTeleopControl());
        act(() => result.current.start("/api/mowglinext/publish/joy"));
        deliver("stopped", 1);

        // The accepted mode request can reach the API before the socket's
        // available broadcast reaches React. The explicit intent must survive
        // that initial snapshot, then acquire as soon as the session opens.
        act(() => result.current.requestControl());
        expect(mockSocket.sendJsonMessage).not.toHaveBeenCalled();
        deliver("available", 2);
        expect(mockSocket.sendJsonMessage).toHaveBeenCalledWith({type: "acquire"});
    });

    it("reacquires after a transient socket loss, but not after STOP", () => {
        const {result} = renderHook(() => useTeleopControl());
        act(() => result.current.start("/api/mowglinext/publish/joy"));
        deliver("available", 1);
        act(() => result.current.requestControl());
        deliver("owner", 2);
        const countAfterInitialAcquire = mockSocket.sendJsonMessage.mock.calls.length;

        act(() => mockSocket.options?.onClose?.());
        deliver("available", 3);
        expect(mockSocket.sendJsonMessage).toHaveBeenCalledTimes(countAfterInitialAcquire + 1);
        expect(mockSocket.sendJsonMessage).toHaveBeenLastCalledWith({type: "acquire"});

        deliver("owner", 4);
        deliver("stopped", 5);
        const countAfterStop = mockSocket.sendJsonMessage.mock.calls.length;
        act(() => mockSocket.options?.onClose?.());
        deliver("available", 6);
        expect(mockSocket.sendJsonMessage).toHaveBeenCalledTimes(countAfterStop);
    });

    it("does not queue an automatic takeover when another client owns control", () => {
        const {result} = renderHook(() => useTeleopControl());
        act(() => result.current.start("/api/mowglinext/publish/joy"));
        deliver("busy", 1);
        act(() => result.current.requestControl());
        deliver("available", 2);
        expect(mockSocket.sendJsonMessage).not.toHaveBeenCalledWith({type: "acquire"});
    });
});
