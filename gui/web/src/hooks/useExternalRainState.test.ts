import {afterEach, beforeEach, describe, expect, it, vi} from "vitest";
import {act, cleanup, renderHook} from "@testing-library/react";
import {createElement, StrictMode, type PropsWithChildren} from "react";
import {parseExternalRainState, useExternalRainState} from "./useExternalRainState.ts";

const socket = vi.hoisted(() => ({
    listeners: new Set<(raw: unknown) => void>(),
    unsubscribe: vi.fn(),
}));
vi.mock("./multiplexedSocket.ts", () => ({
    getMultiplexedSocket: () => ({
        subscribe: (topic: string, listener: (raw: unknown) => void) => {
            expect(topic).toBe("externalRainState");
            socket.listeners.add(listener);
            return () => {
                socket.listeners.delete(listener);
                socket.unsubscribe();
            };
        },
    }),
}));
beforeEach(() => { socket.listeners.clear(); socket.unsubscribe.mockClear(); });
afterEach(cleanup);

const wrap = (obj: unknown) => ({data: JSON.stringify(obj)});

describe("parseExternalRainState", () => {
    it("reads the JSON out of a std_msgs/String", () => {
        const state = parseExternalRainState(wrap({
            state: "wet", enabled: true, received: true, wet: true, active: true, onboard_wet: false,
            age_s: 312, expires_in_s: 6888, max_age_s: 7200,
        }));
        expect(state?.state).toBe("wet");
        expect(state?.active).toBe(true);
        expect(state?.expires_in_s).toBe(6888);
    });

    it("keeps nulls for ages that do not apply", () => {
        const state = parseExternalRainState(wrap({
            state: "no_data", enabled: true, received: false, wet: false, active: false, onboard_wet: false,
            age_s: null, expires_in_s: null, max_age_s: 0,
        }));
        expect(state?.age_s).toBeNull();
        expect(state?.expires_in_s).toBeNull();
    });

    it("ignores anything that is not a usable state, so the last good one stays", () => {
        expect(parseExternalRainState(undefined)).toBeUndefined();
        expect(parseExternalRainState(null)).toBeUndefined();
        expect(parseExternalRainState({data: ""})).toBeUndefined();
        expect(parseExternalRainState({data: "not json"})).toBeUndefined();
        expect(parseExternalRainState({data: "{}"})).toBeUndefined();
        expect(parseExternalRainState({data: 5})).toBeUndefined();
    });

    for (const field of ["received", "active", "onboard_wet"] as const) {
        it.each(["true", "false", 0, 1, {}, []])(`rejects malformed ${field}: %j`, value => {
            expect(parseExternalRainState(wrap({state: "wet", [field]: value}))).toBeUndefined();
        });
        it.each([null, false, true])(`preserves valid ${field}: %j`, value => {
            const payload = {state: "wet", [field]: value};
            expect(parseExternalRainState(wrap(payload))).toEqual(payload);
        });
    }

    for (const field of ["age_s", "expires_in_s"] as const) {
        it.each(["15", {}, [], true, false])(`rejects malformed ${field}: %j`, value => {
            expect(parseExternalRainState(wrap({state: "wet", [field]: value}))).toBeUndefined();
        });
        it.each(["1e999", "-1e999"])(`rejects JSON overflow in ${field}: %s`, literal => {
            expect(parseExternalRainState({data: `{"state":"wet","${field}":${literal}}`})).toBeUndefined();
        });
        it.each([null, 0, -1, 1.25, Number.MAX_VALUE, -Number.MAX_VALUE])(`preserves finite/null ${field}: %j`, value => {
            const payload = {state: "wet", [field]: value};
            expect(parseExternalRainState(wrap(payload))).toEqual(payload);
        });
    }

    it.each(["disabled", "no_data", "dry", "wet", "expired", "future_state", ""])("keeps state %j with absent optional fields", state => {
        const parsed = parseExternalRainState(wrap({state}));
        expect(parsed).toEqual({state});
        expect(Object.keys(parsed ?? {})).toEqual(["state"]);
    });

    it("preserves extra and unconsumed fields without imposing their types", () => {
        const payload = {state: "future_state", enabled: {future: true}, wet: "unknown", max_age_s: null, extra: [1, "two"]};
        expect(parseExternalRainState(wrap(payload))).toEqual(payload);
    });

    it.each([null, [], ["wet"], {}, "wet", 1, true, {state: null}, {state: 1}, {state: false}])("rejects an invalid JSON root/state: %j", payload => {
        expect(parseExternalRainState(wrap(payload))).toBeUndefined();
    });

    it("returns undefined, never null, for unreadable JSON and invalid envelopes", () => {
        for (const raw of [undefined, null, [], "text", {data: 1}, {data: ""}, {data: "{"}, {data: '{"state":"wet","age_s":NaN}'}]) {
            expect(parseExternalRainState(raw)).toBeUndefined();
        }
    });
});

describe("useExternalRainState with real useTopic", () => {
    const emit = (raw: unknown) => act(() => { socket.listeners.forEach(listener => listener(raw)); });

    it("retains the previous object on rejection and accepts the next valid frame", () => {
        const view = renderHook(useExternalRainState);
        expect(view.result.current).toBeNull();
        emit(wrap({state: "wet", received: true, active: true, age_s: 12, expires_in_s: 30}));
        const previous = view.result.current;
        expect(previous?.state).toBe("wet");
        for (const raw of [
            {data: "broken"}, wrap(null), wrap({}),
            wrap({state: "dry", received: "false"}),
            wrap({state: "dry", active: []}),
            wrap({state: "dry", onboard_wet: 1}),
            wrap({state: "dry", age_s: {bad: true}}),
            {data: '{"state":"dry","expires_in_s":1e999}'},
        ]) {
            emit(raw);
            expect(view.result.current).toBe(previous);
        }
        emit(wrap({state: "future_state", received: null, age_s: null, extra: "kept"}));
        expect(view.result.current).toEqual({state: "future_state", received: null, age_s: null, extra: "kept"});
        expect(view.result.current).not.toBe(previous);
        view.unmount();
        expect(socket.listeners.size).toBe(0);
        expect(socket.unsubscribe).toHaveBeenCalledOnce();
    });

    it("cleans up subscriptions through StrictMode and real remount", () => {
        const wrapper = ({children}: PropsWithChildren) => createElement(StrictMode, null, children);
        const first = renderHook(useExternalRainState, {wrapper});
        expect(socket.listeners.size).toBe(1);
        emit(wrap({state: "wet"}));
        expect(first.result.current?.state).toBe("wet");
        first.unmount();
        expect(socket.listeners.size).toBe(0);
        const second = renderHook(useExternalRainState, {wrapper});
        expect(second.result.current).toBeNull();
        expect(socket.listeners.size).toBe(1);
        emit(wrap({state: "dry"}));
        expect(second.result.current?.state).toBe("dry");
        second.unmount();
        expect(socket.listeners.size).toBe(0);
        expect(socket.unsubscribe).toHaveBeenCalled();
    });
});
