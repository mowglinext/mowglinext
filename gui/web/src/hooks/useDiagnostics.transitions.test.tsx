import {act, cleanup, renderHook} from '@testing-library/react';
import {StrictMode, useCallback, useLayoutEffect, useRef} from 'react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import {useDiagnostics, type DiagnosticStatus} from './useDiagnostics';
import {deriveDiagnosticHealth} from '../utils/diagnosticHealth';
import {healthy} from '../test/diagnosticHealthFixture';

const transport = vi.hoisted(() => ({listeners: new Set<(raw: unknown) => void>(), errors: new Set<() => void>(),
    start: vi.fn(), stop: vi.fn()}));
// Only transport is mocked. Commit callbacks and subscription lifetimes mirror
// useWS, including StrictMode; no diagnostic admission/projection is simulated.
vi.mock('./useWS.ts', () => ({
    useWS(error: () => void, _open: unknown, receive: (raw: unknown) => void) {
        const committed = useRef({receive, error});
        useLayoutEffect(() => { committed.current = {receive, error}; }, [receive, error]);
        const listener = useCallback((raw: unknown) => committed.current.receive(raw), []);
        const onError = useCallback(() => committed.current.error(), []);
        const start = useCallback((uri: string) => {
            transport.start(uri); transport.listeners.add(listener); transport.errors.add(onError);
        }, [listener, onError]);
        const stop = useCallback(() => {
            transport.stop(); transport.listeners.delete(listener); transport.errors.delete(onError);
        }, [listener, onError]);
        return {start, stop};
    },
}));
const context = JSON.stringify(['endpoint-A', 'robot-A', 'direct']);
const entry = (level: number, name = 'state', hardware_id = 'source'): DiagnosticStatus => ({
    name, hardware_id, level, message: `level ${level}`, values: [],
});
const frame = (sec: number | undefined, ...status: DiagnosticStatus[]) => ({
    ...(sec === undefined ? {} : {header: {stamp: {sec, nanosec: 0}}}), status,
});
const emit = (raw: unknown) => act(() => { for (const listener of transport.listeners) listener(raw); });
const flush = () => act(() => { vi.advanceTimersByTime(1000); });
const mount = () => renderHook(() => useDiagnostics(context));
const health = (state: ReturnType<typeof useDiagnostics>) => deriveDiagnosticHealth({...healthy(),
    diagnostics: state.diagnostics.status ?? [], nowMs: Date.now(), nowMonotonic: performance.now(), diagnosticsDisconnectedAt: state.disconnectedAt});

beforeEach(() => {
    vi.useFakeTimers(); vi.setSystemTime(100000);
    transport.start.mockClear(); transport.stop.mockClear();
});
afterEach(() => {
    cleanup();
    expect(transport.listeners.size).toBe(0); expect(transport.errors.size).toBe(0);
    expect(vi.getTimerCount()).toBe(0);
    vi.useRealTimers();
});

describe('02A: incremental observations before snapshot coalescence, without visual retention', () => {
    it.each([
        {baseline: 0, sequence: [0, 1, 0], observed: [1, 0]},
        {baseline: 1, sequence: [1, 0, 1], observed: [0, 1]},
        {baseline: 0, sequence: [2, 0], observed: [2, 0]},
    ])('retains $sequence within one window independently of the current snapshot', ({baseline, sequence, observed}) => {
        const hook = mount();
        emit(frame(10, entry(baseline))); flush();
        const before = hook.result.current;
        const cursor = before.readTransitions().cursor;
        sequence.forEach((level, i) => emit(frame(11 + i, entry(level))));
        expect(hook.result.current).toBe(before); // No event-triggered React update.
        const read = before.readTransitions(cursor);
        expect(read.transitions.map(e => e.level)).toEqual(observed);
        expect(read.transitions.map(e => e.order)).toEqual(observed.map((_, i) => cursor.order + i + 1));
        expect(read.missed).toBe(0);
        expect(before.readTransitions(read.cursor).transitions).toEqual([]);
        expect(vi.getTimerCount()).toBe(1);
        flush();
        expect(hook.result.current.diagnostics.status?.[0].level).toBe(sequence[sequence.length - 1]);
        expect(hook.result.current.readTransitions).toBe(before.readTransitions);
        expect(hook.result.current.readTransitions(read.cursor).transitions).toEqual([]);
    });

    it('does not add renders, timers, notifications or retained records for a repeated measured level', () => {
        let renders = 0;
        const hook = renderHook(() => { renders++; return useDiagnostics(context); });
        emit(frame(1, entry(1))); flush();
        const before = hook.result.current.readTransitions();
        const count = renders;
        act(() => {
            for (let i = 0; i < 1000; i++) {
                for (const listener of transport.listeners) listener(frame(2 + i, {...entry(1), values: [{key: 'angle', value: `${i}.1234`}]}));
            }
        });
        expect(renders).toBe(count);
        expect(vi.getTimerCount()).toBe(1);
        const after = hook.result.current.readTransitions();
        expect(after.cursor).toEqual(before.cursor);
        expect(after.transitions[0]).toBe(before.transitions[0]);
        flush();
        expect(renders).toBe(count + 1);
        expect(hook.result.current.diagnostics.status?.[0].values[0].value).toBe('999.1234');
        expect(vi.getTimerCount()).toBe(0);
        expect(transport.start).toHaveBeenCalledTimes(1);
    });

    it('preserves the exact current snapshot with memory enabled or disabled for all provenance cases', () => {
        const hook = renderHook(() => ({enabled: useDiagnostics(context), disabled: useDiagnostics()}));
        const inputs: unknown[] = [frame(10, entry(1)), frame(10, entry(0)), frame(9, entry(0)),
            frame(undefined, entry(2)), frame(11, entry(0)), frame(1, entry(1, 'other')),
            {status: [{...entry(2), level: 'invalid'}]}, null, frame(12, entry(3)), frame(13, entry(0))];
        for (const raw of inputs) {
            emit(raw); flush();
            expect(hook.result.current.enabled.diagnostics).toEqual(hook.result.current.disabled.diagnostics);
            expect(health(hook.result.current.enabled)).toEqual(health(hook.result.current.disabled));
        }
        act(() => { for (const error of transport.errors) error(); });
        expect(hook.result.current.enabled.diagnostics).toEqual(hook.result.current.disabled.diagnostics);
        expect(hook.result.current.enabled.disconnectedAt).toBe(hook.result.current.disabled.disconnectedAt);
        expect(hook.result.current.disabled.readTransitions()).toMatchObject({active: false, transitions: []});
    });

    it('records equal stamps with changed levels without renewing their existing receipt evidence', () => {
        const hook = mount();
        emit(frame(10, entry(0))); flush();
        const at = hook.result.current.diagnostics.status?.[0].receivedAt;
        emit(frame(10, entry(1))); emit(frame(10, entry(0))); flush();
        expect(hook.result.current.readTransitions().transitions.map(e => [e.level, e.publicationStamp])).toEqual([[1, '10:0'], [0, '10:0']]);
        expect(hook.result.current.diagnostics.status?.[0].receivedAt).toBe(at);
        expect(hook.result.current.diagnostics.status?.[0].level).toBe(0);
    });

    it('does not certify a regressive OK as resolution, and keeps legacy and historical-stamp admission', () => {
        const hook = mount();
        emit(frame(10, entry(1))); flush();
        emit(frame(9, entry(0))); flush();
        expect(hook.result.current.readTransitions().transitions.slice(-1)[0]).toMatchObject({level: 0, healthBlocked: true});
        expect(health(hook.result.current).checks.find(c => c.id === 'ros')?.level).toBe('unknown');
        emit(frame(11, entry(0))); flush();
        expect(health(hook.result.current).checks.find(c => c.id === 'ros')?.level).toBe('ok');
        emit(frame(undefined, entry(1, 'legacy'))); emit(frame(undefined, entry(0, 'legacy')));
        emit(frame(1, entry(2, 'historical-first'))); flush();
        expect(hook.result.current.readTransitions().transitions.filter(e => e.name === 'legacy').map(e => e.publicationStamp)).toEqual([null, null]);
        expect(hook.result.current.diagnostics.status?.find(e => e.name === 'historical-first')?.healthBlocked).toBe(false);
    });

    it('keeps distinct sources ordered without changing the current name-keyed collision policy', () => {
        const hook = mount();
        emit(frame(1, entry(1, 'A', 'hardware-A'), entry(2, 'B', 'hardware-B')));
        emit(frame(2, entry(0, 'B', 'hardware-B')));
        emit(frame(3, entry(2, 'A', 'other-hardware')));
        expect(hook.result.current.readTransitions().transitions.map(e => [e.name, e.hardware_id, e.level]))
            .toEqual([['A', 'hardware-A', 1], ['B', 'hardware-B', 2], ['B', 'hardware-B', 0], ['A', 'other-hardware', 2]]);
        flush();
        expect(hook.result.current.diagnostics.status).toHaveLength(2);
        expect(hook.result.current.diagnostics.status?.find(e => e.name === 'A')?.healthBlocked).toBe(true);
    });

    it('reports saturation before the one-second timer without losing its final current state', () => {
        const hook = mount();
        emit(frame(1, entry(0))); flush();
        const cursor = hook.result.current.readTransitions().cursor;
        for (let i = 1; i <= 80; i++) emit(frame(i + 1, entry(i % 2)));
        const read = hook.result.current.readTransitions(cursor);
        expect(read.transitions).toHaveLength(64);
        expect(read.transitions.map(e => e.order)).toEqual(Array.from({length: 64}, (_, i) => i + 17));
        expect(read.missed).toBe(16); expect(read.evicted).toBe(16);
        flush();
        expect(hook.result.current.diagnostics.status?.[0].level).toBe(0);
    });

    it('never rejects a current payload solely because its identity is too large for the small memory', () => {
        const hook = mount();
        const raw = frame(1, entry(2, 'x'.repeat(5000)));
        emit(raw); flush();
        expect(hook.result.current.diagnostics.status?.[0].name).toBe(raw.status[0].name);
        expect(hook.result.current.readTransitions()).toMatchObject({transitions: [], omittedOversizedIdentity: 1, missed: 1});
    });

    it.each(['endpoint', 'robot', 'backend'])('clears memory when %s context changes, without resubscribing or resetting the snapshot', (part) => {
        const hook = renderHook(({scope}) => useDiagnostics(scope), {initialProps: {scope: context}});
        emit(frame(10, entry(1))); flush();
        const before = hook.result.current;
        const read = before.readTransitions();
        const parts = ['endpoint-A', 'robot-A', 'direct'];
        parts[['endpoint', 'robot', 'backend'].indexOf(part)] += '-changed';
        hook.rerender({scope: JSON.stringify(parts)});
        expect(hook.result.current.diagnostics).toBe(before.diagnostics);
        expect(hook.result.current.readTransitions(read.cursor)).toMatchObject({contextChanged: true, transitions: [], evicted: 0});
        emit(frame(11, entry(1)));
        expect(hook.result.current.readTransitions().transitions[0]).toMatchObject({previousLevel: null, level: 1, order: 1});
        expect(transport.start).toHaveBeenCalledTimes(1); expect(transport.stop).not.toHaveBeenCalled();
    });

    it('does not treat disconnect copies as observations in a new context or invent a disconnect transition', () => {
        const hook = renderHook(({scope}) => useDiagnostics(scope), {initialProps: {scope: context}});
        emit(frame(10, entry(1))); flush();
        const first = hook.result.current.readTransitions();
        act(() => { for (const error of transport.errors) error(); });
        emit(frame(11, entry(1)));
        expect(hook.result.current.readTransitions().cursor).toEqual(first.cursor);
        hook.rerender({scope: 'robot-B/direct'});
        act(() => { for (const error of transport.errors) error(); });
        expect(hook.result.current.readTransitions().transitions).toEqual([]);
        emit(frame(12, entry(1)));
        expect(hook.result.current.readTransitions().transitions).toHaveLength(1);
        expect(hook.result.current.readTransitions().transitions[0]).toMatchObject({level: 1, previousLevel: null});
    });

    it('clears on opt-out and unmount, cancels pending timers, and remounts with a fresh cursor', () => {
        const initialProps: {scope: string | undefined} = {scope: context};
        const hook = renderHook(({scope}) => useDiagnostics(scope), {initialProps});
        emit(frame(1, entry(2)));
        const read = hook.result.current.readTransitions();
        hook.rerender({scope: undefined});
        expect(hook.result.current.readTransitions(read.cursor)).toMatchObject({active: false, transitions: [], contextChanged: true});
        hook.rerender({scope: context});
        emit(frame(2, entry(1)));
        const readAfterUnmount = hook.result.current.readTransitions;
        hook.unmount();
        expect(vi.getTimerCount()).toBe(0);
        expect(readAfterUnmount()).toMatchObject({active: false, transitions: [], evicted: 0});
        flush();
        const remount = mount();
        expect(remount.result.current.diagnostics).toEqual({});
        expect(remount.result.current.readTransitions(read.cursor)).toMatchObject({active: true, contextChanged: true, transitions: []});
    });

    it('retains one active subscription under StrictMode and no duplicate observations', () => {
        const hook = renderHook(() => useDiagnostics(context), {wrapper: StrictMode});
        expect(transport.listeners.size).toBe(1);
        emit(frame(1, entry(1))); emit(frame(2, entry(0)));
        expect(hook.result.current.readTransitions().transitions.map(e => e.level)).toEqual([1, 0]);
        flush();
        expect(hook.result.current.diagnostics.status?.[0].level).toBe(0);
        hook.unmount();
        expect(transport.stop).toHaveBeenCalledTimes(2);
    });

    it('observes an ERROR immediately internally without delaying or fabricating the existing direct emergency path', () => {
        const hook = mount();
        emit(frame(1, entry(0))); flush();
        emit(frame(2, {...entry(2, 'Motors', 'mowgli/motors'), values: [{key: 'motor_temperature_c', value: '85'}]}));
        expect(hook.result.current.readTransitions().transitions.slice(-1)[0]?.level).toBe(2);
        expect(health(hook.result.current).checks.find(c => c.id === 'ros')?.level).toBe('ok');
        const emergency = deriveDiagnosticHealth({...healthy(), nowMs: Date.now(), nowMonotonic: performance.now(), diagnostics: hook.result.current.diagnostics.status ?? [],
            emergency: {active_emergency: true, latched_emergency: false}});
        expect(emergency.checks.find(c => c.id === 'emergency')?.level).toBe('critical');
        flush();
        expect(health(hook.result.current).checks.find(c => c.id === 'ros')?.level).toBe('critical');
    });
});
