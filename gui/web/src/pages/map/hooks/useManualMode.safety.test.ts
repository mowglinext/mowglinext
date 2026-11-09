import {act, renderHook} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import {useManualMode} from './useManualMode';
import type {TwistStamped} from '../../../types/ros';

describe('manual command safety', () => {
    beforeEach(() => vi.useFakeTimers());
    afterEach(() => vi.useRealTimers());

    const move = {type: 'move' as const, x: 0, y: 1, direction: null, distance: null};
    function setup(stop: () => Promise<void> = () => Promise.resolve()) {
        const send = vi.fn<(message: unknown, keep?: boolean) => void>();
        const stream = {sendJsonMessage: send, start: vi.fn()};
        const action = vi.fn((_name: string, params: Record<string, unknown>) => params.Command === 8 ? stop : () => Promise.resolve());
        const hook = renderHook(({stateName}) => useManualMode({mowerAction: action, joyStream: stream, stateName}), {initialProps: {stateName: 'MANUAL_MOWING'}});
        const begin = () => {
            // On the old implementation every move implicitly began a gesture.
            (hook.result.current as {handleJoyStart?: () => void}).handleJoyStart?.();
            hook.result.current.handleJoyMove(move);
        };
        return {...hook, send, action, begin};
    }
    const nonzero = (calls: unknown[][]) => calls.filter(([m]) => {
        const twist = (m as TwistStamped).twist;
        return twist?.linear?.x !== 0 || twist?.angular?.z !== 0;
    });

    it('cancels locally before a slow STOP and remains cancelled after refusal', async () => {
        let rejectStop!: (error: Error) => void;
        const stop = new Promise<void>((_resolve, reject) => { rejectStop = reject; });
        const h = setup(() => stop);
        act(() => h.begin());
        expect(nonzero(h.send.mock.calls)).toHaveLength(1);
        let pending!: Promise<void>;
        act(() => { pending = h.result.current.handleStopManualMode(); });
        const rejected = expect(pending).rejects.toThrow('STOP refused');
        const before = nonzero(h.send.mock.calls).length;
        act(() => { h.begin(); });
        act(() => { vi.advanceTimersByTime(1000); });
        expect(nonzero(h.send.mock.calls)).toHaveLength(before);
        expect(h.result.current.manualMode).toBe(false);
        await act(async () => { rejectStop(new Error('STOP refused')); await rejected; });
        act(() => { vi.advanceTimersByTime(1000); });
        expect(nonzero(h.send.mock.calls)).toHaveLength(before);
        h.unmount();
    });

    it('does not resurrect the previous twist on mode exit and return', () => {
        const h = setup();
        act(() => h.begin());
        h.rerender({stateName: 'IDLE'});
        const before = nonzero(h.send.mock.calls).length;
        act(() => { vi.advanceTimersByTime(1300); });
        h.rerender({stateName: 'MANUAL_MOWING'});
        act(() => { vi.advanceTimersByTime(1000); });
        expect(nonzero(h.send.mock.calls)).toHaveLength(before);
        act(() => { h.result.current.handleJoyMove(move); });
        expect(nonzero(h.send.mock.calls)).toHaveLength(before);
        act(() => h.begin());
        expect(nonzero(h.send.mock.calls)).toHaveLength(before + 1);
        h.unmount();
    });

    it('keeps a slow accepted STOP cancelled until an explicit new manual entry and gesture', async () => {
        let resolveStop!: () => void;
        const stop = new Promise<void>(resolve => { resolveStop = resolve; });
        const h = setup(() => stop);
        act(() => h.begin());
        let pending!: Promise<void>;
        act(() => { pending = h.result.current.handleStopManualMode(); });
        await act(async () => { await expect(h.result.current.handleManualMode()).rejects.toThrow('STOP request is still pending'); });
        const before = nonzero(h.send.mock.calls).length;
        act(() => { vi.advanceTimersByTime(1000); });
        expect(nonzero(h.send.mock.calls)).toHaveLength(before);
        await act(async () => { resolveStop(); await pending; });
        act(() => { h.begin(); vi.advanceTimersByTime(500); });
        expect(nonzero(h.send.mock.calls)).toHaveLength(before);
        await act(async () => { await h.result.current.handleManualMode(); });
        act(() => { vi.advanceTimersByTime(500); });
        expect(nonzero(h.send.mock.calls)).toHaveLength(before);
        act(() => h.begin());
        expect(nonzero(h.send.mock.calls)).toHaveLength(before + 1);
        h.unmount();
    });

    it.each(['MANUAL_MOWING', 'RECORDING'])('requires a fresh pointer gesture in %s and after release', stateName => {
        const h = setup();
        h.rerender({stateName});
        act(() => h.result.current.handleJoyMove(move));
        expect(nonzero(h.send.mock.calls)).toHaveLength(0);
        act(() => h.begin());
        expect(nonzero(h.send.mock.calls)).toHaveLength(1);
        act(() => h.result.current.handleJoyStop());
        act(() => { h.result.current.handleJoyMove(move); vi.advanceTimersByTime(500); });
        expect(nonzero(h.send.mock.calls)).toHaveLength(1);
        expect(h.send).toHaveBeenLastCalledWith(expect.objectContaining({twist: {linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}}), false);
        h.unmount();
    });

    it('does not rearm a stopped gesture when an older manual START response arrives', async () => {
        let resolveStart!: () => void;
        const start = new Promise<void>(resolve => { resolveStart = resolve; });
        const h = setup();
        h.action.mockImplementation((_name, params) => params.Command === 7 ? () => start : () => Promise.resolve());
        let starting!: Promise<void>;
        act(() => { starting = h.result.current.handleManualMode(); });
        await act(async () => { await h.result.current.handleStopManualMode(); });
        await act(async () => { resolveStart(); await starting; });
        act(() => { h.begin(); vi.advanceTimersByTime(500); });
        expect(h.result.current.manualMode).toBe(false);
        expect(nonzero(h.send.mock.calls)).toHaveLength(0);
        h.unmount();
    });

    it('clears motion on unmount and ignores late joystick callbacks', () => {
        const h = setup();
        act(() => h.begin());
        const callbacks = h.result.current;
        h.unmount();
        const before = nonzero(h.send.mock.calls).length;
        act(() => { callbacks.handleJoyStart(); callbacks.handleJoyMove(move); vi.advanceTimersByTime(500); });
        expect(nonzero(h.send.mock.calls)).toHaveLength(before);
        expect(vi.getTimerCount()).toBe(0);
    });

    it('still sends STOP if the best-effort neutral send throws', async () => {
        const h = setup();
        act(() => h.begin());
        h.send.mockImplementation(() => { throw new Error('socket closed'); });
        await act(async () => { await h.result.current.handleStopManualMode(); });
        expect(h.action).toHaveBeenCalledWith('high_level_control', {Command: 8});
        expect(h.result.current.manualMode).toBe(false);
        expect(vi.getTimerCount()).toBe(0);
        h.unmount();
    });
});
