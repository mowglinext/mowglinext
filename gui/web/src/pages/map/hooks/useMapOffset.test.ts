import {describe, it, expect, vi, beforeEach, afterEach} from 'vitest';
import {renderHook, act} from '@testing-library/react';
import type {ArgsProps} from 'antd/es/notification/interface';
import {useMapOffset} from './useMapOffset.ts';

// `act` is handed an async callback so React flushes its microtask queue after
// the fake timers fire; the body itself has nothing of its own to await, hence
// the explicit `Promise.resolve()`.
const advanceTimers = async (ms: number) => {
    await act(async () => {
        vi.advanceTimersByTime(ms);
        await Promise.resolve();
    });
};

describe('useMapOffset', () => {
    let setConfig: ReturnType<typeof vi.fn<(values: Record<string, string>) => Promise<void>>>;
    let notification: {error: ReturnType<typeof vi.fn<(args: ArgsProps) => void>>};

    beforeEach(() => {
        setConfig = vi.fn().mockResolvedValue(undefined);
        notification = {error: vi.fn<(args: ArgsProps) => void>()};
        vi.useFakeTimers();
    });

    afterEach(() => {
        vi.useRealTimers();
    });

    it('initializes offsets from config', () => {
        const config = {'gui.map.offset.x': '1.5', 'gui.map.offset.y': '-2.3'};
        const {result} = renderHook(() =>
            useMapOffset({config, setConfig, notification})
        );
        expect(result.current.offsetX).toBe(1.5);
        expect(result.current.offsetY).toBe(-2.3);
    });

    it('defaults to 0 when config is empty', () => {
        const {result} = renderHook(() =>
            useMapOffset({config: {}, setConfig, notification})
        );
        expect(result.current.offsetX).toBe(0);
        expect(result.current.offsetY).toBe(0);
    });

    it('handleOffsetX updates immediately and debounces save', async () => {
        const config = {'gui.map.offset.x': '0', 'gui.map.offset.y': '0'};
        const {result} = renderHook(() =>
            useMapOffset({config, setConfig, notification})
        );

        act(() => {
            result.current.handleOffsetX(5.5);
        });
        expect(result.current.offsetX).toBe(5.5);
        expect(setConfig).not.toHaveBeenCalled();

        // After debounce timeout
        await advanceTimers(1100);
        expect(setConfig).toHaveBeenCalledWith({'gui.map.offset.x': '5.5'});
    });

    it('handleOffsetY updates immediately and debounces save', async () => {
        const config = {'gui.map.offset.x': '0', 'gui.map.offset.y': '0'};
        const {result} = renderHook(() =>
            useMapOffset({config, setConfig, notification})
        );

        act(() => {
            result.current.handleOffsetY(-3.2);
        });
        expect(result.current.offsetY).toBe(-3.2);

        await advanceTimers(1100);
        expect(setConfig).toHaveBeenCalledWith({'gui.map.offset.y': '-3.2'});
    });

    it('ignores NaN config values', () => {
        const config = {'gui.map.offset.x': 'invalid', 'gui.map.offset.y': 'abc'};
        const {result} = renderHook(() =>
            useMapOffset({config, setConfig, notification})
        );
        expect(result.current.offsetX).toBe(0);
        expect(result.current.offsetY).toBe(0);
    });

    it('debounces multiple rapid changes', async () => {
        const config = {'gui.map.offset.x': '0', 'gui.map.offset.y': '0'};
        const {result} = renderHook(() =>
            useMapOffset({config, setConfig, notification})
        );

        act(() => {
            result.current.handleOffsetX(1);
            result.current.handleOffsetX(2);
            result.current.handleOffsetX(3);
        });

        await advanceTimers(1100);

        // Only the last value should be saved
        expect(setConfig).toHaveBeenCalledTimes(1);
        expect(setConfig).toHaveBeenCalledWith({'gui.map.offset.x': '3'});
    });
});

it('retains independent X/Y drafts when another preference is saved', async () => {
    vi.useFakeTimers();
    try {
        const setConfig = vi.fn<(values: Record<string, string>) => Promise<void>>().mockResolvedValue(undefined);
        const notification = {error: vi.fn<(args: ArgsProps) => void>()};
        const view = renderHook(({config}) => useMapOffset({config, setConfig, notification}), {initialProps: {config: {'gui.map.offset.x': '1', 'gui.map.offset.y': '2'}}});
        act(() => {view.result.current.handleOffsetX(5); view.result.current.handleOffsetY(6);});
        view.rerender({config: {'gui.map.offset.x': '1', 'gui.map.offset.y': '3'}});
        expect(view.result.current.offsetX).toBe(5); expect(view.result.current.offsetY).toBe(6);
        await advanceTimers(1000);
        expect(setConfig.mock.calls).toEqual([[{'gui.map.offset.x': '5'}], [{'gui.map.offset.y': '6'}]]);
        view.unmount();
    } finally {vi.useRealTimers();}
});

it('flushes independent instances on navigation once, without delayed error notifications', async () => {
    vi.useFakeTimers();
    try {
        const setConfig = vi.fn<(values: Record<string, string>) => Promise<void>>().mockRejectedValue({error: {error: 'Offline'}});
        const notification = {error: vi.fn<(args: ArgsProps) => void>()};
        const a = renderHook(() => useMapOffset({config: {}, setConfig, notification}));
        const b = renderHook(() => useMapOffset({config: {}, setConfig, notification}));
        act(() => {a.result.current.handleOffsetX(1); b.result.current.handleOffsetX(2);});
        a.unmount(); b.unmount(); await advanceTimers(2000);
        expect(setConfig.mock.calls).toEqual([[{'gui.map.offset.x': '1'}], [{'gui.map.offset.x': '2'}]]);
        expect(notification.error).not.toHaveBeenCalled();
    } finally {vi.useRealTimers();}
});

it('rejects malformed stored offsets and nonfinite edits while preserving numeric zero as a preference', async () => {
    vi.useFakeTimers();
    try {
        const setConfig = vi.fn<(values: Record<string, string>) => Promise<void>>().mockResolvedValue(undefined);
        const notification = {error: vi.fn<(args: ArgsProps) => void>()};
        const view = renderHook(({config}) => useMapOffset({config, setConfig, notification}), {initialProps: {config: {'gui.map.offset.x': '12', 'gui.map.offset.y': '3'}}});
        view.rerender({config: {'gui.map.offset.x': 'Infinity', 'gui.map.offset.y': '3m'}});
        expect(view.result.current.offsetX).toBe(12); expect(view.result.current.offsetY).toBe(3);
        act(() => {view.result.current.handleOffsetX(Infinity); view.result.current.handleOffsetY(NaN);});
        await advanceTimers(1000); expect(setConfig).not.toHaveBeenCalled();
        act(() => {view.result.current.handleOffsetX(0); view.result.current.handleOffsetY(0);});
        await advanceTimers(1000);
        expect(setConfig.mock.calls).toEqual([[{'gui.map.offset.x': '0'}], [{'gui.map.offset.y': '0'}]]);
        view.unmount();
    } finally {vi.useRealTimers();}
});
