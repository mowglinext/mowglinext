import {act, renderHook} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import type {ArgsProps} from 'antd/es/notification/interface';
import {useMapBearing} from './useMapBearing';

const key = 'gui.map.display.bearing';
const advance = async (ms: number) => {await act(async () => {vi.advanceTimersByTime(ms); await Promise.resolve();});};
function options(config: Record<string, string> = {}) {
    return {config, setConfig: vi.fn<(values: Record<string, string>) => Promise<void>>().mockResolvedValue(undefined), notification: {error: vi.fn<(args: ArgsProps) => void>()}};
}
beforeEach(() => vi.useFakeTimers());
afterEach(() => vi.useRealTimers());

describe('map display bearing contract', () => {
    it.each(['0', '-35.5', ' 1e2 '])('loads complete finite preference %s', raw => {
        const opts = options({[key]: raw}); const {result} = renderHook(() => useMapBearing(opts));
        expect(result.current.bearing).toBe(Number(raw)); expect(opts.setConfig).not.toHaveBeenCalled();
    });
    it.each(['Infinity', 'NaN', '45deg', '', '1,2', '0x20', '1e999'])('rejects malformed %s without losing a usable bearing', raw => {
        const opts = options({[key]: '35'});
        const {result, rerender} = renderHook(({config}) => useMapBearing({...opts, config}), {initialProps: {config: opts.config}});
        rerender({config: {[key]: raw}}); expect(result.current.bearing).toBe(35);
        expect(opts.setConfig).not.toHaveBeenCalled();
    });
    it.each([NaN, Infinity, -Infinity])('ignores a nonfinite edit %s without an API write', async value => {
        const opts = options({[key]: '10'}); const {result} = renderHook(() => useMapBearing(opts));
        act(() => result.current.handleBearing(value)); await advance(500);
        expect(result.current.bearing).toBe(10); expect(opts.setConfig).not.toHaveBeenCalled();
    });
    it('preserves zero, normalisation, debounce and navigation flush exactly once', async () => {
        const opts = options(); const view = renderHook(() => useMapBearing(opts));
        act(() => view.result.current.handleBearing(270)); expect(view.result.current.bearing).toBe(-90);
        await advance(399); expect(opts.setConfig).not.toHaveBeenCalled();
        await advance(1); expect(opts.setConfig).toHaveBeenCalledExactlyOnceWith({[key]: '-90'});
        act(() => view.result.current.handleBearing(0)); view.unmount();
        await advance(1000); expect(opts.setConfig.mock.calls).toEqual([[{[key]: '-90'}], [{[key]: '0'}]]);
    });
    it('does not flush on rerender and uses the latest committed writer at the debounce deadline', async () => {
        const opts = options(); const second = vi.fn<(values: Record<string, string>) => Promise<void>>().mockResolvedValue(undefined);
        const view = renderHook(({setConfig}) => useMapBearing({...opts, setConfig}), {initialProps: {setConfig: opts.setConfig}});
        act(() => view.result.current.handleBearing(30)); await advance(100);
        view.rerender({setConfig: second}); await advance(299);
        expect(opts.setConfig).not.toHaveBeenCalled(); expect(second).not.toHaveBeenCalled();
        await advance(1); expect(second).toHaveBeenCalledExactlyOnceWith({[key]: '30'});
    });
    it('serializes overlapping edits, preserving the newer draft against old config echoes', async () => {
        const opts = options({[key]: '0'});
        let resolve: () => void = () => {};
        opts.setConfig.mockReturnValueOnce(new Promise<void>(r => {resolve = r;}));
        const view = renderHook(({config}) => useMapBearing({...opts, config}), {initialProps: {config: opts.config}});
        act(() => view.result.current.handleBearing(10)); await advance(400);
        act(() => view.result.current.handleBearing(20)); await advance(400);
        expect(opts.setConfig).toHaveBeenCalledTimes(1); expect(view.result.current.bearing).toBe(20);
        view.rerender({config: {[key]: '10'}}); expect(view.result.current.bearing).toBe(20);
        await act(async () => {resolve(); await Promise.resolve();});
        expect(opts.setConfig.mock.calls).toEqual([[{[key]: '10'}], [{[key]: '20'}]]);
        view.rerender({config: {[key]: '20'}}); expect(view.result.current.bearing).toBe(20);
    });
    it('reports a structured API failure once, keeps the failed draft and never automatically retries', async () => {
        const opts = options(); opts.setConfig.mockRejectedValue({error: {error: 'Storage unavailable'}});
        const view = renderHook(({config}) => useMapBearing({...opts, config}), {initialProps: {config: opts.config}});
        act(() => view.result.current.handleBearing(30)); await advance(400);
        expect(opts.notification.error).toHaveBeenCalledOnce();
        expect(opts.notification.error.mock.calls[0][0].description).toBe('Storage unavailable');
        view.rerender({config: {[key]: '5'}}); expect(view.result.current.bearing).toBe(30);
        view.unmount(); await advance(1000); expect(opts.setConfig).toHaveBeenCalledOnce();
    });
    it('suppresses obsolete failures and continues with the explicitly requested newer edit', async () => {
        const opts = options(); let reject: (error: Error) => void = () => {};
        opts.setConfig.mockReturnValueOnce(new Promise<void>((_resolve, r) => {reject = r;}));
        const {result} = renderHook(() => useMapBearing(opts));
        act(() => result.current.handleBearing(10)); await advance(400);
        act(() => result.current.handleBearing(20)); await advance(400);
        await act(async () => {reject(new Error('Old write failed')); await Promise.resolve();});
        expect(opts.setConfig).toHaveBeenCalledTimes(2); expect(result.current.bearing).toBe(20);
        expect(opts.notification.error).not.toHaveBeenCalled();
    });
    it('avoids notifying after an in-flight write fails following unmount', async () => {
        const opts = options(); let reject: (error: Error) => void = () => {};
        opts.setConfig.mockReturnValue(new Promise<void>((_resolve, r) => {reject = r;}));
        const view = renderHook(() => useMapBearing(opts));
        act(() => view.result.current.handleBearing(25)); await advance(400); view.unmount();
        await act(async () => {reject(new Error('Offline')); await Promise.resolve();});
        expect(opts.notification.error).not.toHaveBeenCalled(); expect(opts.setConfig).toHaveBeenCalledOnce();
    });
});
