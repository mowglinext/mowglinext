import {act, renderHook} from '@testing-library/react';
import {afterEach, beforeEach, expect, it, vi} from 'vitest';
import {useGnssRuntimeConfig} from './useGnssRuntimeConfig';
const mocks = vi.hoisted(() => ({api: {request: vi.fn<() => Promise<{data?: unknown; error?: unknown; ok?: boolean}>>()}}));
vi.mock('./useApi', () => ({useApi: () => mocks.api}));
const payload = {serial_device: {active_value: '/dev/example', source: 'config'}, serial_devices: [], active_serial_device_exists: false};
beforeEach(() => {vi.clearAllMocks(); vi.useFakeTimers({toFake: ['setInterval', 'clearInterval']}); mocks.api.request.mockResolvedValue({data: payload});});
afterEach(() => vi.useRealTimers());
const flush = async () => {await act(async () => {await Promise.resolve();});};
it('polls at five seconds, keeps a stable subscription and clears the timer', async () => {
    const hook = renderHook(useGnssRuntimeConfig); await flush();
    expect(hook.result.current.runtimeConfig).toEqual(payload);
    hook.rerender(); expect(mocks.api.request).toHaveBeenCalledTimes(1);
    await act(() => vi.advanceTimersByTimeAsync(4999)); expect(mocks.api.request).toHaveBeenCalledTimes(1);
    await act(() => vi.advanceTimersByTimeAsync(1)); expect(mocks.api.request).toHaveBeenCalledTimes(2);
    hook.unmount(); await vi.advanceTimersByTimeAsync(5000); expect(mocks.api.request).toHaveBeenCalledTimes(2);
});
it.each(['malformed', 'http', 'network'])('a %s refresh clears previous configuration and exposes an error', async failure => {
    const hook = renderHook(useGnssRuntimeConfig); await flush();
    if (failure === 'network') mocks.api.request.mockRejectedValue({error: {error: 'offline'}});
    else mocks.api.request.mockResolvedValue(failure === 'http' ? {ok: false, data: undefined} : {data: {serial_device: false}});
    await act(() => vi.advanceTimersByTimeAsync(5000));
    expect(hook.result.current.runtimeConfig).toBeNull(); expect(hook.result.current.error).toBeTruthy(); hook.unmount();
});
it('does not accept old or post-unmount responses', async () => {
    const finishes: Array<(response: {data: unknown}) => void> = [];
    mocks.api.request.mockImplementation(() => new Promise(resolve => {finishes.push(resolve);}));
    const hook = renderHook(useGnssRuntimeConfig);
    await act(() => vi.advanceTimersByTimeAsync(5000));
    await act(async () => {finishes[1]({data: payload}); await Promise.resolve();});
    await act(async () => {finishes[0]({data: {serial_device: {active_value: '/dev/old'}}}); await Promise.resolve();});
    expect(hook.result.current.runtimeConfig).toEqual(payload);
    await act(() => vi.advanceTimersByTimeAsync(5000)); hook.unmount();
    await act(async () => {finishes[2]({data: {serial_device: {active_value: '/dev/late'}}}); await Promise.resolve();});
    expect(hook.result.current.runtimeConfig).toEqual(payload);
});
