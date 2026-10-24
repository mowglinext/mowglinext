import {act, cleanup, renderHook} from '@testing-library/react';
import {afterEach, beforeEach, expect, it, vi} from 'vitest';
import {useContainerRestart} from './useContainerRestart';
const notification = vi.hoisted(() => ({success: vi.fn(), warning: vi.fn<(arg: {message: string; description?: string}) => void>(), error: vi.fn<(arg: {message: string; description?: string}) => void>()}));
vi.mock('antd', () => ({App: {useApp: () => ({notification})}}));
class Socket {
    static instances: Socket[] = [];
    onmessage: (() => void) | null = null;
    onerror: (() => void) | null = null;
    close = vi.fn();
    constructor(readonly url: string) {Socket.instances.push(this);}
}
beforeEach(() => {vi.useFakeTimers(); vi.clearAllMocks(); Socket.instances = []; vi.stubGlobal('WebSocket', Socket);});
afterEach(() => {cleanup(); vi.useRealTimers(); vi.unstubAllGlobals();});
it('requires acknowledgement then stream receipt, prevents concurrent triggers and closes probe', async () => {
    const success = vi.fn(), action = vi.fn().mockResolvedValue(undefined);
    const {result} = renderHook(() => useContainerRestart({onSuccess: success}));
    let run: Promise<void> | undefined;
    await act(async () => {run = result.current.run(action); await Promise.resolve();});
    expect(result.current.pending).toBe(true); expect(notification.success).not.toHaveBeenCalled();
    await act(async () => {await result.current.run(action);}); expect(action).toHaveBeenCalledOnce();
    await act(async () => {Socket.instances[0].onmessage?.(); await run;});
    expect(success).toHaveBeenCalledOnce(); expect(notification.success).toHaveBeenCalledOnce();
    expect(Socket.instances[0].close).toHaveBeenCalledOnce(); expect(result.current.pending).toBe(false);
});
it('stream errors and timeout do not imply success or issue another command', async () => {
    const action = vi.fn().mockResolvedValue(undefined), success = vi.fn();
    const {result} = renderHook(() => useContainerRestart({timeoutMs: 1000, onSuccess: success}));
    let run: Promise<void> | undefined;
    await act(async () => {run = result.current.run(action); await Promise.resolve();});
    Socket.instances[0].onerror?.();
    await act(async () => {await vi.advanceTimersByTimeAsync(1000); await run;});
    expect(notification.warning).toHaveBeenCalledOnce(); expect(success).not.toHaveBeenCalled();
    expect(notification.success).not.toHaveBeenCalled(); expect(action).toHaveBeenCalledOnce(); expect(result.current.pending).toBe(false);
});
it.each([new Error('lost response'), {status: 500, error: {error: 'Docker rejected'}}, {message: 12}, null])('handles unknown rejections safely: %j', async error => {
    const {result} = renderHook(() => useContainerRestart());
    await act(async () => {await result.current.run(vi.fn<() => Promise<void>>().mockRejectedValue(error));});
    const toast = typeof error === 'object' && error !== null && 'status' in error ? notification.error : notification.warning;
    expect(toast).toHaveBeenCalledOnce(); expect(notification.success).not.toHaveBeenCalled();
    expect(Socket.instances).toHaveLength(0); expect(result.current.pending).toBe(false);
    expect(toast.mock.calls[0][0].description).toEqual(expect.any(String));
});
it('retains explicit no-probe mode', async () => {
    const {result} = renderHook(() => useContainerRestart({skipReadinessProbe: true}));
    await act(async () => {await result.current.run(() => Promise.resolve());});
    expect(Socket.instances).toHaveLength(0); expect(notification.success).toHaveBeenCalledOnce();
});
