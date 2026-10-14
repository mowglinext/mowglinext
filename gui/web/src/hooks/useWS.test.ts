import {StrictMode, useEffect} from "react";
import {act, renderHook} from '@testing-library/react';
import {beforeEach, describe, expect, it, vi} from 'vitest';
import type {MultiplexStatus} from './multiplexedSocket';

const mock = vi.hoisted(() => ({
    statusListeners: new Set<(status: MultiplexStatus) => void>(),
    subscribe: vi.fn(),
    sendJsonMessage: vi.fn(),
    listeners: new Map<string, (data: unknown, first?: boolean) => void>(),
}));

vi.mock('react-use-websocket', () => ({default: () => ({sendJsonMessage: mock.sendJsonMessage})}));
vi.mock('./multiplexedSocket.ts', () => ({
    getMultiplexedSocket: () => ({
        subscribe: mock.subscribe,
        onStatusChange: (listener: (status: MultiplexStatus) => void) => {
            mock.statusListeners.add(listener);
            return () => mock.statusListeners.delete(listener);
        },
    }),
    isMultiplexableSubscribeUri: (uri: string) => uri.startsWith('/api/mowglinext/subscribe/'),
    topicFromSubscribeUri: (uri: string) => uri.split('/').pop(),
}));

import {useWS} from './useWS';

describe('useWS multiplex teardown', () => {
    beforeEach(() => {
        mock.statusListeners.clear();
        mock.subscribe.mockReset();
        mock.listeners.clear();
        mock.subscribe.mockImplementation((topic: string, callback: (data: unknown, first?: boolean) => void) => {
            mock.listeners.set(topic, callback);
            return () => {
                mock.listeners.delete(topic);
                mock.statusListeners.forEach(listener => listener('closed'));
            };
        });
    });

    it('keeps its API stable and dispatches to updated callbacks', () => {
        const first = vi.fn();
        const next = vi.fn();
        const {result, rerender} = renderHook(({callback}) => useWS(vi.fn(), vi.fn(), callback), {initialProps: {callback: first}});
        const api = result.current;
        act(() => api.start('/api/mowglinext/subscribe/status'));
        rerender({callback: next});
        expect(result.current).toBe(api);
        act(() => mock.listeners.get('status')?.('updated', true));
        expect(first).not.toHaveBeenCalled();
        expect(next).toHaveBeenCalledExactlyOnceWith('updated', true);
        expect(mock.subscribe).toHaveBeenCalledOnce();
        act(() => api.stop());
    });

    it('replays StrictMode cleanly and only reconnects for an actual topic transition', () => {
        const onData = vi.fn();
        const onError = vi.fn();
        const {rerender, unmount} = renderHook(({topic}) => {
            const {start, stop} = useWS(onError, vi.fn(), onData);
            useEffect(() => {
                start(`/api/mowglinext/subscribe/${topic}`);
                return stop;
            }, [topic, start, stop]);
        }, {initialProps: {topic: 'status'}, wrapper: StrictMode});
        expect(mock.listeners.size).toBe(1);
        expect(mock.statusListeners.size).toBe(1);
        const starts = mock.subscribe.mock.calls.length;
        rerender({topic: 'status'});
        expect(mock.subscribe).toHaveBeenCalledTimes(starts);
        rerender({topic: 'map'});
        expect(mock.listeners.size).toBe(1);
        expect(mock.listeners.has('status')).toBe(false);
        expect(mock.listeners.has('map')).toBe(true);
        unmount();
        expect(mock.listeners.size).toBe(0);
        expect(mock.statusListeners.size).toBe(0);
        expect(onError).not.toHaveBeenCalled();
    });

    it('reports a connection drop but does not report intentional stop as an error', () => {
        const onError = vi.fn();
        const onInfo = vi.fn();
        const {result} = renderHook(() => useWS(onError, onInfo, vi.fn()));
        act(() => result.current.start('/api/mowglinext/subscribe/status'));
        act(() => mock.statusListeners.forEach(listener => listener('closed')));
        expect(onError).toHaveBeenCalledWith(new Error('Stream closed'));
        act(() => mock.statusListeners.forEach(listener => listener('open')));
        expect(onInfo).toHaveBeenCalledWith('Stream connected');
        onError.mockClear();
        act(() => result.current.stop());
        expect(onError).not.toHaveBeenCalled();
        expect(mock.statusListeners.size).toBe(0);
    });

    it('switches topics without reporting the previous subscription teardown as an error', () => {
        const onError = vi.fn();
        const {result} = renderHook(() => useWS(onError, vi.fn(), vi.fn()));
        act(() => result.current.start('/api/mowglinext/subscribe/status'));
        act(() => result.current.start('/api/mowglinext/subscribe/map'));
        expect(onError).not.toHaveBeenCalled();
        expect(mock.statusListeners.size).toBe(1);
        act(() => result.current.stop());
    });
});
