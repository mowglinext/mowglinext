import {StrictMode, useEffect} from "react";
import {act, renderHook} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import type {Options} from 'react-use-websocket';
import type {MultiplexStatus} from './multiplexedSocket';

const mock = vi.hoisted(() => {
    const dedicated: {uri: string | null; options: Options} = {uri: null, options: {}};
    return {
        dedicated,
        statusListeners: new Set<(status: MultiplexStatus) => void>(),
        subscribe: vi.fn(),
        sendJsonMessage: vi.fn(),
        listeners: new Map<string, (data: unknown, first?: boolean) => void>(),
    };
});

vi.mock('react-use-websocket', () => ({default: (uri: string | null, options: Options) => {
    mock.dedicated = {uri, options};
    return {sendJsonMessage: mock.sendJsonMessage};
}}));

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
    it('does not queue outbound commands for a later connection', () => {
        const {result} = renderHook(() => useWS(vi.fn(), vi.fn(), vi.fn()));
        result.current.sendJsonMessage({twist: {}});
        expect(mock.sendJsonMessage).toHaveBeenCalledWith({twist: {}}, false);
        result.current.sendJsonMessage({twist: {linear: 1}}, true);
        expect(mock.sendJsonMessage).toHaveBeenLastCalledWith({twist: {linear: 1}}, false);
    });
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

const base64 = (bytes: Uint8Array) => btoa(String.fromCharCode(...bytes));
const message = (data: unknown) => act(() => mock.dedicated.options.onMessage?.(new MessageEvent<unknown>('message', {data})));
const open = () => act(() => mock.dedicated.options.onOpen?.(new Event('open')));

describe('useWS dedicated base64 stream', () => {
    beforeEach(() => {
        mock.subscribe.mockClear();
        mock.sendJsonMessage.mockClear();
    });
    afterEach(() => vi.restoreAllMocks());

    function stream() {
        const onData = vi.fn();
        const onError = vi.fn();
        const onInfo = vi.fn();
        const hook = renderHook(() => useWS<string>(onError, onInfo, onData));
        act(() => hook.result.current.start('/api/containers/fixture/logs'));
        return {...hook, onData, onError, onInfo};
    }

    it('decodes text on a dedicated socket and preserves send/start/stop and reconnect options', () => {
        const {result, onData, onInfo} = stream();
        expect(mock.dedicated.uri).toMatch(/\/api\/containers\/fixture\/logs$/);
        expect(mock.subscribe).not.toHaveBeenCalled();
        expect(mock.dedicated.options.share).toBe(true);
        expect(mock.dedicated.options.shouldReconnect?.(new CloseEvent('close'))).toBe(true);
        expect(mock.dedicated.options.reconnectAttempts).toBe(Infinity);
        const interval = mock.dedicated.options.reconnectInterval;
        expect(typeof interval).toBe('function');
        if (typeof interval !== 'function') throw new Error('Missing reconnect policy');
        expect([0, 1, 5, 20].map(interval)).toEqual([1000, 2000, 30000, 30000]);
        open();
        expect(onInfo).toHaveBeenCalledExactlyOnceWith('Stream connected');
        message(btoa('first line'));
        message(btoa('next line'));
        expect(onData.mock.calls).toEqual([['first line', true], ['next line', false]]);
        expect(typeof result.current.sendJsonMessage).toBe('function');
        result.current.sendJsonMessage({fixture: 'command'});
        expect(mock.sendJsonMessage).toHaveBeenCalledExactlyOnceWith({fixture: 'command'}, false);
        act(() => result.current.stop());
        expect(mock.dedicated.uri).toBeNull();
    });

    it('buffers split UTF-8 characters without consuming first until text is produced', () => {
        const {onData} = stream();
        const bytes = new TextEncoder().encode('🌿 météo µs');
        message(base64(bytes.slice(0, 2)));
        expect(onData).not.toHaveBeenCalled();
        message(base64(bytes.slice(2)));
        expect(onData).toHaveBeenCalledExactlyOnceWith('🌿 météo µs', true);
        message(btoa('next'));
        expect(onData).toHaveBeenLastCalledWith('next', false);
    });

    it('drops invalid base64, retains first and buffered UTF-8, and recovers', () => {
        const warn = vi.spyOn(console, 'warn');
        const {onData, onError} = stream();
        message(base64(new Uint8Array([0xc2])));
        message('not base64!');
        expect(onData).not.toHaveBeenCalled();
        expect(warn).toHaveBeenCalledOnce();
        message(base64(new Uint8Array([0xb5])));
        expect(onData).toHaveBeenCalledExactlyOnceWith('µ', true);
        expect(onError).not.toHaveBeenCalled();
    });

    it.each([
        ['Blob', () => new Blob(['text'])],
        ['ArrayBuffer', () => new ArrayBuffer(4)],
        ['synthetic number', () => 1234],
    ])('drops %s without coercion or consuming first, then accepts text', (_name, payload) => {
        const warn = vi.spyOn(console, 'warn');
        const {onData, onError} = stream();
        message(payload());
        expect(onData).not.toHaveBeenCalled();
        expect(warn).toHaveBeenCalledExactlyOnceWith('useWS: dropping non-base64 frame',
            {uri: mock.dedicated.uri}, expect.any(Error));
        message(btoa('recovered'));
        expect(onData).toHaveBeenCalledExactlyOnceWith('recovered', true);
        expect(onError).not.toHaveBeenCalled();
    });

    it('warns once per start, including across reconnect, and continues receiving', () => {
        const warn = vi.spyOn(console, 'warn');
        const {result, onData} = stream();
        message('!');
        message(new Blob());
        message(new ArrayBuffer(2));
        open();
        message('!');
        expect(warn).toHaveBeenCalledOnce();
        message(btoa('first stream'));
        expect(onData).toHaveBeenCalledExactlyOnceWith('first stream', true);
        act(() => result.current.start('/api/containers/other/logs'));
        message('!');
        expect(warn).toHaveBeenCalledTimes(2);
        message(btoa('second stream'));
        expect(onData).toHaveBeenLastCalledWith('second stream', true);
        expect(mock.subscribe).not.toHaveBeenCalled();
    });

    it('resets partial decoder bytes on open without resetting an already consumed first flag', () => {
        const {onData} = stream();
        message(base64(new Uint8Array([0xc2])));
        open();
        message(btoa('fresh'));
        expect(onData).toHaveBeenCalledExactlyOnceWith('fresh', true);
        message(base64(new Uint8Array([0xc2])));
        open();
        message(btoa('reconnected'));
        expect(onData).toHaveBeenLastCalledWith('reconnected', false);
    });

    it('resets partial decoder bytes on a new start', () => {
        const {result, onData} = stream();
        message(base64(new Uint8Array([0xc2])));
        act(() => result.current.start('/api/containers/other/logs'));
        message(btoa('new stream'));
        expect(onData).toHaveBeenCalledExactlyOnceWith('new stream', true);
    });

    it('uses the latest callbacks after rerender with a stable API', () => {
        const previous = {onData: vi.fn(), onInfo: vi.fn(), onError: vi.fn()};
        const next = {onData: vi.fn(), onInfo: vi.fn(), onError: vi.fn()};
        const {result, rerender} = renderHook(({onData, onInfo, onError}) => useWS<string>(onError, onInfo, onData),
            {initialProps: previous});
        const api = result.current;
        act(() => api.start('/api/containers/fixture/logs'));
        rerender(next);
        expect(result.current).toBe(api);
        open();
        message(btoa('latest'));
        act(() => mock.dedicated.options.onError?.(new Event('error')));
        act(() => mock.dedicated.options.onClose?.(new CloseEvent('close')));
        expect(next.onData).toHaveBeenCalledExactlyOnceWith('latest', true);
        expect(next.onInfo).toHaveBeenCalledExactlyOnceWith('Stream connected');
        expect(next.onError.mock.calls).toEqual([[new Error('Stream error')], [new Error('Stream closed')]]);
        for (const callback of Object.values(previous)) expect(callback).not.toHaveBeenCalled();
        expect(mock.subscribe).not.toHaveBeenCalled();
    });
});
