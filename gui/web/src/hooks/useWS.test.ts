import {act, renderHook} from '@testing-library/react';
import {beforeEach, describe, expect, it, vi} from 'vitest';
import type {MultiplexStatus} from './multiplexedSocket';

const mock = vi.hoisted(() => ({
    statusListeners: new Set<(status: MultiplexStatus) => void>(),
    subscribe: vi.fn(),
}));

vi.mock('react-use-websocket', () => ({default: () => ({sendJsonMessage: vi.fn()})}));
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
        mock.subscribe.mockImplementation(() => () => {
            mock.statusListeners.forEach(listener => listener('closed'));
        });
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
