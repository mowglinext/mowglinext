import {StrictMode} from 'react';
import {act, renderHook} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import type {MultiplexStatus} from './multiplexedSocket';
import {deriveGpsStatus} from '../utils/gpsStatus';

const mock = vi.hoisted(() => ({listeners: new Map<string, Set<(raw: unknown) => void>>(),
    statuses: new Set<(state: MultiplexStatus) => void>(), subscribe: vi.fn()}));
vi.mock('react-use-websocket', () => ({default: () => ({sendJsonMessage: () => {}})}));
vi.mock('./multiplexedSocket.ts', () => ({
    getMultiplexedSocket: () => ({subscribe: mock.subscribe,
        onStatusChange: (listener: (state: MultiplexStatus) => void) => {
            mock.statuses.add(listener); return () => mock.statuses.delete(listener);
        }}),
    isMultiplexableSubscribeUri: (uri: string) => uri.startsWith('/api/mowglinext/subscribe/'),
    topicFromSubscribeUri: (uri: string) => uri.split('/').pop(),
}));
import {useGnssStatus, useGnssStatusState} from './useGnssStatus';
function emit(topic: string, raw: unknown) {
    act(() => mock.listeners.get(topic)?.forEach(listener => listener(raw)));
}
const sample = (sec: number) => ({fix_type: 3, fix_valid: true, header: {stamp: {sec, nanosec: 0}, frame_id: 'gps'}});
const diagnostic = {header: {stamp: {sec: 1, nanosec: 0}}, status: [{name: 'GPS', level: 0, hardware_id: '', message: '', values: [{key: 'fix_status', value: '0'}]}]};
beforeEach(() => {
    vi.useFakeTimers(); mock.listeners.clear(); mock.statuses.clear(); mock.subscribe.mockReset();
    mock.subscribe.mockImplementation((topic: string, listener: (raw: unknown) => void) => {
        const listeners = mock.listeners.get(topic) ?? new Set(); mock.listeners.set(topic, listeners); listeners.add(listener);
        return () => { listeners.delete(listener); if (!listeners.size) mock.listeners.delete(topic); };
    });
});
afterEach(() => vi.useRealTimers());

describe('GNSS decoded websocket boundary and lifecycle', () => {
    it.each([{}, null, {fix_type: '3'}, {fix_valid: true, fix_type: 255}, {rtk_mode: 255}])(
        'an invalid update %j after a valid fix cannot retain it or renew receipt/identity', raw => {
            const hook = renderHook(useGnssStatusState);
            emit('gnssStatus', sample(10)); expect(deriveGpsStatus(hook.result.current.status).percent).toBe(100);
            const before = hook.result.current.telemetry;
            act(() => { vi.advanceTimersByTime(1000); }); emit('gnssStatus', raw);
            expect(deriveGpsStatus(hook.result.current.status).percent).toBe(0);
            expect(hook.result.current.telemetry.lastValidReceiptAt).toBe(before.lastValidReceiptAt);
            expect(hook.result.current.telemetry.stampChangedAt).toBe(before.stampChangedAt);
            expect(mock.subscribe).toHaveBeenCalledTimes(2); hook.unmount();
        });
    it('tracks receipt separately from unchanged publication identity, even through an invalid frame', () => {
        const hook = renderHook(useGnssStatusState);
        emit('gnssStatus', sample(10)); const at = hook.result.current.telemetry.stampChangedAt;
        act(() => { vi.advanceTimersByTime(1000); }); emit('gnssStatus', null); emit('gnssStatus', sample(10));
        expect(hook.result.current.telemetry.stampChangedAt).toBe(at);
        expect(hook.result.current.telemetry.lastValidReceiptAt).not.toBeNull();
        emit('gnssStatus', sample(11)); expect(hook.result.current.telemetry.stampIdentity).toBe('11:0'); hook.unmount();
    });
    it('allows legacy diagnostic-only producers but never promotes them to 3D/RTK', () => {
        const hook = renderHook(useGnssStatus);
        emit('diagnostics', diagnostic); act(() => { vi.advanceTimersByTime(1000); });
        expect(hook.result.current.fix_valid).toBe(true); expect(deriveGpsStatus(hook.result.current).percent).toBe(0);
        emit('gnssStatus', {fix_valid: false, fix_type: 3, rtk_mode: 3});
        expect(hook.result.current.fix_valid).toBe(false);
        emit('diagnostics', diagnostic); act(() => { vi.advanceTimersByTime(1000); });
        expect(deriveGpsStatus(hook.result.current).fixType).toBe('NO_FIX');
        emit('gnssStatus', {}); expect(hook.result.current.fix_valid).toBeUndefined(); hook.unmount();
    });
    it('preserves diagnostic-only fallback across reconnects without pretending typed data arrived', () => {
        const hook = renderHook(useGnssStatusState);
        emit('diagnostics', diagnostic); act(() => { vi.advanceTimersByTime(1000); });
        act(() => mock.statuses.forEach(listener => listener('closed')));
        act(() => mock.statuses.forEach(listener => listener('open')));
        emit('diagnostics', {...diagnostic, header: {stamp: {sec: 2, nanosec: 0}}});
        act(() => { vi.advanceTimersByTime(1000); });
        expect(hook.result.current.status.fix_valid).toBe(true);
        expect(hook.result.current.telemetry.lastValidReceiptAt).toBeNull();
        expect(hook.result.current.telemetry.hasReceivedFrame).toBe(false);
        emit('gnssStatus', null);
        act(() => mock.statuses.forEach(listener => listener('closed')));
        act(() => mock.statuses.forEach(listener => listener('open')));
        expect(hook.result.current.status).toEqual({}); hook.unmount();
    });
    it('connection-open and diagnostic fallback cannot resurrect a disconnected typed fix', () => {
        const hook = renderHook(useGnssStatusState); emit('gnssStatus', sample(10));
        const at = hook.result.current.telemetry.lastValidReceiptAt;
        act(() => mock.statuses.forEach(listener => listener('closed')));
        act(() => mock.statuses.forEach(listener => listener('open')));
        expect(hook.result.current.status).toEqual({}); expect(hook.result.current.telemetry.lastValidReceiptAt).toBe(at);
        emit('gnssStatus', sample(11)); expect(deriveGpsStatus(hook.result.current.status).percent).toBe(100); hook.unmount();
    });
    it('keeps subscriptions stable on status/diagnostic rerenders and cleans StrictMode subscriptions/timers', () => {
        const hook = renderHook(useGnssStatus, {wrapper: StrictMode});
        const count = mock.subscribe.mock.calls.length;
        emit('gnssStatus', sample(10)); hook.rerender(); emit('diagnostics', diagnostic);
        expect(mock.subscribe).toHaveBeenCalledTimes(count);
        expect(mock.listeners.get('gnssStatus')?.size).toBe(1); expect(mock.listeners.get('diagnostics')?.size).toBe(1);
        hook.unmount(); expect(mock.listeners.size).toBe(0); expect(mock.statuses.size).toBe(0);
        expect(vi.getTimerCount()).toBe(0);
    });
});
