import {StrictMode} from 'react';
import {act, renderHook} from '@testing-library/react';
import {beforeEach, describe, expect, it, vi} from 'vitest';
import type {MultiplexStatus} from './multiplexedSocket';
import type {Emergency} from '../types/ros';
import {decodeEmergency, retainEmergency, hasEmergency} from '../utils/statusTelemetry';

const mock = vi.hoisted(() => ({
    listeners: new Map<string, Set<(data: unknown, first?: boolean) => void>>(),
    statusListeners: new Set<(status: MultiplexStatus) => void>(),
    subscribe: vi.fn(), send: vi.fn(),
}));
vi.mock('react-use-websocket', () => ({default: () => ({sendJsonMessage: mock.send})}));
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
import {useEmergency} from './useEmergency';
import {useStatus} from './useStatus';
import {useHighLevelStatus} from './useHighLevelStatus';
import {useValidatedTelemetry} from './useValidatedTelemetry';

function emit(topic: string, value: unknown) {
    act(() => mock.listeners.get(topic)?.forEach(listener => listener(value)));
}
function connection(status: MultiplexStatus) {
    act(() => mock.statusListeners.forEach(listener => listener(status)));
}
beforeEach(() => {
    mock.listeners.clear(); mock.statusListeners.clear(); mock.subscribe.mockReset(); mock.send.mockReset();
    mock.subscribe.mockImplementation((topic: string, listener: (data: unknown, first?: boolean) => void) => {
        const set = mock.listeners.get(topic) ?? new Set();
        mock.listeners.set(topic, set); set.add(listener);
        return () => { set.delete(listener); if (set.size === 0) mock.listeners.delete(topic); };
    });
});
describe('validated status hooks', () => {
    it('retains a reported emergency through malformed frames, sparse frames, disconnect and reconnect', () => {
        const {result} = renderHook(useEmergency);
        expect(result.current).toEqual({clearanceUnverified: false});
        emit('emergency', {active_emergency: true, latched_emergency: true, reason: 'STOP'});
        emit('emergency', {active_emergency: false, latched_emergency: 'false'});
        expect(result.current).toEqual({active_emergency: true, latched_emergency: true, clearanceUnverified: false});
        emit('emergency', {reason: 'partial'});
        expect(result.current.active_emergency).toBe(true);
        connection('closed'); connection('open');
        expect(result.current).toEqual({active_emergency: true, latched_emergency: true, clearanceUnverified: false});
        emit('emergency', {active_emergency: false});
        expect(result.current).toEqual({active_emergency: false, latched_emergency: true, clearanceUnverified: true});
        emit('emergency', {active_emergency: false, latched_emergency: false});
        expect(result.current).toEqual({active_emergency: false, latched_emergency: false, clearanceUnverified: true});
        expect(mock.send).not.toHaveBeenCalled();
    });
    it('does not acknowledge a replayed clear bit, including after reconnect or a larger publication stamp', () => {
        const {result} = renderHook(useEmergency);
        const cachedClear = {stamp: {sec: 10, nanosec: 0}, active_emergency: false, latched_emergency: false};
        emit('emergency', cachedClear);
        emit('emergency', {stamp: {sec: 11, nanosec: 0}, active_emergency: true, latched_emergency: true});
        connection('closed'); connection('open');
        emit('emergency', cachedClear);
        expect(hasEmergency(result.current, {})).toBe(true);
        expect(result.current.clearanceUnverified).toBe(true);
        // MAVROS stamps each projection's publication time: larger is not
        // proof of a newly acquired physical clear. Native lacks a producer epoch.
        emit('emergency', {...cachedClear, stamp: {sec: 12, nanosec: 0}});
        expect(hasEmergency(result.current, {})).toBe(true);
        expect(result.current.clearanceUnverified).toBe(true);
        // Legacy command consumers retain the latest valid message bits;
        // display history is separate and is never an invented firmware latch.
        expect(result.current.active_emergency).toBe(false);
        expect(result.current.latched_emergency).toBe(false);
        expect(mock.send).not.toHaveBeenCalled();
    });
    it('does not clear a previously observed unstamped high-level assertion from a cached false bit', () => {
        const {result} = renderHook(useHighLevelStatus);
        emit('highLevelStatus', {state: 0, emergency: true});
        emit('highLevelStatus', {state: 1, state_name: 'IDLE', emergency: false});
        expect(result.current.telemetry.assertions).toEqual({emergency: true});
        expect(hasEmergency({}, result.current.highLevelStatus)).toBe(true);
    });
    it('keeps frontend clearance metadata independent of untrusted extension fields', () => {
        const {result} = renderHook(useEmergency);
        emit('emergency', {active_emergency: false, latched_emergency: false, clearanceUnverified: 'not-a-boolean'});
        expect(result.current.clearanceUnverified).toBe(false);
    });
    it('drops healthy operational/firmware values on invalid frames and socket loss', () => {
        const {result} = renderHook(useStatus);
        emit('status', {mower_status: 255, firmware_compatible: true, is_charging: true});
        expect(result.current.mower_status).toBe(255);
        emit('status', {mower_status: 255, firmware_compatible: 'true'});
        expect(result.current).toEqual({});
        emit('status', {mower_status: 255, firmware_compatible: true});
        connection('closed'); connection('open');
        expect(result.current).toEqual({});
        emit('status', {mower_status: 0});
        expect(result.current).toEqual({mower_status: 0});
    });
    it('rejects unsupported state codes and restores status only on a valid subsequent frame', () => {
        const {result} = renderHook(useHighLevelStatus);
        emit('highLevelStatus', {state: 2, state_name: 'MOWING', emergency: true});
        emit('highLevelStatus', {state: 99, state_name: 'IDLE', emergency: false});
        expect(result.current.highLevelStatus).toEqual({emergency: true, clearanceUnverified: false});
        expect(result.current.telemetry.condition).toBe('unsupported');
        connection('closed'); connection('open');
        expect(result.current.telemetry.condition).toBe('disconnected');
        emit('highLevelStatus', {state: 1, state_name: 'IDLE', emergency: false});
        expect(result.current.highLevelStatus).toEqual({state: 1, state_name: 'IDLE', emergency: false, clearanceUnverified: true});
        expect(result.current.telemetry.condition).toBe('valid');
    });
    it('keeps receipt and producer identity times distinct across repeated/cached reconnect frames', () => {
        const clock = vi.spyOn(performance, 'now');
        clock.mockReturnValue(100);
        const {result} = renderHook(() => useValidatedTelemetry<Emergency>('emergency', {}, decodeEmergency, retainEmergency));
        const sample = {stamp: {sec: 123, nanosec: 0}, active_emergency: false, latched_emergency: false};
        emit('emergency', sample);
        expect(result.current.telemetry).toMatchObject({lastValidReceiptAt: 100, stampChangedAt: 100, stampIdentity: '123:0'});
        clock.mockReturnValue(200);
        emit('emergency', {...sample});
        expect(result.current.telemetry).toMatchObject({lastValidReceiptAt: 200, stampChangedAt: 100});
        connection('closed'); connection('open');
        expect(result.current.telemetry.condition).toBe('disconnected');
        clock.mockReturnValue(300);
        emit('emergency', sample);
        expect(result.current.telemetry).toMatchObject({lastValidReceiptAt: 300, stampChangedAt: 100});
        emit('emergency', {active_emergency: 'false'});
        expect(result.current.telemetry.lastValidReceiptAt).toBe(300);
        clock.mockReturnValue(400);
        emit('emergency', {...sample, stamp: {sec: 124, nanosec: 0}});
        expect(result.current.telemetry.stampChangedAt).toBe(400);
        clock.mockRestore();
    });
    it('does not treat unstamped high-level republication as acquisition', () => {
        const {result} = renderHook(useHighLevelStatus);
        emit('highLevelStatus', {state: 2, state_name: 'MOWING'});
        emit('highLevelStatus', {state: 2, state_name: 'MOWING'});
        expect(result.current.telemetry.stampIdentity).toBeNull();
        expect(result.current.telemetry.stampChangedAt).toBeNull();
    });
    it('avoids publishing another state for repeated rejected frames', () => {
        const {result} = renderHook(useHighLevelStatus);
        emit('highLevelStatus', null);
        const state = result.current.telemetry;
        emit('highLevelStatus', []);
        expect(result.current.telemetry).toBe(state);
    });
    it('keeps callback/start/stop identities and one effective subscription per hook through StrictMode', () => {
        const {result, rerender, unmount} = renderHook(() => {
            const emergency = useEmergency(); const status = useStatus(); const high = useHighLevelStatus();
            return {emergency, status, high};
        }, {wrapper: StrictMode});
        expect(mock.listeners.size).toBe(3);
        expect(mock.statusListeners.size).toBe(3);
        for (const listeners of mock.listeners.values()) expect(listeners.size).toBe(1);
        const calls = mock.subscribe.mock.calls.length;
        const {start, stop} = result.current.high;
        rerender();
        emit('highLevelStatus', {state: 3, state_name: 'RECORDING'});
        expect(result.current.high.start).toBe(start); expect(result.current.high.stop).toBe(stop);
        expect(mock.subscribe).toHaveBeenCalledTimes(calls);
        act(() => stop());
        expect(mock.listeners.has('highLevelStatus')).toBe(false);
        // Intentional pause retains the established legacy state snapshot.
        expect(result.current.high.highLevelStatus.state).toBe(3);
        act(() => start('/api/mowglinext/subscribe/highLevelStatus'));
        expect(mock.listeners.get('highLevelStatus')?.size).toBe(1);
        unmount();
        expect(mock.listeners.size).toBe(0); expect(mock.statusListeners.size).toBe(0);
        expect(mock.send).not.toHaveBeenCalled();
    });
});
