import {bridgeFixedWireFixture} from '../test/gnssBridgeFixedFixture';
import sanitizedMavros from '../test/fixtures/gnss-mavros-sanitized.json';
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

describe('Lot 25 stale/replayed GNSS regression', () => {
    it('does not resurrect Fixed when a reconnect repeats the same publication', () => {
        const hook = renderHook(useGnssStatusState); emit('gnssStatus', sample(10));
        act(() => mock.statuses.forEach(listener => listener('closed')));
        act(() => mock.statuses.forEach(listener => listener('open')));
        emit('gnssStatus', sample(10));
        expect(deriveGpsStatus(hook.result.current.status).percent).toBe(0);
        hook.unmount();
    });
    it('expires a frozen position identity even when other runtime fields/stamps are republished', () => {
        const hook = renderHook(useGnssStatusState);
        emit('gnssStatus', {...sample(10), position_observation_sequence: 42});
        for (let i = 0; i < 31; i++) {
            act(() => {vi.advanceTimersByTime(1000);});
            emit('gnssStatus', {...sample(11+i), position_observation_sequence: 42});
        }
        expect(deriveGpsStatus(hook.result.current.status).percent).toBe(0);
        hook.unmount();
    });
    it('expires diagnostic-only state without another message', () => {
        const hook = renderHook(useGnssStatus);
        emit('diagnostics', diagnostic); act(() => {vi.advanceTimersByTime(1000);});
        expect(hook.result.current.fix_valid).toBe(true);
        act(() => {vi.advanceTimersByTime(31000);});
        expect(hook.result.current.fix_valid).toBeUndefined(); hook.unmount();
    });
});

describe('Lot 25 source-owned identity and availability', () => {
    it('keeps valid fix/zero accuracy after the Go transport sanitizes unavailable optional channels', () => {
        const hook = renderHook(useGnssStatusState); emit('gnssStatus', sanitizedMavros);
        expect(deriveGpsStatus(hook.result.current.status).percent).toBe(100);
        expect(hook.result.current.status.horizontal_accuracy_m).toBe(0);
        expect(hook.result.current.status.vertical_accuracy_m).toBeUndefined(); hook.unmount();
    });
    it('requires a new position after No Fix, even when a navigation update changes the header', () => {
        const hook = renderHook(useGnssStatusState);
        emit('gnssStatus', {...sample(10), position_observation_sequence: 42});
        emit('gnssStatus', {...sample(11), position_observation_sequence: 42, fix_valid: false});
        emit('gnssStatus', {...sample(12), position_observation_sequence: 42});
        expect(deriveGpsStatus(hook.result.current.status).percent).toBe(0);
        emit('gnssStatus', {...sample(13), position_observation_sequence: 43});
        expect(deriveGpsStatus(hook.result.current.status).percent).toBe(100); hook.unmount();
    });
    it('does not let a regressed sequence reset a frozen observation lifetime', () => {
        const hook = renderHook(useGnssStatusState);
        emit('gnssStatus', {...sample(10), position_observation_sequence: 42});
        act(() => {vi.advanceTimersByTime(31000);});
        emit('gnssStatus', {...sample(11), position_observation_sequence: 41});
        emit('gnssStatus', {...sample(12), position_observation_sequence: 42});
        expect(deriveGpsStatus(hook.result.current.status).percent).toBe(0); hook.unmount();
    });
    it('replaces the entire status on backend changes, without carrying receiver/correction fields', () => {
        const hook = renderHook(useGnssStatusState);
        emit('gnssStatus', {...sanitizedMavros, receiver_model: 'Fictional receiver A'});
        emit('gnssStatus', {...sample(20), backend: 'universal', position_observation_sequence: 1, fix_type: 1});
        expect(hook.result.current.status.receiver_model).toBeUndefined();
        expect(hook.result.current.status.horizontal_accuracy_m).toBeUndefined();
        expect(deriveGpsStatus(hook.result.current.status).percent).toBe(0); hook.unmount();
    });
    it('does not carry a previous robot into a newly mounted GUI session', () => {
        const robotA = renderHook(useGnssStatus); emit('gnssStatus', sanitizedMavros);
        expect(deriveGpsStatus(robotA.result.current).percent).toBe(100); robotA.unmount();
        const robotB = renderHook(useGnssStatus);
        expect(robotB.result.current).toEqual({});
        emit('gnssStatus', {...sample(20), backend: 'universal', fix_valid: false});
        expect(deriveGpsStatus(robotB.result.current).percent).toBe(0); robotB.unmount();
    });
    it('cannot resurrect diagnostic-only data by reconnecting and replaying an older publication', () => {
        const hook = renderHook(useGnssStatus);
        emit('diagnostics', diagnostic); act(() => {vi.advanceTimersByTime(1000);});
        act(() => mock.statuses.forEach(listener => listener('closed')));
        act(() => mock.statuses.forEach(listener => listener('open')));
        emit('diagnostics', diagnostic); act(() => {vi.advanceTimersByTime(1000);});
        expect(hook.result.current).toEqual({}); hook.unmount();
    });
});

it('does not downgrade an established position sequence to header-only freshness', () => {
    const hook = renderHook(useGnssStatusState);
    emit('gnssStatus', {...sample(10), position_observation_sequence: 42});
    emit('gnssStatus', {...sample(11), position_observation_sequence: 0});
    expect(deriveGpsStatus(hook.result.current.status).percent).toBe(0);
    hook.unmount();
});

it('does not resurrect Fixed from a navigation-only update following dead reckoning', () => {
    const hook = renderHook(useGnssStatusState);
    emit('gnssStatus', {...sample(10), position_observation_sequence: 42});
    emit('gnssStatus', {...sample(11), position_observation_sequence: 42, fix_type: 4});
    emit('gnssStatus', {...sample(12), position_observation_sequence: 42});
    expect(deriveGpsStatus(hook.result.current.status).percent).toBe(0); hook.unmount();
});

it.each(['disconnect', 'malformed', 'silence'])('does not accept an unidentified legacy replay after %s', cause => {
    const hook = renderHook(useGnssStatusState);
    const unidentified = {fix_valid: true, fix_type: 3};
    emit('gnssStatus', unidentified);
    expect(deriveGpsStatus(hook.result.current.status).percent).toBe(100);
    if (cause === 'disconnect') {
        act(() => mock.statuses.forEach(listener => listener('closed')));
        act(() => mock.statuses.forEach(listener => listener('open')));
    } else if (cause === 'malformed') emit('gnssStatus', null);
    else act(() => {vi.advanceTimersByTime(31000);});
    expect(deriveGpsStatus(hook.result.current.status).percent).toBe(0);
    emit('gnssStatus', unidentified);
    expect(deriveGpsStatus(hook.result.current.status).percent).toBe(0);
    // A later, identifiable publication can establish delivery progress.
    emit('gnssStatus', sample(50));
    expect(deriveGpsStatus(hook.result.current.status).percent).toBe(100);
    hook.unmount();
});

it('does not accept an older stamped legacy publication after reconnect', () => {
    const hook = renderHook(useGnssStatusState);
    emit('gnssStatus', sample(10));
    act(() => mock.statuses.forEach(listener => listener('closed')));
    act(() => mock.statuses.forEach(listener => listener('open')));
    emit('gnssStatus', sample(9));
    expect(deriveGpsStatus(hook.result.current.status).percent).toBe(0);
    emit('gnssStatus', sample(10));
    expect(deriveGpsStatus(hook.result.current.status).percent).toBe(0);
    emit('gnssStatus', sample(11));
    expect(deriveGpsStatus(hook.result.current.status).percent).toBe(100); hook.unmount();
});

describe('source round-trip replay barriers', () => {
    const frame = (backend: string | undefined, sequence: number, valid = true) => ({...sample(100 + sequence), ...(backend === undefined ? {} : {backend}),
        position_observation_sequence: sequence, fix_valid: valid, fix_type: valid ? 3 : 0});
    it.each([false, true])('does not restore known A42 after B valid=%s', valid => {
        const hook = renderHook(useGnssStatusState);
        emit('gnssStatus', frame('universal', 42));
        emit('gnssStatus', frame('mavros_gps1', 1, valid));
        emit('gnssStatus', frame('universal', 42));
        expect(deriveGpsStatus(hook.result.current.status).percent).toBe(0);
        emit('gnssStatus', frame('universal', 43));
        expect(deriveGpsStatus(hook.result.current.status).percent).toBe(100); hook.unmount();
    });
    it('accepts direct A progress after B No Fix without an intervening replay', () => {
        const hook = renderHook(useGnssStatusState);
        emit('gnssStatus', frame('universal', 42));
        emit('gnssStatus', frame('mavros_gps1', 1, false));
        emit('gnssStatus', frame('universal', 43));
        expect(deriveGpsStatus(hook.result.current.status).percent).toBe(100); hook.unmount();
    });
    it('rejects A replay even after B has established its own progress', () => {
        const hook = renderHook(useGnssStatusState);
        emit('gnssStatus', frame('universal', 42));
        emit('gnssStatus', frame('mavros_gps1', 1));
        emit('gnssStatus', frame('mavros_gps1', 2));
        expect(deriveGpsStatus(hook.result.current.status).percent).toBe(100);
        emit('gnssStatus', frame('universal', 42));
        expect(deriveGpsStatus(hook.result.current.status).percent).toBe(0); hook.unmount();
    });
    it('does not recover A without identity after B No Fix', () => {
        const hook = renderHook(useGnssStatusState);
        emit('gnssStatus', frame('universal', 42));
        emit('gnssStatus', frame('mavros_gps1', 1, false));
        emit('gnssStatus', {backend: 'universal', fix_type: 3, fix_valid: true});
        expect(deriveGpsStatus(hook.result.current.status).percent).toBe(0); hook.unmount();
    });
    it('recovers on the same source with observation progress after No Fix', () => {
        const hook = renderHook(useGnssStatusState);
        emit('gnssStatus', frame('universal', 42));
        emit('gnssStatus', frame('universal', 42, false));
        emit('gnssStatus', frame('universal', 43));
        expect(deriveGpsStatus(hook.result.current.status).percent).toBe(100); hook.unmount();
    });
    it('retains A high-water mark across reconnect, regression and source switches', () => {
        const hook = renderHook(useGnssStatusState);
        emit('gnssStatus', frame('universal', 42));
        act(() => mock.statuses.forEach(listener => listener('closed')));
        act(() => mock.statuses.forEach(listener => listener('open')));
        emit('gnssStatus', frame('mavros_gps1', 1, false));
        for (const sequence of [41, 42]) {
            emit('gnssStatus', frame('universal', sequence));
            expect(deriveGpsStatus(hook.result.current.status).percent).toBe(0);
        }
        emit('gnssStatus', frame('universal', 43));
        expect(deriveGpsStatus(hook.result.current.status).percent).toBe(100); hook.unmount();
    });
    it('cannot bypass known source barriers by omitting backend', () => {
        const hook = renderHook(useGnssStatusState);
        emit('gnssStatus', frame('universal', 42));
        emit('gnssStatus', frame('mavros_gps1', 1, false));
        emit('gnssStatus', frame(undefined, 42));
        expect(deriveGpsStatus(hook.result.current.status).percent).toBe(0);
        emit('gnssStatus', frame('universal', 43));
        expect(deriveGpsStatus(hook.result.current.status).percent).toBe(100); hook.unmount();
    });
    it('retains per-source header barriers for legacy stamped producers', () => {
        const hook = renderHook(useGnssStatusState);
        emit('gnssStatus', {...sample(10), backend: 'universal'});
        emit('gnssStatus', {...sample(1), backend: 'mavros_gps1', fix_valid: false});
        for (const sec of [9, 10]) {
            emit('gnssStatus', {...sample(sec), backend: 'universal'});
            expect(deriveGpsStatus(hook.result.current.status).percent).toBe(0);
        }
        emit('gnssStatus', {...sample(11), backend: 'universal'});
        expect(deriveGpsStatus(hook.result.current.status).percent).toBe(100); hook.unmount();
    });
    it('does not erase A high-water after repeated switches through multiple sources', () => {
        const hook = renderHook(useGnssStatusState);
        emit('gnssStatus', frame('universal', 42));
        for (const backend of ['mavros_gps1', 'mavros_gps2', 'mavros_gps1']) {
            emit('gnssStatus', frame(backend, 1, false));
            emit('gnssStatus', frame('universal', 42));
            expect(deriveGpsStatus(hook.result.current.status).percent).toBe(0);
        }
        emit('gnssStatus', frame('universal', 43));
        expect(deriveGpsStatus(hook.result.current.status).percent).toBe(100); hook.unmount();
    });
    it('requires progress from a never-seen source after an established source', () => {
        const hook = renderHook(useGnssStatusState);
        emit('gnssStatus', frame('universal', 42));
        for (let i = 0; i < 2; i++) {
            emit('gnssStatus', frame('mavros_gps1', 1));
            expect(deriveGpsStatus(hook.result.current.status).percent).toBe(0);
        }
        emit('gnssStatus', frame('mavros_gps1', 2));
        expect(deriveGpsStatus(hook.result.current.status).percent).toBe(100); hook.unmount();
    });
});

it('retains initial named legacy reporting without pretending it proves acquisition freshness', () => {
    const hook = renderHook(useGnssStatusState);
    emit('gnssStatus', {backend: 'universal', fix_type: 3, fix_valid: true});
    expect(deriveGpsStatus(hook.result.current.status).percent).toBe(100);
    emit('gnssStatus', {backend: 'mavros_gps1', fix_type: 0, fix_valid: false});
    emit('gnssStatus', {backend: 'universal', fix_type: 3, fix_valid: true});
    expect(deriveGpsStatus(hook.result.current.status).percent).toBe(0); hook.unmount();
});

it('preserves expiry, No Fix and reconnect barriers for the actual 3D + Fixed classification', () => {
    const hook = renderHook(useGnssStatusState);
    const fixed = bridgeFixedWireFixture;
    const sequence = fixed.position_observation_sequence;
    emit('gnssStatus', fixed);
    expect(deriveGpsStatus(hook.result.current.status).percent).toBe(100);
    act(() => {vi.advanceTimersByTime(30000);});
    expect(deriveGpsStatus(hook.result.current.status).percent).toBe(0);
    emit('gnssStatus', fixed);
    expect(deriveGpsStatus(hook.result.current.status).percent).toBe(0);
    const progressed = {...fixed, position_observation_sequence: sequence + 1};
    emit('gnssStatus', progressed);
    expect(deriveGpsStatus(hook.result.current.status).percent).toBe(100);
    act(() => mock.statuses.forEach(listener => listener('closed')));
    act(() => mock.statuses.forEach(listener => listener('open')));
    emit('gnssStatus', progressed);
    expect(deriveGpsStatus(hook.result.current.status).percent).toBe(0);
    emit('gnssStatus', {...fixed, position_observation_sequence: sequence + 2});
    expect(deriveGpsStatus(hook.result.current.status).percent).toBe(100);
    emit('gnssStatus', {...fixed, fix_valid: false, position_observation_sequence: sequence + 3});
    expect(deriveGpsStatus(hook.result.current.status).percent).toBe(0);
    emit('gnssStatus', {...fixed, position_observation_sequence: sequence + 2});
    expect(deriveGpsStatus(hook.result.current.status).percent).toBe(0);
    emit('gnssStatus', {...fixed, position_observation_sequence: sequence + 4});
    expect(deriveGpsStatus(hook.result.current.status).percent).toBe(100);
    hook.unmount();
});
