import {StrictMode} from 'react';
import {act, renderHook} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import type {MultiplexStatus} from './multiplexedSocket';
import {wheelControllerFromDiagnostics} from '../utils/motorTelemetry';

const mock = vi.hoisted(() => ({
    listeners: new Map<string, Set<(raw: unknown) => void>>(),
    statuses: new Set<(state: MultiplexStatus) => void>(),
    subscribe: vi.fn(),
}));
vi.mock('react-use-websocket', () => ({default: () => ({sendJsonMessage: () => {}})}));
vi.mock('./multiplexedSocket.ts', () => ({
    getMultiplexedSocket: () => ({
        subscribe: mock.subscribe,
        onStatusChange: (listener: (state: MultiplexStatus) => void) => {
            mock.statuses.add(listener); return () => mock.statuses.delete(listener);
        },
    }),
    isMultiplexableSubscribeUri: (uri: string) => uri.startsWith('/api/mowglinext/subscribe/'),
    topicFromSubscribeUri: (uri: string) => uri.split('/').pop(),
}));
import {usePower} from './usePower';
import {useBladeTelemetry} from './useBladeTelemetry';
import {useWheelRpm} from './useWheelRpm';
import {useWheelTicks} from './useWheelTicks';
import {useDiagnostics} from './useDiagnostics';

function emit(topic: string, raw: unknown) {
    act(() => mock.listeners.get(topic)?.forEach(listener => listener(raw)));
}
function advance(ms: number) { act(() => { vi.advanceTimersByTime(ms); }); }
const blade = (sec: number) => ({stamp: {sec, nanosec: 0}, blade_status_stamp: {sec, nanosec: 0},
    mower_motor_rpm: 0, mower_esc_current: 0, mower_esc_temperature: 0});
const ticks = (sec: number, count = 0) => ({stamp: {sec, nanosec: 0}, wheel_tick_factor: 319.305,
    valid_wheels: 12, wheel_ticks_rl: count, wheel_ticks_rr: count, wheel_direction_rl: 1, wheel_direction_rr: 1});
const diagnostic = (sec: number, fields: Record<string, string> = {}) => ({header: {stamp: {sec, nanosec: 0}}, status: [{
    name: 'mowgli_mavros_bridge/vesc_left_wheel', level: 0, message: 'online', hardware_id: '',
    values: Object.entries({online: 'true', stale: 'false', age_ms: '100', current_a: '0', voltage_v: '24',
        temperature_valid: 'true', temperature_c: '0', rpm_valid: 'true', rpm_abs: '0', ...fields})
        .map(([key, value]) => ({key, value})),
}]});
beforeEach(() => {
    vi.useFakeTimers(); vi.setSystemTime(10000); mock.listeners.clear(); mock.statuses.clear(); mock.subscribe.mockReset();
    mock.subscribe.mockImplementation((topic: string, listener: (raw: unknown) => void) => {
        const set = mock.listeners.get(topic) ?? new Set(); mock.listeners.set(topic, set); set.add(listener);
        return () => { set.delete(listener); if (!set.size) mock.listeners.delete(topic); };
    });
});
afterEach(() => vi.useRealTimers());
describe('physical hooks with real subscription lifecycle', () => {
    it('validates sparse/partial power and removes readings on malformed frames or connection loss', () => {
        const hook = renderHook(usePower);
        emit('power', {v_battery: 26, charge_current: 0});
        expect(hook.result.current).toEqual({v_battery: 26, charge_current: 0});
        emit('power', {v_battery: '26', charge_current: NaN});
        expect(hook.result.current).toEqual({});
        emit('power', {v_battery: 24, charge_current: 0});
        act(() => mock.statuses.forEach(listener => listener('closed')));
        act(() => mock.statuses.forEach(listener => listener('open')));
        expect(hook.result.current).toEqual({});
        emit('power', {v_battery: 25, v_charge: NaN});
        expect(hook.result.current).toEqual({v_battery: 25});
        hook.unmount();
    });
    it.each(['mowgli', 'mavros', 'openmower'])('invalid blade frames cannot resurrect a stale identity for %s', backend => {
        const hook = renderHook(() => useBladeTelemetry(backend));
        emit('status', blade(10)); expect(hook.result.current.rpm).toBe(0);
        advance(6000); expect(hook.result.current.rpm).toBeNaN();
        emit('status', {blade_status_stamp: {sec: 10, nanosec: 'bad'}, mower_motor_rpm: 0});
        expect(hook.result.current.rpm).toBeNaN();
        emit('status', blade(10)); expect(hook.result.current.rpm).toBeNaN();
        emit('status', blade(16)); expect(hook.result.current.rpm).toBe(0);
        expect(hook.result.current.current).toBe(0); hook.unmount();
    });
    it('a missing blade stamp cannot reset the valid identity and make cached data new', () => {
        const hook = renderHook(() => useBladeTelemetry('openmower'));
        emit('status', blade(10)); advance(6000);
        emit('status', {mower_motor_rpm: 0}); expect(hook.result.current.rpm).toBeNaN();
        emit('status', blade(10)); expect(hook.result.current.rpm).toBeNaN(); hook.unmount();
    });
    it('an unavailable MAVROS temperature does not hide measured stopped RPM/current', () => {
        const hook = renderHook(() => useBladeTelemetry('mavros'));
        emit('status', {...blade(10), mower_esc_temperature: NaN});
        expect(hook.result.current.rpm).toBe(0); expect(hook.result.current.current).toBe(0);
        expect(hook.result.current.escTemp).toBeNaN(); expect(hook.result.current.motorTemp).toBeNaN(); hook.unmount();
    });
    it('keeps wheel acquisition continuity through malformed frames without renewing cached RPM', () => {
        const hook = renderHook(useWheelRpm);
        emit('ticks', ticks(10)); advance(1000); emit('ticks', ticks(11));
        expect(hook.result.current.rl).toBe(0);
        advance(5000); expect(hook.result.current.rl).toBeNaN();
        emit('ticks', {...ticks(11), wheel_ticks_rl: '0'});
        emit('ticks', ticks(11)); expect(hook.result.current.rl).toBeNaN();
        emit('ticks', ticks(16)); expect(hook.result.current.rl).toBeNaN();
        advance(1000); emit('ticks', ticks(17)); expect(hook.result.current.rl).toBe(0); hook.unmount();
    });
    it('validates raw wheel display values before calibration formatting', () => {
        const hook = renderHook(useWheelTicks);
        emit('ticks', ticks(10)); expect(hook.result.current.wheel_tick_factor).toBe(319.305);
        emit('ticks', {wheel_tick_factor: '319.305'}); expect(hook.result.current).toEqual({});
        emit('ticks', null); expect(hook.result.current).toEqual({}); hook.unmount();
    });
    it('does not mix browser time with negative/reset simulation stamps', () => {
        const hook = renderHook(useWheelRpm);
        emit('ticks', ticks(-10)); advance(1000); emit('ticks', ticks(-9));
        expect(hook.result.current.rl).toBe(0);
        emit('ticks', ticks(-20)); expect(hook.result.current.rl).toBeNaN();
        advance(1000); emit('ticks', ticks(-19)); expect(hook.result.current.rl).toBe(0); hook.unmount();
    });
    it('retains diagnostic receipt identity across invalid updates and cached publication replay', () => {
        const hook = renderHook(useDiagnostics);
        const frame = diagnostic(10);
        emit('diagnostics', frame); advance(1000);
        expect(wheelControllerFromDiagnostics(hook.result.current.diagnostics, 'left_wheel', Date.now())?.current).toBe(0);
        advance(5000);
        emit('diagnostics', {...frame, status: [{...frame.status[0], values: [{key: 'age_ms', value: 0}]}]}); advance(1000);
        expect(wheelControllerFromDiagnostics(hook.result.current.diagnostics, 'left_wheel', Date.now())).toBeUndefined();
        emit('diagnostics', frame); advance(1000);
        expect(wheelControllerFromDiagnostics(hook.result.current.diagnostics, 'left_wheel', Date.now())?.state).toBe('stale');
        emit('diagnostics', diagnostic(18, {age_ms: '10000'})); advance(1000);
        expect(wheelControllerFromDiagnostics(hook.result.current.diagnostics, 'left_wheel', Date.now())?.state).toBe('stale');
        emit('diagnostics', diagnostic(19)); advance(1000);
        expect(wheelControllerFromDiagnostics(hook.result.current.diagnostics, 'left_wheel', Date.now())?.current).toBe(0);
        hook.unmount();
    });
    it('ignores malformed diagnostic envelopes and keeps the existing throttle/cleanup', () => {
        const hook = renderHook(useDiagnostics);
        emit('diagnostics', null); emit('diagnostics', {status: 'bad'});
        expect(hook.result.current.diagnostics).toEqual({});
        emit('diagnostics', diagnostic(10));
        expect(hook.result.current.diagnostics).toEqual({}); advance(1000);
        expect(hook.result.current.diagnostics.status).toHaveLength(1);
        hook.unmount(); expect(vi.getTimerCount()).toBe(0);
    });
    it('uses only the four existing topics with one effective listener per hook through StrictMode', () => {
        const hook = renderHook(() => {
            usePower(); useBladeTelemetry('mavros'); useWheelRpm(); useDiagnostics();
        }, {wrapper: StrictMode});
        expect([...mock.listeners.keys()].sort()).toEqual(['diagnostics', 'power', 'status', 'ticks']);
        for (const listeners of mock.listeners.values()) expect(listeners.size).toBe(1);
        const calls = mock.subscribe.mock.calls.length; hook.rerender();
        expect(mock.subscribe).toHaveBeenCalledTimes(calls);
        hook.unmount(); expect(mock.listeners.size).toBe(0); expect(mock.statuses.size).toBe(0);
        expect(vi.getTimerCount()).toBe(0);
    });
});
