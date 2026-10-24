import {act, cleanup, render, renderHook} from '@testing-library/react';
import {StrictMode} from 'react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import {useDiagnostics, isDiagnosticStale} from './useDiagnostics';
import {useGnssStatus} from './useGnssStatus';
import {useMavrosDiagnostics} from './useMavrosDiagnostics';
import {deriveDiagnosticHealth} from '../utils/diagnosticHealth';
import {advanceReceptionWatches, missingReceptionSources} from '../utils/diagnosticAlertPresentation';
import {wheelControllerFromDiagnostics} from '../utils/motorTelemetry';
import {healthy} from '../test/diagnosticHealthFixture';
import {GnssStatusConstants as G} from '../types/ros';
import {DiagnosticAlertsPanel} from '../components/diagnostics/DiagnosticAlertsPanel';

const transport = vi.hoisted(() => ({listeners: new Map<string, Set<(raw: unknown) => void>>(),
    statuses: new Set<(status: string) => void>(), subscribe: vi.fn()}));
vi.mock('react-use-websocket', () => ({default: () => ({sendJsonMessage: () => {}})}));
vi.mock('./multiplexedSocket.ts', () => ({
    getMultiplexedSocket: () => ({subscribe: transport.subscribe,
        onStatusChange: (listener: (status: string) => void) => {
            transport.statuses.add(listener); return () => transport.statuses.delete(listener);
        }}),
    isMultiplexableSubscribeUri: (uri: string) => uri.startsWith('/api/mowglinext/subscribe/'),
    topicFromSubscribeUri: (uri: string) => uri.split('/').pop(),
}));
let monotonic: number;
const emit = (raw: unknown) => act(() => {transport.listeners.get('diagnostics')?.forEach(listener => listener(raw));});
const connection = (status: string) => act(() => {transport.statuses.forEach(listener => listener(status));});
const advance = (ms: number) => act(() => {monotonic += ms; vi.advanceTimersByTime(ms);});
const frame = (sec: number | undefined, level = 1, name = 'qualified/report', values: {key: string; value: string}[] = []) => ({
    ...(sec === undefined ? {} : {header: {stamp: {sec, nanosec: 0}}}),
    status: [{name, hardware_id: '', level, message: 'native diagnostic', values}],
});
const current = (state: ReturnType<typeof useDiagnostics>) => state.diagnostics.status![0];
const ros = (state: ReturnType<typeof useDiagnostics>) => deriveDiagnosticHealth({...healthy(), diagnostics: state.diagnostics.status ?? [],
    nowMs: Date.now(), nowMonotonic: monotonic, diagnosticsDisconnectedAt: state.disconnectedAt}).checks.find(c => c.id === 'ros')?.level;
beforeEach(() => {
    vi.useFakeTimers(); vi.setSystemTime(100000); monotonic = 5000;
    vi.spyOn(performance, 'now').mockImplementation(() => monotonic);
    transport.subscribe.mockReset();
    transport.subscribe.mockImplementation((topic: string, listener: (raw: unknown) => void) => {
        const listeners = transport.listeners.get(topic) ?? new Set();
        listeners.add(listener); transport.listeners.set(topic, listeners);
        return () => {listeners.delete(listener); if (!listeners.size) transport.listeners.delete(topic);};
    });
});
afterEach(() => {
    cleanup(); expect(transport.listeners.size).toBe(0); expect(transport.statuses.size).toBe(0);
    expect(vi.getTimerCount()).toBe(0); vi.restoreAllMocks(); vi.useRealTimers();
});

describe('diagnostic elapsed time independent from browser civil time', () => {
    it.each([1000, 9_000_000])('a wall-clock jump to %i without monotonic progress does not change freshness', wall => {
        const hook = renderHook(useDiagnostics); emit(frame(10)); advance(1000);
        expect(ros(hook.result.current)).toBe('warning');
        vi.setSystemTime(wall);
        expect(ros(hook.result.current)).toBe('warning');
        expect(current(hook.result.current)).toMatchObject({receivedAt: 100000, receivedMonotonic: 5000, healthProgressAt: 5000});
    });
    it('rolling civil time back into the old fresh window cannot rejuvenate an expired report', () => {
        const hook = renderHook(useDiagnostics); emit(frame(10)); advance(30001);
        expect(ros(hook.result.current)).toBe('unknown'); vi.setSystemTime(100001);
        expect(ros(hook.result.current)).toBe('unknown');
    });
    it('does not invent a monotonic receipt from civil time when local metadata is unavailable', () => {
        expect(isDiagnosticStale({receivedMonotonic: NaN}, monotonic)).toBe(true);
        const state = healthy();
        state.diagnostics[0].receivedMonotonic = NaN;
        state.nowMonotonic = monotonic;
        expect(deriveDiagnosticHealth(state).checks.find(check => check.id === 'ros')?.level).toBe('unknown');
    });
    it.each([29999, 30000, 30001])('preserves both original comparison predicates at elapsed %i ms', elapsed => {
        const hook = renderHook(useDiagnostics); emit(frame(10)); advance(1000); vi.setSystemTime(9_000_000);
        advance(elapsed - 1000);
        expect(ros(hook.result.current)).toBe(elapsed < 30000 ? 'warning' : 'unknown');
        expect(isDiagnosticStale(current(hook.result.current), monotonic)).toBe(elapsed > 30000);
    });
    it.each([59999, 60000, 60001])('keeps reception presentation at 60 seconds, elapsed %i ms', elapsed => {
        const h = renderHook(useDiagnostics); emit(frame(10)); advance(1000); connection('closed');
        expect(h.result.current.disconnectedAt).toBe(monotonic);
        const view = render(<DiagnosticAlertsPanel alerts={[]} connected={false} now={monotonic}/>);
        const source = {key: 'hardware', label: 'hardware', publication: monotonic, ageMs: 0};
        const watches = advanceReceptionWatches([], [source], monotonic);
        vi.setSystemTime(-9_000_000); advance(elapsed);
        expect(missingReceptionSources(watches, monotonic)).toHaveLength(elapsed >= 60000 ? 1 : 0);
        view.rerender(<DiagnosticAlertsPanel alerts={[]} connected={false} now={monotonic}/>);
        expect(view.container.querySelectorAll('[data-source="communication"]')).toHaveLength(elapsed >= 60000 ? 1 : 0);
    });
    it.each(['mowgli', 'mavros', 'openmower'])('keeps replay, equal and regressive barriers for %s after a civil correction', backend => {
        const h = renderHook(() => useDiagnostics(`robot/${backend}`)); emit(frame(10)); advance(1000);
        const original = current(h.result.current); vi.setSystemTime(9_000_000);
        emit(frame(10)); advance(1000);
        expect(current(h.result.current).receivedAt).toBe(original.receivedAt);
        expect(current(h.result.current).receivedMonotonic).toBe(original.receivedMonotonic);
        connection('closed'); connection('open');
        for (const stamp of [9, 10]) {emit(frame(stamp, 0)); advance(1000); expect(ros(h.result.current)).toBe('unknown');}
        emit(frame(11, 0)); advance(1000); expect(ros(h.result.current)).toBe('ok');
        expect(transport.subscribe).toHaveBeenCalledTimes(1);
    });
    it('keeps unstamped legacy receipts separate from stamped progress and does not drop known barriers', () => {
        const h = renderHook(useDiagnostics); emit(frame(undefined, 0)); advance(1000);
        vi.setSystemTime(1); emit(frame(undefined, 0)); advance(1000);
        expect(ros(h.result.current)).toBe('ok'); expect(current(h.result.current).receivedMonotonic).toBe(6000);
        emit(frame(10, 0)); advance(1000); emit(frame(undefined, 0)); advance(1000);
        expect(ros(h.result.current)).toBe('unknown');
    });
    it('retains the existing historical and delayed-content admission without inventing physical age', () => {
        const h = renderHook(useDiagnostics); emit(frame(1, 0)); advance(31000);
        expect(ros(h.result.current)).toBe('unknown'); vi.setSystemTime(1);
        emit(frame(2, 0)); advance(1000); expect(ros(h.result.current)).toBe('ok');
        h.unmount(); const next = renderHook(useDiagnostics); emit(frame(1, 0)); advance(1000);
        expect(ros(next.result.current)).toBe('ok');
    });
    it('clears only the transition scope at context change, without resetting temporal barriers', () => {
        const h = renderHook(({scope}) => useDiagnostics(scope), {initialProps: {scope: 'A/direct'}});
        emit(frame(20)); advance(1000); const cursor = h.result.current.readTransitions().cursor;
        h.rerender({scope: 'B/mavros'});
        expect(h.result.current.readTransitions(cursor)).toMatchObject({contextChanged: true, transitions: []});
        expect(current(h.result.current).publicationStamp).toBe('20:0');
        emit(frame(1, 0)); advance(1000); expect(ros(h.result.current)).toBe('unknown');
    });
    it('preserves StrictMode cleanup, 64 records, eviction, cursors and the single coalescing timer', () => {
        let renders = 0; const h = renderHook(() => {renders++; return useDiagnostics('robot');}, {wrapper: StrictMode});
        const before = renders;
        act(() => {for (let i = 0; i < 70; i++) transport.listeners.get('diagnostics')?.forEach(fn => fn(frame(10 + i, i % 2 ? 0 : 1)));});
        const read = h.result.current.readTransitions();
        expect(read.transitions).toHaveLength(64); expect(read.evicted).toBe(6);
        expect(renders).toBe(before); expect(vi.getTimerCount()).toBe(1);
        expect(h.result.current.readTransitions(read.cursor).transitions).toHaveLength(0);
        const get = h.result.current.readTransitions; h.unmount(); advance(1000);
        expect(get(read.cursor)).toMatchObject({active: false, contextChanged: true, transitions: []});
    });
    it('keeps direct urgent emergency critical before the diagnostic timer', () => {
        const h = renderHook(useDiagnostics); emit(frame(10, 0));
        expect(h.result.current.diagnostics.status).toBeUndefined();
        const health = deriveDiagnosticHealth({...healthy(), emergency: {active_emergency: true}, nowMs: Date.now(), nowMonotonic: monotonic});
        expect(health.level).toBe('critical');
    });
    it('keeps diagnostic-only GNSS category and expiration independent from the wall clock', () => {
        const h = renderHook(useGnssStatus);
        emit(frame(10, 0, 'GPS', [{key: 'fix_status', value: '0'}])); advance(1000);
        expect(h.result.current.fix_type).toBe(G.FIX_TYPE_GPS_FIX);
        vi.setSystemTime(9_000_000); advance(1000); expect(h.result.current.fix_type).toBe(G.FIX_TYPE_GPS_FIX);
        advance(28000); expect(h.result.current).toEqual({});
    });
    it('keeps MAVROS diagnostic fields and ESC age calculations in the local monotonic domain', () => {
        const h = renderHook(() => ({mavros: useMavrosDiagnostics(), diagnostics: useDiagnostics()}));
        emit(frame(10, 0, 'mowgli_mavros_bridge/wheel_lift', [{key: 'state_valid', value: 'true'}])); advance(1000);
        vi.setSystemTime(9_000_000); advance(1000);
        expect(h.result.current.mavros.lift?.fields.state_valid).toBe('true');
        emit(frame(11, 0, 'mowgli_mavros_bridge/vesc_left_wheel', [{key: 'online', value: 'true'}, {key: 'stale', value: 'false'}, {key: 'age_ms', value: '100'}])); advance(1000);
        expect(wheelControllerFromDiagnostics(h.result.current.diagnostics.diagnostics, 'left_wheel', monotonic)?.state).toBe('online');
        advance(5000); expect(wheelControllerFromDiagnostics(h.result.current.diagnostics.diagnostics, 'left_wheel', monotonic)?.state).toBe('stale');
    });
});
