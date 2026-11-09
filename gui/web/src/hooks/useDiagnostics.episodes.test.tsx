import {act, cleanup, renderHook} from '@testing-library/react';
import {StrictMode} from 'react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import {useDiagnostics, type DiagnosticStatus} from './useDiagnostics';
import {deriveDiagnosticHealth} from '../utils/diagnosticHealth';
import {healthy} from '../test/diagnosticHealthFixture';

// Exercise the real useWS (committed callbacks and teardown); only the socket is simulated.
const transport = vi.hoisted(() => ({listeners: new Set<(raw: unknown) => void>(), statuses: new Set<(state: string) => void>(), subscribe: vi.fn()}));
vi.mock('react-use-websocket', () => ({default: () => ({sendJsonMessage: () => {}})}));
vi.mock('./multiplexedSocket.ts', () => ({
    getMultiplexedSocket: () => ({subscribe: transport.subscribe, onStatusChange: (fn: (state: string) => void) => {
        transport.statuses.add(fn); return () => transport.statuses.delete(fn);
    }}),
    isMultiplexableSubscribeUri: (uri: string) => uri.startsWith('/api/mowglinext/subscribe/'),
    topicFromSubscribeUri: (uri: string) => uri.split('/').pop(),
}));
let now: number;
const context = 'endpoint-A/robot-A/mowgli';
const admission = {connected: true, lidarEnabled: false};
const entry = (level: number, name = 'declared/report', hardware_id = 'source', message = `level ${level}`): DiagnosticStatus =>
    ({level, name, hardware_id, message, values: []});
const frame = (sec: number | undefined, ...status: DiagnosticStatus[]) => ({
    ...(sec === undefined ? {} : {header: {stamp: {sec, nanosec: 0}}}), status,
});
const emit = (raw: unknown) => act(() => {transport.listeners.forEach(fn => fn(raw));});
const advance = (ms: number) => act(() => {now += ms; vi.advanceTimersByTime(ms);});
const connection = (state: string) => act(() => {transport.statuses.forEach(fn => fn(state));});
const mount = () => renderHook(() => useDiagnostics(context, admission));
const episodes = (hook: ReturnType<typeof mount>) => hook.result.current.readEpisodes()!.episodes;

beforeEach(() => {
    vi.useFakeTimers(); vi.setSystemTime(100000); now = 5000;
    vi.spyOn(performance, 'now').mockImplementation(() => now);
    transport.subscribe.mockReset(); transport.subscribe.mockImplementation((_topic: string, fn: (raw: unknown) => void) => {
        transport.listeners.add(fn); return () => transport.listeners.delete(fn);
    });
});
afterEach(() => {
    cleanup(); expect(transport.listeners.size).toBe(0); expect(transport.statuses.size).toBe(0);
    expect(vi.getTimerCount()).toBe(0); vi.restoreAllMocks(); vi.useRealTimers();
});

describe('02B-6B: qualified episodes before React commit, no visual consumer', () => {
    it('retains WARN -> admissible OK in 100ms before any React snapshot', () => {
        const h = mount(); const before = h.result.current;
        emit(frame(10, entry(1))); advance(100); emit(frame(11, entry(0)));
        expect(h.result.current).toBe(before); expect(before.diagnostics).toEqual({});
        expect(episodes(h)).toMatchObject([{state: 'resolved', peakSeverity: 1, unconsulted: true}]);
        expect(before.readTransitions().transitions.map(t => t.level)).toEqual([1, 0]);
        expect(vi.getTimerCount()).toBe(1); advance(900);
        expect(h.result.current.diagnostics.status?.[0].level).toBe(0);
    });
    it('retains WARN -> ERROR -> OK before the coalescing timer', () => {
        const h = mount(); emit(frame(1, entry(1))); emit(frame(2, entry(2))); emit(frame(3, entry(0)));
        expect(h.result.current.diagnostics).toEqual({});
        expect(episodes(h)).toMatchObject([{state: 'resolved', peakSeverity: 2, lastFaultSeverity: 2}]);
    });
    it('refuses a regressive WARN and then accepts the same WARN level with progression', () => {
        const h = mount(); emit(frame(10, entry(0))); emit(frame(9, entry(1)));
        expect(episodes(h)).toEqual([]); emit(frame(11, entry(1)));
        expect(episodes(h)).toMatchObject([{state: 'active', peakSeverity: 1}]);
    });
    it('does not resolve with a blocked OK; a subsequent OK at the same level can resolve', () => {
        const h = mount(); emit(frame(10, entry(1))); emit(frame(9, entry(0)));
        expect(episodes(h)).toMatchObject([{state: 'unknown', unconsulted: true}]);
        emit(frame(11, entry(0))); expect(episodes(h)).toMatchObject([{state: 'resolved', continuity: 'unknown'}]);
    });
    it('uses messages and values even at the same ROS level: unavailability then declared fault', () => {
        const h = mount();
        const wheel = {...entry(1, 'mowgli_mavros_bridge/wheel_odometry_source', '', 'missing_or_stale'), values: [{key: 'required', value: 'true'}]};
        emit(frame(1, wheel)); expect(episodes(h)).toEqual([]);
        emit(frame(2, {...wheel, message: 'declared sensor fault'}));
        expect(episodes(h)).toMatchObject([{state: 'active', peakSeverity: 1}]);
        emit(frame(3, wheel)); expect(episodes(h)).toMatchObject([{state: 'unknown'}]);
        emit(frame(4, {...wheel, level: 0})); expect(episodes(h)).toMatchObject([{state: 'resolved'}]);
    });
    it('does not interpret standard missing-input monitoring WARN or STALE as a material fault', () => {
        const h = mount();
        emit(frame(1, entry(1, 'Hardware Bridge', 'mowgli/hardware_bridge'), entry(3, 'stale')));
        expect(episodes(h)).toEqual([]);
        emit(frame(2, {...entry(1, 'GPS', 'mowgli/gps'), values: [{key: 'fix_status', value: '-1'}]}));
        expect(episodes(h)).toMatchObject([{identity: {name: 'GPS'}, state: 'active'}]);
    });
    it('preserves disabled LiDAR exclusion without erasing an unrelated explicit driver fault', () => {
        const h = mount(); emit(frame(1, entry(1, 'LiDAR', 'mowgli/lidar'))); expect(episodes(h)).toEqual([]);
        emit(frame(2, entry(2, 'lidar_driver', 'physical/lidar', 'motor fault')));
        expect(episodes(h)).toMatchObject([{identity: {name: 'lidar_driver'}, peakSeverity: 2}]);
    });
    it('does not resolve by omission from a partial array, an empty array or an unattributable malformed frame', () => {
        const h = mount(); emit(frame(1, entry(1)));
        emit(frame(2, entry(0, 'other'))); emit(frame(3)); emit(null); emit({status: 'bad'});
        expect(episodes(h)).toMatchObject([{state: 'active'}]);
    });
    it('invalidates named malformed evidence without resolution, then requires new admissible progress', () => {
        const h = mount(); emit(frame(10, entry(1)));
        emit({status: [{...entry(0), values: 'malformed'}]});
        expect(episodes(h)).toMatchObject([{state: 'unknown', continuity: 'unknown'}]);
        emit(frame(10, entry(0))); expect(episodes(h)[0].state).toBe('unknown');
        emit(frame(11, entry(0))); expect(episodes(h)[0].state).toBe('resolved');
    });
    it.each([29999, 30000, 30001])('uses existing health freshness at %i ms, including when the stream is silent', elapsed => {
        const h = mount(); emit(frame(10, entry(1))); advance(elapsed);
        expect(episodes(h)[0].state).toBe(elapsed < 30000 ? 'active' : 'expired');
        expect(vi.getTimerCount()).toBe(0);
        emit(frame(11, entry(1))); expect(episodes(h)).toHaveLength(1);
        expect(episodes(h)[0]).toMatchObject({state: 'active', continuity: elapsed < 30000 ? 'continuous' : 'unknown'});
    });
    it('detects the silent expiry before new progress even if nobody read episodes during the gap', () => {
        const h = mount(); emit(frame(10, entry(1))); advance(31000); emit(frame(11, entry(1)));
        expect(episodes(h)).toMatchObject([{id: 1, state: 'active', continuity: 'unknown'}]);
    });
    it('accepts equal stamps within freshness but does not renew the progression anchor', () => {
        const h = mount(); emit(frame(10, entry(1))); advance(100); emit(frame(10, entry(0)));
        expect(episodes(h)[0].state).toBe('resolved'); emit(frame(10, entry(1)));
        expect(episodes(h)).toHaveLength(2); advance(29900); emit(frame(10, entry(0)));
        expect(episodes(h)[1].state).toBe('expired');
    });
    it('preserves equal-stamp and regressive barriers across disconnect/reconnect', () => {
        const h = mount(); emit(frame(10, entry(1))); connection('closed'); connection('open');
        for (const sec of [9, 10]) {emit(frame(sec, entry(0))); expect(episodes(h)[0].state).toBe('unknown');}
        emit(frame(11, entry(0))); expect(episodes(h)[0].state).toBe('resolved');
    });
    it('does not promote a disconnected initial source until committed connectivity is available', () => {
        const h = renderHook(({connected}) => useDiagnostics(context, {connected}), {initialProps: {connected: false}});
        emit(frame(10, entry(1))); expect(h.result.current.readEpisodes()!.episodes).toEqual([]);
        h.rerender({connected: true}); emit(frame(11, entry(1)));
        expect(h.result.current.readEpisodes()!.episodes[0].state).toBe('active');
    });
    it('blocks a newly seen identity immediately on stream close before connectivity props change', () => {
        const h = mount(); connection('closed'); emit(frame(10, entry(1)));
        expect(episodes(h)).toEqual([]);
        connection('open'); emit(frame(11, entry(1)));
        expect(episodes(h)).toMatchObject([{state: 'active', continuity: 'unknown'}]);
        emit(frame(12, entry(1, 'first seen after gap')));
        expect(episodes(h)[1]).toMatchObject({state: 'active', opening: 'unknown_history', continuity: 'unknown'});
    });
    it('preserves historical first-delivery and unstamped legacy limitations without claiming acquisition', () => {
        const h = mount(); emit(frame(1, entry(1))); emit(frame(undefined, entry(1, 'legacy', '')));
        expect(episodes(h).map(e => e.identity.name)).toEqual(['declared/report', 'legacy']);
        expect(episodes(h)).toHaveLength(2); advance(100); emit(frame(undefined, entry(0, 'legacy', '')));
        expect(episodes(h)[1].state).toBe('resolved');
    });
    it('treats delayed WARN and OK delivery as local report evidence, not as physical acquisition age', () => {
        const h = mount(); advance(60000); emit(frame(1, entry(1)));
        expect(episodes(h)).toMatchObject([{state: 'active', unconsulted: true}]);
        advance(100); emit(frame(2, entry(0)));
        expect(episodes(h)).toMatchObject([{state: 'resolved', unconsulted: true}]);
        advance(900); expect(h.result.current.diagnostics.status?.[0]).toMatchObject({healthProgressAt: 65100, publicationStamp: '2:0'});
    });
    it('does not resolve another identity or retire it because a selected backend changed', () => {
        const h = mount(); emit(frame(1, entry(1, 'A'), entry(2, 'B'))); emit(frame(2, entry(0, 'B')));
        expect(episodes(h).map(e => [e.identity.name, e.state])).toEqual([['A', 'active'], ['B', 'resolved']]);
    });
    it('keeps the existing source-change barrier and does not clear a homonymous source', () => {
        const h = mount(); emit(frame(10, entry(1, 'A', 'one'))); emit(frame(1, entry(0, 'A', 'two')));
        expect(episodes(h)).toHaveLength(1); expect(episodes(h)[0].state).toBe('active');
        emit(frame(2, entry(0, 'A', 'two'))); expect(episodes(h)[0].state).toBe('active');
        emit(frame(11, entry(0, 'A', 'one'))); expect(episodes(h)[0].state).toBe('resolved');
    });
    it.each(['endpoint', 'robot', 'backend'])('isolates committed %s context changes without resetting the existing snapshot or progress barriers', part => {
        const h = renderHook(({scope}) => useDiagnostics(scope, admission), {initialProps: {scope: context}});
        emit(frame(20, entry(1))); advance(1000);
        const before = h.result.current, old = before.readEpisodes()!;
        h.rerender({scope: `${context}/${part}-B`});
        expect(h.result.current.diagnostics).toBe(before.diagnostics);
        expect(h.result.current.readEpisodes()!.receipt.scope).not.toBe(old.receipt.scope);
        expect(h.result.current.readEpisodes()!.episodes).toEqual([]);
        emit(frame(1, entry(2))); expect(h.result.current.readEpisodes()!.episodes).toEqual([]);
        emit(frame(21, entry(1))); expect(h.result.current.readEpisodes()!.episodes).toHaveLength(1);
        expect(transport.subscribe).toHaveBeenCalledTimes(1);
    });
    it('keeps two mounted robot/backend contexts and their lifetimes separate', () => {
        const a = mount(), b = renderHook(() => useDiagnostics('endpoint-B/robot-B/mavros', admission));
        emit(frame(10, entry(1)));
        expect(a.result.current.readEpisodes()!.receipt.scope).not.toBe(b.result.current.readEpisodes()!.receipt.scope);
        const readA = a.result.current.readEpisodes; a.unmount(); expect(readA()).toBeNull();
        expect(b.result.current.readEpisodes()!.episodes).toHaveLength(1);
    });
    it('has bounded neutral omissions, with no fabricated resolution or false opening at saturation', () => {
        const h = mount();
        for (let i = 1; i <= 70; i++) emit(frame(i, entry(1, `source-${i}`)));
        const read = h.result.current.readEpisodes()!;
        expect(read.episodes).toHaveLength(64); expect(read.losses).toMatchObject({evicted: 0, omitted: 6, pending: true});
        emit(frame(71, entry(0, 'source-70')));
        expect(episodes(h)).toHaveLength(64); expect(episodes(h).every(e => e.state === 'active')).toBe(true);
    });
    it('also protects 64 resolved but unread episodes without automatic acknowledgement', () => {
        const h = mount();
        for (let i = 0; i < 65; i++) {
            emit(frame(2 * i + 1, entry(1, `source-${i}`)));
            emit(frame(2 * i + 2, entry(0, `source-${i}`)));
        }
        const read = h.result.current.readEpisodes()!;
        expect(read.episodes).toHaveLength(64);
        expect(read.episodes.every(e => e.state === 'resolved' && e.unconsulted)).toBe(true);
        expect(read.losses).toMatchObject({omitted: 1, evicted: 0, unconsultedEvicted: 0, pending: true});
    });
    it('does not add per-publication renders, subscriptions, timers or full-payload copies', () => {
        let renders = 0; const h = renderHook(() => {renders++; return useDiagnostics(context, admission);});
        const before = renders, read = h.result.current.readEpisodes;
        emit(frame(1, entry(1))); const episode = episodes(h)[0];
        for (let i = 2; i <= 500; i++) emit(frame(i, {...entry(1), values: [{key: 'angle', value: `${i}.125`}]}));
        expect(renders).toBe(before); expect(episodes(h)[0]).toBe(episode); expect(vi.getTimerCount()).toBe(1);
        expect(episode).not.toHaveProperty('values'); expect(transport.subscribe).toHaveBeenCalledTimes(1);
        advance(1000); expect(renders).toBe(before + 1); expect(h.result.current.readEpisodes).toBe(read);
    });
    it('cleans subscriptions, pending timers and episode scope under StrictMode and remounts empty', () => {
        const h = renderHook(() => useDiagnostics(context, admission), {wrapper: StrictMode});
        expect(transport.listeners.size).toBe(1); emit(frame(1, entry(1))); emit(frame(2, entry(0)));
        expect(h.result.current.readEpisodes()!.episodes).toHaveLength(1);
        const read = h.result.current.readEpisodes; h.unmount(); expect(read()).toBeNull();
        expect(transport.listeners.size).toBe(0); expect(vi.getTimerCount()).toBe(0);
        const next = mount(); expect(next.result.current.readEpisodes()!.episodes).toEqual([]);
    });
    it('does not qualify without explicit admission inputs and clears opt-out without changing raw memory', () => {
        const h = renderHook(({enabled}) => useDiagnostics(context, enabled ? admission : undefined), {initialProps: {enabled: true}});
        emit(frame(10, entry(1))); const cursor = h.result.current.readTransitions().cursor;
        h.rerender({enabled: false}); expect(h.result.current.readEpisodes()).toBeNull();
        expect(h.result.current.readTransitions().cursor).toEqual(cursor);
        h.rerender({enabled: true}); expect(h.result.current.readEpisodes()!.episodes).toEqual([]);
    });
    it('does not qualify before configuration is ready or erase an existing episode during a temporary reload', () => {
        const h = renderHook(({ready}) => useDiagnostics(context, {...admission, ready}), {initialProps: {ready: false}});
        emit(frame(1, entry(1))); expect(h.result.current.readEpisodes()!.episodes).toEqual([]);
        h.rerender({ready: true}); emit(frame(2, entry(1)));
        const receipt = h.result.current.readEpisodes()!.receipt;
        h.rerender({ready: false}); emit(frame(3, entry(0)));
        expect(h.result.current.readEpisodes()!.episodes[0]).toMatchObject({state: 'unknown', unconsulted: true});
        expect(h.result.current.readEpisodes()!.receipt.scope).toBe(receipt.scope);
        h.rerender({ready: true}); emit(frame(4, entry(0)));
        expect(h.result.current.readEpisodes()!.episodes[0]).toMatchObject({state: 'resolved', unconsulted: true});
        expect(transport.subscribe).toHaveBeenCalledTimes(1);
    });
    it.each([{label: 'absent', scope: undefined}, {label: 'oversized', scope: 'x'.repeat(4097)}])('does not invent an episode scope when the context is $label', ({scope}) => {
        const h = renderHook(() => useDiagnostics(scope, admission)); emit(frame(1, entry(1)));
        expect(h.result.current.readEpisodes()).toBeNull(); advance(1000);
        expect(h.result.current.diagnostics.status?.[0].level).toBe(1);
    });
    it('preserves current snapshots, health verdicts and direct emergencies with episodes enabled or disabled', () => {
        const h = renderHook(() => ({on: useDiagnostics(context, admission), off: useDiagnostics(context)}));
        const health = (state: ReturnType<typeof useDiagnostics>) => deriveDiagnosticHealth({...healthy(),
            diagnostics: state.diagnostics.status ?? [], nowMonotonic: now, diagnosticsDisconnectedAt: state.disconnectedAt});
        for (const raw of [frame(10, entry(1)), frame(9, entry(0)), frame(11, entry(0)), frame(12, entry(3)),
            frame(13, entry(1, 'LiDAR', 'mowgli/lidar')), {status: [{...entry(0), level: 'bad'}]}, frame(14, entry(2))]) {
            emit(raw); advance(1000); h.result.current.on.readEpisodes();
            expect(h.result.current.on.diagnostics).toEqual(h.result.current.off.diagnostics);
            expect(health(h.result.current.on)).toEqual(health(h.result.current.off));
        }
        connection('closed'); expect(health(h.result.current.on)).toEqual(health(h.result.current.off));
        expect(deriveDiagnosticHealth({...healthy(), emergency: {active_emergency: true}}).level).toBe('critical');
    });
});
