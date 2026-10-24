import {act, cleanup, renderHook} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import {useFusionGraphDiagnostics} from './useFusionGraphDiagnostics';

const stream = vi.hoisted(() => ({receive: (message: unknown) => {void message;},
    error: (error: Error) => {void error;}, info: (message: string) => {void message;}, start: vi.fn(), stop: vi.fn()}));
vi.mock('./useWS', () => ({useWS: (error: (e: Error) => void, info: (msg: string) => void, receive: (message: unknown) => void) => {
    stream.receive = receive; stream.error = error; stream.info = info;
    return {start: stream.start, stop: stream.stop};
}}));

const frame = (sec: number, name = 'fusion_graph') => ({header: {stamp: {sec, nanosec: 25}},
    status: [{name, hardware_id: 'mower-a', level: 0, message: 'OK', values: [{key: 'total_nodes', value: '12'}]}]});

describe('fusion diagnostic provenance', () => {
    beforeEach(() => vi.clearAllMocks());
    afterEach(() => {cleanup(); vi.restoreAllMocks();});
    it('keeps repeated publication identity separate from delivery liveness', () => {
        const clock = vi.spyOn(performance, 'now').mockReturnValue(100);
        const {result} = renderHook(useFusionGraphDiagnostics);
        act(() => stream.receive(frame(1000)));
        clock.mockReturnValue(8100);
        act(() => stream.receive(frame(1000)));
        expect(result.current.stats?.receivedMonotonic).toBe(8100);
        expect(result.current.stats?.distinctMonotonic).toBe(100);
        act(() => stream.receive(frame(1001)));
        expect(result.current.stats?.distinctMonotonic).toBe(8100);
        clock.mockReturnValue(9000);
        act(() => stream.receive(frame(1001, 'replacement-producer')));
        expect(result.current.stats?.distinctMonotonic).toBe(9000);
    });
    it('does not subtract a robot timestamp from the browser clock', () => {
        vi.spyOn(performance, 'now').mockReturnValue(200);
        vi.spyOn(Date, 'now').mockReturnValue(1);
        const {result} = renderHook(useFusionGraphDiagnostics);
        act(() => stream.receive(frame(2_000_000_000)));
        expect(result.current.stats?.sourceStamp).toBe('2000000000.000000025');
        expect(result.current.stats?.receivedMonotonic).toBe(200);
    });
    it('leaves source identity unknown when no publication stamp exists', () => {
        const {result} = renderHook(useFusionGraphDiagnostics);
        act(() => stream.receive({status: [{name: 'fusion_graph', message: '', level: 0, values: []}]}));
        expect(result.current.stats?.sourceIdentity).toBeUndefined();
    });
});


describe('fusion diagnostic boundary', () => {
    beforeEach(() => vi.clearAllMocks());
    afterEach(() => {cleanup(); vi.restoreAllMocks();});
    it('preserves zero, empty text, string measurements and legacy absent metadata', () => {
        const {result} = renderHook(useFusionGraphDiagnostics);
        act(() => stream.receive({status: [{name: '', level: 0, message: '', values: [{key: 'total_nodes', value: '0'}, {key: 'optional', value: ''}]}]}));
        expect(result.current.stats).toMatchObject({level: 0, message: '', values: {total_nodes: '0', optional: ''}});
        expect(result.current.stats?.sourceIdentity).toBeUndefined();
        act(() => stream.receive({status: [{name: 'fusion_graph', level: 0, message: 'waiting init'}]}));
        expect(result.current.stats?.values).toEqual({});
    });
    it('accepts a zero ROS stamp without inventing producer identity', () => {
        const {result} = renderHook(useFusionGraphDiagnostics);
        act(() => stream.receive({...frame(0), header: {stamp: {sec: 0, nanosec: 0}}}));
        expect(result.current.stats?.sourceStamp).toBeUndefined();
        expect(result.current.stats?.level).toBe(0);
    });
    it.each([null, [], 'not decoded', {}, {status: []}, {status: {}}, {status: [null]},
        {status: [{name: 'fusion_graph', level: 0}]},
        ...[NaN, Infinity, -1, 4, '0'].map(level => ({status: [{...frame(1).status[0], level}]})),
        ...[null, {}, [{key: 'count', value: 0}], [{key: 0, value: '0'}]].map(values => ({status: [{...frame(1).status[0], values}]})),
        {status: [{...frame(1).status[0], hardware_id: 0}]},
        ...[null, {sec: 1}, {sec: 1, nanosec: NaN}, {sec: 1.5, nanosec: 0}, {sec: 1, nanosec: 1e9}].map(stamp => ({...frame(1), header: {stamp}})),
    ].map(value => [value]))('rejects malformed frames without renewing the valid snapshot: %j', raw => {
        const {result} = renderHook(useFusionGraphDiagnostics);
        act(() => stream.receive(raw)); expect(result.current.stats).toBeNull();
        act(() => stream.receive(frame(1))); const previous = result.current.stats;
        act(() => stream.receive(raw)); expect(result.current.stats).toBe(previous);
    });
    it('preserves literal diagnostic keys and last duplicate values', () => {
        const {result} = renderHook(useFusionGraphDiagnostics);
        act(() => stream.receive({status: [{...frame(1).status[0], values: [{key: '__proto__', value: 'literal'}, {key: 'n', value: '1'}, {key: 'n', value: '0'}]}]}));
        expect(result.current.stats?.values.n).toBe('0');
        expect(Object.getOwnPropertyDescriptor(result.current.stats?.values, '__proto__')?.value).toBe('literal');
    });
    it('retains first-status selection, state on disconnect, and stable subscription across rerenders', () => {
        const {result, rerender, unmount} = renderHook(useFusionGraphDiagnostics);
        act(() => stream.receive({status: [...frame(1).status, {message: 'ignored'}]}));
        const previous = result.current.stats;
        act(() => {stream.error(new Error('closed')); stream.info('reconnected');});
        rerender(); expect(result.current.stats?.values).toBe(previous?.values);
        expect(result.current.stats?.sourceIdentity).toBe(previous?.sourceIdentity);
        expect(result.current.stats?.healthBlocked).toBe(true);
        expect(result.current.stats?.lastDisconnectAt).toBeTypeOf('number');
        expect(stream.start).toHaveBeenCalledExactlyOnceWith('/api/mowglinext/subscribe/fusionDiag');
        expect(stream.stop).not.toHaveBeenCalled(); unmount(); expect(stream.stop).toHaveBeenCalledOnce();
    });
});

it('retains a disconnect barrier across cached replay and requires distinct publication progress', () => {
    const clock = vi.spyOn(performance, 'now').mockReturnValue(100);
    const hook = renderHook(useFusionGraphDiagnostics);
    act(() => stream.receive(frame(1)));
    clock.mockReturnValue(200); act(() => stream.error(new Error('fixture close')));
    clock.mockReturnValue(300); act(() => stream.receive(frame(1)));
    expect(hook.result.current.stats?.distinctMonotonic).toBe(100);
    expect(hook.result.current.stats?.lastDisconnectAt).toBe(200);
    act(() => stream.receive(frame(2)));
    expect(hook.result.current.stats?.distinctMonotonic).toBe(300);
    hook.unmount(); vi.restoreAllMocks();
});

it('does not restore a healthy localizer verdict from an older different publication after reconnect',async()=>{
    const {deriveDiagnosticHealth}=await import('../utils/diagnosticHealth');
    const {healthy}=await import('../test/diagnosticHealthFixture');
    const clock=vi.spyOn(performance,'now').mockReturnValue(100);
    const hook=renderHook(useFusionGraphDiagnostics);
    act(()=>stream.receive(frame(10)));clock.mockReturnValue(200);act(()=>stream.error(new Error('fixture close')));
    clock.mockReturnValue(300);act(()=>stream.receive(frame(9)));
    const verdict=()=>deriveDiagnosticHealth({...healthy(),highLevel:{state:2},nowMonotonic:300,fusion:hook.result.current.stats});
    expect(verdict().checks.find(c=>c.id==='localizer')?.level).toBe('unknown');
    act(()=>stream.receive(frame(10)));expect(verdict().checks.find(c=>c.id==='localizer')?.level).toBe('unknown');
    act(()=>stream.receive(frame(11)));expect(verdict().checks.find(c=>c.id==='localizer')?.level).toBe('ok');
    hook.unmount();clock.mockRestore();
});
it('retains per-source health barriers across a FusionGraph diagnostic source round trip',()=>{
    const clock=vi.spyOn(performance,'now').mockReturnValue(100);
    const hook=renderHook(useFusionGraphDiagnostics);
    act(()=>stream.receive(frame(42,'source-A')));
    clock.mockReturnValue(200);act(()=>stream.receive(frame(1,'source-B')));
    clock.mockReturnValue(300);act(()=>stream.receive(frame(42,'source-A')));
    expect(hook.result.current.stats?.healthBlocked).toBe(true);
    act(()=>stream.receive(frame(43,'source-A')));expect(hook.result.current.stats?.healthBlocked).toBe(false);
    hook.unmount();clock.mockRestore();
});
