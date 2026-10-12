import {act, renderHook} from '@testing-library/react';
import {afterEach, describe, expect, it, vi} from 'vitest';
import {useFusionGraphDiagnostics} from './useFusionGraphDiagnostics';

const stream = vi.hoisted((): {receive: (message: unknown) => void} => ({receive: vi.fn()}));
vi.mock('./useWS', () => ({useWS: (_error: unknown, _info: unknown, receive: (message: unknown) => void) => {
    stream.receive = receive;
    return {start: vi.fn(), stop: vi.fn()};
}}));

const frame = (sec: number, name = 'fusion_graph') => ({header: {stamp: {sec, nanosec: 25}},
    status: [{name, hardware_id: 'mower-a', level: 0, message: 'OK', values: [{key: 'total_nodes', value: '12'}]}]});

describe('fusion diagnostic provenance', () => {
    afterEach(() => vi.restoreAllMocks());
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
        act(() => stream.receive({status: [{name: 'fusion_graph', level: 0, values: []}]}));
        expect(result.current.stats?.sourceIdentity).toBeUndefined();
    });
});
