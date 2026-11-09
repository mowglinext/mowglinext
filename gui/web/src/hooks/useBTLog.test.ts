import {act, cleanup, renderHook} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import {useBTLog} from './useBTLog';

const stream = vi.hoisted(() => ({receive: (message: unknown) => {void message;},
    error: (error: Error) => {void error;}, info: (message: string) => {void message;}, start: vi.fn(), stop: vi.fn()}));
vi.mock('./useWS', () => ({useWS: (error: (e: Error) => void, info: (msg: string) => void, receive: (message: unknown) => void) => {
    stream.receive = receive; stream.error = error; stream.info = info;
    return {start: stream.start, stop: stream.stop};
}}));
const event = (current_status = 'RUNNING') => ({node_name: 'Spin', uid: 0, previous_status: 'IDLE', current_status,
    timestamp: {sec: 0, nanosec: 0}});
const flush = () => {act(() => {vi.advanceTimersByTime(500);});};
beforeEach(() => {vi.useFakeTimers(); vi.clearAllMocks();});
afterEach(() => {cleanup(); vi.useRealTimers();});

describe('BT diagnostic projection', () => {
    it('preserves zero receipt time and accepts a full producer event with uid zero', () => {
        vi.setSystemTime(0);
        const {result} = renderHook(useBTLog);
        act(() => stream.receive({timestamp: {sec: 0, nanosec: 0}, event_log: [event()]}));
        expect(result.current.size).toBe(0); flush();
        expect(result.current.get('Spin')).toEqual({status: 'RUNNING', updatedAt: 0});
    });
    it('requires only consumed fields, preserves status text and last-event-wins order', () => {
        const {result} = renderHook(useBTLog);
        act(() => stream.receive({event_log: [{node_name: 'Spin', current_status: 'SUCCESS'}, {node_name: 'Spin', current_status: 'SKIPPED'}, {node_name: '', current_status: ''}]}));
        flush(); expect(result.current.get('Spin')?.status).toBe('SKIPPED'); expect(result.current.get('')?.status).toBe('');
        act(() => stream.receive({event_log: [{node_name: 'Spin', current_status: 'FUTURE_STATUS'}]}));
        flush(); expect(result.current.get('Spin')?.status).toBe('FUTURE_STATUS');
    });
    it.each([null, [], 'not decoded', {event_log: null}, {event_log: {}}, {event_log: [null]},
        {event_log: [{node_name: 'Spin'}]}, {event_log: [{node_name: 0, current_status: 'RUNNING'}]},
        {event_log: [event('SUCCESS'), {node_name: 'BackUp', current_status: false}]},
    ].map(value => [value]))('rejects the whole invalid frame without corrupting or renewing state: %j', raw => {
        const {result} = renderHook(useBTLog);
        act(() => stream.receive(raw)); flush(); expect(result.current.size).toBe(0);
        act(() => stream.receive({event_log: [event()]})); flush(); const previous = result.current;
        act(() => stream.receive(raw)); flush(); expect(result.current).toBe(previous);
        act(() => stream.receive({event_log: []})); flush();
        expect(result.current.get('Spin')).toEqual(previous.get('Spin'));
    });
    it('keeps absent/empty logs from inventing nodes or renewing timestamps', () => {
        const {result} = renderHook(useBTLog);
        act(() => stream.receive({})); flush(); expect(result.current.size).toBe(0);
        act(() => stream.receive({event_log: [event()]})); flush(); const previous = result.current.get('Spin');
        act(() => stream.receive({event_log: []})); flush(); expect(result.current.get('Spin')).toEqual(previous);
    });
    it('keeps the 500 ms throttle and cancels pending publication on unmount', () => {
        const {result, unmount} = renderHook(useBTLog);
        act(() => stream.receive({event_log: [event()]}));
        act(() => {vi.advanceTimersByTime(250);});
        act(() => stream.receive({event_log: [event('SUCCESS')]}));
        act(() => {vi.advanceTimersByTime(249);}); expect(result.current.size).toBe(0);
        act(() => {vi.advanceTimersByTime(1);}); expect(result.current.get('Spin')?.status).toBe('SUCCESS');
        act(() => stream.receive({event_log: [event()]})); unmount();
        expect(vi.getTimerCount()).toBe(0); expect(stream.stop).toHaveBeenCalledOnce();
    });
    it('retains states on disconnect and does not resubscribe on state changes or rerenders', () => {
        const {result, rerender, unmount} = renderHook(useBTLog);
        act(() => stream.receive({event_log: [event()]})); flush(); const previous = result.current;
        act(() => {stream.error(new Error('closed')); stream.info('reconnected');}); rerender();
        expect(result.current).toBe(previous);
        expect(stream.start).toHaveBeenCalledExactlyOnceWith('/api/mowglinext/subscribe/btLog');
        expect(stream.stop).not.toHaveBeenCalled(); unmount(); expect(stream.stop).toHaveBeenCalledOnce();
    });
});
