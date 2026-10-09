import {pack} from 'msgpackr';
import {afterEach, beforeEach, describe, expect, it, vi, type Mock} from 'vitest';
import {DELTA_TOPICS, MultiplexedSocket} from './multiplexedSocket';

class FakeWebSocket {
    static instances: FakeWebSocket[] = [];
    binaryType = '';
    onopen: (() => void) | null = null;
    onmessage: ((event: MessageEvent) => void) | null = null;
    onerror: (() => void) | null = null;
    onclose: (() => void) | null = null;
    send = vi.fn<(value: string) => void>();
    close = vi.fn();

    constructor(public url: string) {
        FakeWebSocket.instances.push(this);
    }

    frame(value: unknown): void {
        const bytes = pack(value) as Uint8Array;
        this.onmessage?.({data: Uint8Array.from(bytes).buffer} as MessageEvent);
    }

    operations(): unknown[] {
        return this.send.mock.calls.map(([value]) => JSON.parse(value) as unknown);
    }
}

type Grid = {header: {stamp: number}; info: {width: number; height: number}; data: number[]};
const grid = (stamp: number, data: number[]): Grid => ({header: {stamp}, info: {width: data.length, height: 1}, data});

describe('MultiplexedSocket grid deltas', () => {
    let socket: MultiplexedSocket;
    let cleanups: (() => void)[];
    let ws: FakeWebSocket;
    let listener: Mock<(data: unknown, first: boolean) => void>;

    const open = (topic = 'mowProgress') => {
        listener = vi.fn<(data: unknown, first: boolean) => void>();
        cleanups.push(socket.subscribe(topic, listener));
        ws = FakeWebSocket.instances[FakeWebSocket.instances.length - 1];
        ws.onopen?.();
    };
    const lastGrid = () => listener.mock.calls[listener.mock.calls.length - 1][0] as Grid;

    beforeEach(() => {
        vi.useFakeTimers();
        vi.stubGlobal('WebSocket', FakeWebSocket);
        FakeWebSocket.instances = [];
        cleanups = [];
        socket = new MultiplexedSocket('ws://robot/api/mowglinext/multiplex');
    });

    afterEach(() => {
        cleanups.forEach(unsubscribe => unsubscribe());
        expect(vi.getTimerCount()).toBe(0);
        vi.unstubAllGlobals();
        vi.useRealTimers();
        vi.restoreAllMocks();
    });

    it('asks for deltas on the grid topics only', () => {
        expect(DELTA_TOPICS.has('mowProgress')).toBe(true);
        open('mowProgress');
        expect(ws.operations()).toEqual([{op: 'subscribe', topic: 'mowProgress', delta: true}]);
        const other = vi.fn<(data: unknown, first: boolean) => void>();
        cleanups.push(socket.subscribe('pose', other));
        expect(ws.operations()[1]).toEqual({op: 'subscribe', topic: 'pose'});
    });

    it('hands listeners a complete grid after each patch', () => {
        open();
        ws.frame({topic: 'mowProgress', seq: 1, data: grid(1, [0, 0, 0, 0, 0, 0])});
        expect(lastGrid().data).toEqual([0, 0, 0, 0, 0, 0]);

        ws.frame({topic: 'mowProgress', patch: {base: 1, seq: 2, header: {stamp: 2}, gaps: [1, 3], vals: [100, 50]}});
        expect(listener).toHaveBeenCalledTimes(2);
        expect(lastGrid().data).toEqual([0, 100, 0, 0, 50, 0]);
        expect(lastGrid().header).toEqual({stamp: 2});
        expect(lastGrid().info).toEqual({width: 6, height: 1});

        ws.frame({topic: 'mowProgress', patch: {base: 2, seq: 3, header: {stamp: 3}, gaps: [0], vals: [-1]}});
        expect(lastGrid().data).toEqual([-1, 100, 0, 0, 50, 0]);
        expect(ws.operations().filter(o => (o as {op: string}).op === 'resync')).toEqual([]);
    });

    it('delivers a new object per patch but never an old picture', () => {
        open();
        ws.frame({topic: 'mowProgress', seq: 1, data: grid(1, [0, 0, 0])});
        const first = lastGrid();
        ws.frame({topic: 'mowProgress', patch: {base: 1, seq: 2, header: {stamp: 2}, gaps: [2], vals: [100]}});
        expect(lastGrid()).not.toBe(first);
        expect(lastGrid().data[2]).toBe(100);
    });

    it('drops a patch with the wrong base, asks once for a full grid, and recovers with it', () => {
        open();
        ws.frame({topic: 'mowProgress', seq: 1, data: grid(1, [0, 0, 0])});
        listener.mockClear();
        ws.send.mockClear();

        ws.frame({topic: 'mowProgress', patch: {base: 5, seq: 6, header: {stamp: 6}, gaps: [1], vals: [100]}});
        ws.frame({topic: 'mowProgress', patch: {base: 6, seq: 7, header: {stamp: 7}, gaps: [1], vals: [100]}});
        expect(listener).not.toHaveBeenCalled();
        expect(ws.operations()).toEqual([{op: 'resync', topic: 'mowProgress'}]);

        ws.frame({topic: 'mowProgress', seq: 8, data: grid(8, [0, 100, 0])});
        expect(lastGrid().data).toEqual([0, 100, 0]);
        ws.frame({topic: 'mowProgress', patch: {base: 8, seq: 9, header: {stamp: 9}, gaps: [2], vals: [50]}});
        expect(lastGrid().data).toEqual([0, 100, 50]);
        expect(ws.operations()).toHaveLength(1);
    });

    it('asks for a full grid when a patch arrives before any grid, or points outside the grid', () => {
        open();
        ws.send.mockClear();
        ws.frame({topic: 'mowProgress', patch: {base: 1, seq: 2, header: {stamp: 2}, gaps: [0], vals: [1]}});
        expect(listener).not.toHaveBeenCalled();
        expect(ws.operations()).toEqual([{op: 'resync', topic: 'mowProgress'}]);

        ws.frame({topic: 'mowProgress', seq: 3, data: grid(3, [0, 0])});
        ws.send.mockClear();
        listener.mockClear();
        ws.frame({topic: 'mowProgress', patch: {base: 3, seq: 4, header: {stamp: 4}, gaps: [5], vals: [1]}});
        expect(listener).not.toHaveBeenCalled();
        expect(ws.operations()).toEqual([{op: 'resync', topic: 'mowProgress'}]);
    });

    it('forgets the held grid when the last listener leaves, so an old patch cannot apply to a new subscription', () => {
        open();
        ws.frame({topic: 'mowProgress', seq: 1, data: grid(1, [0, 0, 0])});
        cleanups.pop()?.(); // the connection is retired with its last listener

        listener = vi.fn<(data: unknown, first: boolean) => void>();
        cleanups.push(socket.subscribe('mowProgress', listener));
        ws = FakeWebSocket.instances[FakeWebSocket.instances.length - 1];
        ws.onopen?.();
        ws.send.mockClear();
        ws.frame({topic: 'mowProgress', patch: {base: 1, seq: 2, header: {stamp: 2}, gaps: [1], vals: [100]}});
        expect(listener).not.toHaveBeenCalled();
        expect(ws.operations()).toEqual([{op: 'resync', topic: 'mowProgress'}]);
    });

    it('treats a frame without a seq as an ordinary full frame', () => {
        open();
        ws.frame({topic: 'mowProgress', data: grid(1, [0, 0])});
        expect(lastGrid().data).toEqual([0, 0]);
        ws.send.mockClear();
        ws.frame({topic: 'mowProgress', patch: {base: 1, seq: 2, header: {stamp: 2}, gaps: [0], vals: [1]}});
        expect(ws.operations()).toEqual([{op: 'resync', topic: 'mowProgress'}]);
    });
});
