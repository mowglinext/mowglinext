import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import {MultiplexedSocket, PAUSE_WHEN_HIDDEN} from './multiplexedSocket';

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

    operations(): unknown[] {
        return this.send.mock.calls.map(([value]) => JSON.parse(value) as unknown);
    }
}

describe('MultiplexedSocket in a hidden tab', () => {
    let socket: MultiplexedSocket;
    let cleanups: (() => void)[];

    const subscribe = (topic: string) => {
        cleanups.push(socket.subscribe(topic, vi.fn()));
    };
    const current = () => FakeWebSocket.instances[FakeWebSocket.instances.length - 1];

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

    it('stops only the visual topics while hidden and restarts them when shown', () => {
        subscribe('highLevelStatus');
        subscribe('mowProgress');
        subscribe('lidar');
        const ws = current();
        ws.onopen?.();
        ws.send.mockClear();

        socket.setHidden(true);
        expect(ws.operations()).toEqual([
            {op: 'unsubscribe', topic: 'mowProgress'},
            {op: 'unsubscribe', topic: 'lidar'},
        ]);

        ws.send.mockClear();
        socket.setHidden(false);
        expect(ws.operations()).toEqual([
            {op: 'subscribe', topic: 'mowProgress', delta: true},
            {op: 'subscribe', topic: 'lidar'},
        ]);
    });

    it('never pauses what background code acts on', () => {
        for (const topic of ['highLevelStatus', 'status', 'emergency', 'power', 'diagnostics', 'gps', 'gnssStatus']) {
            expect(PAUSE_WHEN_HIDDEN.has(topic), topic).toBe(false);
        }
    });

    it('never pauses a topic that publishes only on change, which could come back empty', () => {
        for (const topic of ['map', 'plan', 'path', 'obstacles', 'recordingTrajectory', 'lidarMap', 'cogHeading', 'magYaw']) {
            expect(PAUSE_WHEN_HIDDEN.has(topic), topic).toBe(false);
        }
    });

    it('does nothing when the visibility did not change', () => {
        subscribe('mowProgress');
        const ws = current();
        ws.onopen?.();
        ws.send.mockClear();
        socket.setHidden(false);
        expect(ws.operations()).toEqual([]);
        socket.setHidden(true);
        ws.send.mockClear();
        socket.setHidden(true);
        expect(ws.operations()).toEqual([]);
    });

    it('does not subscribe a visual topic that is added while hidden, until the tab is shown', () => {
        subscribe('highLevelStatus');
        const ws = current();
        ws.onopen?.();
        socket.setHidden(true);
        ws.send.mockClear();

        subscribe('mowProgress');
        expect(ws.operations()).toEqual([]);

        socket.setHidden(false);
        expect(ws.operations()).toEqual([{op: 'subscribe', topic: 'mowProgress', delta: true}]);
    });

    it('connecting while hidden subscribes only the topics that are still served', () => {
        socket.setHidden(true);
        subscribe('status');
        subscribe('mowProgress');
        subscribe('pose');
        const ws = current();
        ws.onopen?.();
        expect(ws.operations()).toEqual([{op: 'subscribe', topic: 'status'}]);
    });

    it('does not send an unsubscribe for a paused topic whose last listener leaves', () => {
        subscribe('highLevelStatus');
        const leave = socket.subscribe('mowProgress', vi.fn());
        const ws = current();
        ws.onopen?.();
        socket.setHidden(true);
        ws.send.mockClear();
        leave();
        expect(ws.operations()).toEqual([]);
        socket.setHidden(false);
        expect(ws.operations()).toEqual([]);
    });

    it('does not mistake the paused streams for a dead connection', () => {
        subscribe('pose');
        const ws = current();
        ws.onopen?.();
        socket.setHidden(true);
        // pose is a continuous topic, silent on purpose now: no watchdog, no reconnect.
        vi.advanceTimersByTime(120_000);
        expect(ws.close).not.toHaveBeenCalled();
        expect(socket.getStatus()).toBe('open');
        // Shown again: the watchdog is back and grants a full grace period.
        socket.setHidden(false);
        vi.advanceTimersByTime(29_999);
        expect(ws.close).not.toHaveBeenCalled();
        vi.advanceTimersByTime(1);
        expect(ws.close).toHaveBeenCalledTimes(1);
    });
});
