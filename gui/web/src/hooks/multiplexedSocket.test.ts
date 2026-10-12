import {pack} from 'msgpackr';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import {MultiplexedSocket} from './multiplexedSocket';

class FakeWebSocket {
    static instances: FakeWebSocket[] = [];
    binaryType = '';
    onopen: (() => void) | null = null;
    onmessage: ((event: MessageEvent) => void) | null = null;
    onerror: (() => void) | null = null;
    onclose: (() => void) | null = null;
    send = vi.fn<(value: string) => void>();
    // Deliberately do not emit onclose: a half-open socket may never do so.
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

describe('MultiplexedSocket recovery', () => {
    let socket: MultiplexedSocket;
    let cleanups: (() => void)[];

    function subscribe(topic: string, listener = vi.fn()) {
        const unsubscribe = socket.subscribe(topic, listener);
        cleanups.push(unsubscribe);
        return {listener, unsubscribe};
    }

    function current(): FakeWebSocket {
        return FakeWebSocket.instances[FakeWebSocket.instances.length - 1];
    }

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

    it('recovers a silent open socket without waiting for close and restores active topics', () => {
        const {listener} = subscribe('status');
        subscribe('map');
        const old = current();
        old.onopen?.();
        expect(old.binaryType).toBe('arraybuffer');
        vi.advanceTimersByTime(29_999);
        expect(old.close).not.toHaveBeenCalled();
        vi.advanceTimersByTime(1);
        expect(old.close).toHaveBeenCalledTimes(1);
        expect(socket.getStatus()).toBe('closed');
        vi.advanceTimersByTime(1_000);
        const replacement = current();
        expect(replacement).not.toBe(old);
        replacement.onopen?.();
        expect(replacement.operations()).toEqual([
            {op: 'subscribe', topic: 'status'}, {op: 'subscribe', topic: 'map'},
        ]);
        replacement.frame({topic: 'status', data: {state: 'idle'}});
        expect(listener).toHaveBeenCalledWith({state: 'idle'}, true);
    });

    it('extends the deadline on valid subscribed traffic without resetting first delivery', () => {
        const {listener} = subscribe('status');
        const old = current();
        old.onopen?.();
        vi.advanceTimersByTime(29_000);
        old.frame({topic: 'status', data: 1});
        vi.advanceTimersByTime(29_999);
        expect(old.close).not.toHaveBeenCalled();
        vi.advanceTimersByTime(1_001);
        current().onopen?.();
        current().frame({topic: 'status', data: 2});
        expect(listener.mock.calls).toEqual([[1, true], [2, false]]);
    });

    it('leaves a latched-only connection quiet and grants newly subscribed telemetry a full deadline', () => {
        subscribe('map');
        subscribe('path');
        subscribe('plan');
        const ws = current();
        ws.onopen?.();
        vi.advanceTimersByTime(120_000);
        expect(ws.close).not.toHaveBeenCalled();
        subscribe('status');
        vi.advanceTimersByTime(29_999);
        expect(ws.close).not.toHaveBeenCalled();
        vi.advanceTimersByTime(1);
        expect(ws.close).toHaveBeenCalledTimes(1);
    });

    it('stops checking silence when the last continuous topic is removed', () => {
        subscribe('map');
        const {unsubscribe} = subscribe('status');
        const ws = current();
        ws.onopen?.();
        vi.advanceTimersByTime(20_000);
        unsubscribe();
        vi.advanceTimersByTime(120_000);
        expect(ws.close).not.toHaveBeenCalled();
        expect(ws.operations()).toContainEqual({op: 'unsubscribe', topic: 'status'});
        expect(vi.getTimerCount()).toBe(0);
    });

    it('keeps a shared topic subscribed until its final listener leaves', () => {
        const first = subscribe('status');
        const second = subscribe('status');
        const ws = current();
        ws.onopen?.();
        first.unsubscribe();
        ws.frame({topic: 'status', data: 7});
        expect(first.listener).not.toHaveBeenCalled();
        expect(second.listener).toHaveBeenCalledWith(7, true);
        expect(ws.operations()).toEqual([{op: 'subscribe', topic: 'status'}]);
        second.unsubscribe();
        expect(ws.close).toHaveBeenCalledTimes(1);
        vi.advanceTimersByTime(120_000);
        expect(FakeWebSocket.instances).toHaveLength(1);
    });

    it.each(['connecting', 'open', 'awaiting reconnect'])('cancels all work on final unsubscribe while %s', phase => {
        const {unsubscribe} = subscribe('status');
        const ws = current();
        if (phase !== 'connecting') ws.onopen?.();
        if (phase === 'awaiting reconnect') ws.onclose?.();
        unsubscribe();
        expect(socket.getStatus()).toBe('closed');
        vi.advanceTimersByTime(120_000);
        expect(FakeWebSocket.instances).toHaveLength(1);
        expect(ws.onopen).toBeNull();
        expect(ws.onclose).toBeNull();
    });

    it('ignores queued events from a retired generation, including during immediate resubscription', () => {
        const first = subscribe('status');
        const old = current();
        const lateOpen = old.onopen;
        const lateMessage = old.onmessage;
        const lateError = old.onerror;
        const lateClose = old.onclose;
        first.unsubscribe();
        const second = subscribe('status');
        const replacement = current();
        replacement.onopen?.();
        lateOpen?.();
        const bytes = pack({topic: 'status', data: 'old'}) as Uint8Array;
        lateMessage?.({data: Uint8Array.from(bytes).buffer} as MessageEvent);
        lateError?.();
        lateClose?.();
        expect(socket.getStatus()).toBe('open');
        expect(replacement.close).not.toHaveBeenCalled();
        expect(replacement.operations()).toEqual([{op: 'subscribe', topic: 'status'}]);
        expect(second.listener).not.toHaveBeenCalled();
        replacement.frame({topic: 'status', data: 'new'});
        expect(second.listener).toHaveBeenCalledWith('new', true);
    });

    it('retains exponential backoff when replacement handshakes fail', () => {
        subscribe('status');
        current().onopen?.();
        current().onclose?.();
        vi.advanceTimersByTime(999);
        expect(FakeWebSocket.instances).toHaveLength(1);
        vi.advanceTimersByTime(1);
        current().onclose?.();
        vi.advanceTimersByTime(1_999);
        expect(FakeWebSocket.instances).toHaveLength(2);
        vi.advanceTimersByTime(1);
        expect(FakeWebSocket.instances).toHaveLength(3);
    });

    it('does not let malformed or unrelated frames conceal a silent stream', () => {
        vi.spyOn(console, 'warn').mockImplementation(() => {});
        subscribe('status');
        const ws = current();
        ws.onopen?.();
        vi.advanceTimersByTime(29_000);
        ws.onmessage?.({data: 'text'} as MessageEvent);
        ws.onmessage?.({data: new Uint8Array([0xc1]).buffer} as MessageEvent);
        ws.frame(null);
        ws.frame({topic: 'status'});
        ws.frame({topic: 'unsubscribed', data: 1});
        vi.advanceTimersByTime(1_000);
        expect(ws.close).toHaveBeenCalledTimes(1);
    });

    it('measures silence independently of wall-clock adjustments', () => {
        subscribe('status');
        const ws = current();
        ws.onopen?.();
        vi.advanceTimersByTime(20_000);
        vi.setSystemTime(Date.now() + 3_600_000);
        vi.advanceTimersByTime(9_999);
        expect(ws.close).not.toHaveBeenCalled();
        vi.advanceTimersByTime(1);
        expect(ws.close).toHaveBeenCalledTimes(1);
    });

    it('reconnects even if closing the silent socket throws', () => {
        subscribe('status');
        const ws = current();
        ws.onopen?.();
        ws.close.mockImplementation(() => { throw new Error('already gone'); });
        vi.advanceTimersByTime(31_000);
        expect(FakeWebSocket.instances).toHaveLength(2);
    });

    it('handles a status listener immediately subscribing on disconnect', () => {
        subscribe('status');
        current().onopen?.();
        const unregister = socket.onStatusChange(status => {
            if (status === 'closed') subscribe('map');
        });
        vi.advanceTimersByTime(30_000);
        unregister();
        expect(socket.getStatus()).toBe('connecting');
        expect(FakeWebSocket.instances).toHaveLength(2);
        expect(vi.getTimerCount()).toBe(0);
    });
});
