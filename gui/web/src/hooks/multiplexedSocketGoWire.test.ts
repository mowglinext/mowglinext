// Regenerate from gui/: MOWGLI_NETWORK_BROWSER_FIXTURE="$PWD/web/src/test/fixtures/network-grid-wire.json" go test ./pkg/api -run "^TestNetworkBrowserFrames$" -count=1
import {unpack} from 'msgpackr';
import {afterEach, describe, expect, it, vi} from 'vitest';
import {MultiplexedSocket} from './multiplexedSocket';
import fixture from '../test/fixtures/network-grid-wire.json';

class WireSocket {
    static latest: WireSocket;
    binaryType = '';
    onopen: (() => void) | null = null;
    onmessage: ((event: MessageEvent) => void) | null = null;
    onclose: (() => void) | null = null;
    send = vi.fn();
    close = vi.fn();
    constructor() { WireSocket.latest = this; }
}

describe('Go-generated multiplex full frame followed by delta', () => {
    afterEach(() => { vi.unstubAllGlobals(); vi.useRealTimers(); });

    it('decodes the actual Go wire arrays with msgpackr and applies the patch', () => {
        vi.useFakeTimers();
        vi.stubGlobal('WebSocket', WireSocket);
        const socket = new MultiplexedSocket('ws://test/multiplex');
        const listener = vi.fn();
        const unsubscribe = socket.subscribe('lidarMap', listener);
        const ws = WireSocket.latest;
        ws.onopen?.();
        expect(JSON.parse(ws.send.mock.calls[0][0] as string)).toEqual({op: 'subscribe', topic: 'lidarMap', delta: true});
        for (const [i, frame] of fixture.frames.entries()) {
            const bytes = Uint8Array.from(atob(frame), char => char.charCodeAt(0));
            const decoded = unpack(bytes) as {data?: {data: unknown[]}; patch?: {vals: number[]}};
            if (i === 0) {
                expect(Array.isArray(decoded.data?.data)).toBe(true);
                expect(decoded.data?.data.every(cell => typeof cell === 'number')).toBe(true);
            } else {
                expect(decoded.patch?.vals).toEqual([-1, 100]);
            }
            ws.onmessage?.({data: bytes.buffer} as MessageEvent);
            expect(listener).toHaveBeenCalledTimes(i + 1);
            expect(listener.mock.calls[i][0]).toEqual(fixture.expected[i]);
            expect(listener.mock.calls[i][1]).toBe(i === 0);
        }
        expect(ws.send).toHaveBeenCalledTimes(1); // no spurious resync
        unsubscribe();
        expect(vi.getTimerCount()).toBe(0);
    });

});
