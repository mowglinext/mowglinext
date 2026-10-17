import {afterEach, describe, expect, it, vi} from 'vitest';
import {GamepadDiagnostics, type GamepadDiagnosticReport} from './diagnostics';

function parseReport(request?: RequestInit): GamepadDiagnosticReport {
    if (typeof request?.body !== 'string') throw new Error('Missing diagnostic body');
    return JSON.parse(request.body) as GamepadDiagnosticReport;
}

afterEach(() => { vi.unstubAllGlobals(); vi.useRealTimers(); });

describe('controller diagnostic recorder', () => {
    it('keeps a bounded history across retries with the last health snapshot', async () => {
        const send = vi.fn<typeof fetch>().mockResolvedValue({ok: true} as Response);
        vi.stubGlobal('fetch', send);
        const diagnostics = new GamepadDiagnostics();
        diagnostics.update({attempt: 'first', mode: 'MANUAL_MOWING'});
        for (let i = 0; i < 40; i++) diagnostics.record('controlState', {i});
        diagnostics.record('sessionEnded', {fault: 'connectionLost'});
        diagnostics.record('enableRequested', {attempt: 'retry'});
        diagnostics.update({attempt: 'retry', state: 'waitingForMode'});
        await diagnostics.flush();
        const [url, request] = send.mock.calls[0] as unknown as [string, RequestInit];
        const report = parseReport(request);
        expect(url).toBe('/api/mowglinext/gamepad/diagnostics');
        expect(request.keepalive).toBe(true);
        expect(report.version).toBe(1);
        expect(report.events).toHaveLength(32);
        expect(report.events.slice(-2).map((event: {event: string}) => event.event)).toEqual(['sessionEnded', 'enableRequested']);
        expect(report.snapshot).toEqual({attempt: 'retry', mode: 'MANUAL_MOWING', state: 'waitingForMode'});
        expect(new TextEncoder().encode(request.body as string).length).toBeLessThan(16_384);
    });

    it('sends a fault captured during a stalled upload after the upload times out', async () => {
        vi.useFakeTimers();
        const send = vi.fn<typeof fetch>()
            .mockImplementationOnce((_url, request) => new Promise((_resolve, reject) => {
                request?.signal?.addEventListener('abort', () => reject(new Error('timeout')), {once: true});
            }))
            .mockResolvedValue({ok: true} as Response);
        vi.stubGlobal('fetch', send);
        const diagnostics = new GamepadDiagnostics();
        diagnostics.record('enableRequested');
        const first = diagnostics.flush();
        diagnostics.record('sessionEnded', {fault: 'inputLost'});
        diagnostics.flushSoon();
        await vi.advanceTimersByTimeAsync(2000);
        await first;
        expect(send).toHaveBeenCalledTimes(2);
        const report = parseReport(send.mock.calls[1][1]);
        expect(report.events[report.events.length - 1].detail.fault).toBe('inputLost');
        await diagnostics.flush();
        expect(send).toHaveBeenCalledTimes(2);
    });

    it('retains diagnostics after a rejected request without throwing into input control', async () => {
        const send = vi.fn<typeof fetch>().mockRejectedValueOnce(new Error('offline')).mockResolvedValue({ok: true} as Response);
        vi.stubGlobal('fetch', send);
        const diagnostics = new GamepadDiagnostics();
        diagnostics.record('sessionEnded', {fault: 'connectionLost'});
        await expect(diagnostics.flush()).resolves.toBeUndefined();
        await diagnostics.flush();
        expect(send).toHaveBeenCalledTimes(2);
        expect(parseReport(send.mock.calls[1][1]).events[0].detail.fault).toBe('connectionLost');
    });
});
