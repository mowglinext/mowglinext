import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import {act, renderHook} from '@testing-library/react';

// The poller keeps module-level state (one shared poller), so every test gets a fresh module.
const loadHook = async () => {
    vi.resetModules();
    return import('./useHostUpdater.ts');
};

const stateBody = (extra: Record<string, unknown> = {}) => JSON.stringify({
    api: 1,
    agent: {version: 'x', revision: 'y', platform: 'linux/arm64'},
    trusted_repositories: [],
    state: {policy: {}, releases: [], notices: [], history: [], last_check: '', last_success: '', next_check: '', ...extra},
});

type Reply = {status: number; body?: string; etag?: string};

const mockFetch = (replies: Reply[]) => {
    const calls: {headers: Record<string, string>}[] = [];
    let index = 0;
    vi.stubGlobal('fetch', vi.fn((_url: string, init?: {headers?: Record<string, string>}) => {
        calls.push({headers: {...(init?.headers ?? {})}});
        const reply = replies[Math.min(index++, replies.length - 1)];
        return Promise.resolve(new Response(reply.status === 304 ? null : reply.body, {
            status: reply.status,
            headers: reply.etag ? {ETag: reply.etag} : {},
        }));
    }));
    return calls;
};

const setHidden = (hidden: boolean) => {
    Object.defineProperty(document, 'hidden', {configurable: true, get: () => hidden});
    document.dispatchEvent(new Event('visibilitychange'));
};

describe('useHostUpdater', () => {
    beforeEach(() => {
        vi.useFakeTimers();
        setHidden(false);
    });
    afterEach(() => {
        vi.useRealTimers();
        vi.unstubAllGlobals();
    });

    it('loads everything on mount, then revalidates with the ETag and keeps its copy on 304', async () => {
        const {POLL_IDLE_MS, useHostUpdater} = await loadHook();
        const calls = mockFetch([{status: 200, body: stateBody(), etag: '"v1"'}, {status: 304}]);
        const {result} = renderHook(() => useHostUpdater());
        await act(async () => { await vi.advanceTimersByTimeAsync(0); });
        expect(result.current.data?.agent.version).toBe('x');
        expect(calls[0].headers['If-None-Match']).toBeUndefined();

        await act(async () => { await vi.advanceTimersByTimeAsync(POLL_IDLE_MS); });
        expect(calls).toHaveLength(2);
        expect(calls[1].headers['If-None-Match']).toBe('"v1"');
        expect(result.current.data?.agent.version).toBe('x');
        expect(result.current.error).toBeUndefined();
    });

    it('polls slowly when idle and fast while an update job is running', async () => {
        const {POLL_FAST_MS, POLL_IDLE_MS, useHostUpdater} = await loadHook();
        const calls = mockFetch([
            {status: 200, body: stateBody(), etag: '"idle"'},
            {status: 200, body: stateBody({active_job_id: 'job-0', job: {id: 'j1', kind: 'containers', phase: 'pulling'}}), etag: '"busy"'},
            {status: 200, body: stateBody({active_job_id: 'job-0', job: {id: 'j1', kind: 'containers', phase: 'pulling'}}), etag: '"busy"'},
        ]);
        renderHook(() => useHostUpdater());
        await act(async () => { await vi.advanceTimersByTimeAsync(0); });
        expect(calls).toHaveLength(1);

        // Idle: nothing happens at the fast rate...
        await act(async () => { await vi.advanceTimersByTimeAsync(POLL_FAST_MS * 2); });
        expect(calls).toHaveLength(1);
        // ...until the slow rate is reached; that poll finds the running job...
        await act(async () => { await vi.advanceTimersByTimeAsync(POLL_IDLE_MS - POLL_FAST_MS * 2); });
        expect(calls).toHaveLength(2);
        // ...and from then on it polls fast.
        await act(async () => { await vi.advanceTimersByTimeAsync(POLL_FAST_MS); });
        expect(calls).toHaveLength(3);
    });

    it('stays on the slow rate after an update has finished (active_job_id names the installed job, it is not a running one)', async () => {
        const {POLL_FAST_MS, POLL_IDLE_MS, useHostUpdater} = await loadHook();
        const calls = mockFetch([
            {status: 200, body: stateBody({active_job_id: 'job-1', job: {id: 'job-1', kind: 'containers', phase: 'succeeded'}}), etag: '"done"'},
            {status: 304},
        ]);
        renderHook(() => useHostUpdater());
        await act(async () => { await vi.advanceTimersByTimeAsync(0); });
        expect(calls).toHaveLength(1);

        await act(async () => { await vi.advanceTimersByTimeAsync(POLL_IDLE_MS - 1); });
        expect(calls).toHaveLength(1);
        expect(POLL_FAST_MS).toBeLessThan(POLL_IDLE_MS - 1);
        await act(async () => { await vi.advanceTimersByTimeAsync(1); });
        expect(calls).toHaveLength(2);
    });

    it('does not poll while the tab is hidden and catches up when it is shown', async () => {
        const {POLL_IDLE_MS, useHostUpdater} = await loadHook();
        const calls = mockFetch([{status: 200, body: stateBody(), etag: '"v1"'}, {status: 304}]);
        renderHook(() => useHostUpdater());
        await act(async () => { await vi.advanceTimersByTimeAsync(0); });
        expect(calls).toHaveLength(1);

        act(() => setHidden(true));
        await act(async () => { await vi.advanceTimersByTimeAsync(POLL_IDLE_MS * 5); });
        expect(calls).toHaveLength(1);

        act(() => setHidden(false));
        await act(async () => { await vi.advanceTimersByTimeAsync(0); });
        expect(calls).toHaveLength(2);
        expect(calls[1].headers['If-None-Match']).toBe('"v1"');
    });

    it('the open updater panel (live) polls fast and always asks for a full copy', async () => {
        const {POLL_FAST_MS, useHostUpdater} = await loadHook();
        const calls = mockFetch([{status: 200, body: stateBody(), etag: '"v1"'}]);
        renderHook(() => useHostUpdater({live: true}));
        await act(async () => { await vi.advanceTimersByTimeAsync(POLL_FAST_MS * 2); });
        expect(calls.length).toBeGreaterThanOrEqual(3);
        for (const call of calls) expect(call.headers['If-None-Match']).toBeUndefined();
    });

    it('retries at the fast rate after a failure and drops the stale ETag', async () => {
        const {POLL_FAST_MS, useHostUpdater} = await loadHook();
        const calls = mockFetch([
            {status: 200, body: stateBody(), etag: '"v1"'},
            {status: 503, body: JSON.stringify({error: 'Host updater unavailable'})},
            {status: 200, body: stateBody(), etag: '"v2"'},
        ]);
        const {result} = renderHook(() => useHostUpdater({live: true}));
        await act(async () => { await vi.advanceTimersByTimeAsync(POLL_FAST_MS); });
        await act(async () => { await vi.advanceTimersByTimeAsync(POLL_FAST_MS); });
        expect(calls.length).toBeGreaterThanOrEqual(3);
        expect(result.current.error).toBeUndefined();
        expect(result.current.data).toBeDefined();
    });

    it('stops polling when the last component unmounts', async () => {
        const {POLL_IDLE_MS, useHostUpdater} = await loadHook();
        const calls = mockFetch([{status: 200, body: stateBody(), etag: '"v1"'}]);
        const {unmount} = renderHook(() => useHostUpdater());
        await act(async () => { await vi.advanceTimersByTimeAsync(0); });
        unmount();
        await act(async () => { await vi.advanceTimersByTimeAsync(POLL_IDLE_MS * 3); });
        expect(calls).toHaveLength(1);
    });
});
