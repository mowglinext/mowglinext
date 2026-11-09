import {StrictMode} from 'react';
import {act, cleanup, fireEvent, render, screen, within} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import {App, message} from 'antd';

vi.mock('../hooks/useDiagnosticsSnapshot.ts', () => ({useDiagnosticsSnapshot: () => ({snapshot: {coverage: []}})}));
// Keep the real shared useApi and generated HTTP client, including its JSON
// parse-error behavior. Only fetch delivery and the unrelated snapshot are fake.
import StatisticsPage from './StatisticsPage';

function deferred<T>() {
    let resolve!: (value: T) => void;
    let reject!: (reason: Error) => void;
    const promise = new Promise<T>((yes, no) => { resolve = yes; reject = no; });
    return {promise, resolve, reject};
}

function response(data: unknown, status = 200, invalidJson = false): Response {
    // An already-delivered HTTP response; no native streaming/timing dependency.
    return {
        ok: status >= 200 && status < 300, status,
        clone() { return this; },
        json: () => invalidJson ? Promise.reject(new SyntaxError('invalid JSON')) : Promise.resolve(data),
    } as Response;
}

type Request = ReturnType<typeof deferred<Response>> & {path: string; method: string};
type View = ReturnType<typeof render>;
let requests: Request[] = [];

function history(count: number, tag = 'initial') {
    return {
        sessions: {sessions: Array.from({length: count}, (_, i) => ({
            id: `${tag}-${i}`, start_time: '2026-10-22T10:00:00Z', end_time: '2026-10-22T11:00:00Z',
            duration_sec: 3600, area_index: i, coverage_percent: 80,
            strips_completed: 8, strips_skipped: 2, distance_meters: 1000,
            status: 'completed', errors: [],
        })), total: count},
        stats: {total_sessions: count, total_duration_sec: count * 3600, total_distance_m: count * 1000,
            total_strips: count * 8, completed: count, aborted: 0, errors: 0, avg_coverage_pct: count ? 80 : 0},
    };
}

async function settle(work: () => void = () => {}) {
    await act(async () => { work(); await Promise.resolve(); });
}

function pair(offset: number) {
    const session = requests[offset];
    const stats = requests[offset + 1];
    expect(session).toMatchObject({method: 'GET', path: '/api/diagnostics/sessions'});
    expect(stats).toMatchObject({method: 'GET', path: '/api/diagnostics/sessions/stats'});
    return {session, stats};
}

async function reply(offset: number, count: number, tag = 'initial') {
    const read = pair(offset), data = history(count, tag);
    await settle(() => { read.session.resolve(response(data.sessions)); read.stats.resolve(response(data.stats)); });
}

async function mount(strict = false) {
    const page = <App><StatisticsPage/></App>;
    const view = render(strict ? <StrictMode>{page}</StrictMode> : page);
    await settle();
    return view;
}

async function loaded() {
    const view = await mount();
    await reply(0, 1);
    expectHistory(view, 1);
    return view;
}

async function poll() {
    const offset = requests.length;
    await settle(() => vi.advanceTimersByTime(30000));
    return offset;
}

function clearButton(view: View) { return within(view.container).getByRole('button', {name: /Clear$/}); }

async function clear(view: View) {
    const offset = requests.length;
    fireEvent.click(clearButton(view));
    const tooltips = screen.getAllByRole('tooltip');
    fireEvent.click(within(tooltips[tooltips.length - 1]).getByRole('button', {name: /Clear$/}));
    await settle();
    expect(requests[offset]).toMatchObject({method: 'DELETE', path: '/api/diagnostics/sessions'});
    return offset;
}

function expectHistory(view: View, count: number, tag = 'initial') {
    expect(within(view.container).getByText(`${count} ${count === 1 ? 'session' : 'sessions'} recorded`)).toBeInTheDocument();
    expect(within(view.container).getByText(`${count} completed`)).toBeInTheDocument();
    expect(Array.from(view.container.querySelectorAll('tr[data-row-key]'), row => row.getAttribute('data-row-key')))
        .toEqual(Array.from({length: count}, (_, i) => `${tag}-${i}`));
    if (count === 0) {
        expect(clearButton(view)).toBeDisabled();
        expect(within(view.container).getByText('No mowing sessions recorded yet.')).toBeInTheDocument();
    } else expect(clearButton(view)).not.toBeDisabled();
}

function loading(view: View) {
    // AntD Spin debounces even a zero-delay change; drain that UI task only.
    act(() => { vi.advanceTimersByTime(0); });
    return view.container.querySelector('.ant-spin-spinning');
}

describe('StatisticsPage read and clear authority', () => {
    beforeEach(() => {
        vi.useFakeTimers();
        vi.setSystemTime(new Date('2026-10-22T12:00:00Z'));
        requests = [];
        vi.stubGlobal('fetch', vi.fn((path: string, init?: RequestInit) => {
            const request = {...deferred<Response>(), path, method: init?.method ?? 'GET'};
            requests.push(request);
            return request.promise;
        }));
        vi.spyOn(message, 'success');
        vi.spyOn(message, 'error');
        vi.spyOn(console, 'error').mockImplementation(() => {});
    });
    afterEach(() => {
        cleanup();
        act(() => message.destroy());
        vi.clearAllTimers();
        vi.useRealTimers();
        vi.unstubAllGlobals();
        vi.restoreAllMocks();
    });

    it.each(['sessions', 'stats'] as const)('distinguishes initial %s read failure from a successful empty history and retries', async endpoint => {
        const view = await mount();
        expect(screen.getByRole('status')).toHaveTextContent('Loading mowing history');
        expect(screen.queryByText('No mowing yet')).not.toBeInTheDocument();
        const read = pair(0);
        await settle(() => {
            read.session.resolve(response(endpoint === 'sessions' ? {error: 'unavailable'} : history(0).sessions, endpoint === 'sessions' ? 503 : 200));
            read.stats.resolve(response(endpoint === 'stats' ? {error: 'unavailable'} : history(0).stats, endpoint === 'stats' ? 503 : 200));
        });
        expect(screen.getByRole('alert')).toHaveTextContent('Mowing history could not be loaded.');
        expect(screen.queryByText('No mowing yet')).not.toBeInTheDocument();
        expect(screen.queryByText('No mowing sessions recorded yet.')).not.toBeInTheDocument();
        fireEvent.click(screen.getByRole('button', {name: 'Retry'}));
        await settle();
        await reply(2, 0);
        expectHistory(view, 0);
        expect(screen.getByText('No mowing yet')).toBeVisible();
        expect(screen.queryByRole('alert')).not.toBeInTheDocument();
    });

    it.each([0, 1])('marks retained history (%s sessions) on refresh failure until a retry succeeds', async count => {
        const view = await mount();
        await reply(0, count);
        const offset = await poll(), read = pair(offset);
        await settle(() => { read.session.reject(new Error('offline')); read.stats.resolve(response(history(2).stats)); });
        expectHistory(view, count);
        expect(screen.getByRole('alert')).toHaveTextContent('Showing the last confirmed data.');
        fireEvent.click(screen.getByRole('button', {name: 'Retry'}));
        await settle();
        expect(screen.getByRole('alert')).toHaveTextContent('Showing the last confirmed data.');
        await reply(offset + 2, 2, 'retry');
        expectHistory(view, 2, 'retry');
        expect(screen.queryByRole('alert')).not.toBeInTheDocument();
    });

    it('does not report retired read failures after newer success or confirmed deletion', async () => {
        const view = await loaded();
        const older = await poll(), newer = await poll();
        await reply(newer, 2, 'new');
        await settle(() => pair(older).session.reject(new Error('retired')));
        expectHistory(view, 2, 'new');
        expect(screen.queryByRole('alert')).not.toBeInTheDocument();
        const old = await poll(), deletion = await clear(view);
        await settle(() => requests[deletion].resolve(response({ok: true})));
        await reply(deletion + 1, 0);
        await settle(() => pair(old).session.reject(new Error('pre-delete')));
        expectHistory(view, 0);
        expect(screen.queryByRole('alert')).not.toBeInTheDocument();
    });

    it('labels all weekly values as focusable controls, including zero runtime', async () => {
        await loaded();
        const bars = screen.getAllByRole('button', {name: /^Week \d+: .* h$/});
        expect(bars).toHaveLength(12);
        expect(bars[0]).toHaveAccessibleName('Week 1: 0 h');
        expect(bars[11]).toHaveAccessibleName('Week 12: 1 h');
        for (const bar of bars) expect(bar.tabIndex).toBe(0);
    });

    it.each(['sessions-first', 'stats-first'] as const)('does not restore a pre-clear poll (%s)', async order => {
        const view = await loaded();
        const old = await poll();
        const deletion = await clear(view);
        await settle(() => requests[deletion].resolve(response({ok: true})));
        await reply(deletion + 1, 0);
        expectHistory(view, 0);
        const stale = pair(old), data = history(2, 'stale');
        await settle(() => order === 'sessions-first'
            ? stale.session.resolve(response(data.sessions)) : stale.stats.resolve(response(data.stats)));
        expectHistory(view, 0);
        await settle(() => order === 'sessions-first'
            ? stale.stats.resolve(response(data.stats)) : stale.session.resolve(response(data.sessions)));
        expectHistory(view, 0);
        expect(message.success).toHaveBeenCalledOnce();
    });

    it('installs only a complete authorized pair when refreshes interleave', async () => {
        const view = await loaded();
        const older = await poll(), newer = await poll();
        const oldRead = pair(older), nextRead = pair(newer);
        await settle(() => oldRead.session.resolve(response(history(2, 'old').sessions)));
        await settle(() => nextRead.stats.resolve(response(history(3, 'new').stats)));
        expectHistory(view, 1);
        await settle(() => nextRead.session.resolve(response(history(3, 'new').sessions)));
        expectHistory(view, 3, 'new');
        await settle(() => oldRead.stats.resolve(response(history(2, 'old').stats)));
        expectHistory(view, 3, 'new');
    });

    it('keeps loading and confirmed data while the newest read is still pending', async () => {
        const view = await loaded();
        const older = await poll(), newer = await poll();
        expect(loading(view)).not.toBeNull();
        await reply(older, 2, 'retired');
        expect(loading(view)).not.toBeNull();
        expectHistory(view, 1);
        await reply(newer, 3, 'new');
        expect(loading(view)).toBeNull();
        expectHistory(view, 3, 'new');
    });

    it('invalidates old reads at clear intent and skips polls while DELETE is pending', async () => {
        const view = await loaded();
        const old = await poll(), deletion = await clear(view);
        await reply(old, 2, 'retired');
        expectHistory(view, 1);
        expect(loading(view)).not.toBeNull();
        const count = requests.length;
        await poll();
        expect(requests).toHaveLength(count);
        await settle(() => requests[deletion].resolve(response({ok: true})));
        await reply(deletion + 1, 0);
        expectHistory(view, 0);
        expect(loading(view)).toBeNull();
    });

    it('allows a genuinely new history in the fresh read after DELETE', async () => {
        const view = await loaded(), deletion = await clear(view);
        await settle(() => requests[deletion].resolve(response({ok: true})));
        await reply(deletion + 1, 2, 'post-clear');
        expectHistory(view, 2, 'post-clear');
        expect(message.success).toHaveBeenCalledWith('Statistics cleared');
    });

    it.each(['http', 'network', 'invalid-json', 'ok-false', 'error-body'] as const)('preserves data on DELETE %s failure', async kind => {
        const view = await loaded(), deletion = await clear(view);
        await settle(() => {
            if (kind === 'network') requests[deletion].reject(new Error('network unavailable'));
            else if (kind === 'http') requests[deletion].resolve(response({error: 'DB unavailable'}, 500));
            else if (kind === 'invalid-json') requests[deletion].resolve(response(null, 200, true));
            else requests[deletion].resolve(response(kind === 'ok-false' ? {ok: false} : {error: 'refused'}));
        });
        expectHistory(view, 1);
        expect(message.success).not.toHaveBeenCalled();
        expect(message.error).toHaveBeenCalledWith('Failed to clear statistics');
        expect(requests).toHaveLength(deletion + 1);
        expect(loading(view)).toBeNull();
        const fresh = await poll();
        await reply(fresh, 2, 'recovered');
        expectHistory(view, 2, 'recovered');
    });

    it.each([
        ['sessions', 'http'], ['stats', 'http'],
        ['sessions', 'invalid-json'], ['stats', 'invalid-json'],
        ['sessions', 'error-body'], ['stats', 'error-body'],
        ['sessions', 'network'], ['stats', 'network'],
    ] as const)('does not install half a pair on %s %s error', async (endpoint, kind) => {
        const view = await loaded(), offset = await poll(), read = pair(offset);
        const bad = endpoint === 'sessions' ? read.session : read.stats;
        const good = endpoint === 'sessions' ? read.stats : read.session;
        await settle(() => {
            good.resolve(response(endpoint === 'sessions' ? history(2).stats : history(2).sessions));
            if (kind === 'network') bad.reject(new Error('network unavailable'));
            else bad.resolve(response({error: 'read refused'}, kind === 'http' ? 500 : 200, kind === 'invalid-json'));
        });
        expectHistory(view, 1);
        expect(loading(view)).toBeNull();
        expect(console.error).toHaveBeenCalled();
        const fresh = await poll();
        await reply(fresh, 2, 'recovered');
        expectHistory(view, 2, 'recovered');
    });

    it('keeps a confirmed deletion empty if the post-clear read fails', async () => {
        const view = await loaded(), deletion = await clear(view);
        await settle(() => requests[deletion].resolve(response({ok: true})));
        const read = pair(deletion + 1);
        await settle(() => {
            read.session.reject(new Error('read lost'));
            read.stats.resolve(response(history(0).stats));
        });
        expectHistory(view, 0);
        expect(screen.getByRole('alert')).toHaveTextContent('Showing the last confirmed data.');
        expect(message.success).toHaveBeenCalledOnce();
        expect(loading(view)).toBeNull();
    });

    it('does not apply a retired GET to a remounted page', async () => {
        const first = await mount();
        first.unmount();
        const second = await mount();
        await reply(2, 2, 'new-instance');
        await reply(0, 3, 'retired');
        expectHistory(second, 2, 'new-instance');
        expect(loading(second)).toBeNull();
    });

    it.each(['success', 'failure'] as const)('does not continue a retired DELETE %s after remount', async result => {
        const first = await loaded(), deletion = await clear(first);
        first.unmount();
        const second = await mount();
        await reply(deletion + 1, 2, 'new-instance');
        const count = requests.length;
        await settle(() => result === 'success'
            ? requests[deletion].resolve(response({ok: true}))
            : requests[deletion].reject(new Error('retired failure')));
        expect(requests).toHaveLength(count);
        expectHistory(second, 2, 'new-instance');
        expect(message.success).not.toHaveBeenCalled();
        expect(message.error).not.toHaveBeenCalled();
    });

    it('distinguishes a failed first clear, a successful retry and both older polls', async () => {
        const view = await loaded(), oldPoll = await poll(), first = await clear(view);
        await settle(() => requests[first].reject(new Error('first failure')));
        expectHistory(view, 1);
        const nextPoll = await poll(), second = await clear(view);
        await settle(() => requests[second].resolve(response({ok: true})));
        await reply(second + 1, 0);
        await reply(nextPoll, 2, 'retired-second');
        await reply(oldPoll, 3, 'retired-first');
        expectHistory(view, 0);
        expect(message.error).toHaveBeenCalledOnce();
        expect(message.success).toHaveBeenCalledOnce();
    });

    it('ignores the initial GET pair retired by StrictMode', async () => {
        const view = await mount(true);
        expect(requests).toHaveLength(4);
        await reply(2, 2, 'active');
        await reply(0, 3, 'retired');
        expectHistory(view, 2, 'active');
    });

    it('retains nominal sessions, aggregates, annual chart and exact 30-second polling', async () => {
        const view = await loaded();
        expect(within(view.container).getByText('Active days').parentElement).toHaveTextContent('1');
        expect(view.container.querySelectorAll('svg rect')).toHaveLength(364);
        expect(screen.getByTitle('Week 12: 1 h')).toBeInTheDocument();
        await settle(() => vi.advanceTimersByTime(29999));
        expect(requests).toHaveLength(2);
        await settle(() => vi.advanceTimersByTime(1));
        expect(requests).toHaveLength(4);
        await reply(2, 2, 'next');
        expectHistory(view, 2, 'next');
        view.unmount();
        await poll();
        expect(requests).toHaveLength(4);
    });
});
