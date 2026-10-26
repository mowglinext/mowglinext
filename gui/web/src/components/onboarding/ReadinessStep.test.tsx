import {act, cleanup, fireEvent, render, screen, waitFor, within} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import {MemoryRouter} from 'react-router-dom';
import {App, ConfigProvider} from 'antd';
import en from '../../i18n/locales/en.json';

const mock = vi.hoisted(() => ({fetch: vi.fn<typeof fetch>(), firmware: true, jump: vi.fn()}));
vi.mock('../../hooks/useApi.ts', async () => {
    const {Api} = await import('../../api/Api');
    const api = new Api<unknown>({customFetch: (...args) => mock.fetch(...args)});
    api.baseUrl = '';
    return {useApi: () => api};
});
vi.mock('../../hooks/useGnssStatus.ts', async () => {
    const {GnssStatusConstants: G} = await import('../../types/ros');
    return {useGnssStatus: () => ({fix_type: G.FIX_TYPE_RTK_FIXED, fix_valid: true,
        rtk_mode: G.RTK_MODE_FIXED, correction_stream_status: G.CORRECTION_STREAM_STATUS_ACTIVE})};
});
vi.mock('../../hooks/useFusionGraphDiagnostics.ts', () => ({useFusionGraphDiagnostics: () => ({stats: {
    receivedAt: Date.now(), level: 0, message: '', values: {total_nodes: '42', cov_xx: '0.0009', cov_yy: '0.0009'},
}})}));
vi.mock('../../hooks/useCalibrationStatus.ts', () => ({useCalibrationStatus: () => ({status: {
    dock: {present: true}, imu: {present: true}, mag: {present: true},
}})}));
vi.mock('../../hooks/useFirmwareStatus.ts', () => ({useFirmwareStatus: () => ({firmwareCompatible: mock.firmware})}));
vi.mock('../../hooks/useMowingMap.ts', () => ({useMowingMap: () => ({working_area: [
    {area: {points: [{x: 0, y: 0}, {x: 1, y: 0}, {x: 1, y: 1}]}},
]})}));
vi.mock('../../theme/ThemeContext.tsx', async () => {
    const {getColors} = await import('../../theme/colors');
    return {useThemeMode: () => ({colors: getColors('dark')})};
});

import {ReadinessStep} from './ReadinessStep';

const t = en.onboardingPage;
const fallback = en.asyncButton.errorOccurred;
interface RequestCall {url: string; method: string; body: BodyInit | null | undefined}
const unhandled: unknown[] = [];
const onUnhandled = (event: PromiseRejectionEvent) => {unhandled.push(event.reason);};
beforeEach(() => {
    mock.fetch.mockReset(); mock.jump.mockReset(); mock.firmware = true;
    unhandled.length = 0;
    vi.stubGlobal('fetch', mock.fetch);
    window.addEventListener('unhandledrejection', onUnhandled);
});
afterEach(() => {
    cleanup(); vi.unstubAllGlobals();
    window.removeEventListener('unhandledrejection', onUnhandled);
    expect(unhandled).toEqual([]);
});
function serve(post: () => Response | Promise<Response>, restartFailure?: string): RequestCall[] {
    const calls: RequestCall[] = [];
    mock.fetch.mockImplementation(async (input, init) => {
        const url = input instanceof Request ? input.url : String(input);
        calls.push({url, method: init?.method ?? 'GET', body: init?.body});
        if (url.endsWith('/api/settings/status')) return post();
        if (url === '/containers') return new Response(JSON.stringify({containers: [
            {id: 'ros-id', names: ['/mowgli-ros2']}, {id: 'gui-id', names: ['/mowgli-gui']},
        ]}));
        if (url === '/containers/ros-id/restart' || url === '/containers/gui-id/restart') {
            const rejected = restartFailure !== undefined && url.includes(restartFailure);
            return new Response(JSON.stringify(rejected ? {error: 'Restart refused'} : {}), {status: rejected ? 500 : 200});
        }
        throw new Error(`Unexpected request ${url}`);
    });
    return calls;
}
function mount() {
    render(<ConfigProvider theme={{token: {motion: false}}}><App><MemoryRouter>
        <ReadinessStep values={{datum_lat: 48.87, datum_lon: 2.17, imu_yaw: 0.12}} onJumpToStep={mock.jump}/>
    </MemoryRouter></App></ConfigProvider>);
}
function finishButton() {
    const button = screen.getByText(t.readinessFinishApply, {exact: true}).closest('button');
    if (!button) throw new Error('Missing Finish & apply button');
    return button;
}
async function finish(gated: boolean) {
    if (gated) {
        expect(finishButton()).toBeDisabled();
        fireEvent.click(screen.getByRole('button', {name: t.readinessFinishAnyway}));
        const dialog = await screen.findByRole('dialog');
        const button = within(dialog).getByText(t.readinessFinishApply, {exact: true}).closest('button');
        if (!button) throw new Error('Missing confirmation button');
        fireEvent.click(button);
    } else {
        expect(finishButton()).toBeEnabled();
        fireEvent.click(finishButton());
    }
    await waitFor(() => expect(screen.queryByText(t.applyingConfigTitle, {exact: true})).not.toBeInTheDocument());
}
function expectOnePost(calls: RequestCall[]) {
    expect(calls).toHaveLength(1);
    expect(calls[0]).toMatchObject({method: 'POST', body: undefined});
    expect(calls[0].url).toMatch(/\/api\/settings\/status$/);
    expect(mock.jump).not.toHaveBeenCalled();
}
const failures = [
    {name: 'HTTP 500 detail', status: 500, raw: JSON.stringify({error: 'DB write unavailable'}), detail: 'DB write unavailable'},
    {name: 'HTTP 400', status: 400, raw: JSON.stringify({error: 'Refused'}), detail: 'Refused'},
    {name: 'HTTP 503', status: 503, raw: JSON.stringify({error: 'Unavailable'}), detail: 'Unavailable'},
    {name: 'HTTP 500 empty body', status: 500, raw: ''},
    {name: 'HTTP 500 invalid JSON', status: 500, raw: '{invalid'},
    {name: 'HTTP 500 empty error', status: 500, raw: '{"error":""}'},
    {name: 'HTTP 500 null error', status: 500, raw: '{"error":null}'},
    {name: 'HTTP 500 numeric error', status: 500, raw: '{"error":42}'},
    {name: 'HTTP 500 object error', status: 500, raw: '{"error":{"detail":"Refused"}}'},
    {name: 'HTTP 500 missing detail', status: 500, raw: '{}'},
    {name: 'network rejection', detail: 'Connection lost'},
];
describe.each([false, true])('Readiness finalization, Finish anyway=%s', gated => {
    beforeEach(() => {mock.firmware = !gated;});
    it.each(failures)('refuses $name without restart or false completion', async fixture => {
        const calls = serve(() => {
            if (fixture.status === undefined) throw new TypeError('Connection lost');
            return new Response(fixture.raw, {status: fixture.status});
        });
        mount(); await finish(gated);
        expectOnePost(calls);
        expect(screen.queryByText(t.allSetTitle, {exact: true})).not.toBeInTheDocument();
        expect(screen.getByText(t.readinessTitle, {exact: true})).toBeVisible();
        await waitFor(() => expect(screen.getAllByText(fixture.detail ?? fallback, {exact: true}).length).toBeGreaterThan(0));
        expect(screen.queryByText(t.restartFailedTitle, {exact: true})).not.toBeInTheDocument();
        await waitFor(() => expect(screen.queryByRole('dialog')).not.toBeInTheDocument());
        expect(finishButton()).not.toHaveClass('ant-btn-loading');
        if (gated) expect(screen.getByRole('button', {name: t.readinessFinishAnyway})).toBeEnabled();
        else expect(finishButton()).toBeEnabled();
        await act(async () => {await new Promise(resolve => setTimeout(resolve, 0));});
        expectOnePost(calls);
    });
    it.each([undefined, 'ros-id', 'gui-id'])('preserves acknowledged POST and restart policy for %s failure', async failure => {
        const calls = serve(() => new Response('{}'), failure);
        mount(); await finish(gated);
        await screen.findByText(t.allSetTitle, {exact: true});
        const posts = calls.filter(call => call.method === 'POST');
        expect(posts.map(call => call.url)).toEqual([
            expect.stringMatching(/\/api\/settings\/status$/), '/containers/ros-id/restart',
            ...(failure === 'ros-id' ? [] : ['/containers/gui-id/restart']),
        ]);
        expect(posts[0].body).toBeUndefined();
        if (failure) expect(screen.getByText(t.restartFailedTitle, {exact: true})).toBeVisible();
        else expect(screen.queryByText(t.restartFailedTitle, {exact: true})).not.toBeInTheDocument();
        expect(mock.jump).not.toHaveBeenCalled();
        await waitFor(() => expect(screen.queryByRole('dialog')).not.toBeInTheDocument());
    });
    it('retries only after another explicit action and reuses the confirmation', async () => {
        let recovered = false;
        const calls = serve(() => new Response(recovered ? '{}' : '{"error":"Refused"}', {status: recovered ? 200 : 500}));
        mount(); await finish(gated);
        expectOnePost(calls);
        expect(screen.queryByText(t.allSetTitle, {exact: true})).not.toBeInTheDocument();
        await waitFor(() => expect(screen.queryByRole('dialog')).not.toBeInTheDocument());
        recovered = true;
        await finish(gated);
        await screen.findByText(t.allSetTitle, {exact: true});
        const posts = calls.filter(call => call.method === 'POST');
        expect(posts.map(call => call.url)).toEqual([
            expect.stringMatching(/\/api\/settings\/status$/), expect.stringMatching(/\/api\/settings\/status$/),
            '/containers/ros-id/restart', '/containers/gui-id/restart',
        ]);
        expect(posts.slice(0, 2).every(call => call.body === undefined)).toBe(true);
        await waitFor(() => expect(screen.queryByRole('dialog')).not.toBeInTheDocument());
        await act(async () => {await new Promise(resolve => setTimeout(resolve, 0));});
    });
});
it('keeps the readiness gate and allows cancelling Finish anyway without requests', async () => {
    mock.firmware = false;
    const calls = serve(() => new Response('{}'));
    mount(); expect(finishButton()).toBeDisabled();
    fireEvent.click(screen.getByRole('button', {name: t.readinessFinishAnyway}));
    const dialog = await screen.findByRole('dialog');
    fireEvent.click(within(dialog).getByRole('button', {name: 'Cancel'}));
    await waitFor(() => expect(screen.queryByRole('dialog')).not.toBeInTheDocument());
    expect(calls).toEqual([]);
    expect(screen.queryByText(t.allSetTitle, {exact: true})).not.toBeInTheDocument();
});
