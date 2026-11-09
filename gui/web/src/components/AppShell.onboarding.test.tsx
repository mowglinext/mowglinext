import {StrictMode} from 'react';
import {act, cleanup, render, screen, waitFor} from '@testing-library/react';
import {createMemoryRouter, RouterProvider} from 'react-router-dom';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import {AppShell} from './AppShell';

vi.mock('./MowerStatus', () => ({MowerStatus: () => null}));
vi.mock('./NotificationBell', () => ({NotificationBell: () => null}));
vi.mock('./LanguageSwitcher', () => ({LanguageSwitcher: () => null}));
vi.mock('./LiveStatusStrip', () => ({LiveStatusStrip: () => null}));
vi.mock('./HardwareBackendBadge', () => ({HardwareBackendBadge: () => null}));
vi.mock('./IOSInstallBanner', () => ({IOSInstallBanner: () => null}));
vi.mock('./dashboard', () => ({KEYFRAMES_CSS: ''}));
vi.mock('../hooks/useIOSInstallPrompt', () => ({useIOSInstallPrompt: () => ({showPrompt: false, dismiss: () => undefined})}));
vi.mock('../hooks/useNotificationCenter', () => ({useAutoNotifications: () => undefined}));
vi.mock('../hooks/useHighLevelStatus', () => ({useHighLevelStatus: () => ({highLevelStatus: {}})}));
vi.mock('../hooks/useEmergency', () => ({useEmergency: () => ({emergency: {}})}));
vi.mock('../hooks/useStatus', () => ({useStatus: () => ({})}));
vi.mock('../hooks/useHostUpdater', () => ({useHostUpdater: () => ({data: undefined, error: null})}));
vi.mock('../hooks/useIsMobile', () => ({useIsMobile: () => false}));

const fetchMock = vi.fn<typeof fetch>();
const reply = (body: unknown, status = 200) => new Response(JSON.stringify(body), {status});
function mountShell(path = '/settings', strict = true) {
    const router = createMemoryRouter([{path: '/', element: <AppShell/>, children: [
        {path: 'settings', element: <div>Settings route</div>},
        {path: 'onboarding', element: <div>Onboarding route</div>},
        {path: 'mowglinext', element: <div>Home route</div>},
        {path: 'map', element: <div>Map route</div>},
    ]}], {initialEntries: [path]});
    const tree = <RouterProvider router={router}/>;
    const view = render(strict ? <StrictMode>{tree}</StrictMode> : tree);
    return {router, view};
}
const mount = (path = '/settings') => mountShell(path).router;
const failedChecks = (calls: unknown[][]) => calls.filter(args => args[0] === 'Failed to check onboarding status');
function expectReadOnlyRequests() {
    expect(fetchMock.mock.calls.length).toBeGreaterThan(0);
    for (const [input, options] of fetchMock.mock.calls) {
        expect(input instanceof Request ? input.url : String(input)).toMatch(/\/api\/settings\/status$/);
        expect(options?.method ?? (input instanceof Request ? input.method : 'GET')).toBe('GET');
    }
}

beforeEach(() => {fetchMock.mockReset(); vi.stubGlobal('fetch', fetchMock);});
afterEach(() => {cleanup(); vi.restoreAllMocks(); vi.unstubAllGlobals();});

describe('AppShell onboarding response boundary', () => {
    it.each([true, false])('preserves the actual route decision for valid completed=%s', async completed => {
        const errors = vi.spyOn(console, 'error'); // Pass through all diagnostics.
        fetchMock.mockImplementation(() => Promise.resolve(reply({onboarding_completed: completed})));
        const router = mount();
        await screen.findByText(completed ? 'Settings route' : 'Onboarding route');
        await act(async () => {await Promise.resolve();});
        expect(router.state.location.pathname).toBe(completed ? '/settings' : '/onboarding');
        expect(failedChecks(errors.mock.calls)).toHaveLength(0);
        expectReadOnlyRequests();
    });

    it.each([true, false])('accepts additional properties with completed=%s', async completed => {
        fetchMock.mockImplementation(() => Promise.resolve(reply({onboarding_completed: completed, future: {value: 42}})));
        const router = mount();
        await screen.findByText(completed ? 'Settings route' : 'Onboarding route');
        await act(async () => {await Promise.resolve();});
        expect(router.state.location.pathname).toBe(completed ? '/settings' : '/onboarding');
        expectReadOnlyRequests();
    });

    it('does not redirect an already active onboarding route', async () => {
        fetchMock.mockImplementation(() => Promise.resolve(reply({onboarding_completed: false})));
        const router = mount('/onboarding');
        const originalKey = router.state.location.key;
        await screen.findByText('Onboarding route');
        await act(async () => {await Promise.resolve();});
        expect(router.state.location.key).toBe(originalKey);
        expectReadOnlyRequests();
    });

    const malformed: unknown[] = [{}, {onboarding_completed: 'false'}, {onboarding_completed: 0},
        {onboarding_completed: null}, null, [], 'status'];
    it.each(malformed.map(body => ({body})))('rejects malformed JSON without redirecting or mutating: %j', async ({body}) => {
        const errors = vi.spyOn(console, 'error');
        fetchMock.mockImplementation(() => Promise.resolve(reply(body)));
        const router = mount();
        await waitFor(() => expect(failedChecks(errors.mock.calls).length).toBeGreaterThan(0));
        await screen.findByText('Settings route');
        expect(router.state.location.pathname).toBe('/settings');
        expect(screen.queryByText('Onboarding route')).not.toBeInTheDocument();
        expectReadOnlyRequests();
    });

    it.each([500, 503].flatMap(status => [{error: 'fixture unavailable'}, {onboarding_completed: true}, {onboarding_completed: false}]
        .map(body => ({status, body}))))('rejects HTTP $status without redirecting, body=$body', async ({status, body}) => {
        const errors = vi.spyOn(console, 'error');
        fetchMock.mockImplementation(() => Promise.resolve(reply(body, status)));
        const router = mount();
        await waitFor(() => expect(failedChecks(errors.mock.calls).length).toBeGreaterThan(0));
        await screen.findByText('Settings route');
        expect(router.state.location.pathname).toBe('/settings');
        expectReadOnlyRequests();
    });

    it.each(['network', 'JSON'])('preserves the existing error path for %s failures', async failure => {
        const errors = vi.spyOn(console, 'error');
        fetchMock.mockImplementation(() => failure === 'network'
            ? Promise.reject(new Error('fixture network error')) : Promise.resolve(new Response('{invalid')));
        const router = mount();
        await waitFor(() => expect(failedChecks(errors.mock.calls).length).toBeGreaterThan(0));
        await screen.findByText('Settings route');
        expect(router.state.location.pathname).toBe('/settings');
        expectReadOnlyRequests();
    });

    it.each([false, true])('finishes the check after failure=%s and does not recheck on later navigation', async failure => {
        fetchMock.mockImplementation(() => failure
            ? Promise.reject(new Error('fixture failure')) : Promise.resolve(reply({onboarding_completed: true})));
        const router = mount();
        await screen.findByText('Settings route');
        await act(async () => {await Promise.resolve();});
        const calls = fetchMock.mock.calls.length;
        await act(async () => {await router.navigate('/map');});
        await screen.findByText('Map route');
        expect(router.state.location.pathname).toBe('/map');
        expect(fetchMock).toHaveBeenCalledTimes(calls);
        expectReadOnlyRequests();
    });
});

interface PendingStatus {
    response: Promise<Response>;
    resolve: (response: Response) => void;
}
function queueStatusRequests() {
    const pending: PendingStatus[] = [];
    fetchMock.mockImplementation(() => {
        let resolve!: (response: Response) => void;
        const response = new Promise<Response>(accept => {resolve = accept;});
        pending.push({response, resolve});
        return response;
    });
    return pending;
}
function statusRequest(request: PendingStatus | undefined): PendingStatus {
    if (!request) throw new Error('Expected a pending status request');
    return request;
}
async function resolveStatus(request: PendingStatus, completed: boolean, status = 200) {
    await act(async () => {
        request.resolve(reply({onboarding_completed: completed}, status));
        await request.response;
    });
}
async function expectRenderedRoute(router: ReturnType<typeof createMemoryRouter>, path: string, text: string) {
    expect(router.state.location.pathname).toBe(path);
    await waitFor(() => expect(screen.getByText(text)).toBeVisible());
}

describe('AppShell obsolete responses', () => {
    it('ignores an obsolete false after the current StrictMode response is true', async () => {
        const pending = queueStatusRequests();
        const {router} = mountShell();
        const old = statusRequest(pending[0]);
        const current = statusRequest(pending[pending.length - 1]);
        expect(old).not.toBe(current); // Establish two effect lifetimes, not an exact request count.
        await resolveStatus(current, true);
        await resolveStatus(old, false);
        await expectRenderedRoute(router, '/settings', 'Settings route');
        expectReadOnlyRequests();
    });

    it('ignores an obsolete StrictMode false even before the current true', async () => {
        const pending = queueStatusRequests();
        const {router} = mountShell();
        const old = statusRequest(pending[0]);
        const current = statusRequest(pending[pending.length - 1]);
        expect(old).not.toBe(current);
        await resolveStatus(old, false);
        await expectRenderedRoute(router, '/settings', 'Settings route');
        await resolveStatus(current, true);
        await expectRenderedRoute(router, '/settings', 'Settings route');
        expectReadOnlyRequests();
    });

    it('does not finish configChecked from an invalidated invocation', async () => {
        const pending = queueStatusRequests();
        const {router} = mountShell();
        const old = statusRequest(pending[0]);
        const current = statusRequest(pending[pending.length - 1]);
        expect(old).not.toBe(current);
        await resolveStatus(old, true);
        // The active request has not completed: a route change must still start its own check.
        await act(async () => {await router.navigate('/map');});
        const latest = statusRequest(pending[pending.length - 1]);
        expect(latest).not.toBe(current);
        await resolveStatus(latest, true);
        await resolveStatus(current, false);
        await expectRenderedRoute(router, '/map', 'Map route');
        expectReadOnlyRequests();
    });

    it('ignores an obsolete response after /settings to /map without StrictMode', async () => {
        const pending = queueStatusRequests();
        const {router} = mountShell('/settings', false);
        const old = statusRequest(pending[pending.length - 1]);
        await act(async () => {await router.navigate('/map');});
        const current = statusRequest(pending[pending.length - 1]);
        expect(current).not.toBe(old);
        await resolveStatus(current, true);
        await resolveStatus(old, false);
        await expectRenderedRoute(router, '/map', 'Map route');
        expectReadOnlyRequests();
    });

    it('does not repeat onboarding navigation from an obsolete captured /settings path', async () => {
        const pending = queueStatusRequests();
        const {router} = mountShell('/settings', false);
        const old = statusRequest(pending[pending.length - 1]);
        await act(async () => {await router.navigate('/onboarding');});
        const key = router.state.location.key;
        const current = statusRequest(pending[pending.length - 1]);
        expect(current).not.toBe(old);
        await resolveStatus(current, true);
        await resolveStatus(old, false);
        expect(router.state.location.key).toBe(key);
        await expectRenderedRoute(router, '/onboarding', 'Onboarding route');
        expectReadOnlyRequests();
    });

    it.each([false, 'invalid'])('ignores an obsolete streamed JSON body: %s', async payload => {
        const errors = vi.spyOn(console, 'error');
        const pending = queueStatusRequests();
        const {router} = mountShell('/settings', false);
        const old = statusRequest(pending[pending.length - 1]);
        let body!: ReadableStreamDefaultController<Uint8Array>;
        const stream = new ReadableStream<Uint8Array>({start(controller) {body = controller;}});
        await act(async () => {old.resolve(new Response(stream)); await old.response;});
        await act(async () => {await router.navigate('/map');});
        const current = statusRequest(pending[pending.length - 1]);
        expect(current).not.toBe(old);
        await resolveStatus(current, true);
        await act(async () => {
            body.enqueue(new TextEncoder().encode(payload === false ? '{"onboarding_completed":false}' : '{invalid'));
            body.close();
            await Promise.resolve();
        });
        await expectRenderedRoute(router, '/map', 'Map route');
        expect(failedChecks(errors.mock.calls)).toHaveLength(0);
        expectReadOnlyRequests();
    });

    it.each([500, 503])('ignores an obsolete HTTP %s after an active valid response', async status => {
        const errors = vi.spyOn(console, 'error');
        const pending = queueStatusRequests();
        const {router} = mountShell();
        const old = statusRequest(pending[0]);
        const current = statusRequest(pending[pending.length - 1]);
        expect(old).not.toBe(current);
        await resolveStatus(current, true);
        await resolveStatus(old, false, status);
        await expectRenderedRoute(router, '/settings', 'Settings route');
        expect(failedChecks(errors.mock.calls)).toHaveLength(0);
        expectReadOnlyRequests();
    });

    it('ignores an obsolete callback after real unmount and remount', async () => {
        const pending = queueStatusRequests();
        const first = mountShell('/settings', false);
        const old = statusRequest(pending[pending.length - 1]);
        first.view.unmount();
        const second = mountShell('/map', false);
        const current = statusRequest(pending[pending.length - 1]);
        expect(current).not.toBe(old);
        await resolveStatus(current, true);
        await resolveStatus(old, false);
        expect(first.router.state.location.pathname).toBe('/settings');
        await expectRenderedRoute(second.router, '/map', 'Map route');
        expectReadOnlyRequests();
    });

    it('preserves an active false even if an obsolete true finishes afterward', async () => {
        const pending = queueStatusRequests();
        const {router} = mountShell();
        const old = statusRequest(pending[0]);
        const current = statusRequest(pending[pending.length - 1]);
        expect(old).not.toBe(current);
        await resolveStatus(current, false);
        await resolveStatus(old, true);
        await expectRenderedRoute(router, '/onboarding', 'Onboarding route');
        expectReadOnlyRequests();
    });
});
