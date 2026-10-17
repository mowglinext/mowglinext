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
function mount(path = '/settings') {
    const router = createMemoryRouter([{path: '/', element: <AppShell/>, children: [
        {path: 'settings', element: <div>Settings route</div>},
        {path: 'onboarding', element: <div>Onboarding route</div>},
        {path: 'mowglinext', element: <div>Home route</div>},
        {path: 'map', element: <div>Map route</div>},
    ]}], {initialEntries: [path]});
    render(<StrictMode><RouterProvider router={router}/></StrictMode>);
    return router;
}
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

    it.each([{error: 'fixture unavailable'}, {onboarding_completed: true}, {onboarding_completed: false}])('rejects HTTP 503 even with a valid-looking body: %j', async body => {
        const errors = vi.spyOn(console, 'error');
        fetchMock.mockImplementation(() => Promise.resolve(reply(body, 503)));
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
