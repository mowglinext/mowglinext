import {act, cleanup, fireEvent, render, screen, waitFor, within} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import type {FleetRobotWire} from '../utils/fleet';
import type {FleetCoordination, MapPushResult} from '../hooks/useFleet';
import {getColors} from '../theme/colors';
import en from '../i18n/locales/en.json';
import {ConfigProvider} from 'antd';
import {Component, type ReactNode} from 'react';

const mock = vi.hoisted(() => {
    const robots: FleetRobotWire[] = [];
    const coordination: FleetCoordination = {
        settings: {enabled: false, yield_distance_m: 3, resume_distance_m: 5, completed_ttl_h: 12, yield_max_hold_s: 45, yield_cooldown_s: 20},
        status: {enabled: false, excluded_areas: [], preferred_start: -1, yielded: false, completed_areas: []},
        peer_feed: {active: false, peers_published: 0, rate_hz: 5, publishes: 0},
    };
    return {
        robots, coordination, realFleet: false,
        addPeer: vi.fn<(address: string) => Promise<string | undefined>>(),
        removePeer: vi.fn<(id: string) => Promise<void>>(),
        call: vi.fn<(id: string, command: string, args: Record<string, unknown>) => Promise<void>>(),
        setCoordination: vi.fn<(settings: FleetCoordination['settings']) => Promise<void>>(),
        startFresh: vi.fn<() => Promise<void>>(),
        pushMap: vi.fn<() => Promise<MapPushResult>>(),
        error: vi.fn<(content: unknown) => void>(),
        success: vi.fn<(content: string) => void>(),
        warning: vi.fn<(content: string, duration: number) => void>(),
    };
});
vi.mock('antd', async importOriginal => {
    const antd = await importOriginal<typeof import('antd')>();
    return {...antd, App: {...antd.App, useApp: () => ({
        message: {error: mock.error, success: mock.success, warning: mock.warning},
        notification: {error: mock.error},
    })}};
});
vi.mock('../hooks/useFleet.ts', async importOriginal => {
    const original = await importOriginal<typeof import('../hooks/useFleet.ts')>();
    return {...original, useFleet: () => mock.realFleet ? original.useFleet() : {...mock, error: undefined, loading: false}};
});
vi.mock('../hooks/useSettings.ts', () => ({useSettings: () => ({settings: {}})}));
vi.mock('../theme/ThemeContext.tsx', () => ({useThemeMode: () => ({colors: getColors('dark')})}));

import FleetPage from './FleetPage';

const robot = (id: string, online = true, api_version = 1): FleetRobotWire => ({
    identity: {id, name: id, api_version}, self: false, online,
});
const addButton = () => screen.getByRole('button', {name: new RegExp(`${en.fleetPage.add}$`)});
function enterAddress() {
    fireEvent.change(screen.getByRole('textbox', {name: en.fleetPage.addPeerTitle}), {target: {value: '  mower-b.local:4006  '}});
}

beforeEach(() => {
    vi.resetAllMocks();
    mock.realFleet = false;
    mock.robots = [];
    mock.addPeer.mockResolvedValue(undefined);
    mock.removePeer.mockResolvedValue(undefined);
    mock.call.mockResolvedValue(undefined);
    mock.setCoordination.mockResolvedValue(undefined);
    mock.startFresh.mockResolvedValue(undefined);
    mock.pushMap.mockResolvedValue({areas: 2, peers: []});
});
afterEach(() => {
    cleanup();
    vi.unstubAllGlobals();
    vi.useRealTimers();
});

describe('FleetPage action errors', () => {
    it.each([
        {name: 'HTTP error', value: new Error('peer API refused the command'), expected: 'peer API refused the command'},
        {name: 'network error', value: new TypeError('Failed to fetch'), expected: 'Failed to fetch'},
        {name: 'string rejection', value: 'plain rejection', expected: 'plain rejection'},
        {name: 'null rejection', value: null, expected: 'null'},
        {name: 'undefined rejection', value: undefined, expected: 'undefined'},
        {name: 'object without message', value: {reason: 'missing'}, expected: '[object Object]'},
        {name: 'textual message', value: {message: 'text detail'}, expected: 'text detail'},
        {name: 'empty message', value: {message: ''}, expected: ''},
        {name: 'null message', value: {message: null}, expected: '[object Object]'},
        {name: 'object message', value: {message: {reason: 'invalid content'}}, expected: '[object Object]'},
        {name: 'numeric message', value: {message: 42}, expected: '[object Object]'},
        {name: 'direct API error', value: {error: 'backend detail'}, expected: 'backend detail'},
        {name: 'nested API error', value: {error: {error: 'nested backend detail'}}, expected: 'nested backend detail'},
    ])('reports only text for $name, clears busy, and permits retry', async ({value, expected}) => {
        mock.addPeer.mockRejectedValueOnce(value);
        render(<FleetPage/>);
        enterAddress();
        // Flush the rejected action and its finally render, not just the
        // synchronous error notification that precedes clearing busy.
        await act(async () => {
            fireEvent.click(addButton());
            await expect(mock.addPeer.mock.results[0].value).rejects.toBe(value);
        });
        await waitFor(() => expect(mock.error).toHaveBeenCalledOnce());
        expect(mock.error).toHaveBeenCalledWith(expected);
        expect(typeof mock.error.mock.calls[0][0]).toBe('string');
        expect(mock.success).not.toHaveBeenCalled();
        expect(mock.warning).not.toHaveBeenCalled();
        await waitFor(() => expect(addButton()).not.toHaveClass('ant-btn-loading'));
        expect(mock.addPeer).toHaveBeenCalledExactlyOnceWith('mower-b.local:4006');
        expect(screen.getByRole('textbox', {name: en.fleetPage.addPeerTitle})).toHaveValue('  mower-b.local:4006  ');
        await act(async () => {
            fireEvent.click(addButton());
            await expect(mock.addPeer.mock.results[1].value).resolves.toBeUndefined();
        });
        await waitFor(() => expect(mock.success).toHaveBeenCalledExactlyOnceWith(en.fleetPage.peerAdded));
        expect(addButton()).not.toHaveClass('ant-btn-loading');
        expect(mock.addPeer.mock.calls).toEqual([['mower-b.local:4006'], ['mower-b.local:4006']]);
        expect(mock.error).toHaveBeenCalledOnce();
        expect(screen.getByRole('textbox', {name: en.fleetPage.addPeerTitle})).toHaveValue('');
    });

    it('shows busy while an action is pending and clears it on failure', async () => {
        let reject: (reason: unknown) => void = () => { throw new Error('Request not started'); };
        mock.addPeer.mockImplementationOnce(() => new Promise((_resolve, rejectRequest) => { reject = rejectRequest; }));
        render(<FleetPage/>);
        enterAddress();
        fireEvent.click(addButton());
        expect(addButton()).toHaveClass('ant-btn-loading');
        await act(async () => {
            reject(new Error('late failure'));
            await Promise.resolve();
        });
        expect(addButton()).not.toHaveClass('ant-btn-loading');
        expect(mock.error).toHaveBeenCalledExactlyOnceWith('late failure');
        expect(mock.success).not.toHaveBeenCalled();
    });

    it('preserves the successful add warning and its duration without success notification', async () => {
        mock.addPeer.mockResolvedValueOnce('Reverse registration pending');
        render(<FleetPage/>);
        enterAddress();
        fireEvent.click(addButton());
        await waitFor(() => expect(mock.warning).toHaveBeenCalledExactlyOnceWith('Reverse registration pending', 8));
        expect(mock.success).not.toHaveBeenCalled();
        expect(mock.error).not.toHaveBeenCalled();
        expect(mock.addPeer).toHaveBeenCalledExactlyOnceWith('mower-b.local:4006');
    });

    it('reports a failed removal, resolves the confirmation, and permits retry for the same peer', async () => {
        mock.robots = [robot('peer-ready')];
        mock.removePeer.mockRejectedValueOnce(new Error('Cannot remove peer'));
        render(<ConfigProvider theme={{token: {motion: false}}}><FleetPage/></ConfigProvider>);
        const card = within(screen.getByTestId('fleet-card-peer-ready'));
        fireEvent.click(card.getByRole('button', {name: /delete/}));
        fireEvent.click(await screen.findByRole('button', {name: 'OK'}));
        await waitFor(() => expect(mock.error).toHaveBeenCalledExactlyOnceWith('Cannot remove peer'));
        expect(mock.success).not.toHaveBeenCalled();
        await waitFor(() => {
            const popup = document.querySelector('.ant-popover');
            expect(popup === null || popup.classList.contains('ant-popover-hidden')).toBe(true);
        });
        fireEvent.click(card.getByRole('button', {name: /delete/}));
        fireEvent.click(await screen.findByRole('button', {name: 'OK'}));
        await waitFor(() => expect(mock.success).toHaveBeenCalledExactlyOnceWith(en.fleetPage.peerRemoved));
        expect(mock.removePeer.mock.calls).toEqual([['peer-ready'], ['peer-ready']]);
    });

    it('preserves coordination settings and map-push warnings', async () => {
        mock.robots = [robot('alpha'), robot('bravo')];
        mock.pushMap.mockResolvedValueOnce({areas: 2, peers: [{id: 'bravo', name: 'bravo', address: 'bravo.local', ok: false, error: 'Map refused'}]});
        render(<FleetPage/>);
        fireEvent.click(screen.getByRole('switch'));
        await waitFor(() => expect(mock.setCoordination).toHaveBeenCalledExactlyOnceWith({...mock.coordination.settings, enabled: true}));
        fireEvent.click(screen.getByRole('button', {name: new RegExp(`${en.fleetPage.pushMap}$`)}));
        fireEvent.click(await screen.findByRole('button', {name: 'OK'}));
        await waitFor(() => expect(mock.warning).toHaveBeenCalledExactlyOnceWith('bravo: Map refused', 8));
        expect(mock.pushMap).toHaveBeenCalledExactlyOnceWith();
        expect(mock.success).not.toHaveBeenCalled();
        expect(mock.error).not.toHaveBeenCalled();
    });

    it('keeps the robot id and command arguments and blocks offline/incompatible robots', async () => {
        mock.robots = [robot('peer-ready'), robot('peer-offline', false), robot('peer-incompatible', true, 99)];
        render(<FleetPage/>);
        const ready = within(screen.getByTestId('fleet-card-peer-ready'));
        fireEvent.click(ready.getByRole('button', {name: new RegExp(`${en.fleetPage.play}$`)}));
        await waitFor(() => expect(mock.call).toHaveBeenCalledExactlyOnceWith('peer-ready', 'high_level_control', {Command: 1}));
        await waitFor(() => expect(mock.success).toHaveBeenCalledOnce());
        for (const id of ['peer-offline', 'peer-incompatible']) {
            const card = within(screen.getByTestId(`fleet-card-${id}`));
            for (const name of [en.fleetPage.play, en.fleetPage.pause, en.fleetPage.home]) {
                expect(card.getByRole('button', {name: new RegExp(`${name}$`)})).toBeDisabled();
            }
        }
        expect(mock.error).not.toHaveBeenCalled();
    });
});


describe('FleetPage map sharing with the real useFleet HTTP boundary', () => {
    const requestUrl = (url: Parameters<typeof fetch>[0]) => typeof url === 'string' ? url : url instanceof URL ? url.href : url.url;
    it.each([
        {name: 'string false', payload: {areas: 2, peers: [{id: 'b', name: 'Bravo', address: 'b', ok: 'false', error: 'Refused'}]}},
        {name: 'missing peers', payload: {areas: 2}},
        {name: 'non-array peers', payload: {areas: 2, peers: {}}},
        {name: 'null peer', payload: {areas: 2, peers: [null]}},
        {name: 'non-textual peer error', payload: {areas: 2, peers: [{id: 'b', name: 'Bravo', address: 'b', ok: false, error: {reason: 'Refused'}}]}},
    ])('rejects $name without business notification, closes confirmation, then permits an explicit retry', async ({payload}) => {
        vi.useFakeTimers({toFake: ['setInterval', 'clearInterval']});
        mock.realFleet = true;
        let responseText = JSON.stringify(payload);
        const fetchMock = vi.fn<typeof fetch>(url => Promise.resolve(new Response(
            requestUrl(url).endsWith('/map/push') ? responseText : requestUrl(url).endsWith('/robots')
                ? JSON.stringify([robot('alpha'), robot('bravo')]) : '{}',
        )));
        vi.stubGlobal('fetch', fetchMock);
        render(<ConfigProvider theme={{token: {motion: false}}}><FleetPage/></ConfigProvider>);
        const button = screen.getByText(en.fleetPage.pushMap, {exact: true}).closest('button');
        if (!button) throw new Error('Missing map sharing button');
        const pushButton = () => button;
        await waitFor(() => expect(button).toBeEnabled());
        fireEvent.click(pushButton());
        fireEvent.click(await screen.findByText('OK', {exact: true}));
        await waitFor(() => expect(mock.error).toHaveBeenCalledExactlyOnceWith(
            'Unusable map sharing result: the response is invalid; remote delivery is not determined.',
        ));
        expect(mock.success).not.toHaveBeenCalled();
        expect(mock.warning).not.toHaveBeenCalled();
        await waitFor(() => expect(pushButton()).not.toHaveClass('ant-btn-loading'));
        await waitFor(() => {
            const popup = document.querySelector('.ant-popover');
            expect(popup === null || popup.classList.contains('ant-popover-hidden')).toBe(true);
        });
        const posts = () => fetchMock.mock.calls.filter(([, init]) => init?.method === 'POST');
        expect(posts()).toHaveLength(1);
        expect(posts()[0]).toEqual([expect.stringMatching(/\/api\/fleet\/map\/push$/), {
            headers: {'Content-Type': 'application/json'}, method: 'POST', body: '{}',
        }]);
        responseText = JSON.stringify({areas: 2, peers: [{id: 'b', name: 'Bravo', address: 'b', ok: true}]});
        fireEvent.click(pushButton());
        fireEvent.click(await screen.findByText('OK', {exact: true}));
        await waitFor(() => expect(mock.success).toHaveBeenCalledExactlyOnceWith(en.fleetPage.mapPushed));
        expect(posts()).toHaveLength(2);
        expect(mock.error).toHaveBeenCalledOnce();
        expect(mock.warning).not.toHaveBeenCalled();
    });
});

class CoordinationRenderBoundary extends Component<{
    children: ReactNode;
    onError: (error: Error) => void;
}, {failed: boolean}> {
    state = {failed: false};
    static getDerivedStateFromError() {return {failed: true};}
    componentDidCatch(error: Error) {this.props.onError(error);}
    render() {return this.state.failed ? <div>Unexpected coordination render failure</div> : this.props.children;}
}

describe('FleetPage malformed coordination with the real hook', () => {
    const requestUrl = (url: Parameters<typeof fetch>[0]) => typeof url === 'string' ? url : url instanceof URL ? url.href : url.url;
    it.each([
        {name: 'excluded areas object', invalid: {settings: {enabled: true}, status: {excluded_areas: {length: 1}}}},
        {name: 'completed areas object', invalid: {settings: {enabled: true}, status: {completed_areas: {length: 1}}}},
        {name: 'last error object', invalid: {settings: {}, status: {last_error: {reason: 'not text'}}}},
        {name: 'enabled text', invalid: {settings: {enabled: 'false'}, status: {}}},
        {name: 'peer active text', invalid: {settings: {}, status: {}, peer_feed: {active: 'false'}}},
        {name: 'distance text', invalid: {settings: {yield_distance_m: '12'}, status: {}}},
    ])('keeps the last valid controls and shows the poll error for $name', async ({invalid}) => {
        vi.useFakeTimers({toFake: ['setInterval', 'clearInterval']});
        mock.realFleet = true;
        let coordination: unknown = mock.coordination;
        const fetchMock = vi.fn<typeof fetch>(url => Promise.resolve(new Response(JSON.stringify(
            requestUrl(url).endsWith('/robots') ? [robot('alpha'), robot('bravo')] : coordination,
        ))));
        vi.stubGlobal('fetch', fetchMock);
        const caught = vi.fn<(error: Error) => void>();
        render(<CoordinationRenderBoundary onError={caught}>
            <ConfigProvider theme={{token: {motion: false}}}><FleetPage/></ConfigProvider>
        </CoordinationRenderBoundary>);
        await screen.findByTestId('fleet-card-alpha');
        expect(screen.getByRole('switch')).not.toBeChecked();
        const distances = screen.getAllByRole('spinbutton');
        expect(distances[0]).toHaveValue('3.0');
        coordination = invalid;
        await act(() => vi.advanceTimersByTimeAsync(2000));
        await screen.findByText('Invalid fleet coordination response');
        expect(caught).not.toHaveBeenCalled();
        expect(screen.queryByText('Unexpected coordination render failure')).not.toBeInTheDocument();
        expect(screen.getByTestId('fleet-card-alpha')).toBeVisible();
        expect(screen.getByTestId('fleet-card-bravo')).toBeVisible();
        expect(screen.getByRole('switch')).not.toBeChecked();
        expect(screen.getAllByRole('spinbutton')[0]).toHaveValue('3.0');
        expect(within(screen.getByTestId('fleet-peer-feed')).getByText(en.fleetPage.peerFeedDown)).toBeVisible();
        expect(mock.success).not.toHaveBeenCalled();
        expect(mock.warning).not.toHaveBeenCalled();
        expect(mock.error).not.toHaveBeenCalled();
        expect(fetchMock.mock.calls).toHaveLength(4);
        expect(fetchMock.mock.calls.every(([, options]) => !options?.method)).toBe(true);
    });
});

describe('FleetPage robot identity boundary with the real hook', () => {
    const at = (id: string, name: string, lat: number) => ({...robot(id), identity: {id, name, api_version: 1},
        topics: {gps: {pose: {pose: {position: {x: lat, y: 2, z: 0}}}}}});
    const urlOf = (url: Parameters<typeof fetch>[0]) => typeof url === 'string' ? url : url instanceof URL ? url.href : url.url;
    it.each(['duplicate', 'name', 'online'])('preserves the valid rendered inventory after %s rejection', async mode => {
        vi.useFakeTimers({toFake: ['setInterval', 'clearInterval']});
        mock.realFleet = true;
        let payload: unknown = [at('a', 'Alpha', 48), at('b', 'Beta', 48.0001)];
        const fetchMock = vi.fn<typeof fetch>(url => Promise.resolve(new Response(JSON.stringify(urlOf(url).endsWith('/robots') ? payload : {}))));
        vi.stubGlobal('fetch', fetchMock);
        const caught = vi.fn<(error: Error) => void>();
        render(<CoordinationRenderBoundary onError={caught}><ConfigProvider theme={{token: {motion: false}}}><FleetPage/></ConfigProvider></CoordinationRenderBoundary>);
        await screen.findByTestId('fleet-card-a');
        if (mode === 'duplicate') payload = [at('same', 'Alpha', 48), at('same', 'Beta', 48.0001)];
        if (mode === 'name') payload = [{...robot('bad'), identity: {id: 'bad', name: {label: 'invalid'}}}];
        if (mode === 'online') payload = [{...robot('bad'), online: 'false'}];
        await act(() => vi.advanceTimersByTimeAsync(2000));
        expect(caught).not.toHaveBeenCalled();
        const map = screen.getByRole('img', {name: 'fleet positions'});
        expect(within(map).getAllByText('Alpha')).toHaveLength(1);
        expect(within(map).getAllByText('Beta')).toHaveLength(1);
        expect(screen.queryByTestId('fleet-card-same')).not.toBeInTheDocument();
        expect(screen.queryByTestId('fleet-card-bad')).not.toBeInTheDocument();
        expect(screen.getByTestId('fleet-card-a')).toBeVisible();
        expect(screen.getByTestId('fleet-card-b')).toBeVisible();
        expect(screen.getByText('Invalid fleet robots response')).toBeVisible();
        const card = screen.getByTestId('fleet-card-b');
        fireEvent.click(within(card).getByRole('button', {name: /Start mowing$/}));
        await waitFor(() => expect(fetchMock.mock.calls.some(([url]) => urlOf(url).endsWith('/robots/b/call/high_level_control'))).toBe(true));
        const command = fetchMock.mock.calls.find(([url]) => urlOf(url).includes('/call/'));
        expect(command?.[1]?.body).toBe('{"Command":1}');
        expect(fetchMock.mock.calls.filter(([url]) => urlOf(url).includes('/call/'))).toHaveLength(1);
    });
    it('does not expose or send a command from two cards with an ambiguous id', async () => {
        mock.realFleet = true;
        const payload = [at('same', 'Alpha', 48), at('same', 'Beta', 48.0001)];
        const fetchMock = vi.fn<typeof fetch>(url => Promise.resolve(new Response(JSON.stringify(urlOf(url).endsWith('/robots') ? payload : {}))));
        vi.stubGlobal('fetch', fetchMock);
        render(<ConfigProvider theme={{token: {motion: false}}}><FleetPage/></ConfigProvider>);
        await waitFor(() => expect(screen.queryAllByTestId('fleet-card-same').length > 0 || screen.queryByText('Invalid fleet robots response') !== null).toBe(true));
        // On HEAD both distinct cards are actionable and send the same target.
        for (const index of [0, 1]) {
            const cards = screen.queryAllByTestId('fleet-card-same');
            const card = cards[index];
            if (card) {
                fireEvent.click(within(card).getByRole('button', {name: /Start mowing$/}));
                await waitFor(() => expect(mock.success).toHaveBeenCalledTimes(index + 1));
            }
        }
        expect(fetchMock.mock.calls.filter(([url]) => urlOf(url).includes('/call/'))).toHaveLength(0);
        expect(screen.queryAllByTestId('fleet-card-same')).toHaveLength(0);
        expect(screen.getByText('Invalid fleet robots response')).toBeVisible();
    });
    it('never exposes Start or sends a command for textual online=false', async () => {
        mock.realFleet = true;
        const payload = [{...robot('bad'), online: 'false'}];
        const fetchMock = vi.fn<typeof fetch>(url => Promise.resolve(new Response(JSON.stringify(urlOf(url).endsWith('/robots') ? payload : {}))));
        vi.stubGlobal('fetch', fetchMock);
        render(<ConfigProvider theme={{token: {motion: false}}}><FleetPage/></ConfigProvider>);
        await waitFor(() => expect(screen.queryByTestId('fleet-card-bad') !== null || screen.queryByText('Invalid fleet robots response') !== null).toBe(true));
        const card = screen.queryByTestId('fleet-card-bad');
        if (card) {
            const start = within(card).getByRole('button', {name: /Start mowing$/});
            expect(start).not.toBeDisabled();
            fireEvent.click(start);
            await waitFor(() => expect(mock.success).toHaveBeenCalledOnce());
        }
        expect(fetchMock.mock.calls.filter(([url]) => urlOf(url).includes('/call/'))).toHaveLength(0);
        expect(screen.queryByTestId('fleet-card-bad')).not.toBeInTheDocument();
        expect(screen.getByText('Invalid fleet robots response')).toBeVisible();
    });

});

describe('FleetPage HighLevelStatus publication boundary', () => {
    it.each([
        {name: 'object state name', value: {state_name: {bad: true}}},
        {name: 'text emergency', value: {emergency: 'false'}},
        {name: 'text charging', value: {is_charging: 'false'}},
        {name: 'text state', value: {state: '2', state_name: 'MOWING'}},
        {name: 'object coverage', value: {state: 2, coverage_percent: {bad: true}}},
        {name: 'infinite coverage', value: {state: 2, coverage_percent: '__OVERFLOW__'}},
        {name: 'object area', value: {state: 2, current_area: {bad: true}}},
    ])('rejects $name before rendering and recovers without changing the command target', async ({value}) => {
        vi.useFakeTimers({toFake: ['setInterval', 'clearInterval']});
        mock.realFleet = true;
        let robots: unknown = [robot('alpha'), robot('bravo')];
        let coordination: unknown = {settings: {yield_distance_m: 7}, status: {}};
        const urlOf = (url: Parameters<typeof fetch>[0]) => typeof url === 'string' ? url : url instanceof URL ? url.href : url.url;
        const fetchMock = vi.fn<typeof fetch>((url, options) => Promise.resolve(new Response(JSON.stringify(
            options?.method === 'POST' ? {} : urlOf(url).endsWith('/robots') ? robots : coordination,
        ).replace('"__OVERFLOW__"', '1e999'))));
        vi.stubGlobal('fetch', fetchMock);
        const caught = vi.fn<(error: Error) => void>();
        render(<CoordinationRenderBoundary onError={caught}><ConfigProvider theme={{token: {motion: false}}}><FleetPage/></ConfigProvider></CoordinationRenderBoundary>);
        await screen.findByTestId('fleet-card-alpha');
        robots = [{...robot('alpha'), topics: {highLevelStatus: value}}, {...robot('bravo'), identity: {id: 'bravo', name: 'New Bravo'}}];
        coordination = {settings: {yield_distance_m: 12}, status: {}};
        await act(() => vi.advanceTimersByTimeAsync(2000));
        expect(caught).not.toHaveBeenCalled();
        expect(screen.getByTestId('fleet-card-alpha')).toBeVisible();
        const bravo = screen.getByTestId('fleet-card-bravo');
        expect(within(bravo).getByText('bravo', {exact: true})).toBeVisible();
        expect(screen.getByText('Invalid fleet highLevelStatus response')).toBeVisible();
        expect(within(screen.getByTestId('fleet-card-alpha')).getByText('Idle', {exact: true})).toBeVisible();
        expect(screen.getAllByRole('spinbutton')[0]).toHaveValue('7.0');
        robots = [{...robot('alpha'), topics: {highLevelStatus: {state: 999, state_name: 'FUTURE_STATE'}}}, robot('bravo')];
        await act(() => vi.advanceTimersByTimeAsync(2000));
        expect(await screen.findByText('FUTURE_STATE')).toBeVisible();
        expect(screen.queryByText('Invalid fleet highLevelStatus response')).not.toBeInTheDocument();
        fireEvent.click(within(screen.getByTestId('fleet-card-bravo')).getByRole('button', {name: /Start mowing$/}));
        await waitFor(() => expect(mock.success).toHaveBeenCalledOnce());
        const commands = fetchMock.mock.calls.filter(([url]) => urlOf(url).includes('/call/'));
        expect(commands).toHaveLength(1);
        expect(urlOf(commands[0][0])).toMatch(/\/robots\/bravo\/call\/high_level_control$/);
        expect(commands[0][1]?.body).toBe('{"Command":1}');
    });
});

describe('FleetPage GNSS publication boundary', () => {
    const base = {fix_type: 6, rtk_mode: 3, fix_valid: true, dead_reckoning: false, capability_flags: 1, value_flags: 1};
    it.each([
        {name: 'text false validity', gnss: {...base, fix_valid: 'false'}},
        {name: 'text true dead reckoning', gnss: {...base, dead_reckoning: 'true'}},
        {name: 'text mask', gnss: {...base, capability_flags: '1'}},
        {name: 'array mask', gnss: {...base, value_flags: [1]}},
        {name: 'negative mask', gnss: {...base, capability_flags: -1}},
        {name: 'fractional mask', gnss: {...base, value_flags: 1.5}},
        {name: 'text enum', gnss: {...base, rtk_mode: '3'}},
        {name: 'JSON overflow enum', gnss: {...base, fix_type: '__OVERFLOW__'}},
    ])('rejects $name before rendering and permits recovery with unchanged commands', async ({gnss}) => {
        vi.useFakeTimers({toFake: ['setInterval', 'clearInterval']});
        mock.realFleet = true;
        let robots: unknown = [robot('alpha'), robot('bravo')];
        const urlOf = (url: Parameters<typeof fetch>[0]) => typeof url === 'string' ? url : url instanceof URL ? url.href : url.url;
        const fetchMock = vi.fn<typeof fetch>((url, init) => Promise.resolve(new Response(JSON.stringify(
            init?.method === 'POST' ? {} : urlOf(url).endsWith('/robots') ? robots : {},
        ).replace('"__OVERFLOW__"', '1e999'))));
        vi.stubGlobal('fetch', fetchMock);
        render(<ConfigProvider theme={{token: {motion: false}}}><FleetPage/></ConfigProvider>);
        await screen.findByTestId('fleet-card-alpha');
        robots = [{...robot('alpha'), topics: {gnssStatus: gnss}}, {...robot('bravo'), identity: {id: 'bravo', name: 'New Bravo'}}];
        await act(() => vi.advanceTimersByTimeAsync(2000));
        const alpha = screen.getByTestId('fleet-card-alpha');
        expect(alpha.textContent).not.toContain('FIX');
        expect(within(screen.getByTestId('fleet-card-bravo')).getByText('bravo', {exact: true})).toBeVisible();
        expect(screen.getByText('Invalid fleet gnssStatus response')).toBeVisible();
        expect(fetchMock.mock.calls.every(([, init]) => !init?.method)).toBe(true);
        robots = [{...robot('alpha'), topics: {gnssStatus: base}}, robot('bravo')];
        await act(() => vi.advanceTimersByTimeAsync(2000));
        expect(screen.getByTestId('fleet-card-alpha').textContent).toContain('FIX');
        expect(screen.queryByText('Invalid fleet gnssStatus response')).not.toBeInTheDocument();
        fireEvent.click(within(screen.getByTestId('fleet-card-bravo')).getByRole('button', {name: /Start mowing$/}));
        await waitFor(() => expect(mock.success).toHaveBeenCalledOnce());
        const commands = fetchMock.mock.calls.filter(([url]) => urlOf(url).includes('/call/'));
        expect(commands).toHaveLength(1);
        expect(urlOf(commands[0][0])).toMatch(/\/robots\/bravo\/call\/high_level_control$/);
        expect(commands[0][1]?.body).toBe('{"Command":1}');
    });
});

describe('FleetPage strict snapshot JSON', () => {
    it.each(['robots', 'coordination'])('keeps usable cards/settings and shows the $path read error', async path => {
        vi.useFakeTimers({toFake: ['setInterval', 'clearInterval']});
        mock.realFleet = true;
        let invalid = false;
        const urlOf = (url: Parameters<typeof fetch>[0]) => typeof url === 'string' ? url : url instanceof URL ? url.href : url.url;
        const fetchMock = vi.fn<typeof fetch>(url => {
            if (invalid && urlOf(url).endsWith(`/${path}`)) return Promise.resolve(new Response('{bad'));
            return Promise.resolve(new Response(JSON.stringify(urlOf(url).endsWith('/robots')
                ? [{...robot('alpha'), identity: {id: 'alpha', name: invalid ? 'Recent Alpha' : 'Initial Alpha'}}, robot('bravo')]
                : {settings: {enabled: true, yield_distance_m: invalid ? 12 : 7}, status: {}})));
        });
        vi.stubGlobal('fetch', fetchMock);
        render(<ConfigProvider theme={{token: {motion: false}}}><FleetPage/></ConfigProvider>);
        await screen.findByTestId('fleet-card-alpha');
        invalid = true;
        await act(() => vi.advanceTimersByTimeAsync(2000));
        expect(screen.getByTestId('fleet-card-alpha')).toBeVisible();
        expect(screen.getByTestId('fleet-card-bravo')).toBeVisible();
        expect(within(screen.getByTestId('fleet-card-alpha')).getByText(path === 'robots' ? 'Initial Alpha' : 'Recent Alpha')).toBeVisible();
        expect(screen.getByRole('switch')).toBeChecked();
        expect(screen.getAllByRole('spinbutton')[0]).toHaveValue('7.0');
        expect(screen.getByText('Invalid fleet snapshot JSON response')).toBeVisible();
        invalid = false;
        await act(() => vi.advanceTimersByTimeAsync(2000));
        expect(screen.queryByText('Invalid fleet snapshot JSON response')).not.toBeInTheDocument();
        expect(screen.getByTestId('fleet-card-bravo')).toBeVisible();
        expect(fetchMock.mock.calls.every(([, init]) => !init?.method)).toBe(true);
    });
});
