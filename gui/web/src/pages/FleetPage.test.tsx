import {act, cleanup, fireEvent, render, screen, waitFor, within} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import type {FleetRobotWire} from '../utils/fleet';
import type {FleetCoordination, MapPushResult} from '../hooks/useFleet';
import {getColors} from '../theme/colors';
import en from '../i18n/locales/en.json';
import {ConfigProvider} from 'antd';

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
        fireEvent.click(addButton());
        await waitFor(() => expect(mock.error).toHaveBeenCalledOnce());
        expect(mock.error).toHaveBeenCalledWith(expected);
        expect(typeof mock.error.mock.calls[0][0]).toBe('string');
        expect(mock.success).not.toHaveBeenCalled();
        expect(mock.warning).not.toHaveBeenCalled();
        await waitFor(() => expect(addButton()).not.toHaveClass('ant-btn-loading'));
        expect(screen.getByRole('textbox', {name: en.fleetPage.addPeerTitle})).toHaveValue('  mower-b.local:4006  ');
        fireEvent.click(addButton());
        await waitFor(() => expect(mock.success).toHaveBeenCalledExactlyOnceWith(en.fleetPage.peerAdded));
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
