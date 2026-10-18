import {act, cleanup, renderHook, waitFor} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import {parseCoordination, useFleet, type FleetCoordination, type MapPushResult} from './useFleet';

const peer = {id: 'arbitrary id / é', name: 'Bravo 🐢', address: 'not a constrained address', ok: true};
const validCases = [
    {name: 'success', payload: {areas: 2, peers: [peer]}},
    {name: 'empty peers', payload: {areas: 0, peers: []}},
    {name: 'failure', payload: {areas: 2, peers: [{...peer, ok: false, error: 'Map refused'}]}},
    {name: 'multiple failures', payload: {areas: 2, peers: [{...peer, ok: false, error: 'Offline'}, {...peer, id: 'other', ok: false, error: 'No datum'}]}},
    {name: 'mixed', payload: {areas: 2, peers: [peer, {...peer, id: 'other', ok: false, error: 'Offline'}]}},
    {name: 'optional error absent on failure', payload: {areas: 2, peers: [{...peer, ok: false}]}},
    {name: 'extra fields and empty strings', payload: {areas: 2, extra: {keep: true}, peers: [{...peer, id: '', name: '', address: '', extra: [1, 2]}]}},
];
const invalidCases = [
    {name: 'null envelope', payload: null},
    {name: 'array envelope', payload: []},
    {name: 'areas missing', payload: {peers: []}},
    ...['2', null, false, 1.5].map(areas => ({name: `invalid areas ${String(areas)}`, payload: {areas, peers: []}})),
    {name: 'peers missing', payload: {areas: 2}},
    ...[null, {}, 'bad', 3].map((peers, index) => ({name: `invalid peers case ${index}`, payload: {areas: 2, peers}})),
    ...[null, 'bad', 42].map(entry => ({name: `invalid entry ${String(entry)}`, payload: {areas: 2, peers: [entry]}})),
    ...['id', 'name', 'address'].flatMap(key => [
        {name: `missing ${key}`, payload: {areas: 2, peers: [Object.fromEntries(Object.entries(peer).filter(([field]) => field !== key))]}},
        {name: `invalid ${key}`, payload: {areas: 2, peers: [{...peer, [key]: 7}]}},
    ]),
    {name: 'missing ok', payload: {areas: 2, peers: [{id: 'b', name: 'Bravo', address: 'b'}]}},
    ...['false', 'true', 1, null, {}].map((ok, index) => ({name: `invalid ok case ${index}`, payload: {areas: 2, peers: [{...peer, ok}]}})),
    ...[null, 42, {reason: 'Refused'}].map((error, index) => ({name: `invalid error case ${index}`, payload: {areas: 2, peers: [{...peer, ok: false, error}]}})),
    {name: 'valid failure followed by invalid entry', payload: {areas: 2, peers: [{...peer, ok: false, error: 'Important detail'}, null]}},
];
const fetchMock = vi.fn<typeof fetch>();
let pushText: string;
let pushStatus: number;
const requestUrl = (url: Parameters<typeof fetch>[0]) => typeof url === 'string' ? url : url instanceof URL ? url.href : url.url;
const requests = () => fetchMock.mock.calls.map(([url, init]) => `${init?.method ?? 'GET'} ${requestUrl(url).split('/api/fleet')[1]}`);

beforeEach(() => {
    pushText = JSON.stringify(validCases[0].payload);
    pushStatus = 200;
    fetchMock.mockReset();
    fetchMock.mockImplementation(url => {
        const push = requestUrl(url).endsWith('/map/push');
        return Promise.resolve(new Response(push ? pushText : requestUrl(url).endsWith('/robots') ? '[]' : '{}', {status: push ? pushStatus : 200}));
    });
    vi.stubGlobal('fetch', fetchMock);
});
afterEach(() => {
    cleanup();
    vi.restoreAllMocks();
    vi.unstubAllGlobals();
});
async function mountedFleet() {
    const hook = renderHook(() => useFleet());
    await waitFor(() => expect(hook.result.current.loading).toBe(false));
    fetchMock.mockClear();
    return hook;
}

describe('useFleet pushMap response boundary', () => {
    it.each(validCases)('preserves the original decoded object for $name', async ({payload}) => {
        pushText = JSON.stringify(payload);
        const parser = vi.spyOn(JSON, 'parse');
        const {result} = await mountedFleet();
        let received: MapPushResult | undefined;
        await act(async () => { received = await result.current.pushMap(); });
        expect(received).toEqual(payload);
        const index = parser.mock.calls.findIndex(([text]) => text === pushText);
        expect(index).toBeGreaterThanOrEqual(0);
        const decoded: unknown = parser.mock.results[index].value;
        expect(received).toBe(decoded);
        expect(requests()).toEqual(['POST /map/push', 'GET /robots', 'GET /coordination']);
        expect(fetchMock.mock.calls[0][1]).toMatchObject({method: 'POST', body: '{}'});
    });

    it.each(invalidCases)('rejects the whole result after refresh for $name', async ({payload}) => {
        pushText = JSON.stringify(payload);
        const {result} = await mountedFleet();
        await act(async () => {
            await expect(result.current.pushMap()).rejects.toThrow('Unusable map sharing result');
        });
        expect(requests()).toEqual(['POST /map/push', 'GET /robots', 'GET /coordination']);
    });

    it.each(['not JSON', '', '{"areas":1e999,"peers":[]}', '{"areas":NaN,"peers":[]}'])('rejects unreadable/nonfinite numeric JSON: %s', async text => {
        pushText = text;
        const {result} = await mountedFleet();
        await act(async () => { await expect(result.current.pushMap()).rejects.toThrow('Unusable map sharing result'); });
        expect(requests()).toEqual(['POST /map/push', 'GET /robots', 'GET /coordination']);
    });

    it('retains the HTTP error detail and does not refresh or repeat a rejected POST', async () => {
        pushText = JSON.stringify({error: 'No recorded area to share'});
        pushStatus = 400;
        const {result} = await mountedFleet();
        await expect(result.current.pushMap()).rejects.toThrow('No recorded area to share');
        expect(requests()).toEqual(['POST /map/push']);
    });

    it('waits for refresh to finish before rejecting an invalid success response', async () => {
        pushText = JSON.stringify({areas: 2, peers: [{...peer, ok: 'false'}]});
        const {result} = await mountedFleet();
        let release: (response: Response) => void = () => { throw new Error('Refresh not started'); };
        const ordinaryFetch = fetchMock.getMockImplementation();
        fetchMock.mockImplementation((url, init) => {
            if (requestUrl(url).endsWith('/coordination')) return new Promise(resolve => { release = resolve; });
            if (!ordinaryFetch) throw new Error('Missing fetch fixture');
            return ordinaryFetch(url, init);
        });
        const settled = vi.fn();
        const promise = result.current.pushMap();
        const assertion = expect(promise).rejects.toThrow('Unusable map sharing result');
        const completion = promise.then(settled, settled);
        await waitFor(() => expect(requests()).toEqual(['POST /map/push', 'GET /robots', 'GET /coordination']));
        expect(settled).not.toHaveBeenCalled();
        await act(async () => {
            release(new Response('{}'));
            await assertion;
            await completion;
        });
        expect(settled).toHaveBeenCalledOnce();
    });
});

const coordinationDefaults: FleetCoordination = {
    settings: {enabled: false, yield_distance_m: 3, resume_distance_m: 5, completed_ttl_h: 12, yield_max_hold_s: 45, yield_cooldown_s: 20},
    status: {enabled: false, excluded_areas: [], preferred_start: -1, yielded: false, completed_areas: []},
    peer_feed: {active: false, peers_published: 0, rate_hz: 5, publishes: 0},
};
const validCoordination = {
    settings: {...coordinationDefaults.settings, enabled: true},
    status: {...coordinationDefaults.status, excluded_areas: [1, 2], completed_areas: [3], last_error: 'Peer temporarily unavailable'},
    peer_feed: {...coordinationDefaults.peer_feed, active: true},
};
const invalidCoordination = [
    {name: 'null envelope', payload: null},
    {name: 'array envelope', payload: []},
    ...['settings', 'status', 'peer_feed'].flatMap(section => [null, [], 'invalid'].map(value => ({
        name: `${section} invalid shape ${JSON.stringify(value)}`, payload: {...validCoordination, [section]: value},
    }))),
    {name: 'enabled text', payload: {...validCoordination, settings: {...validCoordination.settings, enabled: 'false'}}},
    {name: 'status enabled text', payload: {...validCoordination, status: {...validCoordination.status, enabled: 'false'}}},
    {name: 'yielded numeric', payload: {...validCoordination, status: {...validCoordination.status, yielded: 1}}},
    {name: 'peer feed active text', payload: {...validCoordination, peer_feed: {...validCoordination.peer_feed, active: 'false'}}},
    ...['yield_distance_m', 'resume_distance_m', 'completed_ttl_h', 'yield_max_hold_s', 'yield_cooldown_s'].map(key => ({
        name: `${key} text`, payload: {...validCoordination, settings: {...validCoordination.settings, [key]: '3'}},
    })),
    ...['excluded_areas', 'completed_areas'].flatMap(key => [null, {length: 1}, '1,2', [1, '2'], [null]].map(value => ({
        name: `${key} invalid ${JSON.stringify(value)}`, payload: {...validCoordination, status: {...validCoordination.status, [key]: value}},
    }))),
    {name: 'preferred start text', payload: {...validCoordination, status: {...validCoordination.status, preferred_start: '0'}}},
    ...['peers_published', 'rate_hz', 'publishes'].map(key => ({
        name: `${key} text`, payload: {...validCoordination, peer_feed: {...validCoordination.peer_feed, [key]: '5'}},
    })),
    ...['status', 'peer_feed'].flatMap(section => ['last_error', section === 'status' ? 'last_push_at' : 'last_publish_at'].map(key => ({
        name: `${section}.${key} object`, payload: {...validCoordination, [section]: {...validCoordination[section as 'status' | 'peer_feed'], [key]: {reason: 'not text'}}},
    }))),
];

describe('useFleet coordination response boundary', () => {
    let coordination: unknown;
    let coordinationStatus: number;
    let robots: unknown;
    beforeEach(() => {
        coordination = validCoordination;
        coordinationStatus = 200;
        robots = [{identity: {id: 'a', name: 'Initial', api_version: 1}, online: true, self: false}];
        fetchMock.mockImplementation(url => Promise.resolve(requestUrl(url).endsWith('/coordination')
            ? new Response(JSON.stringify(coordination), {status: coordinationStatus})
            : new Response(JSON.stringify(robots))));
    });
    afterEach(() => vi.useRealTimers());

    it.each(invalidCoordination)('retains valid coordination, publishes valid robots and reports $name', async ({payload}) => {
        const hook = await mountedFleet();
        expect(hook.result.current.coordination).toEqual(validCoordination);
        coordination = payload;
        robots = [{identity: {id: 'b', name: 'Current', api_version: 1}, online: true, self: false}];
        await act(() => hook.result.current.refresh());
        expect(hook.result.current.error).toBe('Invalid fleet coordination response');
        expect(hook.result.current.coordination).toEqual(validCoordination);
        expect(hook.result.current.robots).toEqual(robots);
        expect(hook.result.current.loading).toBe(false);
        expect(requests()).toEqual(['GET /robots', 'GET /coordination']);
        coordination = {settings: {}, status: {}};
        await act(() => hook.result.current.refresh());
        expect(hook.result.current.coordination).toEqual(coordinationDefaults);
        expect(hook.result.current.error).toBeUndefined();
    });
    it.each([
        {}, {settings: {}, status: {}}, {settings: {}}, {status: {}},
        {settings: {enabled: true}}, {status: {yielded: true}},
    ])('preserves existing defaults for absent fields and sections: %j', async payload => {
        coordination = payload;
        const hook = await mountedFleet();
        expect(hook.result.current.coordination).toEqual(coordinationDefaults);
        expect(hook.result.current.error).toBeUndefined();
    });
    it('preserves additional properties, zero, negatives, decimals and text without new business restrictions', async () => {
        const payload = {
            extension: {keep: true},
            settings: {...validCoordination.settings, yield_distance_m: 0, resume_distance_m: -.5, completed_ttl_h: 1e100, extension: ['keep']},
            status: {...validCoordination.status, excluded_areas: [-1, .5], completed_areas: [], preferred_start: -.5, last_push_at: 'opaque date', last_error: '', extra: true},
            peer_feed: {...validCoordination.peer_feed, peers_published: -.5, rate_hz: 0, publishes: 1e100, last_publish_at: 'opaque date', last_error: '', extra: 'keep'},
        };
        coordination = payload;
        const hook = await mountedFleet();
        expect(hook.result.current.coordination).toEqual(payload);
        expect(hook.result.current.error).toBeUndefined();
        const parsed = parseCoordination(payload);
        expect(parsed.status.excluded_areas).toBe(payload.status.excluded_areas);
        expect(parsed.settings).toMatchObject({extension: payload.settings.extension});
    });
    it('keeps defaults on an invalid first response without losing the valid robots', async () => {
        coordination = {...validCoordination, status: {...validCoordination.status, last_error: {reason: 'bad'}}};
        const hook = await mountedFleet();
        expect(hook.result.current.coordination).toEqual(coordinationDefaults);
        expect(hook.result.current.robots).toEqual(robots);
        expect(hook.result.current.error).toBe('Invalid fleet coordination response');
    });
    it.each(['http', 'network'])('retains robots returned alongside a coordination %s failure', async failure => {
        const hook = await mountedFleet();
        const ordinaryFetch = fetchMock.getMockImplementation();
        coordinationStatus = 503;
        coordination = {error: 'Coordination unavailable'};
        robots = [{identity: {id: 'b', name: 'Current'}, online: false, self: false}];
        fetchMock.mockImplementation((url, init) => {
            if (failure === 'network' && requestUrl(url).endsWith('/coordination')) return Promise.reject(new TypeError('Coordination offline'));
            if (!ordinaryFetch) throw new Error('Missing fetch implementation');
            return ordinaryFetch(url, init);
        });
        await act(() => hook.result.current.refresh());
        expect(hook.result.current.coordination).toEqual(validCoordination);
        expect(hook.result.current.robots).toEqual(robots);
        expect(hook.result.current.error).toBe(failure === 'network' ? 'Coordination offline' : 'Coordination unavailable');
    });
    it('rejects overflowed JSON numbers without coercion', async () => {
        const hook = await mountedFleet();
        fetchMock.mockImplementation(url => Promise.resolve(new Response(requestUrl(url).endsWith('/coordination')
            ? '{"settings":{"yield_distance_m":1e999},"status":{}}' : JSON.stringify(robots))));
        await act(() => hook.result.current.refresh());
        expect(hook.result.current.error).toBe('Invalid fleet coordination response');
        expect(hook.result.current.coordination).toEqual(validCoordination);
    });
    it('retains the 2000 ms cadence and keeps publishing robots on invalid coordination', async () => {
        vi.useFakeTimers();
        const hook = renderHook(() => useFleet());
        await act(async () => {await Promise.resolve();});
        fetchMock.mockClear();
        coordination = {...validCoordination, settings: {...validCoordination.settings, enabled: 'false'}};
        await act(() => vi.advanceTimersByTimeAsync(1999));
        expect(fetchMock).not.toHaveBeenCalled();
        await act(() => vi.advanceTimersByTimeAsync(1));
        expect(requests()).toEqual(['GET /robots', 'GET /coordination']);
        expect(hook.result.current.error).toBe('Invalid fleet coordination response');
        expect(hook.result.current.coordination).toEqual(validCoordination);
        expect(hook.result.current.robots).toEqual(robots);
        hook.unmount();
        expect(vi.getTimerCount()).toBe(0);
    });
});
