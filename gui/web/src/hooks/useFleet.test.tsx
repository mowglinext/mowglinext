import {act, cleanup, renderHook, waitFor} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import {parseCoordination, useFleet, type FleetCoordination, type MapPushResult} from './useFleet';
import {deriveFleetRow} from '../utils/fleet';
import {useFleetPeers} from './useFleetPeers';
import {peerMapPoses} from '../utils/fleetPeers';

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

describe('Fleet snapshot order and lifetime', () => {
    function deferred() {
        let resolve!: (value: Response) => void;
        let reject!: (reason: unknown) => void;
        const promise = new Promise<Response>((yes, no) => {resolve = yes; reject = no;});
        return {promise, resolve, reject};
    }
    let reads: ReturnType<typeof deferred>[];
    const snapshotRobots = (name: string) => [{identity: {id: 'a', name, api_version: 1}, online: true, self: false}];
    const snapshotCoordination = (distance: number) => ({settings: {yield_distance_m: distance}, status: {}});
    const response = (data: unknown, status = 200) => new Response(JSON.stringify(data), {status});
    async function finish(cycle: number, name: string, distance: number) {
        await act(async () => {
            reads[cycle * 2].resolve(response(snapshotRobots(name)));
            reads[cycle * 2 + 1].resolve(response(snapshotCoordination(distance)));
            await Promise.resolve();
        });
    }
    beforeEach(() => {
        vi.useFakeTimers();
        reads = [];
        fetchMock.mockImplementation((_url, init) => {
            if (init?.method) return Promise.resolve(response({}));
            const read = deferred();
            reads.push(read);
            return read.promise;
        });
    });
    afterEach(() => vi.useRealTimers());

    it.each(['old-first', 'new-first'])('keeps the latest published snapshot with %s completion', async order => {
        const hook = renderHook(() => useFleet());
        await act(() => vi.advanceTimersByTimeAsync(2000));
        if (order === 'old-first') {
            await finish(0, 'Old', 2);
            expect(hook.result.current.robots).toEqual(snapshotRobots('Old'));
            await finish(1, 'New', 5);
        } else {
            await finish(1, 'New', 5);
            await finish(0, 'Old', 2);
        }
        expect(hook.result.current.robots).toEqual(snapshotRobots('New'));
        expect(hook.result.current.coordination.settings.yield_distance_m).toBe(5);
        expect(hook.result.current.error).toBeUndefined();
        expect(requests()).toEqual(['GET /robots', 'GET /coordination', 'GET /robots', 'GET /coordination']);
    });
    it.each(['robots', 'coordination'])('ignores an old %s error after a newer success', async field => {
        const hook = renderHook(() => useFleet());
        await act(() => vi.advanceTimersByTimeAsync(2000));
        await finish(1, 'New', 5);
        await act(async () => {
            if (field === 'robots') {
                reads[0].reject(new TypeError('Old robots failure'));
                reads[1].resolve(response(snapshotCoordination(2)));
            } else {
                reads[0].resolve(response(snapshotRobots('Old')));
                reads[1].reject(new TypeError('Old coordination failure'));
            }
            await Promise.resolve();
        });
        expect(hook.result.current.robots).toEqual(snapshotRobots('New'));
        expect(hook.result.current.coordination.settings.yield_distance_m).toBe(5);
        expect(hook.result.current.error).toBeUndefined();
    });
    it('keeps a recent robots error and the previous snapshot when an old success arrives', async () => {
        const hook = renderHook(() => useFleet());
        await finish(0, 'Initial', 7);
        await act(() => vi.advanceTimersByTimeAsync(4000));
        await act(async () => {
            reads[4].reject(new TypeError('Current robots failure'));
            reads[5].resolve(response(snapshotCoordination(9)));
            await Promise.resolve();
        });
        await finish(1, 'Old', 2);
        expect(hook.result.current.robots).toEqual(snapshotRobots('Initial'));
        expect(hook.result.current.coordination.settings.yield_distance_m).toBe(7);
        expect(hook.result.current.error).toBe('Current robots failure');
        await act(() => vi.advanceTimersByTimeAsync(2000));
        await finish(3, 'Recovered', 8);
        expect(hook.result.current.robots).toEqual(snapshotRobots('Recovered'));
        expect(hook.result.current.error).toBeUndefined();
    });
    it.each(['invalid', 'http', 'network'])('keeps recent robots with %s coordination against an old full success', async failure => {
        const hook = renderHook(() => useFleet());
        await finish(0, 'Initial', 7);
        await act(() => vi.advanceTimersByTimeAsync(2000));
        let manual!: Promise<void>;
        act(() => {manual = hook.result.current.refresh();});
        await act(async () => {
            if (failure === 'network') reads[5].reject(new TypeError('Current coordination failure'));
            else reads[5].resolve(response(failure === 'http'
                ? {error: 'Current coordination failure'} : {settings: {enabled: 'false'}, status: {}}, failure === 'http' ? 503 : 200));
            await Promise.resolve();
        });
        // Preserve the existing wait for both GETs, including on coordination failure.
        expect(hook.result.current.robots).toEqual(snapshotRobots('Initial'));
        await act(async () => {reads[4].resolve(response(snapshotRobots('New'))); await manual;});
        expect(hook.result.current.robots).toEqual(snapshotRobots('New'));
        expect(hook.result.current.coordination.settings.yield_distance_m).toBe(7);
        const error = failure === 'invalid' ? 'Invalid fleet coordination response' : 'Current coordination failure';
        expect(hook.result.current.error).toBe(error);
        await finish(1, 'Old', 2);
        expect(hook.result.current.robots).toEqual(snapshotRobots('New'));
        expect(hook.result.current.coordination.settings.yield_distance_m).toBe(7);
        expect(hook.result.current.error).toBe(error);
        expect(hook.result.current.loading).toBe(false);
    });
    it('does not reintroduce a removed peer after the action refresh publishes', async () => {
        const hook = renderHook(() => useFleet());
        const initialRobots = [...snapshotRobots('Alpha'), {identity: {id: 'b', name: 'Beta'}, online: true, self: false}];
        await act(async () => {reads[0].resolve(response(initialRobots)); reads[1].resolve(response(snapshotCoordination(3))); await Promise.resolve();});
        await act(() => vi.advanceTimersByTimeAsync(2000));
        let removal!: Promise<void>;
        await act(async () => {removal = hook.result.current.removePeer('b'); await Promise.resolve();});
        await act(async () => {reads[4].resolve(response(snapshotRobots('Alpha'))); reads[5].resolve(response(snapshotCoordination(3))); await removal;});
        await act(async () => {reads[2].resolve(response(initialRobots)); reads[3].resolve(response(snapshotCoordination(3))); await Promise.resolve();});
        expect(hook.result.current.robots).toEqual(snapshotRobots('Alpha'));
        expect(requests()).toEqual(['GET /robots', 'GET /coordination', 'GET /robots', 'GET /coordination',
            'DELETE /peers/b', 'GET /robots', 'GET /coordination']);
    });
    it.each(['strict', 'pollMs'])('invalidates the old setup on %s replay even before the new setup publishes', async mode => {
        const hook = renderHook(({pollMs}) => useFleet(pollMs), {initialProps: {pollMs: 2000}, reactStrictMode: mode === 'strict'});
        if (mode === 'pollMs') hook.rerender({pollMs: 3000});
        expect(reads).toHaveLength(4);
        expect(vi.getTimerCount()).toBe(1);
        await finish(0, 'Old', 2);
        expect(hook.result.current.robots).toEqual([]);
        expect(hook.result.current.loading).toBe(true);
        await finish(1, 'Current', 5);
        expect(hook.result.current.robots).toEqual(snapshotRobots('Current'));
        await act(() => vi.advanceTimersByTimeAsync(mode === 'strict' ? 2000 : 3000));
        expect(reads).toHaveLength(6);
        await finish(2, 'Next tick', 6);
        expect(hook.result.current.robots).toEqual(snapshotRobots('Next tick'));
        hook.unmount();
        expect(vi.getTimerCount()).toBe(0);
    });
    it('ignores the old StrictMode error without replaying updater side effects', async () => {
        const hook = renderHook(() => useFleet(), {reactStrictMode: true});
        await act(async () => {reads[0].reject(new TypeError('Old setup')); reads[1].resolve(response({})); await Promise.resolve();});
        expect(hook.result.current.error).toBeUndefined();
        expect(hook.result.current.loading).toBe(true);
        await act(async () => {reads[2].resolve(response(snapshotRobots('Active'))); reads[3].resolve(response({settings: {enabled: 'false'}, status: {}})); await Promise.resolve();});
        expect(hook.result.current.robots).toEqual(snapshotRobots('Active'));
        expect(hook.result.current.error).toBe('Invalid fleet coordination response');
        await act(() => vi.advanceTimersByTimeAsync(2000));
        await finish(2, 'Recovered', 4);
        expect(hook.result.current.error).toBeUndefined();
        expect(hook.result.current.robots).toEqual(snapshotRobots('Recovered'));
    });
    it('stops processing late data after unmount and clears polling', async () => {
        const hook = renderHook(() => useFleet());
        hook.unmount();
        expect(vi.getTimerCount()).toBe(0);
        await act(() => vi.advanceTimersByTimeAsync(6000));
        expect(fetchMock).toHaveBeenCalledTimes(2);
        const readIdentity = vi.fn(() => ({id: 'old', name: 'Old'}));
        const json = vi.spyOn(JSON, 'parse');
        json.mockImplementationOnce(() => [{get identity() {return readIdentity();}}]);
        await act(async () => {reads[0].resolve(response([])); reads[1].resolve(response({})); await Promise.resolve();});
        expect(readIdentity).not.toHaveBeenCalled();
    });
    it('continues to publish six-second cycles while newer polls are pending', async () => {
        const hook = renderHook(() => useFleet());
        await act(() => vi.advanceTimersByTimeAsync(6000));
        for (let id = 0; id < 4; id++) {
            if (id) await act(() => vi.advanceTimersByTimeAsync(2000));
            expect(reads.length / 2).toBeGreaterThan(id + 1);
            await finish(id, `Cycle ${id}`, 3 + id);
            expect(hook.result.current.robots).toEqual(snapshotRobots(`Cycle ${id}`));
            expect(hook.result.current.coordination.settings.yield_distance_m).toBe(3 + id);
            expect(hook.result.current.error).toBeUndefined();
        }
        expect(reads).toHaveLength(14);
        for (let id = 4; id < 7; id++) await finish(id, `Cycle ${id}`, 3 + id);
        expect(fetchMock).toHaveBeenCalledTimes(14);
    });
    it('waits for both GETs and rejects late cycles completed out of order', async () => {
        const hook = renderHook(() => useFleet());
        await finish(0, 'Initial', 3);
        await act(() => vi.advanceTimersByTimeAsync(2000));
        await act(async () => {reads[2].resolve(response(snapshotRobots('Old'))); await Promise.resolve();});
        expect(hook.result.current.robots).toEqual(snapshotRobots('Initial'));
        await act(() => vi.advanceTimersByTimeAsync(2000));
        await finish(2, 'Current', 5);
        await act(async () => {reads[3].resolve(response(snapshotCoordination(2))); await Promise.resolve();});
        expect(hook.result.current.robots).toEqual(snapshotRobots('Current'));
        expect(hook.result.current.coordination.settings.yield_distance_m).toBe(5);
    });
});

describe('useFleet robot identity rejection', () => {
    it.each([
        {name: 'duplicate ids', robots: [{identity: {id: 'same', name: 'Alpha'}}, {identity: {id: 'same', name: 'Beta'}}]},
        {name: 'malformed name', robots: [{identity: {id: 'a', name: {bad: true}}}]},
        {name: 'text availability', robots: [{identity: {id: 'a'}, online: 'false'}]},
        {name: 'text self', robots: [{identity: {id: 'a'}, self: 'false'}]},
    ])('keeps the last robots and coordination on $name then recovers', async ({robots}) => {
        let payload: unknown = [{identity: {id: 'valid', name: 'Valid'}, online: true, self: false}];
        fetchMock.mockImplementation(url => Promise.resolve(new Response(JSON.stringify(requestUrl(url).endsWith('/robots') ? payload : {}))));
        const {result} = await mountedFleet();
        const previous = result.current.robots;
        const coordination = result.current.coordination;
        payload = robots;
        await act(() => result.current.refresh());
        expect(result.current.robots).toBe(previous);
        expect(result.current.coordination).toBe(coordination);
        expect(result.current.error).toBe('Invalid fleet robots response');
        expect(result.current.loading).toBe(false);
        payload = [{identity: {id: 'new', name: 'New'}}];
        await act(() => result.current.refresh());
        expect(result.current.robots[0].identity.id).toBe('new');
        expect(result.current.error).toBeUndefined();
        expect(requests()).toEqual(['GET /robots', 'GET /coordination', 'GET /robots', 'GET /coordination']);
    });
});

const highLevelFields = ['state', 'state_name', 'sub_state_name', 'emergency', 'is_charging', 'current_area', 'coverage_percent'];
const highLevelInvalid = [
    ...['state', 'current_area', 'coverage_percent'].flatMap(key => [
        {name: `${key} text`, value: {[key]: '2'}},
        {name: `${key} object`, value: {[key]: {bad: true}}},
    ]),
    ...['state_name', 'sub_state_name'].map(key => ({name: `${key} object`, value: {[key]: {bad: true}}})),
    ...['emergency', 'is_charging'].map(key => ({name: `${key} text`, value: {[key]: 'false'}})),
    {name: 'array topic', value: []},
    {name: 'text topic', value: 'bad'},
    {name: 'boolean topic', value: false},
];
describe('useFleet HighLevelStatus publication boundary', () => {
    it.each(highLevelInvalid)('rejects $name and keeps both robots and coordination until recovery', async ({value}) => {
        let robots: unknown = [{identity: {id: 'a', name: 'Alpha'}}, {identity: {id: 'b', name: 'Beta'}}];
        let coordination: unknown = {settings: {yield_distance_m: 7}, status: {}};
        fetchMock.mockImplementation(url => Promise.resolve(new Response(JSON.stringify(requestUrl(url).endsWith('/robots') ? robots : coordination))));
        const {result} = await mountedFleet();
        const previous = result.current.robots;
        const previousCoordination = result.current.coordination;
        robots = [{identity: {id: 'new'}, topics: {highLevelStatus: value}}, {identity: {id: 'b', name: 'New Beta'}}];
        coordination = {settings: {yield_distance_m: 12}, status: {}};
        await act(() => result.current.refresh());
        expect(result.current.robots).toBe(previous);
        expect(result.current.coordination).toBe(previousCoordination);
        expect(result.current.error).toBe('Invalid fleet highLevelStatus response');
        expect(result.current.loading).toBe(false);
        robots = [{identity: {id: 'recovered'}, topics: {highLevelStatus: {state: 999, state_name: 'FUTURE'}}}];
        await act(() => result.current.refresh());
        expect(result.current.robots[0].identity.id).toBe('recovered');
        expect(result.current.coordination.settings.yield_distance_m).toBe(12);
        expect(result.current.error).toBeUndefined();
        expect(requests()).toEqual(['GET /robots', 'GET /coordination', 'GET /robots', 'GET /coordination']);
    });
    it.each(highLevelFields)('keeps a missing or null %s with its existing fallback', async field => {
        let topic: unknown = {};
        fetchMock.mockImplementation(url => Promise.resolve(new Response(JSON.stringify(requestUrl(url).endsWith('/robots')
            ? [{identity: {id: 'a'}, online: true, topics: {highLevelStatus: topic}}] : {}))));
        const {result} = await mountedFleet();
        const absent = deriveFleetRow(result.current.robots[0]);
        topic = {[field]: null};
        await act(() => result.current.refresh());
        expect(result.current.error).toBeUndefined();
        expect(deriveFleetRow(result.current.robots[0])).toEqual(absent);
    });
    it.each([undefined, null, {}])('accepts optional topic %s', async topic => {
        fetchMock.mockImplementation(url => Promise.resolve(new Response(JSON.stringify(requestUrl(url).endsWith('/robots')
            ? [{identity: {id: 'a'}, topics: {highLevelStatus: topic}}] : {}))));
        const {result} = await mountedFleet();
        expect(result.current.error).toBeUndefined();
        expect(result.current.robots).toHaveLength(1);
    });
    it.each(['state', 'current_area', 'coverage_percent'])('rejects nonfinite %s decoded from an actual JSON number', async field => {
        fetchMock.mockImplementation(url => Promise.resolve(new Response(requestUrl(url).endsWith('/robots')
            ? `[{"identity":{"id":"a"},"topics":{"highLevelStatus":{"${field}":1e999}}}]` : '{}')));
        const {result} = await mountedFleet();
        expect(result.current.robots).toEqual([]);
        expect(result.current.error).toBe('Invalid fleet highLevelStatus response');
    });
    it('does not apply HighLevelStatus validation to the peer hook or its valid poses', async () => {
        const raw = [{identity: {id: 'a', name: 'Alpha'}, online: true, self: false,
            topics: {highLevelStatus: {state_name: {bad: true}}, pose: {pose: {pose: {position: {x: 3, y: 4}}}, motion_heading: 0.5}}}];
        fetchMock.mockImplementation(() => Promise.resolve(new Response(JSON.stringify(raw))));
        const {result} = renderHook(() => useFleetPeers());
        await waitFor(() => expect(result.current).toHaveLength(1));
        expect(result.current).toEqual(raw);
        const positions = peerMapPoses(result.current, [48, 2, 0]);
        expect(positions).toHaveLength(1);
        expect(positions[0].x).toBeCloseTo(3);
        expect(positions[0].y).toBeCloseTo(4);
    });
});

const fleetGnssBase = {fix_type: 6, rtk_mode: 3, fix_valid: true, dead_reckoning: false, capability_flags: 1, value_flags: 1};
const fleetGnssInvalid = [
    ...['fix_valid', 'dead_reckoning'].flatMap(field => ['false', 'true', 1, {}, []].map((value, index) => ({name: `${field} invalid ${index}`, value: {...fleetGnssBase, [field]: value}}))),
    ...['fix_type', 'rtk_mode'].flatMap(field => ['3', true, {}, []].map((value, index) => ({name: `${field} invalid ${index}`, value: {...fleetGnssBase, [field]: value}}))),
    ...['capability_flags', 'value_flags'].flatMap(field => ['1', [1], -1, 1.5, 0x100000000].map((value, index) => ({name: `${field} invalid ${index}`, value: {...fleetGnssBase, [field]: value}}))),
    ...[[], 'bad', 0, false].map((value, index) => ({name: `topic shape ${index}`, value})),
];
describe('useFleet GNSS publication boundary', () => {
    it.each(fleetGnssInvalid)('rejects $name and preserves the entire previous snapshot until recovery', async ({value}) => {
        let robots: unknown = [{identity: {id: 'a', name: 'Alpha'}}, {identity: {id: 'b', name: 'Beta'}}];
        let coordination: unknown = {settings: {yield_distance_m: 7}, status: {}};
        fetchMock.mockImplementation(url => Promise.resolve(new Response(JSON.stringify(requestUrl(url).endsWith('/robots') ? robots : coordination))));
        const {result} = await mountedFleet();
        const previous = result.current.robots;
        const previousCoordination = result.current.coordination;
        robots = [{identity: {id: 'a'}, topics: {gnssStatus: value}}, {identity: {id: 'b', name: 'New Beta'}}];
        coordination = {settings: {yield_distance_m: 12}, status: {}};
        await act(() => result.current.refresh());
        expect(result.current.robots).toBe(previous);
        expect(result.current.coordination).toBe(previousCoordination);
        expect(result.current.loading).toBe(false);
        expect(result.current.error).toBe('Invalid fleet gnssStatus response');
        robots = [{identity: {id: 'recovered'}, topics: {gnssStatus: fleetGnssBase}}];
        await act(() => result.current.refresh());
        expect(result.current.robots[0].identity.id).toBe('recovered');
        expect(result.current.coordination.settings.yield_distance_m).toBe(12);
        expect(result.current.error).toBeUndefined();
        expect(requests()).toEqual(['GET /robots', 'GET /coordination', 'GET /robots', 'GET /coordination']);
    });
    it.each([
        ['fix_type', 100, 0], ['rtk_mode', 40, 0], ['fix_valid', 100, 100],
        ['dead_reckoning', 100, 100], ['capability_flags', 100, 40], ['value_flags', 100, 40],
    ] as const)('preserves distinct absent/null behavior for %s', async (field, absentPercent, nullPercent) => {
        const gnss: Record<string, unknown> = {...fleetGnssBase};
        delete gnss[field];
        fetchMock.mockImplementation(url => Promise.resolve(new Response(JSON.stringify(requestUrl(url).endsWith('/robots')
            ? [{identity: {id: 'a'}, online: true, topics: {gnssStatus: gnss}}] : {}))));
        const {result} = await mountedFleet();
        expect(result.current.error).toBeUndefined();
        expect(deriveFleetRow(result.current.robots[0]).gpsPercent).toBe(absentPercent);
        gnss[field] = null;
        await act(() => result.current.refresh());
        expect(result.current.error).toBeUndefined();
        expect(result.current.robots[0].topics?.gnssStatus).toEqual(gnss);
        expect(deriveFleetRow(result.current.robots[0]).gpsPercent).toBe(nullPercent);
    });
    it.each([undefined, null, {}])('accepts absent/null/empty GNSS topic %s', async value => {
        fetchMock.mockImplementation(url => Promise.resolve(new Response(JSON.stringify(requestUrl(url).endsWith('/robots')
            ? [{identity: {id: 'a'}, topics: {gnssStatus: value}}] : {}))));
        const {result} = await mountedFleet();
        expect(result.current.error).toBeUndefined();
        expect(result.current.robots).toHaveLength(1);
    });
    it.each(['fix_type', 'rtk_mode', 'capability_flags', 'value_flags'])('rejects JSON overflow in %s', async field => {
        fetchMock.mockImplementation(url => Promise.resolve(new Response(requestUrl(url).endsWith('/robots')
            ? `[{"identity":{"id":"a"},"topics":{"gnssStatus":{"${field}":1e999}}}]` : '{}')));
        const {result} = await mountedFleet();
        expect(result.current.robots).toEqual([]);
        expect(result.current.error).toBe('Invalid fleet gnssStatus response');
    });
    it('leaves the peer hook and its valid positions independent from GNSS validation', async () => {
        const robots = [{identity: {id: 'a'}, online: true, self: false, topics: {gnssStatus: {fix_valid: 'false'},
            pose: {pose: {pose: {position: {x: 3, y: 4}}}, motion_heading: 0.5}}}];
        fetchMock.mockImplementation(() => Promise.resolve(new Response(JSON.stringify(robots))));
        const {result} = renderHook(() => useFleetPeers());
        await waitFor(() => expect(result.current).toHaveLength(1));
        expect(result.current).toEqual(robots);
        const [pose] = peerMapPoses(result.current, [48, 2, 0]);
        expect(pose.x).toBeCloseTo(3);
        expect(pose.y).toBeCloseTo(4);
    });
});

describe('useFleet GNSS publication ordering', () => {
    it.each(['new-error', 'new-success'])('keeps the latest accepted result with %s', async mode => {
        const reads: Array<(response: Response) => void> = [];
        fetchMock.mockImplementation(() => new Promise(resolve => {reads.push(resolve);}));
        const hook = renderHook(() => useFleet());
        const good = (name: string) => [{identity: {id: 'a', name}, topics: {gnssStatus: fleetGnssBase}}];
        const bad = [{identity: {id: 'a'}, topics: {gnssStatus: {fix_valid: 'false'}}}];
        await act(async () => {reads[0](new Response(JSON.stringify(good('Initial')))); reads[1](new Response('{}')); await Promise.resolve();});
        const previous = hook.result.current.robots;
        let old!: Promise<void>;
        let recent!: Promise<void>;
        act(() => {old = hook.result.current.refresh(); recent = hook.result.current.refresh();});
        await act(async () => {reads[4](new Response(JSON.stringify(mode === 'new-error' ? bad : good('Recent')))); reads[5](new Response('{}')); await recent;});
        await act(async () => {reads[2](new Response(JSON.stringify(mode === 'new-error' ? good('Old') : bad))); reads[3](new Response('{}')); await old;});
        if (mode === 'new-error') {
            expect(hook.result.current.robots).toBe(previous);
            expect(hook.result.current.error).toBe('Invalid fleet gnssStatus response');
        } else {
            expect(hook.result.current.robots[0].identity.name).toBe('Recent');
            expect(hook.result.current.error).toBeUndefined();
        }
    });
});

describe('useFleet strict snapshot JSON', () => {
    it.each(['robots', 'coordination'].flatMap(path => [
        {name: 'invalid JSON', text: '{bad'}, {name: 'empty body', text: ''}, {name: 'whitespace', text: '  \n '},
    ].map(item => ({path, ...item}))))('rejects $path $name and recovers without losing usable data', async ({path, text}) => {
        let invalid = false;
        fetchMock.mockImplementation(url => {
            const robots = requestUrl(url).endsWith('/robots');
            if (invalid && requestUrl(url).endsWith(`/${path}`)) return Promise.resolve(new Response(text));
            return Promise.resolve(new Response(JSON.stringify(robots
                ? [{identity: {id: 'a', name: invalid ? 'Recent' : 'Initial'}}]
                : {settings: {yield_distance_m: invalid ? 12 : 7}, status: {}})));
        });
        const {result} = await mountedFleet();
        const previous = result.current.robots;
        const previousCoordination = result.current.coordination;
        invalid = true;
        await act(() => result.current.refresh());
        if (path === 'robots') expect(result.current.robots).toBe(previous);
        else expect(result.current.robots[0].identity.name).toBe('Recent');
        expect(result.current.coordination).toBe(previousCoordination);
        expect(result.current.error).toBe('Invalid fleet snapshot JSON response');
        expect(result.current.loading).toBe(false);
        invalid = false;
        await act(() => result.current.refresh());
        expect(result.current.robots[0].identity.name).toBe('Initial');
        expect(result.current.error).toBeUndefined();
        expect(requests()).toEqual(['GET /robots', 'GET /coordination', 'GET /robots', 'GET /coordination']);
    });
    it.each(['{}', 'null', '[]'])('keeps the existing decoded-shape behavior for %s', async text => {
        fetchMock.mockImplementation(() => Promise.resolve(new Response(text, {headers: {'Content-Type': 'text/plain'}})));
        const {result} = await mountedFleet();
        expect(result.current.robots).toEqual([]);
        if (text === '{}') {
            expect(result.current.error).toBeUndefined();
            expect(result.current.coordination.settings.yield_distance_m).toBe(3);
        } else expect(result.current.error).toBe('Invalid fleet coordination response');
    });
    it.each(['robots', 'coordination'])('keeps the existing %s body-read error path', async path => {
        let unreadable = false;
        fetchMock.mockImplementation(url => {
            if (unreadable && requestUrl(url).endsWith(`/${path}`)) {
                return Promise.resolve(new Response(new ReadableStream({start(controller) {controller.error(new TypeError('Body read failed'));}})));
            }
            return Promise.resolve(new Response(JSON.stringify(requestUrl(url).endsWith('/robots')
                ? [{identity: {id: 'a', name: unreadable ? 'Recent' : 'Initial'}}] : {})));
        });
        const {result} = await mountedFleet();
        const previous = result.current.robots;
        const previousCoordination = result.current.coordination;
        unreadable = true;
        await act(() => result.current.refresh());
        expect(result.current.error).toBe('Body read failed');
        expect(result.current.coordination).toBe(previousCoordination);
        if (path === 'robots') expect(result.current.robots).toBe(previous);
        else expect(result.current.robots[0].identity.name).toBe('Recent');
        unreadable = false;
        await act(() => result.current.refresh());
        expect(result.current.error).toBeUndefined();
    });
    it.each(['call', 'removePeer', 'addPeer', 'setCoordination', 'startFresh'] as const)('preserves an empty HTTP204 acknowledgement for %s', async action => {
        fetchMock.mockImplementation((url, init) => Promise.resolve(init?.method
            ? new Response(null, {status: 204})
            : new Response(requestUrl(url).endsWith('/robots') ? '[]' : '{}')));
        const {result} = await mountedFleet();
        await act(async () => {
            switch (action) {
                case 'call': await result.current.call('a', 'high_level_control', {Command: 1}); break;
                case 'removePeer': await result.current.removePeer('a'); break;
                case 'addPeer': expect(await result.current.addPeer('peer.local')).toBeUndefined(); break;
                case 'setCoordination': await result.current.setCoordination(coordinationDefaults.settings); break;
                case 'startFresh': await result.current.startFresh(); break;
            }
        });
        expect(result.current.error).toBeUndefined();
        expect(fetchMock).toHaveBeenCalledTimes(3);
        expect(requests().slice(1)).toEqual(['GET /robots', 'GET /coordination']);
    });
    it.each([
        {text: '{"error":{"reason":"untyped"}}', error: 'HTTP 503'},
        {text: '{"error":""}', error: 'HTTP 503'},
        {text: '{"error":"   "}', error: 'HTTP 503'},
        {text: '{"error":{"error":"DB failed"}}', error: 'DB failed'},
        {text: '{"error":"  Native detail  "}', error: '  Native detail  '},
        {text: '{bad', error: 'HTTP 503'},
        {text: '', error: 'HTTP 503'},
    ])('HTTP non-OK messages: normalizes $text without changing refusal', async ({text, error}) => {
        fetchMock.mockImplementation(url => Promise.resolve(requestUrl(url).endsWith('/robots')
            ? new Response(text, {status: 503}) : new Response('{}')));
        const {result} = await mountedFleet();
        expect(result.current.error).toBe(error);
    });
    it.each(['new-error', 'new-success'])('keeps accepted publication order with a JSON $mode', async mode => {
        const reads: Array<(res: Response) => void> = [];
        fetchMock.mockImplementation(() => new Promise(resolve => {reads.push(resolve);}));
        const hook = renderHook(() => useFleet());
        const robots = (name: string) => JSON.stringify([{identity: {id: 'a', name}}]);
        await act(async () => {reads[0](new Response(robots('Initial'))); reads[1](new Response('{}')); await Promise.resolve();});
        const previous = hook.result.current.robots;
        let old!: Promise<void>;
        let recent!: Promise<void>;
        act(() => {old = hook.result.current.refresh(); recent = hook.result.current.refresh();});
        await act(async () => {reads[4](new Response(mode === 'new-error' ? '{bad' : robots('Recent'))); reads[5](new Response('{}')); await recent;});
        await act(async () => {reads[2](new Response(mode === 'new-error' ? robots('Old') : '{bad')); reads[3](new Response('{}')); await old;});
        if (mode === 'new-error') {
            expect(hook.result.current.robots).toBe(previous);
            expect(hook.result.current.error).toBe('Invalid fleet snapshot JSON response');
        } else {
            expect(hook.result.current.robots[0].identity.name).toBe('Recent');
            expect(hook.result.current.error).toBeUndefined();
        }
    });
});

describe('useFleet HTTP non-OK messages', () => {
    it.each(['robots', 'coordination'])('retains usable data after an unreadable HTTP503 %s body and recovers', async path => {
        let refused = false;
        fetchMock.mockImplementation(url => {
            if (refused && requestUrl(url).endsWith(`/${path}`)) {
                return Promise.resolve(new Response(new ReadableStream({start(controller) {
                    controller.error(new TypeError('Body read failed'));
                }}), {status: 503}));
            }
            return Promise.resolve(new Response(JSON.stringify(requestUrl(url).endsWith('/robots')
                ? [{identity: {id: 'a', name: refused ? 'Recent' : 'Initial'}}]
                : {settings: {yield_distance_m: refused ? 12 : 7}, status: {}})));
        });
        const {result} = await mountedFleet();
        const previous = result.current.robots;
        const coordination = result.current.coordination;
        refused = true;
        await act(() => result.current.refresh());
        expect(result.current.error).toBe('HTTP 503');
        expect(result.current.coordination).toBe(coordination);
        if (path === 'robots') expect(result.current.robots).toBe(previous);
        else expect(result.current.robots[0].identity.name).toBe('Recent');
        refused = false;
        await act(() => result.current.refresh());
        expect(result.current.error).toBeUndefined();
        expect(requests()).toEqual(['GET /robots', 'GET /coordination', 'GET /robots', 'GET /coordination']);
    });

    it.each(['robots', 'coordination'])('preserves snapshots and 2000ms polling after a textual HTTP503 %s failure', async path => {
        vi.useFakeTimers({toFake: ['setInterval', 'clearInterval']});
        try {
            let refused = false;
            fetchMock.mockImplementation(url => Promise.resolve(refused && requestUrl(url).endsWith(`/${path}`)
                ? new Response('{"error":{"error":"DB failed"}}', {status: 503})
                : new Response(JSON.stringify(requestUrl(url).endsWith('/robots')
                    ? [{identity: {id: 'a', name: refused ? 'Recent' : 'Initial'}}]
                    : {settings: {yield_distance_m: 7}, status: {}}))));
            const {result} = renderHook(() => useFleet());
            await act(() => vi.advanceTimersByTimeAsync(0));
            expect(result.current.loading).toBe(false);
            fetchMock.mockClear();
            const previous = result.current.robots;
            const coordination = result.current.coordination;
            refused = true;
            await act(() => vi.advanceTimersByTimeAsync(1999));
            expect(fetchMock).not.toHaveBeenCalled();
            await act(() => vi.advanceTimersByTimeAsync(1));
            expect(result.current.error).toBe('DB failed');
            expect(result.current.coordination).toBe(coordination);
            if (path === 'robots') expect(result.current.robots).toBe(previous);
            else expect(result.current.robots[0].identity.name).toBe('Recent');
            expect(requests()).toEqual(['GET /robots', 'GET /coordination']);
            refused = false;
            await act(() => vi.advanceTimersByTimeAsync(2000));
            expect(result.current.error).toBeUndefined();
            expect(fetchMock).toHaveBeenCalledTimes(4);
        } finally { cleanup(); vi.useRealTimers(); }
    });

    it.each(['addPeer', 'call', 'pushMap', 'startFresh', 'setCoordination'] as const)('normalizes refused %s action without retry or refresh', async action => {
        const {result} = await mountedFleet();
        fetchMock.mockResolvedValue(new Response('{"error":" "}', {status: 503}));
        await act(async () => {
            const request = action === 'addPeer' ? result.current.addPeer('peer.local')
                : action === 'call' ? result.current.call('a', 'high_level_control', {Command: 1})
                : action === 'pushMap' ? result.current.pushMap()
                : action === 'setCoordination' ? result.current.setCoordination(coordinationDefaults.settings)
                : result.current.startFresh();
            await expect(request).rejects.toThrow('HTTP 503');
        });
        expect(fetchMock).toHaveBeenCalledOnce();
    });

    it('keeps a fetch network rejection unchanged and does not refresh', async () => {
        const {result} = await mountedFleet();
        const failure = new TypeError('Network unavailable');
        fetchMock.mockRejectedValueOnce(failure);
        await expect(result.current.call('a', 'high_level_control', {Command: 1})).rejects.toBe(failure);
        expect(requests()).toEqual(['POST /robots/a/call/high_level_control']);
    });
});

describe('useFleet addPeer warning acknowledgement', () => {
    afterEach(() => vi.useRealTimers());

    async function ready(payload: unknown, status = 200) {
        vi.useFakeTimers({toFake: ['setInterval', 'clearInterval']});
        fetchMock.mockImplementation((url, init) => Promise.resolve(init?.method === 'POST'
            ? new Response(status === 204 ? null : JSON.stringify(payload), {status})
            : new Response(requestUrl(url).endsWith('/robots') ? '[]' : '{}')));
        const hook = renderHook(() => useFleet());
        await act(() => vi.advanceTimersByTimeAsync(0));
        expect(hook.result.current.loading).toBe(false);
        fetchMock.mockClear();
        return hook;
    }

    it.each([null, false, true, 42, {reason: 'partial registration'}, ['detail'], ' \t\n '].map(warning => ({warning})))(
        'signals an unusable warning without rejecting the acknowledged action: $warning', async ({warning}) => {
            const {result} = await ready({warning});
            await act(async () => { expect(await result.current.addPeer('peer.local:4006')).toBeNull(); });
            expect(requests()).toEqual(['POST /peers', 'GET /robots', 'GET /coordination']);
            expect(fetchMock.mock.calls[0][1]).toEqual({headers: {'Content-Type': 'application/json'}, method: 'POST', body: '{"address":"peer.local:4006"}'});
        },
    );

    it.each([
        {payload: {}, expected: undefined, status: 200},
        {payload: {warning: ''}, expected: '', status: 200},
        {payload: {warning: '  Reverse registration pending \n'}, expected: '  Reverse registration pending \n', status: 200},
        {payload: {}, expected: undefined, status: 204},
    ])('preserves the native acknowledgement: $payload / $status', async ({payload, expected, status}) => {
        const {result} = await ready(payload, status);
        await act(async () => { expect(await result.current.addPeer('peer.local')).toBe(expected); });
        expect(requests()).toEqual(['POST /peers', 'GET /robots', 'GET /coordination']);
    });

    it('waits for the existing refresh before resolving an unusable warning', async () => {
        const {result} = await ready({warning: false});
        let release: (value: Response) => void = () => { throw new Error('Refresh not started'); };
        const original = fetchMock.getMockImplementation()!;
        fetchMock.mockImplementation((url, init) => requestUrl(url).endsWith('/coordination')
            ? new Promise(resolve => { release = resolve; }) : original(url, init));
        const settled = vi.fn();
        const pending = result.current.addPeer('peer.local').then(settled);
        await act(() => vi.advanceTimersByTimeAsync(0));
        await waitFor(() => expect(requests()).toEqual(['POST /peers', 'GET /robots', 'GET /coordination']));
        expect(settled).not.toHaveBeenCalled();
        await act(async () => { release(new Response('{}')); await pending; });
        expect(settled).toHaveBeenCalledExactlyOnceWith(null);
    });

    it.each(['HTTP', 'network'])('does not retry or refresh after a %s refusal', async kind => {
        const {result} = await ready({});
        const failure = new TypeError('Network unavailable');
        if (kind === 'HTTP') fetchMock.mockResolvedValueOnce(new Response('{"error":"Registration refused"}', {status: 503}));
        else fetchMock.mockRejectedValueOnce(failure);
        await expect(result.current.addPeer('peer.local')).rejects.toThrow(kind === 'HTTP' ? 'Registration refused' : 'Network unavailable');
        expect(requests()).toEqual(['POST /peers']);
    });
});
