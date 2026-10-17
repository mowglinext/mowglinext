import {act, cleanup, renderHook, waitFor} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import {useFleet, type MapPushResult} from './useFleet';

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
