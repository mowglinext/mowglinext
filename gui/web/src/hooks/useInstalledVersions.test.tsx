import {act, cleanup, renderHook} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import {useInstalledVersions} from './useInstalledVersions';

const complete = () => ({
    observed_at: 'fixture-time', docker_available: true,
    server: {revision: 'abc', version: 'custom version', built_at: '', modified: false, extra: 42},
    components: [{component: 'future-component', name: 'fixture', state: 'future-state',
        image: 'image:custom', image_id: 'opaque-id', digests: ['opaque digest', ''],
        revision: '', version: 'no prescribed format', built_at: '', architecture: 'future-arch',
        metadata_available: true, extra: {retained: true}}],
    extra: ['retained'],
});
const fetchMock = vi.fn<typeof fetch>();
const reply = (body: unknown) => new Response(JSON.stringify(body), {status: 200});
const flush = async () => { await act(async () => { await vi.advanceTimersByTimeAsync(0); }); };
const deferred = () => {
    let resolve!: (response: Response) => void;
    const promise = new Promise<Response>(done => {resolve = done;});
    return {promise, resolve};
};

beforeEach(() => {
    vi.useFakeTimers();
    fetchMock.mockReset();
    vi.stubGlobal('fetch', fetchMock);
    vi.spyOn(document, 'hidden', 'get').mockReturnValue(false);
});
afterEach(() => {
    cleanup();
    vi.restoreAllMocks();
    vi.unstubAllGlobals();
    vi.useRealTimers();
});

describe('installed inventory boundary', () => {
    it.each([complete(), {components: [], docker_available: false},
        {components: [{}, {component: 'unknown', state: 'unknown', version: '', metadata_available: false, digests: []}], docker_available: true, server: {}}])('preserves a valid inventory including optional and additional fields: %j', async body => {
        const response = reply(body);
        // Preserve identity to detect normalization or filtering by the hook.
        const decoded: unknown = body;
        vi.spyOn(response, 'json').mockResolvedValue(decoded);
        fetchMock.mockResolvedValue(response);
        const {result} = renderHook(useInstalledVersions);
        await flush();
        expect(result.current).toMatchObject({loading: false, error: false});
        expect(result.current.data).toBe(body);
        expect(fetchMock.mock.calls[0][0]).toMatch(/\/api\/system\/versions$/);
        expect(fetchMock.mock.calls[0][1]?.signal).toBeInstanceOf(AbortSignal);
        expect(fetchMock.mock.calls[0][1]?.cache).toBe('no-store');
    });

    const invalid: unknown[] = [null, [], 1, 'inventory', {},
        {components: []}, {docker_available: true},
        {components: [], docker_available: 'false'}, {components: null, docker_available: true},
        ...[null, 4, 'component', []].map(entry => ({components: [complete().components[0], entry], docker_available: true})),
        ...['architecture', 'built_at', 'component', 'image', 'image_id', 'name', 'revision', 'state', 'version']
            .flatMap(key => [4, null, {}, false].map(value => ({...complete(), components: [{[key]: value}]}))),
        ...[null, 'digest', [null], [2], [{}], [true]].map(digests => ({...complete(), components: [{digests}]})),
        ...[null, 1, 'true'].map(metadata_available => ({...complete(), components: [{metadata_available}]})),
        ...[null, [], 'server', 2].map(server => ({...complete(), server})),
        ...['built_at', 'revision', 'version'].flatMap(key => [1, null, {}].map(value => ({...complete(), server: {[key]: value}}))),
        ...[null, 0, 'false'].map(modified => ({...complete(), server: {modified}})),
        ...[null, {}, 1].map(observed_at => ({...complete(), observed_at})),
    ];
    it.each(invalid)('rejects the whole malformed inventory before exposing it: %j', async body => {
        fetchMock.mockResolvedValue(reply(body));
        const {result} = renderHook(useInstalledVersions);
        await flush();
        expect(result.current).toMatchObject({data: null, error: true, loading: false});
    });

    it('keeps the previous valid inventory on invalid data and recovers on the next valid response', async () => {
        const first = complete();
        const recovered = {components: [], docker_available: false};
        fetchMock.mockResolvedValueOnce(reply(first)).mockResolvedValueOnce(reply({components: [null], docker_available: true}))
            .mockResolvedValueOnce(reply(recovered));
        const {result} = renderHook(useInstalledVersions);
        await flush();
        const previous = result.current.data;
        await act(async () => { await result.current.refresh(); });
        expect(result.current).toMatchObject({data: first, error: true, loading: false});
        expect(result.current.data).toBe(previous);
        await act(async () => { await result.current.refresh(); });
        expect(result.current).toMatchObject({data: recovered, error: false, loading: false});
    });

    it.each(['network', 'HTTP', 'JSON', 'AbortError'])('uses the existing failure state for %s failures and keeps prior data', async failure => {
        fetchMock.mockResolvedValueOnce(reply(complete()));
        const {result} = renderHook(useInstalledVersions);
        await flush();
        const previous = result.current.data;
        if (failure === 'HTTP') fetchMock.mockResolvedValueOnce(new Response(null, {status: 503}));
        else if (failure === 'JSON') fetchMock.mockResolvedValueOnce(new Response('{invalid'));
        else fetchMock.mockRejectedValueOnce(failure === 'AbortError' ? new DOMException('Aborted', 'AbortError') : new Error('Network'));
        await act(async () => {await result.current.refresh();});
        expect(result.current.data).toBe(previous);
        expect(result.current).toMatchObject({error: true, loading: false});
    });
});

describe('existing polling and cancellation', () => {
    it('polls at 30 seconds only when visible, refreshes on focus and cleans up on unmount', async () => {
        fetchMock.mockImplementation(() => Promise.resolve(reply(complete())));
        const {unmount} = renderHook(useInstalledVersions);
        await flush();
        await act(async () => {await vi.advanceTimersByTimeAsync(29999);});
        expect(fetchMock).toHaveBeenCalledTimes(1);
        await act(async () => {await vi.advanceTimersByTimeAsync(1);});
        expect(fetchMock).toHaveBeenCalledTimes(2);
        vi.spyOn(document, 'hidden', 'get').mockReturnValue(true);
        await act(async () => {await vi.advanceTimersByTimeAsync(30000);});
        expect(fetchMock).toHaveBeenCalledTimes(2);
        act(() => {window.dispatchEvent(new Event('focus'));});
        await flush();
        expect(fetchMock).toHaveBeenCalledTimes(3);
        const signal = fetchMock.mock.calls[2][1]?.signal;
        unmount();
        expect(signal?.aborted).toBe(true);
        await act(async () => {window.dispatchEvent(new Event('focus')); await vi.advanceTimersByTimeAsync(60000);});
        expect(fetchMock).toHaveBeenCalledTimes(3);
    });

    it('aborts a superseded request and ignores its late response', async () => {
        const old = deferred();
        const current = deferred();
        fetchMock.mockReturnValueOnce(old.promise).mockReturnValueOnce(current.promise);
        const {result} = renderHook(useInstalledVersions);
        const oldSignal = fetchMock.mock.calls[0][1]?.signal;
        let refresh!: Promise<void>;
        act(() => {refresh = result.current.refresh();});
        expect(oldSignal?.aborted).toBe(true);
        await act(async () => {old.resolve(reply(complete())); await old.promise;});
        expect(result.current).toMatchObject({data: null, loading: true, error: false});
        await act(async () => {current.resolve(reply({components: [], docker_available: false})); await refresh;});
        expect(result.current).toMatchObject({data: {components: [], docker_available: false}, loading: false, error: false});
    });

    it('does not publish an error for a request cancelled on unmount', async () => {
        let reject!: (error: unknown) => void;
        fetchMock.mockImplementationOnce((_url, init) => new Promise<Response>((_resolve, fail) => {
            reject = fail;
            init?.signal?.addEventListener('abort', () => reject(new DOMException('Aborted', 'AbortError')));
        }));
        const {result, unmount} = renderHook(useInstalledVersions);
        unmount();
        await flush();
        expect(result.current).toMatchObject({data: null, error: false});
    });
});
