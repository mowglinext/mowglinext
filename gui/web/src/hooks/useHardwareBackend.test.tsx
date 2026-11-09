import {renderHook, waitFor} from '@testing-library/react';
import {afterEach, describe, expect, it, vi} from 'vitest';
const mock = vi.hoisted(() => ({api: {request: vi.fn()}}));
vi.mock('./useApi', () => ({useApi: () => mock.api}));
import {useHardwareBackend} from './useHardwareBackend';
afterEach(() => vi.resetAllMocks());
const payload = (backend: string) => ({backend, parameter_routes: {}, runtime_routing: 'available', default_overrides: {}, robot_name: 'Garden'});

describe('hardware metadata authority', () => {
    it.each(['mowgli', 'mavros', 'openmower'])('confirms only a valid %s response', async backend => {
        mock.api.request.mockResolvedValue({ok: true, data: payload(backend)});
        const hook = renderHook(useHardwareBackend);
        await waitFor(() => expect(hook.result.current.authority).toBe('confirmed'));
        expect(hook.result.current.backend).toBe(backend); hook.unmount();
    });
    it.each([[404, 'legacy_absent'], [405, 'legacy_absent'], [503, 'unavailable']])('separates HTTP %s from confirmed identity', async (status, authority) => {
        mock.api.request.mockRejectedValue({status});
        const hook = renderHook(useHardwareBackend);
        await waitFor(() => expect(hook.result.current.authority).toBe(authority));
        expect(hook.result.current.parameterRoutes).toEqual({}); expect(hook.result.current.backend).toBe('unknown'); hook.unmount();
    });
    it('network failure is not legacy Mowgli', async () => {
        mock.api.request.mockRejectedValue(new Error('offline'));
        const hook = renderHook(useHardwareBackend);
        await waitFor(() => expect(hook.result.current.authority).toBe('unavailable')); hook.unmount();
    });
    it.each([[{backend: 'bogus'}, 'unsupported'], [{backend: 'mowgli', runtime_routing: 'future'}, 'malformed'], [null, 'malformed']])(
        'separates successful but untrusted metadata %j', async (data, authority) => {
            mock.api.request.mockResolvedValue({ok: true, data}); const hook = renderHook(useHardwareBackend);
            await waitFor(() => expect(hook.result.current.authority).toBe(authority));
            expect(hook.result.current.parameterRoutes).toEqual({}); hook.unmount();
        });
    it('loading, rerenders and late responses cannot manufacture routes', async () => {
        let resolve: (value: unknown) => void = () => {};
        mock.api.request.mockImplementation(() => new Promise<unknown>(done => {resolve = done;}));
        const hook = renderHook(useHardwareBackend);
        expect(hook.result.current.backend).toBe('unknown'); expect(hook.result.current.parameterRoutes).toEqual({});
        hook.rerender(); expect(mock.api.request).toHaveBeenCalledTimes(1); hook.unmount();
        resolve({ok: true, data: payload('mowgli')}); await Promise.resolve();
        expect(hook.result.current.authority).toBe('loading');
    });
});

it('classifies a real HTTP Response and the producer unsupported error distinctly', async () => {
    mock.api.request.mockRejectedValue(Object.assign(new Response(null, {status: 503}), {error: {state: 'unsupported'}}));
    const hook = renderHook(useHardwareBackend);
    await waitFor(() => expect(hook.result.current.authority).toBe('unsupported')); hook.unmount();
    mock.api.request.mockRejectedValue(new Response(null, {status: 404}));
    const legacy = renderHook(useHardwareBackend);
    await waitFor(() => expect(legacy.result.current.authority).toBe('legacy_absent')); legacy.unmount();
});
