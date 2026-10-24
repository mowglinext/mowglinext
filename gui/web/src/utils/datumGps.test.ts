import {describe, expect, it, vi} from 'vitest';
import {Api, type ApiErrorResponse} from '../api/Api';
import {DatumParseError, requestDatumFromGps} from './datumGps';

describe('requestDatumFromGps existing response contract', () => {
    it.each([
        {error: {error: 'datum refused'}, message: 'datum refused'},
        {error: {error: ''}, message: ''},
        {error: {}, message: ''},
    ])('preserves API error message "$message" without retry', async ({error, message}) => {
        const api = new Api<unknown>();
        const response = Object.assign(new Response(null), {data: {}, error: error satisfies ApiErrorResponse});
        const call = vi.spyOn(api.mowglinext, 'callCreate').mockResolvedValue(response);

        await expect(requestDatumFromGps(api)).rejects.toEqual(new Error(message));
        expect(call).toHaveBeenCalledExactlyOnceWith('set_datum', {});
    });

    it('passes through a rejected client request without replacing its error', async () => {
        const api = new Api<unknown>();
        const failure = new Error('connection lost');
        const call = vi.spyOn(api.mowglinext, 'callCreate').mockRejectedValue(failure);

        await expect(requestDatumFromGps(api)).rejects.toBe(failure);
        expect(call).toHaveBeenCalledExactlyOnceWith('set_datum', {});
    });

    it('retains the existing coordinate interpretation through the real API client', async () => {
        const fetchMock = vi.fn<typeof fetch>().mockResolvedValue(
            new Response(JSON.stringify({message: '48.5,-1.25'}), {status: 200}),
        );
        const api = new Api<unknown>({baseUrl: 'http://gui.test', customFetch: fetchMock});

        await expect(requestDatumFromGps(api)).resolves.toEqual({lat: 48.5, lon: -1.25});
        expect(fetchMock).toHaveBeenCalledTimes(1);
        expect(fetchMock.mock.calls[0][0]).toBe('http://gui.test/mowglinext/call/set_datum');
        expect(fetchMock.mock.calls[0][1]?.body).toBe('{}');
    });

    it('retains DatumParseError for an unreadable coordinate message', async () => {
        const fetchMock = vi.fn<typeof fetch>().mockResolvedValue(
            new Response(JSON.stringify({message: 'not,a-position'}), {status: 200}),
        );
        const api = new Api<unknown>({customFetch: fetchMock});

        await expect(requestDatumFromGps(api)).rejects.toEqual(new DatumParseError('not,a-position'));
        expect(fetchMock).toHaveBeenCalledTimes(1);
    });
});
