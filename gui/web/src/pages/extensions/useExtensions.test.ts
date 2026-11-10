import {act, renderHook, waitFor} from '@testing-library/react';
import {afterEach, describe, expect, it, vi} from 'vitest';
import {useExtensions} from './useExtensions';

describe('extension refresh cadence', () => {
    afterEach(() => { vi.useRealTimers(); vi.unstubAllGlobals(); Object.defineProperty(document, 'visibilityState', {configurable: true, value: 'visible'}); });
    it('does not poll an idle page', async () => {
        const fetchMock = vi.fn(() => Promise.resolve(new Response(JSON.stringify({api: 1, configured: true, entries: []}))));
        vi.stubGlobal('fetch', fetchMock);
        const {unmount} = renderHook(() => useExtensions());
        await waitFor(() => expect(fetchMock).toHaveBeenCalledTimes(1));
        vi.useFakeTimers();
        await act(() => vi.advanceTimersByTimeAsync(10000));
        expect(fetchMock).toHaveBeenCalledTimes(1);
        unmount();
    });

    it('polls while running, pauses hidden tabs and stops on completion', async () => {
        const fetchMock = vi.fn(() => Promise.resolve(new Response(JSON.stringify({api: 1, configured: true, entries: [], operation: {action: 'install', id: 'test', phase: 'running'}}))));
        vi.stubGlobal('fetch', fetchMock);
        vi.useFakeTimers();
        const {result, unmount} = renderHook(() => useExtensions());
        await act(async () => {});
        expect(result.current.running).toBe(true);
        await act(() => vi.advanceTimersByTimeAsync(1000));
        expect(fetchMock).toHaveBeenCalledTimes(2);
        Object.defineProperty(document, 'visibilityState', {configurable: true, value: 'hidden'});
        await act(() => document.dispatchEvent(new Event('visibilitychange')));
        await act(() => vi.advanceTimersByTimeAsync(5000));
        expect(fetchMock).toHaveBeenCalledTimes(2);
        fetchMock.mockImplementation(() => Promise.resolve(new Response(JSON.stringify({api: 1, configured: true, entries: [], operation: {action: 'install', id: 'test', phase: 'succeeded'}}))));
        Object.defineProperty(document, 'visibilityState', {configurable: true, value: 'visible'});
        await act(() => document.dispatchEvent(new Event('visibilitychange')));
        await act(() => vi.advanceTimersByTimeAsync(1000));
        expect(result.current.running).toBe(false);
        await act(() => vi.advanceTimersByTimeAsync(5000));
        expect(fetchMock).toHaveBeenCalledTimes(3);
        unmount();
    });
});
