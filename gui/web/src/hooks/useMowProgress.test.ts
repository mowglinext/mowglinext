import {act, renderHook} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';

const subscribe = vi.fn();
const unsubscribe = vi.fn();
vi.mock('./multiplexedSocket.ts', () => ({
    getMultiplexedSocket: () => ({subscribe}),
}));

import {useDocumentVisible} from './useDocumentVisible';
import {useMowProgress} from './useMowProgress';

/**
 * The mow-progress OccupancyGrid is 351 kB per message and arrives at ~0.6 Hz
 * — ~207 kB/s into the browser, measured on the field robot 2026-09-16. The
 * hook's throttleMs only limits React re-renders: the websocket still delivers
 * every byte. A backgrounded tab must therefore hold NO subscription at all,
 * and must pick the stream straight back up when the operator returns.
 */

/** Drive document.visibilityState and fire the event the hook listens on. */
const setVisibility = (state: 'visible' | 'hidden') => {
    Object.defineProperty(document, 'visibilityState', {
        configurable: true,
        get: () => state,
    });
    act(() => {
        document.dispatchEvent(new Event('visibilitychange'));
    });
};

describe('useMowProgress visibility gating', () => {
    beforeEach(() => {
        subscribe.mockReset();
        unsubscribe.mockReset();
        subscribe.mockImplementation(() => unsubscribe);
        setVisibility('visible');
    });

    afterEach(() => {
        setVisibility('visible');
    });

    it('subscribes while the tab is visible and the grid is rendered', () => {
        renderHook(() => useMowProgress(useDocumentVisible()));

        expect(subscribe).toHaveBeenCalledTimes(1);
        expect(subscribe.mock.calls[0][0]).toBe('mowProgress');
    });

    it('holds no subscription while the page that renders the grid is unmounted', () => {
        // Nothing on screen draws the grid -> the caller passes enabled=false.
        renderHook(() => useMowProgress(false));

        expect(subscribe).not.toHaveBeenCalled();
    });

    it('drops the subscription when the tab is hidden and restores it on return', () => {
        renderHook(() => useMowProgress(useDocumentVisible()));
        expect(subscribe).toHaveBeenCalledTimes(1);

        setVisibility('hidden');
        expect(unsubscribe).toHaveBeenCalledTimes(1);
        expect(subscribe).toHaveBeenCalledTimes(1); // no re-subscribe while hidden

        setVisibility('visible');
        expect(subscribe).toHaveBeenCalledTimes(2);
        expect(unsubscribe).toHaveBeenCalledTimes(1);
    });

    it('does not keep showing a stale grid after the tab is hidden', () => {
        let listener: ((raw: unknown) => void) | undefined;
        subscribe.mockImplementation((_topic: string, cb: (raw: unknown) => void) => {
            listener = cb;
            return unsubscribe;
        });

        const {result} = renderHook(() => useMowProgress(useDocumentVisible()));
        act(() => listener?.({info: {width: 2, height: 2, resolution: 0.1}, data: [100, 0, 0, 0]}));
        expect(result.current.info?.width).toBe(2);

        setVisibility('hidden');
        expect(result.current).toEqual({});
    });
});

describe('useDocumentVisible', () => {
    beforeEach(() => setVisibility('visible'));
    afterEach(() => setVisibility('visible'));

    it('reports the visibility state at mount', () => {
        setVisibility('hidden');
        const {result} = renderHook(() => useDocumentVisible());
        expect(result.current).toBe(false);
    });

    it('tracks visibilitychange in both directions', () => {
        const {result} = renderHook(() => useDocumentVisible());
        expect(result.current).toBe(true);

        setVisibility('hidden');
        expect(result.current).toBe(false);

        setVisibility('visible');
        expect(result.current).toBe(true);
    });

    it('removes its listener on unmount', () => {
        const remove = vi.spyOn(document, 'removeEventListener');
        const {unmount} = renderHook(() => useDocumentVisible());
        unmount();
        expect(remove).toHaveBeenCalledWith('visibilitychange', expect.any(Function));
        remove.mockRestore();
    });
});
