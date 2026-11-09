import {act, cleanup, renderHook} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import {useFleetPeers, FLEET_PEERS_IDLE_POLL_MS, FLEET_PEERS_POLL_MS} from './useFleetPeers';
import {peerMapPoses, peerDisplayFeatures} from '../utils/fleetPeers';
import {DEFAULT_GEOMETRY} from './useRobotDescription';

const peer = (id: string, x = 3) => ({identity: {id, name: id}, online: true, self: false,
    topics: {pose: {pose: {pose: {position: {x, y: 4, z: 0}}}, motion_heading: 0.5}}});
const fetchMock = vi.fn<typeof fetch>();
let payload: unknown;
beforeEach(() => {
    vi.useFakeTimers();
    payload = [peer('valid')];
    fetchMock.mockReset();
    fetchMock.mockImplementation(() => Promise.resolve(new Response(JSON.stringify(payload))));
    vi.stubGlobal('fetch', fetchMock);
});
afterEach(() => {cleanup(); vi.unstubAllGlobals(); vi.useRealTimers();});
const flush = () => act(async () => {await vi.advanceTimersByTimeAsync(0);});

describe('useFleetPeers robot identity boundary', () => {
    it.each([
        {name: 'duplicates', value: [peer('same'), peer('same', 8)]},
        {name: 'name object', value: [{...peer('bad'), identity: {id: 'bad', name: {label: 'bad'}}}]},
        {name: 'self text', value: [{...peer('bad'), self: 'false'}]},
        {name: 'online text', value: [{...peer('bad'), online: 'false'}]},
    ])('keeps usable peers and map features when rejecting $name', async ({value}) => {
        const {result} = renderHook(() => useFleetPeers());
        await flush();
        const previous = result.current;
        const poses = peerMapPoses(previous, [48, 2, 0]);
        const features = peerDisplayFeatures(poses, 0, 0, [48, 2, 0], DEFAULT_GEOMETRY);
        payload = value;
        await act(() => vi.advanceTimersByTimeAsync(FLEET_PEERS_POLL_MS));
        expect(result.current).toBe(previous);
        expect(peerMapPoses(result.current, [48, 2, 0])).toEqual(poses);
        expect(peerDisplayFeatures(peerMapPoses(result.current, [48, 2, 0]), 0, 0, [48, 2, 0], DEFAULT_GEOMETRY)).toEqual(features);
        // A rejected parse follows the existing failed-read path: idle delay.
        payload = [peer('recovered')];
        await act(() => vi.advanceTimersByTimeAsync(FLEET_PEERS_IDLE_POLL_MS));
        expect(result.current[0].identity.id).toBe('recovered');
        expect(fetchMock).toHaveBeenCalledTimes(3);
        expect(fetchMock.mock.calls.every(([, init]) => !init?.method)).toBe(true);
    });
    it('preserves absent fields and only excludes true self or offline peers', async () => {
        payload = [peer('visible'), {...peer('self'), self: true}, {...peer('offline'), online: false}, {identity: {id: 'unknown'}}];
        const {result} = renderHook(() => useFleetPeers());
        await flush();
        expect(result.current).toEqual(payload);
        expect(peerMapPoses(result.current, [48, 2, 0]).map(p => p.id)).toEqual(['visible']);
    });
    it('keeps nominal peer/idle polling periods and cleans up the timer', async () => {
        const view = renderHook(() => useFleetPeers());
        await flush();
        await act(() => vi.advanceTimersByTimeAsync(1999));
        expect(fetchMock).toHaveBeenCalledOnce();
        payload = [];
        await act(() => vi.advanceTimersByTimeAsync(1));
        expect(fetchMock).toHaveBeenCalledTimes(2);
        await act(() => vi.advanceTimersByTimeAsync(14999));
        expect(fetchMock).toHaveBeenCalledTimes(2);
        await act(() => vi.advanceTimersByTimeAsync(1));
        expect(fetchMock).toHaveBeenCalledTimes(3);
        view.unmount();
        expect(vi.getTimerCount()).toBe(0);
    });
});
