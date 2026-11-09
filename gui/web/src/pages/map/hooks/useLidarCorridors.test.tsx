import {act, renderHook, waitFor} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import {useLidarCorridors} from './useLidarCorridors';
import type {LidarIgnoreCorridor} from '../../../types/ros';

type Result = {data?: Record<string, unknown>; error?: {error: string; outcome?: string}};
const mocks = vi.hoisted(() => ({callCreate: vi.fn<(command: string, body: unknown) => Promise<Result>>()}));
// The API object must be stable like the real useApi singleton.
const api = {mowglinext: {callCreate: mocks.callCreate}};
vi.mock('../../../hooks/useApi.ts', () => ({useApi: () => api}));
const line: LidarIgnoreCorridor = {name: 'kept', id: 41, width_m: .4, polyline: {points: [{x: 0, y: 0, z: 0}, {x: 1, y: 0, z: 0}]}};
const get = (corridors = [line]) => ({data: {success: true, corridors}});
const commit = () => ({data: {success: true, committed: true, persisted: true, published: true, outcome: 'COMMITTED', corridors: [line]}});
const deferred = <T,>() => {let resolve!: (value: T) => void; const promise = new Promise<T>(r => {resolve = r;}); return {promise, resolve};};

beforeEach(() => {mocks.callCreate.mockReset(); mocks.callCreate.mockImplementation((command: string) => Promise.resolve(command === 'get_lidar_ignore_corridors' ? get() : commit()));});
afterEach(() => {vi.useRealTimers();});
describe('transactional corridor save', () => {
    it('sends one complete replacement and reloads canonical IDs', async () => {
        const hook = renderHook(() => useLidarCorridors());
        await waitFor(() => expect(hook.result.current.corridors).toEqual([line]));
        await act(async () => {await hook.result.current.save([line]);});
        expect(mocks.callCreate.mock.calls.filter(([command]) => command === 'set_lidar_ignore_corridors')).toHaveLength(1);
        expect(mocks.callCreate).toHaveBeenCalledWith('set_lidar_ignore_corridors', {corridors: [line]});
        expect(hook.result.current.busy).toBe(false);
    });
    it.each(['UNKNOWN_AFTER_SEND', 'COMMITTED_DURABILITY_UNCONFIRMED', 'COMMITTED_PUBLICATION_PENDING'])('reloads %s without retry or rollback', async outcome => {
        const hook = renderHook(() => useLidarCorridors());
        await waitFor(() => expect(hook.result.current.corridors).toEqual([line]));
        mocks.callCreate.mockImplementation((command: string) => Promise.resolve(command === 'get_lidar_ignore_corridors' ? get() : {error: {error: `producer: ${outcome}`, outcome}}));
        await act(async () => {await expect(hook.result.current.save([])).rejects.toThrow(outcome);});
        expect(hook.result.current.corridors).toEqual([line]);
        expect(mocks.callCreate.mock.calls.map(([command]) => command)).toEqual(['get_lidar_ignore_corridors', 'set_lidar_ignore_corridors', 'get_lidar_ignore_corridors']);
        expect(hook.result.current.busy).toBe(false);
    });
    it('does not silently accept a functional refusal carried by HTTP 200', async () => {
        const hook = renderHook(() => useLidarCorridors());
        await waitFor(() => expect(hook.result.current.corridors).toEqual([line]));
        mocks.callCreate.mockImplementation((command: string) => Promise.resolve(command === 'get_lidar_ignore_corridors' ? get() : {data: {success: false, committed: false, message: 'not applied'}}));
        await act(async () => {await expect(hook.result.current.save([])).rejects.toThrow('not applied');});
    });
    it('does not let an older GET erase the state reloaded after save', async () => {
        const hook = renderHook(() => useLidarCorridors());
        await waitFor(() => expect(hook.result.current.corridors).toEqual([line]));
        const old = deferred<ReturnType<typeof get>>();
        mocks.callCreate.mockReturnValueOnce(old.promise);
        let pending: Promise<void>;
        act(() => {pending = hook.result.current.reload();});
        await act(async () => {await hook.result.current.save([line]);});
        await act(async () => {old.resolve(get([])); await pending;});
        expect(hook.result.current.corridors).toEqual([line]);
    });
    it('retains the last confirmed snapshot on malformed or rejected reload', async () => {
        const hook = renderHook(() => useLidarCorridors());
        await waitFor(() => expect(hook.result.current.corridors).toEqual([line]));
        mocks.callCreate.mockResolvedValueOnce({data: {success: false, corridors: []}});
        await act(async () => {await hook.result.current.reload();});
        expect(hook.result.current.corridors).toEqual([line]);
        mocks.callCreate.mockResolvedValueOnce({data: {success: true}});
        await act(async () => {await hook.result.current.reload();});
        expect(hook.result.current.corridors).toEqual([line]);
    });
    it('cleans up the periodic refresh on unmount', async () => {
        vi.useFakeTimers();const hook = renderHook(() => useLidarCorridors());
        await act(async () => {await Promise.resolve();});
        hook.unmount();mocks.callCreate.mockClear();
        await act(async () => {await vi.advanceTimersByTimeAsync(30000);});
        expect(mocks.callCreate).not.toHaveBeenCalled();
    });
});
