import {act, renderHook, waitFor} from '@testing-library/react';
import {beforeEach, describe, expect, it, vi} from 'vitest';
import {useRosbag} from './useRosbag';

const mock = vi.hoisted(() => ({request: vi.fn()}));
vi.mock('./useApi.ts', () => { const api = {request: mock.request}; return {useApi: () => api}; });

describe('rosbag request error boundary', () => {
    beforeEach(() => { mock.request.mockReset(); });

    it.each([
        [{error: {error: 'Capture refused'}}, 'Capture refused'],
        [{error: 'Capture refused'}, 'Capture refused'],
        [{message: 'Capture refused'}, 'Capture refused'],
        [{error: {error: 42}}, 'start failed'],
        [null, 'start failed'],
    ])('rejects a failed start and restores its busy state', async (failure, expected) => {
        mock.request.mockResolvedValueOnce({data: {active: false, recordings: []}}).mockRejectedValueOnce(failure);
        const {result} = renderHook(() => useRosbag());
        await waitFor(() => expect(result.current.status?.active).toBe(false));
        await act(async () => { await expect(result.current.start()).rejects.toThrow(expected); });
        expect(result.current.busy).toBeNull();
        expect(result.current.status?.active).toBe(false);
    });
});
