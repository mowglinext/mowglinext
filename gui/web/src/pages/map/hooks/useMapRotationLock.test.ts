import {act, renderHook, waitFor} from '@testing-library/react';
import {describe, expect, it, vi} from 'vitest';
import type {ArgsProps} from 'antd/es/notification/interface';
import {useMapRotationLock} from './useMapRotationLock';
import {MAP_ROTATION_LOCKED_KEY} from './mapRotationConfig';

function options(config: Record<string, string> = {}) {
    return {
        config,
        setConfig: vi.fn<(values: Record<string, string>) => Promise<void>>().mockResolvedValue(undefined),
        notification: {error: vi.fn<(args: ArgsProps) => void>()},
    };
}

describe('map rotation lock', () => {
    it('reads the mower-wide lock from config', () => {
        expect(renderHook(() => useMapRotationLock(options({[MAP_ROTATION_LOCKED_KEY]: 'true'}))).result.current.rotationLocked).toBe(true);
        expect(renderHook(() => useMapRotationLock(options())).result.current.rotationLocked).toBe(false);
    });

    it('saves the requested lock state without reporting an error on success', async () => {
        const opts = options();
        const {result} = renderHook(() => useMapRotationLock(opts));
        act(() => result.current.setRotationLocked(true));
        await opts.setConfig.mock.results[0].value;
        expect(opts.setConfig).toHaveBeenCalledExactlyOnceWith({[MAP_ROTATION_LOCKED_KEY]: 'true'});
        expect(opts.notification.error).not.toHaveBeenCalled();
    });

    it('notifies the operator when the save fails', async () => {
        const opts = options({[MAP_ROTATION_LOCKED_KEY]: 'true'});
        opts.setConfig.mockRejectedValue(new Error('backend unavailable'));
        const {result} = renderHook(() => useMapRotationLock(opts));
        act(() => result.current.setRotationLocked(false));
        await waitFor(() => expect(opts.notification.error).toHaveBeenCalledOnce());
        expect(opts.setConfig).toHaveBeenCalledExactlyOnceWith({[MAP_ROTATION_LOCKED_KEY]: 'false'});
        expect(opts.notification.error.mock.calls[0][0]).toMatchObject({description: 'backend unavailable'});
    });
});
