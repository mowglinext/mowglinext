import {act, renderHook} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import {useConfig} from '../../../hooks/useConfig';
import {useMapOffset} from './useMapOffset';
import type {ArgsProps} from 'antd/es/notification/interface';

const mocks = vi.hoisted(() => ({
    api: {config: {keysGetCreate: vi.fn(), keysSetCreate: vi.fn()}},
    notification: {error: vi.fn<(args: ArgsProps) => void>()},
}));
vi.mock('../../../hooks/useApi.ts', () => ({useApi: () => mocks.api}));
vi.mock('antd', () => ({App: {useApp: () => ({notification: mocks.notification})}}));
const key = 'gui.map.offset.x';
const advance = async (ms: number) => {await act(async () => {vi.advanceTimersByTime(ms); await Promise.resolve();});};
function useDisplay() {
    const db = useConfig([key]);
    const offsets = useMapOffset({...db, notification: mocks.notification});
    return {...offsets, dbConfig: db.config, loading: db.loading};
}
beforeEach(() => {
    vi.useFakeTimers();
    mocks.api.config.keysGetCreate.mockReset().mockResolvedValue({data: {[key]: '0'}});
    mocks.api.config.keysSetCreate.mockReset(); mocks.notification.error.mockClear();
});
afterEach(() => vi.useRealTimers());

describe('DB to map display preference boundary', () => {
    it.each(['empty_echo', 'http_failure', 'malformed_echo'])('does not confirm or retry a %s write', async failure => {
        mocks.api.config.keysSetCreate.mockResolvedValue(failure === 'empty_echo' ? {data: {}}
            : failure === 'http_failure' ? {data: {[key]: '5'}, error: {error: 'Storage failed'}} : {data: {[key]: 5}});
        const view = renderHook(useDisplay);
        await act(async () => {await Promise.resolve();});
        expect(view.result.current.loading).toBe(false);
        act(() => view.result.current.handleOffsetX(5)); await advance(1000);
        expect(view.result.current.offsetX).toBe(5); // Local unsaved draft, never a measurement.
        expect(view.result.current.dbConfig[key]).toBe('0');
        expect(mocks.notification.error).toHaveBeenCalledOnce();
        view.unmount(); await advance(1000);
        expect(mocks.api.config.keysSetCreate).toHaveBeenCalledExactlyOnceWith({[key]: '5'});
    });
    it('preserves a newer edit through a delayed initial DB read and then accepts a validated zero echo', async () => {
        let finish: (value: unknown) => void = () => {};
        mocks.api.config.keysGetCreate.mockReturnValue(new Promise(resolve => {finish = resolve;}));
        mocks.api.config.keysSetCreate.mockResolvedValue({data: {[key]: '0'}});
        const view = renderHook(useDisplay);
        act(() => view.result.current.handleOffsetX(0));
        await act(async () => {finish({data: {[key]: '12'}}); await Promise.resolve();});
        expect(view.result.current.offsetX).toBe(0);
        await advance(1000);
        expect(view.result.current.dbConfig[key]).toBe('0');
        expect(mocks.notification.error).not.toHaveBeenCalled();
    });
    it('rejects a malformed DB read without treating it as a valid numeric setting or issuing a write', async () => {
        mocks.api.config.keysGetCreate.mockResolvedValue({data: {[key]: null}});
        const {result} = renderHook(useDisplay);
        await act(async () => {await Promise.resolve();});
        expect(result.current.dbConfig).toEqual({}); expect(result.current.loading).toBe(false);
        expect(mocks.notification.error).toHaveBeenCalledOnce();
        expect(mocks.api.config.keysSetCreate).not.toHaveBeenCalled();
    });
});
