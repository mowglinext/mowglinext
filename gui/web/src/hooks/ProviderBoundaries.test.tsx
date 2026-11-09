import {StrictMode, type PropsWithChildren} from 'react';
import {act, renderHook} from '@testing-library/react';
import {beforeEach, describe, expect, it, vi} from 'vitest';
import {NotificationCenterProvider} from './NotificationCenterProvider';
import {TimeFormatProvider} from './TimeFormatProvider';
import {useNotificationCenter} from './useNotificationCenter';
import {useTimeFormat} from './useTimeFormat';
import {LOG_TIME_ZONE_STORAGE_KEY} from '../utils/logTime';

const host = vi.hoisted(() => ({data: undefined, refresh: vi.fn<() => Promise<void>>()}));
vi.mock('./useHostUpdater', () => ({
    useHostUpdater: () => host,
    updaterRequest: vi.fn<() => Promise<void>>(),
}));

function Providers({children}: PropsWithChildren) {
    return <StrictMode><NotificationCenterProvider><TimeFormatProvider>
        {children}
    </TimeFormatProvider></NotificationCenterProvider></StrictMode>;
}

beforeEach(() => {localStorage.clear(); vi.clearAllMocks();});

describe('extracted providers and original public hooks', () => {
    it('retains usable defaults outside the providers', () => {
        const {result} = renderHook(() => ({notices: useNotificationCenter(), time: useTimeFormat()}));
        expect(result.current.notices.items).toEqual([]);
        expect(result.current.notices.unread).toBe(0);
        expect(typeof result.current.time.formatLogTime(0)).toBe('string');
    });

    it('shares each context between its moved provider and its original hook under StrictMode', () => {
        const {result} = renderHook(() => ({notices: useNotificationCenter(), time: useTimeFormat()}), {wrapper: Providers});
        act(() => {result.current.notices.push({level: 'info', title: 'one'});});
        expect(result.current.notices.items.map(n => n.title)).toEqual(['one']);
        expect(result.current.notices.unread).toBe(1);
        act(() => {result.current.notices.markRead(result.current.notices.items[0].id);});
        expect(result.current.notices.unread).toBe(0);
        act(() => {result.current.time.setTimeZoneMode('utc');});
        expect(result.current.time.timeZoneMode).toBe('utc');
        expect(localStorage.getItem(LOG_TIME_ZONE_STORAGE_KEY)).toBe('utc');
        expect(result.current.time.zoneLabel).toBe('UTC');
    });

    it('keeps notification state local to each provider instance', () => {
        const first = renderHook(useNotificationCenter, {wrapper: Providers});
        const second = renderHook(useNotificationCenter, {wrapper: Providers});
        act(() => {first.result.current.push({level: 'warning', title: 'first only'});});
        expect(first.result.current.items).toHaveLength(1);
        expect(second.result.current.items).toEqual([]);
        act(() => {first.result.current.clear();});
        expect(first.result.current.items).toEqual([]);
        expect(second.result.current.items).toEqual([]);
    });

    it('retains persisted time preference on a genuine provider remount', () => {
        const first = renderHook(useTimeFormat, {wrapper: Providers});
        act(() => {first.result.current.setTimeZoneMode('utc');});
        first.unmount();
        const second = renderHook(useTimeFormat, {wrapper: Providers});
        expect(second.result.current.timeZoneMode).toBe('utc');
        expect(second.result.current.zoneLabel).toBe('UTC');
    });
});
