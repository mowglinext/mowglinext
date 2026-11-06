import {act, renderHook} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';

const subscribe = vi.fn();
const unsubscribe = vi.fn();
vi.mock('./multiplexedSocket.ts', () => ({
    getMultiplexedSocket: () => ({subscribe}),
}));

import {useTopic} from './useTopic';

// The upstream foxglove subscription opens on the first subscriber for a topic
// and closes on the last. A page that only shows a high-rate topic inside one
// collapsed panel must be able to hold NO subscription while it is hidden —
// /imu/data runs at ~90 Hz and foxglove_bridge serialises every message for as
// long as anyone is subscribed, whatever the client-side throttle.
describe('useTopic enabled flag', () => {
    beforeEach(() => {
        subscribe.mockReset();
        unsubscribe.mockReset();
        subscribe.mockImplementation(() => unsubscribe);
    });

    it('holds no subscription while disabled', () => {
        renderHook(() => useTopic('imu', {}, {enabled: false}));
        expect(subscribe).not.toHaveBeenCalled();
    });

    it('subscribes by default', () => {
        renderHook(() => useTopic('imu', {}));
        expect(subscribe).toHaveBeenCalledTimes(1);
        expect(subscribe.mock.calls[0][0]).toBe('imu');
    });

    it('subscribes when enabled turns on, and releases plus resets when it turns off', () => {
        let listener: ((raw: unknown) => void) | undefined;
        subscribe.mockImplementation((_topic: string, cb: (raw: unknown) => void) => {
            listener = cb;
            return unsubscribe;
        });
        const initial = {};
        const {result, rerender} = renderHook(
            ({enabled}) => useTopic<Record<string, unknown>>('imu', initial, {enabled}),
            {initialProps: {enabled: false}},
        );
        expect(subscribe).not.toHaveBeenCalled();

        rerender({enabled: true});
        expect(subscribe).toHaveBeenCalledTimes(1);
        act(() => listener?.({angular_velocity: {z: 0.5}}));
        expect(result.current.data).toEqual({angular_velocity: {z: 0.5}});

        rerender({enabled: false});
        expect(unsubscribe).toHaveBeenCalledTimes(1);
        // A hidden panel must not keep showing the last live sample as current.
        expect(result.current.data).toBe(initial);
    });
});


describe('useTopic delivery timestamps', () => {
    beforeEach(() => {
        vi.useFakeTimers();
        vi.setSystemTime(new Date('2026-10-15T00:00:00Z'));
        subscribe.mockReset();
        unsubscribe.mockReset();
    });
    afterEach(() => { vi.useRealTimers(); });

    it('preserves wall timestamps by default and offers monotonic delivery liveness', () => {
        const listeners = new Map<string, (raw: unknown) => void>();
        subscribe.mockImplementation((topic: string, listener: (raw: unknown) => void) => {
            listeners.set(topic, listener);
            return unsubscribe;
        });
        const {result} = renderHook(() => ({
            wall: useTopic('wall', {}, {withTimestamp: true}),
            monotonic: useTopic('monotonic', {}, {withTimestamp: true, timestampClock: 'monotonic'}),
        }));
        act(() => {
            listeners.get('wall')?.({state: 3});
            listeners.get('monotonic')?.({state: 3});
        });
        const firstWall = result.current.wall.lastMessageAt!;
        const firstMonotonic = result.current.monotonic.lastMessageAt!;
        expect(firstWall).toBe(Date.now());
        expect(firstMonotonic).toBe(performance.now());

        act(() => {
            vi.setSystemTime(Date.now() - 60 * 60 * 1000);
            vi.advanceTimersByTime(500);
            listeners.get('wall')?.({state: 4});
            listeners.get('monotonic')?.({state: 4});
        });
        expect(result.current.wall.lastMessageAt).toBe(firstWall - 60 * 60 * 1000 + 500);
        expect(result.current.monotonic.lastMessageAt).toBe(firstMonotonic + 500);
    });
});
