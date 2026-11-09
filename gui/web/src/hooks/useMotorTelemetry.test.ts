import {act, renderHook} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import {useBladeTelemetry} from './useBladeTelemetry';
import {useWheelRpm} from './useWheelRpm';
import type {Status, WheelTick} from '../types/ros';
const stream = vi.hoisted(() => ({status: {data: {} as Status, lastMessageAt: null as number | null},
    ticks: {data: {} as WheelTick, lastMessageAt: null as number | null}}));
vi.mock('./useTopic', () => ({useTopic: (key: 'status' | 'ticks', _initial: unknown, options?: {select?: (raw: unknown) => unknown}) => ({
    ...stream[key], data: options?.select ? options.select(stream[key].data) : stream[key].data,
})}));

describe('shared display hooks: acquisition versus cached delivery', () => {
    beforeEach(() => {
        vi.useFakeTimers(); vi.setSystemTime(10000);
        stream.status = {data: {stamp: {sec: 10, nanosec: 0}, blade_status_stamp: {sec: 10, nanosec: 0},
            mower_motor_rpm: 2000, mower_esc_current: 1, mower_esc_temperature: 40}, lastMessageAt: 10000};
        stream.ticks = {data: {stamp: {sec: 10, nanosec: 0}, wheel_tick_factor: 1000, valid_wheels: 12,
            wheel_ticks_rl: 0, wheel_ticks_rr: 0, wheel_direction_rl: 1, wheel_direction_rr: 1}, lastMessageAt: 10000};
    });
    afterEach(() => vi.useRealTimers());
    it('cached Status republication cannot renew a frozen blade observation', () => {
        const hook = renderHook(() => useBladeTelemetry('mavros'));
        expect(hook.result.current.rpm).toBe(2000);
        void act(() => vi.advanceTimersByTime(4000));
        stream.status = {...stream.status, lastMessageAt: Date.now()}; hook.rerender();
        void act(() => vi.advanceTimersByTime(2000));
        expect(hook.result.current.rpm).toBeNaN();
        stream.status = {data: {...stream.status.data, stamp: {sec: 16, nanosec: 0},
            blade_status_stamp: {sec: 16, nanosec: 0}, mower_motor_rpm: 0}, lastMessageAt: Date.now()};
        hook.rerender(); expect(hook.result.current.rpm).toBe(0); hook.unmount();
    });
    it('waits for two valid wheel acquisitions, ages cached ticks, and recovers without replay', () => {
        const hook = renderHook(() => useWheelRpm());
        expect(hook.result.current.rl).toBeNaN();
        void act(() => vi.advanceTimersByTime(1000));
        stream.ticks = {data: {...stream.ticks.data, stamp: {sec: 11, nanosec: 0}, wheel_ticks_rl: 100,
            wheel_ticks_rr: 100}, lastMessageAt: Date.now()};
        hook.rerender(); expect(hook.result.current.rl).toBeGreaterThan(0);
        void act(() => vi.advanceTimersByTime(4000));
        stream.ticks = {...stream.ticks, lastMessageAt: Date.now()}; hook.rerender();
        void act(() => vi.advanceTimersByTime(1000)); expect(hook.result.current.rl).toBeNaN();
        stream.ticks = {data: {...stream.ticks.data, stamp: {sec: 16, nanosec: 0}}, lastMessageAt: Date.now()};
        hook.rerender(); expect(hook.result.current.rl).toBeNaN();
        stream.ticks = {data: {...stream.ticks.data, stamp: {sec: 17, nanosec: 0}}, lastMessageAt: Date.now()+1000};
        hook.rerender(); expect(hook.result.current.rl).toBe(0);
        stream.ticks = {data: {...stream.ticks.data, valid_wheels: 0}, lastMessageAt: Date.now()+1000};
        hook.rerender(); expect(hook.result.current.rl).toBeNaN(); hook.unmount();
    });
});
