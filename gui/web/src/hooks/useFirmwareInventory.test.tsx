import {act, cleanup, renderHook} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import type {Status} from '../types/ros';
import {useFirmwareInventory} from './useFirmwareInventory';

const socket = vi.hoisted(() => {
    type Connection = 'closed' | 'connecting' | 'open';
    const state: {status: Connection} = {status: 'closed'};
    const listeners = new Set<(status: Connection) => void>();
    const frames = new Set<(frame: unknown) => void>();
    const register = (listener: (status: Connection) => void) => {
        listeners.add(listener);
        return () => {listeners.delete(listener);};
    };
    return {state, listeners, frames, register,
        getStatus: vi.fn(() => state.status),
        onStatusChange: vi.fn(register),
        subscribe: vi.fn((_topic: string, listener: (frame: unknown) => void) => {
            frames.add(listener);
            return () => {frames.delete(listener);};
        }),
    };
});
vi.mock('./multiplexedSocket', () => ({getMultiplexedSocket: () => socket}));
const sample = (sec = 1, compatible = true): Status => ({stamp: {sec, nanosec: 0},
    firmware_version: 'fixture-version', firmware_protocol_version: 6, firmware_compatible: compatible});
const frame = (data: Status) => act(() => {socket.frames.forEach(listener => listener(data));});
const status = (value: typeof socket.state.status) => act(() => {
    socket.state.status = value;
    socket.listeners.forEach(listener => listener(value));
});

beforeEach(() => {
    vi.useFakeTimers();
    vi.setSystemTime(new Date('2026-10-17T08:00:00Z'));
    socket.state.status = 'closed';
    socket.listeners.clear(); socket.frames.clear();
    socket.onStatusChange.mockReset().mockImplementation(socket.register);
    socket.getStatus.mockClear(); socket.subscribe.mockClear();
});
afterEach(() => {cleanup(); vi.useRealTimers();});

describe('firmware inventory connection, reception and progression', () => {
    it('reports waiting when mounted after the socket opened, without inventing receipt or compatibility', () => {
        socket.state.status = 'open';
        const {result} = renderHook(useFirmwareInventory);
        expect(result.current).toEqual({data: {}, state: 'waiting'});
        expect(socket.subscribe).toHaveBeenCalledWith('status', expect.any(Function));
    });

    it('reports disconnected initially and waiting on a later open without any frame', () => {
        const {result} = renderHook(useFirmwareInventory);
        expect(result.current.state).toBe('disconnected');
        status('connecting');
        expect(result.current.state).toBe('disconnected');
        status('open');
        expect(result.current).toEqual({data: {}, state: 'waiting'});
    });

    it('requires distinct timestamps rather than a single or repeatedly delivered frame', () => {
        socket.state.status = 'open';
        const {result} = renderHook(useFirmwareInventory);
        frame(sample());
        expect(result.current).toEqual({data: sample(), state: 'waiting'});
        act(() => {vi.advanceTimersByTime(2000);});
        frame(sample()); frame(sample());
        expect(result.current.state).toBe('waiting');
        frame(sample(2));
        expect(result.current.state).toBe('compatible');
    });

    it('preserves the six-second expiry and does not renew it for repeated timestamps', () => {
        socket.state.status = 'open';
        const {result} = renderHook(useFirmwareInventory);
        frame(sample(1)); frame(sample(2));
        expect(result.current.state).toBe('compatible');
        act(() => {vi.advanceTimersByTime(6000);});
        frame(sample(2));
        expect(result.current.state).toBe('compatible');
        // The existing clock ticks once per second; >6000 becomes visible at 7000.
        act(() => {vi.advanceTimersByTime(1000);});
        expect(result.current.state).toBe('stale');
        frame(sample(2));
        expect(result.current.state).toBe('stale');
        frame(sample(3));
        expect(result.current.state).toBe('compatible');
    });

    it('preserves incomplete metadata and a reported incompatible verdict after progression', () => {
        socket.state.status = 'open';
        const {result} = renderHook(useFirmwareInventory);
        frame(sample(1)); frame({...sample(2), firmware_version: ''});
        expect(result.current.state).toBe('waiting');
        frame(sample(3, false));
        expect(result.current.state).toBe('incompatible');
    });

    it('forgets progression on disconnect and requires new progression after reconnect', () => {
        socket.state.status = 'open';
        const {result} = renderHook(useFirmwareInventory);
        frame(sample(1)); frame(sample(2));
        expect(result.current.state).toBe('compatible');
        status('closed');
        expect(result.current).toEqual({data: sample(2), state: 'disconnected'});
        status('open');
        expect(result.current.state).toBe('waiting');
        frame(sample(2)); frame(sample(2));
        expect(result.current.state).toBe('waiting');
        frame(sample(3));
        expect(result.current.state).toBe('compatible');
    });

    it.each(['open', 'closed'] as const)('rechecks a transition to %s during listener registration', next => {
        socket.state.status = next === 'open' ? 'closed' : 'open';
        socket.onStatusChange.mockImplementationOnce(listener => {
            // The transition occurs just before registration, so no event is replayed.
            socket.state.status = next;
            return socket.register(listener);
        });
        const {result} = renderHook(useFirmwareInventory);
        expect(result.current).toEqual({data: {}, state: next === 'open' ? 'waiting' : 'disconnected'});
    });

    it('removes all listeners and its timer so no notifications remain after unmount', () => {
        socket.state.status = 'open';
        const renders = vi.fn();
        const {unmount} = renderHook(() => {const value = useFirmwareInventory(); renders(value); return value;});
        expect(socket.listeners.size).toBeGreaterThan(0);
        expect(socket.frames.size).toBe(1);
        unmount();
        const calls = renders.mock.calls.length;
        expect(socket.listeners.size).toBe(0);
        expect(socket.frames.size).toBe(0);
        expect(vi.getTimerCount()).toBe(0);
        status('closed'); status('open'); frame(sample(9));
        act(() => {vi.advanceTimersByTime(10000);});
        expect(renders).toHaveBeenCalledTimes(calls);
    });
});
