import {StrictMode, createRef, useState} from 'react';
import {act, renderHook} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import type {Map as MapboxMap} from 'mapbox-gl';
import {useMapStreams} from './useMapStreams';

type Stream = {start: ReturnType<typeof vi.fn<(uri: string) => void>>; stop: ReturnType<typeof vi.fn<() => void>>; sendJsonMessage: ReturnType<typeof vi.fn<(message: unknown) => void>>};
const mock = vi.hoisted(() => ({
    stateName: 'IDLE',
    streams: [] as Stream[],
    highStart: vi.fn<(uri: string) => void>(), highStop: vi.fn<() => void>(),
}));
vi.mock('../../../hooks/useWS.ts', () => ({useWS: () => {
    const [stream] = useState(() => {
        const value = {start: vi.fn<(uri: string) => void>(), stop: vi.fn<() => void>(), sendJsonMessage: vi.fn<(message: unknown) => void>()};
        mock.streams.push(value);
        return value;
    });
    return stream;
}}));
vi.mock('../../../hooks/useHighLevelStatus.ts', () => ({useHighLevelStatus: () => ({
    highLevelStatus: {state_name: mock.stateName}, start: mock.highStart, stop: mock.highStop,
})}));
vi.mock('../../../hooks/useRobotDescription.ts', () => ({useRobotDescription: () => ({})}));
vi.mock('../../../theme/ThemeContext.tsx', () => ({useThemeMode: () => ({displayMode: 'balanced'})}));

describe('map stream dependency transitions', () => {
    beforeEach(() => { mock.streams.length = 0; mock.stateName = 'IDLE'; mock.highStart.mockClear(); mock.highStop.mockClear(); vi.useFakeTimers(); });
    afterEach(() => vi.useRealTimers());

    const options = (): Parameters<typeof useMapStreams>[0] => ({
        editMap: false, settings: {datum_lon: '2', datum_lat: '48'}, offsetX: 0, offsetY: 0,
        datum: [2, 48, 0],
        setFeatures: vi.fn(), setEditMap: vi.fn(), setMapKey: vi.fn(),
        mapInstanceRef: createRef<MapboxMap>(), robotPoseRef: createRef<{x: number; y: number; heading: number}>(),
    });
    const starts = () => mock.streams.reduce((sum, stream) => sum + stream.start.mock.calls.length, mock.highStart.mock.calls.length);
    const stops = () => mock.streams.reduce((sum, stream) => sum + stream.stop.mock.calls.length, mock.highStop.mock.calls.length);

    it('does not reconnect on settings object churn and stops cleanly through StrictMode replay', () => {
        const initial = options();
        const {rerender, unmount} = renderHook(props => useMapStreams(props), {initialProps: initial, wrapper: StrictMode});
        const beforeStarts = starts();
        const beforeStops = stops();
        rerender({...initial, settings: {...initial.settings}});
        expect(starts()).toBe(beforeStarts);
        expect(stops()).toBe(beforeStops);
        rerender({...initial, editMap: true});
        expect(starts()).toBe(beforeStarts);
        expect(stops()).toBeGreaterThan(beforeStops);
        rerender(initial);
        expect(starts()).toBeGreaterThan(beforeStarts);
        const beforeUnmount = stops();
        unmount();
        expect(stops()).toBeGreaterThan(beforeUnmount);
        const afterUnmount = stops();
        act(() => { vi.runAllTimers(); });
        expect(stops()).toBe(afterUnmount);
    });

    it('keeps the 1200 ms manual-exit debounce without repeated joy starts on telemetry renders', () => {
        mock.stateName = 'MANUAL_MOWING';
        const initial = options();
        const {rerender, result, unmount} = renderHook(props => useMapStreams(props), {initialProps: initial});
        const joy = result.current.joyStream;
        expect(joy.start).toHaveBeenCalledOnce();
        rerender({...initial, settings: {...initial.settings}});
        expect(joy.start).toHaveBeenCalledOnce();
        mock.stateName = 'IDLE';
        rerender(initial);
        act(() => { vi.advanceTimersByTime(1199); });
        expect(joy.stop).not.toHaveBeenCalled();
        mock.stateName = 'MANUAL_MOWING';
        rerender(initial);
        act(() => { vi.advanceTimersByTime(1); });
        expect(joy.stop).not.toHaveBeenCalled();
        mock.stateName = 'IDLE';
        rerender(initial);
        act(() => { vi.advanceTimersByTime(1200); });
        expect(joy.stop).toHaveBeenCalledOnce();
        unmount();
    });
});
