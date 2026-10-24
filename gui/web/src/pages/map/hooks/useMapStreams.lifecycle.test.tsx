import {StrictMode, createRef, useState} from 'react';
import {act, renderHook} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import type {Map as MapboxMap} from 'mapbox-gl';
import {useMapStreams} from './useMapStreams';
import {useManualMode} from './useManualMode';
import type {TwistStamped} from '../../../types/ros';

type Stream = {start: ReturnType<typeof vi.fn<(uri: string) => void>>; stop: ReturnType<typeof vi.fn<() => void>>; sendJsonMessage: ReturnType<typeof vi.fn<(message: unknown, keep?: boolean) => void>>; open: () => void; error: () => void};
const mock = vi.hoisted(() => ({
    stateName: 'IDLE',
    streams: [] as Stream[],
    highStart: vi.fn<(uri: string) => void>(), highStop: vi.fn<() => void>(),
}));
vi.mock('../../../hooks/useWS.ts', () => ({useWS: (onError: (error: Error) => void, onInfo: (message: string) => void) => {
    const [stream] = useState(() => {
        const value = {start: vi.fn<(uri: string) => void>(), stop: vi.fn<() => void>(), sendJsonMessage: vi.fn<(message: unknown, keep?: boolean) => void>(), open: () => onInfo("Stream connected"), error: () => onError(new Error("Stream closed"))};
        mock.streams.push(value);
        return value;
    });
    // The wrapper may change identity even when its adapters remain stable.
    return {...stream};
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
    it('keeps trajectory and visual streams stable across controller ownership and render churn', () => {
        mock.stateName = 'RECORDING';
        const initial = options();
        const {rerender, result, unmount} = renderHook(props => useMapStreams(props), {initialProps: initial});
        const trajectory = mock.streams.find(stream => stream.start.mock.calls.some(([uri]) => uri.endsWith('/recordingTrajectory')))!;
        const joy = result.current.joyStream;
        const beforeStarts = starts();
        const beforeStops = stops();
        expect(trajectory.start).toHaveBeenCalledOnce();
        expect(trajectory.stop).not.toHaveBeenCalled();

        rerender({...initial, controllerOwnsInput: true});
        expect(joy.stop).toHaveBeenCalledOnce();
        rerender({...initial, controllerOwnsInput: true, settings: {...initial.settings}});
        expect(starts()).toBe(beforeStarts);
        expect(stops()).toBe(beforeStops + 1);
        rerender({...initial, controllerOwnsInput: false});
        expect(joy.start).toHaveBeenCalledTimes(2);
        expect(starts()).toBe(beforeStarts + 1);
        expect(trajectory.start).toHaveBeenCalledOnce();
        expect(trajectory.stop).not.toHaveBeenCalled();

        mock.stateName = 'IDLE';
        rerender(initial);
        expect(trajectory.stop).toHaveBeenCalledOnce();
        unmount();
    });

    it.each(['MANUAL_MOWING', 'RECORDING'])('does not restore an old touch gesture or keepalive after controller return in %s', stateName => {
        mock.stateName = stateName;
        const initial = options();
        const {result, rerender, unmount} = renderHook(props => {
            const streams = useMapStreams(props);
            const manual = useManualMode({mowerAction: () => () => Promise.resolve(), joyStream: streams.joyStream, stateName: mock.stateName, controllerOwnsInput: props.controllerOwnsInput});
            return {streams, manual};
        }, {initialProps: initial});
        const raw = mock.streams.find(s => s.start === result.current.streams.joyStream.start)!;
        const move = {type: 'move' as const, x: 0, y: 1, direction: null, distance: null};
        const nonzero = () => raw.sendJsonMessage.mock.calls.filter(([m]) => (m as TwistStamped).twist?.linear?.x !== 0).length;
        act(() => { raw.open(); result.current.manual.handleJoyStart(); result.current.manual.handleJoyMove(move); vi.advanceTimersByTime(200); });
        expect(nonzero()).toBe(3);

        rerender({...initial, controllerOwnsInput: true});
        const afterTakeover = raw.sendJsonMessage.mock.calls.length;
        act(() => { result.current.manual.handleJoyMove(move); vi.advanceTimersByTime(500); });
        expect(raw.sendJsonMessage.mock.calls).toHaveLength(afterTakeover);
        rerender({...initial, controllerOwnsInput: false});
        act(() => { raw.open(); result.current.manual.handleJoyMove(move); vi.advanceTimersByTime(500); });
        expect(nonzero()).toBe(3);
        expect(vi.getTimerCount()).toBe(0);

        act(() => { result.current.manual.handleJoyStart(); result.current.manual.handleJoyMove(move); vi.advanceTimersByTime(100); });
        expect(nonzero()).toBe(5);
        expect(raw.sendJsonMessage.mock.calls.every(([, keep]) => keep === false)).toBe(true);
        unmount();
    });

    it('never queues motion or revives a held gesture across a publisher reconnect', () => {
        mock.stateName = 'MANUAL_MOWING';
        const initial = options();
        const {result, unmount} = renderHook(() => {
            const streams = useMapStreams(initial);
            const manual = useManualMode({mowerAction: () => () => Promise.resolve(), joyStream: streams.joyStream, stateName: mock.stateName});
            return {streams, manual};
        });
        const raw = mock.streams.find(s => s.start === result.current.streams.joyStream.start)!;
        const move = {type: 'move' as const, x: 0, y: 1, direction: null, distance: null};
        const nonzero = () => raw.sendJsonMessage.mock.calls.filter(([m]) => (m as TwistStamped).twist?.linear?.x !== 0).length;
        act(() => { result.current.manual.handleJoyStart(); result.current.manual.handleJoyMove(move); vi.advanceTimersByTime(200); });
        expect(raw.sendJsonMessage).not.toHaveBeenCalled();
        act(() => raw.open());
        act(() => { vi.advanceTimersByTime(200); });
        expect(nonzero()).toBe(0);
        act(() => { result.current.manual.handleJoyStart(); result.current.manual.handleJoyMove(move); });
        expect(nonzero()).toBe(1);
        // Disconnect and reconnect before even one repeat timer gets a turn.
        act(() => { raw.error(); raw.open(); result.current.manual.handleJoyMove(move); vi.advanceTimersByTime(500); });
        expect(nonzero()).toBe(1);
        act(() => { result.current.manual.handleJoyStart(); result.current.manual.handleJoyMove(move); });
        expect(nonzero()).toBe(2);
        expect(raw.sendJsonMessage.mock.calls.every(([, keep]) => keep === false)).toBe(true);
        unmount();
        const count = nonzero();
        act(() => { vi.runAllTimers(); });
        expect(nonzero()).toBe(count);
    });

});
