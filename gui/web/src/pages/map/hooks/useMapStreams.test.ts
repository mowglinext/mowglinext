import {useRef, useState, type Dispatch, type SetStateAction} from 'react';
import {act, cleanup, renderHook} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import {PathFeature, type MowingFeature} from '../../../types/map.ts';
import {useMapStreams} from './useMapStreams.ts';

const mocks = vi.hoisted(() => ({
    stateName: 'IDLE',
    streams: [] as Array<{start: ReturnType<typeof vi.fn>; stop: ReturnType<typeof vi.fn>; onData: (data: unknown) => void}>,
    statusStart: vi.fn(),
    statusStop: vi.fn(),
}));

vi.mock('../../../hooks/useWS.ts', () => ({
    useWS: (_onError: unknown, _onInfo: unknown, onData: (data: unknown) => void) => {
        const stream = useRef<null | typeof mocks.streams[number]>(null);
        if (!stream.current) {
            stream.current = {start: vi.fn(), stop: vi.fn(), onData};
            mocks.streams.push(stream.current);
        }
        stream.current.onData = onData;
        return stream.current;
    },
}));
vi.mock('../../../hooks/useHighLevelStatus.ts', () => ({
    useHighLevelStatus: () => ({
        highLevelStatus: {state_name: mocks.stateName}, start: mocks.statusStart, stop: mocks.statusStop,
    }),
}));
vi.mock('../../../hooks/useRobotDescription.ts', () => ({useRobotDescription: () => ({})}));
vi.mock('../../../theme/ThemeContext.tsx', () => ({useThemeMode: () => ({displayMode: 'balanced'})}));
vi.mock('./useGridImageStream.ts', () => ({
    useGridImageStream: () => {
        const [stream] = useState(() => ({image: null, stream: {start: vi.fn(), stop: vi.fn()}, cancel: vi.fn()}));
        return stream;
    },
}));

type Features = Record<string, MowingFeature>;

function mount(stateName: string, initialFeatures: Features = {}) {
    mocks.stateName = stateName;
    let features = initialFeatures;
    const setFeatures = vi.fn<Dispatch<SetStateAction<Features>>>(next => {
        features = typeof next === 'function' ? next(features) : next;
    });
    const setEditMap = vi.fn();
    const {rerender, unmount} = renderHook(({ownsInput, editMap}) => useMapStreams({
        controllerOwnsInput: ownsInput,
        editMap,
        settings: {datum_lat: '52', datum_lon: '5'},
        offsetX: 0, offsetY: 0, datum: [52, 5, 0],
        setFeatures, setEditMap, setMapKey: vi.fn(),
        mapInstanceRef: {current: null}, robotPoseRef: {current: null},
    }), {initialProps: {ownsInput: false, editMap: false}});
    // useWS calls: pose, map, path, plan, joy, lidar, obstacles, recording trajectory.
    const joy = mocks.streams[4];
    const trajectory = mocks.streams[7];
    return {
        joy, trajectory, setFeatures, setEditMap, unmount,
        features: () => features,
        rerender: (ownsInput: boolean, editMap = false) => rerender({ownsInput, editMap}),
    };
}

beforeEach(() => {
    vi.useFakeTimers();
    mocks.streams = [];
    vi.clearAllMocks();
});

afterEach(() => {
    cleanup();
    vi.useRealTimers();
});

describe('useMapStreams controller ownership', () => {
    it('hands joy ownership over without restarting a recording trajectory subscription', () => {
        const hook = mount('RECORDING');
        expect(hook.trajectory.start).toHaveBeenCalledExactlyOnceWith('/api/mowglinext/subscribe/recordingTrajectory');
        expect(hook.setEditMap).toHaveBeenCalledExactlyOnceWith(false);
        expect(hook.joy.start).toHaveBeenCalledExactlyOnceWith('/api/mowglinext/publish/joy');

        hook.rerender(true);
        expect(hook.joy.stop).toHaveBeenCalledTimes(1);
        hook.rerender(false);
        expect(hook.joy.start).toHaveBeenCalledTimes(2);
        expect(hook.trajectory.start).toHaveBeenCalledTimes(1);
        expect(hook.trajectory.stop).not.toHaveBeenCalled();
        expect(hook.setFeatures).not.toHaveBeenCalled();
    });

    it('preserves an absent trajectory record and does no feature work on idle ownership changes', () => {
        const initial: Features = {};
        const hook = mount('IDLE', initial);
        expect(hook.features()).toBe(initial);
        hook.setFeatures.mockClear();
        hook.trajectory.stop.mockClear();

        hook.rerender(true);
        hook.rerender(false);
        expect(hook.features()).toBe(initial);
        expect(hook.setFeatures).not.toHaveBeenCalled();
        expect(hook.trajectory.start).not.toHaveBeenCalled();
        expect(hook.trajectory.stop).not.toHaveBeenCalled();
    });

    it('removes only the trajectory when recording ends and keeps the joy exit debounce', () => {
        const keep = new PathFeature('keep', [[5, 52], [5.1, 52.1]], '#ffffff', 2);
        const trajectory = new PathFeature('recording-trajectory', [[5, 52], [5.1, 52.1]], '#ff6600', 2);
        const initial = {keep, 'recording-trajectory': trajectory};
        const hook = mount('RECORDING', initial);

        mocks.stateName = 'IDLE';
        hook.rerender(false);
        expect(hook.trajectory.stop).toHaveBeenCalledTimes(1);
        expect(hook.features()).toEqual({keep});
        expect(initial['recording-trajectory']).toBe(trajectory);
        expect(hook.joy.stop).not.toHaveBeenCalled();
        act(() => { vi.advanceTimersByTime(1199); });
        expect(hook.joy.stop).not.toHaveBeenCalled();
        act(() => { vi.advanceTimersByTime(1); });
        expect(hook.joy.stop).toHaveBeenCalledTimes(1);
    });

    it('preserves feature identity when an empty trajectory message has nothing to remove', () => {
        const keep = new PathFeature('keep', [[5, 52], [5.1, 52.1]], '#ffffff', 2);
        const initial = {keep};
        const hook = mount('RECORDING', initial);

        act(() => { hook.trajectory.onData({poses: []}); });
        expect(hook.features()).toBe(initial);
    });

    it('keeps the edit-mode teardown effective across ownership changes', () => {
        const hook = mount('RECORDING');
        hook.rerender(false, true);
        expect(hook.trajectory.stop).toHaveBeenCalledTimes(1);
        hook.trajectory.start.mockClear();
        hook.setEditMap.mockClear();

        hook.rerender(true, true);
        expect(hook.trajectory.start).not.toHaveBeenCalled();
        expect(hook.setEditMap).not.toHaveBeenCalled();
    });
});
