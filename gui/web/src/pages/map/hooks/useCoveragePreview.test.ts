import {act, renderHook, waitFor} from '@testing-library/react';
import {beforeEach, describe, expect, it, vi} from 'vitest';
import {useCoveragePreview} from './useCoveragePreview';
import {MowingAreaFeature} from '../../../types/map.ts';
import type {MapArea} from '../../../types/ros.ts';
import {transpose} from '../../../utils/map.tsx';
import type {AreaOverrideFields} from '../coveragePreview.ts';

const mocks = vi.hoisted(() => ({callCreate: vi.fn()}));
vi.mock('react-i18next', () => ({useTranslation: () => ({t: (key: string) => key})}));
vi.mock('../../../hooks/useApi.ts', () => ({useApi: () => ({mowglinext: {callCreate: mocks.callCreate}})}));

const datum: [number, number, number] = [52.0, 5.0, 0];
const SQUARE = [[0, 0], [10, 0], [10, 10], [0, 10]];

const area = (extra: Partial<MapArea> = {}, id = 'a') => {
    const f = new MowingAreaFeature(id, 1);
    const ring = SQUARE.map(([x, y]) => transpose(0, 0, datum, y, x));
    f.geometry = {type: 'Polygon', coordinates: [[...ring, ring[0]]]};
    f.area = {id: 5, name: 'lawn', area: {points: []}, ...extra} as MapArea;
    return f;
};

const base = (over: Partial<Parameters<typeof useCoveragePreview>[0]> = {}) => ({
    areas: [area()],
    obstacles: [],
    datum,
    offsetX: 0,
    offsetY: 0,
    globalAngleDeg: -1,
    globalDirection: 0,
    editMode: true,
    onCommit: vi.fn(),
    ...over,
});

const lastCall = () => mocks.callCreate.mock.calls[mocks.callCreate.mock.calls.length - 1] as [string, Record<string, unknown>];

const answer = (extra: Record<string, unknown> = {}) => ({
    data: {success: true, mow_angle_deg: 72.4, start_adjustable: true, start_x: 10, start_y: 5, rings: [], swaths: [], ...extra},
});

describe('useCoveragePreview', () => {
    beforeEach(() => {
        vi.clearAllMocks();
        mocks.callCreate.mockResolvedValue(answer());
    });

    describe('outside edit mode', () => {
        it('cannot change anything', () => {
            const props = base({editMode: false});
            const {result} = renderHook(() => useCoveragePreview(props));
            act(() => result.current.setDirection(2));
            act(() => result.current.setAngleMode('fixed'));
            act(() => result.current.setAngleDeg(40));
            act(() => result.current.commitAngle());
            act(() => result.current.setStart({x: 1, y: 2}));
            expect(props.onCommit).not.toHaveBeenCalled();
            expect(result.current.choices.direction).toBe('global');
            expect(result.current.choices.angleMode).toBe('global');
            expect(result.current.choices.start).toBeNull();
        });

        it('still shows what the area has', () => {
            const props = base({editMode: false, areas: [area({has_mow_angle: true, mow_angle_deg: 30, has_ring_direction: true, ring_direction: 1})]});
            const {result} = renderHook(() => useCoveragePreview(props));
            expect(result.current.choices).toMatchObject({angleMode: 'fixed', angleDeg: 30, direction: 1});
        });
    });

    describe('in edit mode', () => {
        it('puts a chosen direction into the edit session at once, with a flag per value', () => {
            const props = base();
            const {result} = renderHook(() => useCoveragePreview(props));
            act(() => result.current.setDirection(2));
            expect(props.onCommit).toHaveBeenCalledExactlyOnceWith('a', {
                has_mow_angle: false, mow_angle_deg: 0,
                has_ring_direction: true, ring_direction: 2,
                has_start_point: false, start_x: 0, start_y: 0,
            });
        });

        it('going back to robot-wide commits a cleared override', () => {
            const props = base({areas: [area({has_ring_direction: true, ring_direction: 2})]});
            const {result} = renderHook(() => useCoveragePreview(props));
            act(() => result.current.setDirection('global'));
            expect(props.onCommit).toHaveBeenCalledWith('a', expect.objectContaining({has_ring_direction: false, ring_direction: 0}));
        });

        it('commits "auto" as a negative angle with the flag set', () => {
            const props = base({globalAngleDeg: 30});
            const {result} = renderHook(() => useCoveragePreview(props));
            act(() => result.current.setAngleMode('auto'));
            expect(props.onCommit).toHaveBeenCalledWith('a', expect.objectContaining({has_mow_angle: true, mow_angle_deg: -1}));
        });

        it('a slider drag moves the preview only; letting go commits once', () => {
            const props = base();
            const {result} = renderHook(() => useCoveragePreview(props));
            act(() => result.current.setAngleDeg(40));
            act(() => result.current.setAngleDeg(45));
            act(() => result.current.setAngleDeg(50));
            expect(props.onCommit).not.toHaveBeenCalled();
            expect(result.current.choices).toMatchObject({angleMode: 'fixed', angleDeg: 50});

            act(() => result.current.commitAngle());
            expect(props.onCommit).toHaveBeenCalledExactlyOnceWith('a', expect.objectContaining({has_mow_angle: true, mow_angle_deg: 50}));
        });

        it('starts a fixed angle from the angle the planner is using now', async () => {
            const props = base();
            const {result} = renderHook(() => useCoveragePreview(props));
            act(() => result.current.setEnabled(true));
            await waitFor(() => expect(result.current.result?.mow_angle_deg).toBe(72.4));
            act(() => result.current.setAngleMode('fixed'));
            expect(props.onCommit).toHaveBeenCalledWith('a', expect.objectContaining({has_mow_angle: true, mow_angle_deg: 72}));
        });

        it('a half-made change is dropped when edit mode ends (Cancel or Save)', () => {
            const props = base();
            const {result, rerender} = renderHook((p: typeof props) => useCoveragePreview(p), {initialProps: props});
            act(() => result.current.setAngleDeg(50));
            expect(result.current.choices.angleDeg).toBe(50);
            rerender({...props, editMode: false});
            expect(result.current.choices.angleMode).toBe('global');
            // ...and it does not come back when the next edit session starts.
            rerender({...props, editMode: true});
            expect(result.current.choices.angleMode).toBe('global');
        });

        it('a half-made change belongs to its area', () => {
            const two = [area({id: 5}, 'a'), area({id: 6}, 'b')];
            const props = base({areas: two});
            const {result} = renderHook(() => useCoveragePreview(props));
            act(() => result.current.setAngleDeg(50));
            act(() => result.current.selectArea('b'));
            expect(result.current.choices.angleMode).toBe('global');
        });

        it('converts a dropped start marker to map coordinates and commits it', () => {
            const onCommit = vi.fn<(id: string, overrides: Required<AreaOverrideFields>) => void>();
            const props = base({onCommit});
            const {result} = renderHook(() => useCoveragePreview(props));
            const [lon, lat] = transpose(0, 0, datum, 20, 10); // x = 10 m east, y = 20 m north
            act(() => result.current.moveStartTo(lon, lat));
            expect(onCommit).toHaveBeenCalledOnce();
            const overrides = onCommit.mock.calls[0][1];
            expect(overrides.has_start_point).toBe(true);
            expect(overrides.start_x).toBeCloseTo(10, 4);
            expect(overrides.start_y).toBeCloseTo(20, 4);
        });

        it('going back to the planner\'s own start clears it', () => {
            const props = base({areas: [area({has_start_point: true, start_x: 3, start_y: 4})]});
            const {result} = renderHook(() => useCoveragePreview(props));
            act(() => result.current.setStart(null));
            expect(props.onCommit).toHaveBeenCalledWith('a', expect.objectContaining({has_start_point: false, start_x: 0, start_y: 0}));
        });

        it('a choice keeps the other values the area already has', () => {
            const props = base({areas: [area({has_mow_angle: true, mow_angle_deg: 30, has_start_point: true, start_x: 3, start_y: 4})]});
            const {result} = renderHook(() => useCoveragePreview(props));
            act(() => result.current.setDirection(1));
            expect(props.onCommit).toHaveBeenCalledWith('a', {
                has_mow_angle: true, mow_angle_deg: 30,
                has_ring_direction: true, ring_direction: 1,
                has_start_point: true, start_x: 3, start_y: 4,
            });
        });
    });

    describe('the planner request', () => {
        it('carries the area\'s own angle, winding and start point', async () => {
            const props = base({
                areas: [area({has_mow_angle: true, mow_angle_deg: 35, has_ring_direction: true, ring_direction: 2, has_start_point: true, start_x: 3, start_y: 4})],
            });
            const {result} = renderHook(() => useCoveragePreview(props));
            act(() => result.current.setEnabled(true));
            await waitFor(() => expect(mocks.callCreate).toHaveBeenCalled());
            const [command, body] = lastCall();
            expect(command).toBe('preview_coverage');
            expect(body).toMatchObject({
                mow_angle_deg: 35, ring_direction: 2, has_start_point: true, start_x: 3, start_y: 4,
            });
        });

        it('uses the robot-wide values for an area that overrides nothing, and no start point', async () => {
            const props = base({globalAngleDeg: 60, globalDirection: 1});
            const {result} = renderHook(() => useCoveragePreview(props));
            act(() => result.current.setEnabled(true));
            await waitFor(() => expect(mocks.callCreate).toHaveBeenCalled());
            expect(lastCall()[1]).toMatchObject({
                mow_angle_deg: 60, ring_direction: 1, has_start_point: false,
            });
        });

        it('asks nothing while the overlay is off', () => {
            renderHook(() => useCoveragePreview(base()));
            expect(mocks.callCreate).not.toHaveBeenCalled();
        });
    });

    describe('changedFromServer', () => {
        it('is false when the edit session holds what the server has', () => {
            const server = [{id: 5, has_ring_direction: true, ring_direction: 1}] as MapArea[];
            const props = base({areas: [area({has_ring_direction: true, ring_direction: 1})], savedAreas: server});
            const {result} = renderHook(() => useCoveragePreview(props));
            expect(result.current.changedFromServer).toBe(false);
        });

        it('is true once the session changes this area\'s lines', () => {
            const server = [{id: 5}] as MapArea[];
            const props = base({areas: [area({has_start_point: true, start_x: 1, start_y: 2})], savedAreas: server});
            const {result} = renderHook(() => useCoveragePreview(props));
            expect(result.current.changedFromServer).toBe(true);
        });

        it('treats an area the server does not know yet as unchanged when it has no lines of its own', () => {
            const props = base({savedAreas: []});
            const {result} = renderHook(() => useCoveragePreview(props));
            expect(result.current.changedFromServer).toBe(false);
        });
    });
});
