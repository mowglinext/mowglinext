import {act, renderHook} from '@testing-library/react';
import {beforeEach, describe, expect, it, vi} from 'vitest';
import {useMapEditing, findContainingArea, type UseMapEditingOptions} from './useMapEditing';
import {MowingAreaFeature, ObstacleFeature} from '../../../types/map.ts';

interface DeleteDialog {
    title: string;
    content: string;
    onOk: () => void;
}
const {confirm} = vi.hoisted(() => ({confirm: vi.fn<(dialog: DeleteDialog) => void>()}));
vi.mock('antd', () => ({App: {useApp: () => ({modal: {confirm}})}}));
vi.mock('react-i18next', () => ({useTranslation: () => ({t: (key: string) => key})}));

describe('map delete confirmation', () => {
    beforeEach(() => confirm.mockClear());

    function setup(mode: string, points: number, selected = ['area-0']) {
        const draw = {
            getMode: () => mode,
            getSelectedPoints: () => ({features: Array.from({length: points})}),
            getSelectedIds: () => selected,
            trash: vi.fn(),
        };
        const options = {
            features: {}, setFeatures: vi.fn(), editMap: true, mowingAreas: [],
            drawRef: {current: draw}, notification: {}, mapInstanceRef: {current: null},
        } as unknown as UseMapEditingOptions;
        const hook = renderHook(() => useMapEditing(options));
        act(() => hook.result.current.handleTrash());
        return draw;
    }

    it.each([1, 2])('describes deleting %i selected vertices and waits for confirmation', points => {
        const draw = setup('direct_select', points);
        const dialog = confirm.mock.calls[0][0];
        expect(dialog.title).toBe('mapEditing.deletePointsConfirmTitle');
        expect(dialog.content).toBe('mapEditing.deletePointsConfirmBody');
        expect(draw.trash).not.toHaveBeenCalled();
        act(() => dialog.onOk());
        expect(draw.trash).toHaveBeenCalledOnce();
    });

    it('keeps the area warning for a complete feature selection', () => {
        setup('simple_select', 0);
        expect(confirm.mock.calls[0][0].content).toBe('mapEditing.deleteAreaConfirmBody');
    });

    it('does not offer deletion with no selected feature', () => {
        const draw = setup('simple_select', 0, []);
        expect(confirm).not.toHaveBeenCalled();
        expect(draw.trash).not.toHaveBeenCalled();
    });

    it('describes direct-select deletion even before the selected-point cache updates', () => {
        // The custom midpoint handler selects the new vertex internally before
        // updating Draw's public getSelectedPoints cache.
        const draw = setup('direct_select', 0);
        const dialog = confirm.mock.calls[0][0];
        expect(dialog.title).toBe('mapEditing.deletePointsConfirmTitle');
        act(() => dialog.onOk());
        expect(draw.trash).toHaveBeenCalledOnce();
    });
});

function square(x0: number, y0: number, x1: number, y1: number) {
    return {
        type: 'Polygon' as const,
        coordinates: [[[x0, y0], [x1, y0], [x1, y1], [x0, y1], [x0, y0]]],
    };
}

function areaFeature(id: string, geom: ReturnType<typeof square>) {
    const f = new MowingAreaFeature(id, 1);
    f.setGeometry(geom);
    return f;
}

// mowglinext: obstacle-to-parent-area mismatch after splitting a workarea.
// findContainingArea (used by addObstacle/onCombine/performSplit/
// updateMowingArea) must not silently take the first area whose polygon
// contains a point when more than one candidate does -- two areas that touch
// or slightly overlap at a shared edge (exactly what splitting one area in
// two produces) can both legitimately contain the same obstacle centroid.
describe('findContainingArea', () => {
    it('returns undefined when nothing contains the ring', () => {
        const areas = [areaFeature('a', square(0, 0, 5, 5))];
        expect(findContainingArea([[20, 20], [21, 20], [21, 21], [20, 21]], areas)).toBeUndefined();
    });

    it('picks the single containing area', () => {
        const a = areaFeature('a', square(0, 0, 10, 10));
        const b = areaFeature('b', square(20, 20, 30, 30));
        const ring = [[4, 4], [5, 4], [5, 5], [4, 5]];
        expect(findContainingArea(ring, [a, b])).toBe(a);
    });

    it('prefers the SMALLEST containing area when several overlap the point', () => {
        // A big area and a small one nested inside it both contain the ring --
        // the small, more specific one must win, not whichever came first.
        const big = areaFeature('big', square(0, 0, 100, 100));
        const small = areaFeature('small', square(4, 4, 6, 6));
        const ring = [[4.5, 4.5], [5.5, 4.5], [5.5, 5.5], [4.5, 5.5]];
        expect(findContainingArea(ring, [big, small])).toBe(small);
        expect(findContainingArea(ring, [small, big])).toBe(small); // order-independent
    });
});

describe('performSplit re-parents obstacles by where they actually end up', () => {
    function setup(areaGeom: ReturnType<typeof square>, obstacleGeom: ReturnType<typeof square>) {
        const parent = areaFeature('area-0-area-0', areaGeom);
        const obstacle = new ObstacleFeature('area-0-obstacle-0', parent);
        obstacle.setGeometry(obstacleGeom);

        const draw = {
            changeMode: vi.fn(),
            get: vi.fn(() => undefined),
            delete: vi.fn(),
            add: vi.fn(),
        };
        const options = {
            features: {[parent.id]: parent, [obstacle.id]: obstacle},
            setFeatures: vi.fn(),
            editMap: true,
            mowingAreas: [],
            drawRef: {current: draw},
            notification: {error: vi.fn(), info: vi.fn(), success: vi.fn()},
            mapInstanceRef: {current: null},
        } as unknown as UseMapEditingOptions;
        const hook = renderHook(() => useMapEditing(options));
        // selectedFeatureIds is internal hook state, not an options prop --
        // handleSplit reads it via useState, so it must be set through the
        // hook's own setter before handleSplit() will accept the target.
        act(() => {
            hook.result.current.setSelectedFeatureIds([parent.id]);
        });
        return {hook, parent, obstacle, draw, setFeatures: options.setFeatures as ReturnType<typeof vi.fn>};
    }

    it("reassigns an obstacle to the new half when the cut leaves it out of the original feature's half", () => {
        // 10x10 square cut at x=5: the segment-hit trace lands the RIGHT half
        // (x in [5,10]) on the ORIGINAL feature reference and the LEFT half
        // (x in [0,5]) on the newly-created feature -- see the comment in
        // useMapEditing.ts's performSplit for why the split always keeps the
        // original object identity for one specific side.
        const {hook, parent, obstacle, setFeatures} = setup(
            square(0, 0, 10, 10),
            square(1, 4, 2, 5), // obstacle sits in the LEFT half (x<5)
        );

        act(() => {
            hook.result.current.handleSplit();
        });
        act(() => {
            hook.result.current.onCreate({
                features: [{
                    type: 'Feature', properties: {},
                    geometry: {type: 'LineString', coordinates: [[5, -1], [5, 11]]},
                }],
            });
        });

        expect(setFeatures).toHaveBeenCalled();
        const updater = setFeatures.mock.calls[setFeatures.mock.calls.length - 1][0] as (
            curr: Record<string, unknown>
        ) => Record<string, unknown>;
        const next = updater({[parent.id]: parent, [obstacle.id]: obstacle});

        const newAreaEntry = Object.values(next).find(
            (f) => f instanceof MowingAreaFeature && f !== parent
        );
        expect(newAreaEntry).toBeDefined();
        // The obstacle was originally parented to `parent` (now the RIGHT
        // half) but geometrically sits in the LEFT half -- it must now point
        // at the NEW feature, not silently stay on the old reference.
        expect(obstacle.getMowingArea()).toBe(newAreaEntry);
        expect(obstacle.getMowingArea()).not.toBe(parent);
    });

    it('keeps an obstacle on the original feature when it genuinely stays in that half', () => {
        const {hook, parent, obstacle, setFeatures} = setup(
            square(0, 0, 10, 10),
            square(7, 4, 8, 5), // obstacle sits in the RIGHT half (x>5) -- stays with `parent`
        );

        act(() => {
            hook.result.current.handleSplit();
        });
        act(() => {
            hook.result.current.onCreate({
                features: [{
                    type: 'Feature', properties: {},
                    geometry: {type: 'LineString', coordinates: [[5, -1], [5, 11]]},
                }],
            });
        });

        const updater = setFeatures.mock.calls[setFeatures.mock.calls.length - 1][0] as (
            curr: Record<string, unknown>
        ) => Record<string, unknown>;
        updater({[parent.id]: parent, [obstacle.id]: obstacle});

        expect(obstacle.getMowingArea()).toBe(parent);
    });
});
