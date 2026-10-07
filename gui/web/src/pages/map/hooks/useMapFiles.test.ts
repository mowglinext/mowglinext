import {act, renderHook} from '@testing-library/react';
import {describe, expect, it, vi} from 'vitest';
import {useMapFiles} from './useMapFiles';
import {MowingAreaFeature, NavigationFeature} from '../../../types/map.ts';
import type {Api} from '../../../api/Api.ts';

vi.mock('react-i18next', () => ({useTranslation: () => ({t: (key: string) => key})}));

// mowglinext#637 phase 3: PUT /mowglinext/map clears and re-adds every area
// on ANY save, so map_server can only preserve an area's stable id if the
// GUI actually sends it back. Losing this silently re-indexes the WHOLE map
// on every save, discarding coverage-resume progress even for areas the
// operator never touched.
describe('useMapFiles handleSaveMap id round-trip', () => {
    function area(id: string, mowingOrder: number, areaId: number | undefined) {
        const f = new MowingAreaFeature(id, mowingOrder);
        f.setArea(
            {
                name: 'Area ' + mowingOrder,
                id: areaId,
                area: {points: [{x: 0, y: 0, z: 0}, {x: 1, y: 0, z: 0}, {x: 1, y: 1, z: 0}, {x: 0, y: 1, z: 0}]},
            },
            0, 0, [0, 0, 0],
        );
        return f;
    }

    function navigation(id: string, areaId: number | undefined) {
        const f = new NavigationFeature(id);
        f.setArea(
            {
                name: 'Passage',
                id: areaId,
                area: {points: [{x: 0, y: 0, z: 0}, {x: 1, y: 0, z: 0}, {x: 1, y: 1, z: 0}, {x: 0, y: 1, z: 0}]},
            },
            0, 0, [0, 0, 0],
        );
        return f;
    }

    it('sends each area\'s existing id back, and omits it for a brand-new area', async () => {
        const front = area('area-0-area-0', 1, 501);
        const fresh = area('area-1-area-0', 2, undefined);
        const passage = navigation('navigation-0-area-0', 777);

        const putMowglinext = vi.fn().mockResolvedValue({});
        const guiApi = {mowglinext: {putMowglinext}} as unknown as Api<unknown>;

        const hook = renderHook(() => useMapFiles({
            features: {[front.id]: front, [fresh.id]: fresh, [passage.id]: passage},
            setFeatures: vi.fn(),
            map: undefined,
            setMap: vi.fn(),
            editMap: true,
            setEditMap: vi.fn(),
            setHasUnsavedChanges: vi.fn(),
            offsetX: 0,
            offsetY: 0,
            datum: [0, 0, 0],
            notification: {success: vi.fn(), warning: vi.fn(), error: vi.fn()} as any,
            guiApi,
            dockDirty: false,
            setDockDirty: vi.fn(),
            buildFeaturesFromMap: vi.fn(),
            obstacleOriginals: [],
            restoreObstacleOriginals: vi.fn(),
            corridors: [],
            restoreCorridors: vi.fn(),
        }));

        await act(async () => {
            await hook.result.current.handleSaveMap();
        });

        expect(putMowglinext).toHaveBeenCalledOnce();
        const sentAreas = putMowglinext.mock.calls[0][0].areas as Array<{
            area: {name?: string; id?: number}; is_navigation_area?: boolean;
        }>;
        const workAreas = sentAreas.filter(a => !a.is_navigation_area);
        const navAreas = sentAreas.filter(a => a.is_navigation_area);
        const byName = Object.fromEntries(workAreas.map(a => [a.area.name, a.area]));
        expect(byName['Area 1'].id).toBe(501);
        expect(byName['Area 2'].id).toBeUndefined();
        expect(navAreas).toHaveLength(1);
        expect(navAreas[0].area.id).toBe(777);
    });
});

// An area's own mow angle / perimeter winding ride the same rebuild: clear_map +
// add_area per area. If the save drops them, every map save quietly resets every
// area to the robot-wide settings.
describe('useMapFiles handleSaveMap per-area coverage lines', () => {
    const square = {points: [{x: 0, y: 0, z: 0}, {x: 1, y: 0, z: 0}, {x: 1, y: 1, z: 0}, {x: 0, y: 1, z: 0}]};

    function area(id: string, mowingOrder: number, fields: Record<string, unknown>) {
        const f = new MowingAreaFeature(id, mowingOrder);
        f.setArea({name: 'Area ' + mowingOrder, area: square, ...fields}, 0, 0, [0, 0, 0]);
        return f;
    }

    async function save(features: Record<string, MowingAreaFeature>) {
        const putMowglinext = vi.fn().mockResolvedValue({});
        const hook = renderHook(() => useMapFiles(backupOptions({
            features,
            editMap: true,
            guiApi: {mowglinext: {putMowglinext}} as unknown as Api<unknown>,
        })));
        await act(async () => {
            await hook.result.current.handleSaveMap();
        });
        return putMowglinext.mock.calls[0][0].areas as Array<{area: Record<string, unknown>}>;
    }

    it('sends each area\'s own angle and winding back', async () => {
        const own = area('area-0-area-0', 1, {
            id: 1, has_mow_angle: true, mow_angle_deg: 35, has_ring_direction: true, ring_direction: 2,
        });
        const sent = await save({[own.id]: own});
        expect(sent[0].area).toMatchObject({
            has_mow_angle: true, mow_angle_deg: 35, has_ring_direction: true, ring_direction: 2,
        });
    });

    it('keeps 0 degrees and the planner-default winding as real overrides', async () => {
        const zero = area('area-0-area-0', 1, {
            id: 1, has_mow_angle: true, mow_angle_deg: 0, has_ring_direction: true, ring_direction: 0,
        });
        const sent = await save({[zero.id]: zero});
        expect(sent[0].area).toMatchObject({
            has_mow_angle: true, mow_angle_deg: 0, has_ring_direction: true, ring_direction: 0,
        });
    });

    it('sends each area\'s own start point back, (0, 0) included', async () => {
        const own = area('area-0-area-0', 1, {id: 1, has_start_point: true, start_x: 12.5, start_y: -3});
        const origin = area('area-1-area-0', 2, {id: 2, has_start_point: true, start_x: 0, start_y: 0});
        const plain = area('area-2-area-0', 3, {id: 3});
        const sent = await save({[own.id]: own, [origin.id]: origin, [plain.id]: plain});
        const byName = Object.fromEntries(sent.map((x) => [x.area.name as string, x.area]));
        expect(byName['Area 1']).toMatchObject({has_start_point: true, start_x: 12.5, start_y: -3});
        expect(byName['Area 2']).toMatchObject({has_start_point: true, start_x: 0, start_y: 0});
        expect(byName['Area 3'].has_start_point).toBeFalsy();
    });

    it('keeps an area that follows the robot-wide settings following them', async () => {
        const plain = area('area-0-area-0', 1, {id: 1});
        const sent = await save({[plain.id]: plain});
        expect(sent[0].area.has_mow_angle).toBeFalsy();
        expect(sent[0].area.has_ring_direction).toBeFalsy();
    });

    it('does not treat each area as another\'s: overrides stay on their own area', async () => {
        const a = area('area-0-area-0', 1, {id: 1, has_mow_angle: true, mow_angle_deg: 10});
        const b = area('area-1-area-0', 2, {id: 2});
        const sent = await save({[a.id]: a, [b.id]: b});
        const byName = Object.fromEntries(sent.map((x) => [x.area.name as string, x.area]));
        expect(byName['Area 1'].has_mow_angle).toBe(true);
        expect(byName['Area 2'].has_mow_angle).toBeFalsy();
    });
});

// Everything handleBackupMap does NOT care about. Shared by the two backup
// tests below so the untyped antd `notification` stub is cast once.
type BackupOverrides = Partial<Parameters<typeof useMapFiles>[0]>;
function backupOptions(overrides: BackupOverrides): Parameters<typeof useMapFiles>[0] {
    return {
        features: {},
        setFeatures: vi.fn(),
        map: {working_area: [], navigation_areas: []},
        setMap: vi.fn(),
        editMap: false,
        setEditMap: vi.fn(),
        setHasUnsavedChanges: vi.fn(),
        offsetX: 0,
        offsetY: 0,
        datum: [0, 0, 0],
        notification: {success: vi.fn(), warning: vi.fn(), error: vi.fn()} as unknown as
            Parameters<typeof useMapFiles>[0]['notification'],
        guiApi: {} as unknown as Api<unknown>,
        dockDirty: false,
        setDockDirty: vi.fn(),
        buildFeaturesFromMap: vi.fn(),
        obstacleOriginals: [],
        restoreObstacleOriginals: vi.fn(),
        corridors: [],
        restoreCorridors: vi.fn(),
        ...overrides,
    };
}

// The pre-shrink obstacle outlines live in the GUI config store, not in the Map
// message, so a backup that only stringified `map` silently lost them.
describe('useMapFiles backup carries the pre-shrink obstacle outlines', () => {
    it('writes the records that still match an obstacle into map.json', async () => {
        const ring = (x0: number, y0: number, x1: number, y1: number) =>
            [{x: x0, y: y0}, {x: x1, y: y0}, {x: x1, y: y1}, {x: x0, y: y1}];
        const kept = {shrunk: ring(4.3, 4.3, 5.7, 5.7), original: ring(4, 4, 6, 6)};
        const stale = {shrunk: ring(9, 9, 10, 10), original: ring(8, 8, 11, 11)};
        let blob: Blob | undefined;
        Object.assign(window.URL, {createObjectURL: vi.fn((b: Blob) => { blob = b; return 'blob:x'; }), revokeObjectURL: vi.fn()});
        vi.spyOn(HTMLAnchorElement.prototype, 'click').mockImplementation(() => {});

        const hook = renderHook(() => useMapFiles(backupOptions({
            map: {
                working_area: [{
                    name: 'A', area: {points: []}, is_navigation_area: false,
                    obstacles: [{points: [{x: 4.3, y: 4.3, z: 0}, {x: 5.7, y: 4.3, z: 0}, {x: 5.7, y: 5.7, z: 0}, {x: 4.3, y: 5.7, z: 0}]}],
                }],
                navigation_areas: [],
            },
            obstacleOriginals: [kept, stale],
        })));

        hook.result.current.handleBackupMap();

        const saved = JSON.parse(await blob!.text()) as {obstacle_originals: unknown};
        expect(saved.obstacle_originals).toEqual([kept]);
    });
});

// The LiDAR-ignore lines are map_server state, not part of the Map message, so
// a backup that only stringified `map` silently lost them.
describe('useMapFiles backup carries the ignore lines', () => {
    it('writes them into map.json', async () => {
        const line = {name: 'Hedge', polyline: {points: [{x: 0, y: 0, z: 0}, {x: 2, y: 0, z: 0}]}, width_m: 0.4, id: 9};
        let blob: Blob | undefined;
        const createObjectURL = vi.fn((b: Blob) => { blob = b; return 'blob:x'; });
        Object.assign(window.URL, {createObjectURL, revokeObjectURL: vi.fn()});
        vi.spyOn(HTMLAnchorElement.prototype, 'click').mockImplementation(() => {});

        const hook = renderHook(() => useMapFiles(backupOptions({corridors: [line]})));

        hook.result.current.handleBackupMap();

        const saved = JSON.parse(await blob!.text()) as {
            lidar_ignore_corridors: unknown; working_area: unknown;
        };
        expect(saved.lidar_ignore_corridors).toEqual([
            {name: 'Hedge', polyline: line.polyline, width_m: 0.4},
        ]);
        expect(saved.working_area).toEqual([]);
    });
});
