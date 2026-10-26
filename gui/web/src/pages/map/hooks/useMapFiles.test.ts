import {act, fireEvent, renderHook, waitFor} from '@testing-library/react';
import {afterEach, describe, expect, it, vi} from 'vitest';
import {useMapFiles, decodeEditableGeoJSON, decodeImportOpenMowerSummary, decodeImportOutcome, type ImportOpenMowerSummary} from './useMapFiles';
import {MowingAreaFeature, NavigationFeature, DockFeatureBase, ObstacleFeature, featuresFromJSON, serializeFeatures} from '../../../types/map.ts';
import {transpose} from '../../../utils/map.tsx';
import {Api} from '../../../api/Api.ts';
import {parseMapBackup} from '../utils/mapBackup.ts';

// Exercise the real generated API contract against an in-memory fetch only.
function mockApi() {
    return new Api<unknown>({customFetch: vi.fn<typeof fetch>().mockImplementation(() => Promise.resolve(new Response('{}')))});
}
function notifications(): Parameters<typeof useMapFiles>[0]['notification'] {
    return {success: vi.fn(), warning: vi.fn(), error: vi.fn(), info: vi.fn(), open: vi.fn(), destroy: vi.fn()};
}

afterEach(() => {vi.restoreAllMocks(); vi.unstubAllGlobals();});

const geometryRing = (x: number, y: number, size: number) =>
    [[x, y], [x + size, y], [x + size, y + size], [x, y + size], [x, y]]
        .map(([east, north]) => transpose(0, 0, [0, 0, 0], north, east));

it('rejects two GeoJSON holes on import and refuses an older rehydrated draft without losing either ring', async () => {
    const rings = [geometryRing(0, 0, 20), geometryRing(2, 2, 2), geometryRing(10, 10, 3)];
    const source = {type: 'FeatureCollection', features: [{type: 'Feature', id: 'garden-A',
        properties: {feature_type: 'workarea', name: 'Garden A', mowing_order: 1},
        geometry: {type: 'Polygon', coordinates: rings}}]};
    expect(() => decodeEditableGeoJSON(source, geoMessages)).toThrow('unsupported');
    const draft = new MowingAreaFeature('garden-A', 1);
    draft.setName('Garden A');
    draft.setGeometry({type: 'Polygon', coordinates: rings});
    const features = featuresFromJSON(serializeFeatures({[draft.id]: draft}));
    expect(features['garden-A'].geometry).toEqual(source.features[0].geometry);
    expect(serializeFeatures(features)['garden-A'].geometry).toEqual(source.features[0].geometry);
    const options = backupOptions({features, editMap: true, dockDirty: true});
    const put = vi.spyOn(options.guiApi.mowglinext, 'putMowglinext');
    const hook = renderHook(() => useMapFiles(options));
    await hook.result.current.handleSaveMap();
    expect(put).not.toHaveBeenCalled();
    expect(features['garden-A'].geometry.coordinates).toEqual(rings);
    expect(options.notification.error).toHaveBeenCalledExactlyOnceWith({message: 'mapFiles.failedToSaveArea', description: 'mapFiles.unsupportedGeometry mapFiles.affectedPolygons'});
    expect(options.setEditMap).not.toHaveBeenCalled(); expect(options.setHasUnsavedChanges).not.toHaveBeenCalled();
    expect(options.setDockDirty).not.toHaveBeenCalled(); expect(options.notification.success).not.toHaveBeenCalled();
});

describe('geometry integrity before any map/dock write', () => {
    it.each([
        {kind: 'workarea', holes: 1}, {kind: 'workarea', holes: 2},
        {kind: 'navigation', holes: 1}, {kind: 'obstacle', holes: 1},
    ])('rejects import of $kind with $holes holes, retaining the existing draft', async ({kind, holes}) => {
        const outer = new MowingAreaFeature('existing', 1); outer.setGeometry({type: 'Polygon', coordinates: [geometryRing(0, 0, 20)]});
        const before = serializeFeatures({existing: outer});
        const options = backupOptions({features: {existing: outer}, editMap: true, dockDirty: true});
        const put = vi.spyOn(options.guiApi.mowglinext, 'putMowglinext'), dock = vi.spyOn(options.guiApi.mowglinext, 'mapDockingCreate');
        const hook = renderHook(() => useMapFiles(options));
        const geometry = {type: 'Polygon', coordinates: [geometryRing(0, 0, 20), ...Array.from({length: holes}, (_, index) => geometryRing(2 + 5 * index, 2, 2))]};
        selectedFile(hook.result.current.handleUploadGeoJSON, JSON.stringify({type: 'FeatureCollection', features: [{type: 'Feature', id: 'incoming', geometry, properties: {feature_type: kind}}]}));
        await waitFor(() => expect(options.notification.error).toHaveBeenCalledExactlyOnceWith({message: 'mapFiles.unsupportedGeometry'}));
        expect(put).not.toHaveBeenCalled(); expect(dock).not.toHaveBeenCalled();
        for (const setter of [options.setFeatures, options.setEditMap, options.setHasUnsavedChanges, options.setDockDirty]) expect(setter).not.toHaveBeenCalled();
        expect(serializeFeatures(options.features)).toEqual(before); expect(options.notification.success).not.toHaveBeenCalled();
    });
    it.each([
        {type: 'Polygon', coordinates: []}, {type: 'Polygon', coordinates: [[]]},
        {type: 'Polygon', coordinates: [[[0, 0], [1, 1], [0, 1], [1, 0], [0, 0]]]},
        {type: 'Polygon', coordinates: [[[0, 0], [NaN, 0], [0, 1]]]},
        {type: 'Polygon', coordinates: [[[0, 0], [Infinity, 0], [0, 1]]]},
        {type: 'Polygon', coordinates: [[[0, 0], ['1', 0], [0, 1]]]},
        {type: 'MultiPolygon', coordinates: [[geometryRing(0, 0, 20)]]},
    ])('refuses invalid or unsupported geometry in a draft/history entry before saving: %j', raw => {
        const feature = new MowingAreaFeature('draft', 1);
        Reflect.set(feature, 'geometry', raw); // Models a malformed JS/history boundary without a cast.
        const valid = new MowingAreaFeature('valid-first', 2); valid.setGeometry({type: 'Polygon', coordinates: [geometryRing(0, 0, 20)]});
        const dock = new DockFeatureBase([0, 0], 0);
        const options = backupOptions({features: {valid, draft: feature, dock}, editMap: true, dockDirty: true});
        const put = vi.spyOn(options.guiApi.mowglinext, 'putMowglinext'), dockWrite = vi.spyOn(options.guiApi.mowglinext, 'mapDockingCreate');
        const hook = renderHook(() => useMapFiles(options));
        return hook.result.current.handleSaveMap().then(() => {
            expect(put).not.toHaveBeenCalled(); expect(dockWrite).not.toHaveBeenCalled();
            expect(options.notification.error).toHaveBeenCalledOnce(); expect(options.notification.success).not.toHaveBeenCalled();
            expect(options.setEditMap).not.toHaveBeenCalled(); expect(options.setDockDirty).not.toHaveBeenCalled();
            expect(options.setHasUnsavedChanges).not.toHaveBeenCalled(); expect(feature.geometry).toBe(raw);
        });
    });
    it('exports every ring from an older unsupported draft unchanged', async () => {
        const area = new MowingAreaFeature('recoverable', 1); area.setName('Recoverable garden');
        const rings = [geometryRing(0, 0, 20), geometryRing(2, 2, 2), geometryRing(10, 10, 2)];
        area.setGeometry({type: 'Polygon', coordinates: rings});
        let blob: Blob | undefined;
        Object.assign(window.URL, {createObjectURL: vi.fn((value: Blob) => {blob = value; return 'blob:recovery';}), revokeObjectURL: vi.fn()});
        vi.spyOn(HTMLAnchorElement.prototype, 'click').mockImplementation(() => {});
        const options = backupOptions({features: {recoverable: area}, editMap: true});
        renderHook(() => useMapFiles(options)).result.current.handleDownloadGeoJSON();
        if (!blob) throw new Error('No export');
        const actual: unknown = JSON.parse(await blob.text());
        expect(actual).toMatchObject({type: 'FeatureCollection', features: [{id: 'recoverable', properties: {name: 'Recoverable garden'}, geometry: {type: 'Polygon', coordinates: rings}}]});
        expect(options.setHasUnsavedChanges).not.toHaveBeenCalled();
    });
    it('keeps two explicit obstacles separate across import, save and ROS-style rehydration', async () => {
        const outer = geometryRing(0, 0, 20), holeA = geometryRing(2, 2, 2), holeB = geometryRing(10, 10, 2);
        const source = {type: 'FeatureCollection', features: [
            {type: 'Feature', id: 'garden', properties: {feature_type: 'workarea', name: 'Garden', mowing_order: 0}, geometry: {type: 'Polygon', coordinates: [outer]}},
            ...[holeA, holeB].map((ring, i) => ({type: 'Feature', id: `exclusion-${i}`, properties: {feature_type: 'obstacle', mowing_area: 'garden'}, geometry: {type: 'Polygon', coordinates: [ring]}})),
        ]};
        const features = featuresFromJSON(decodeEditableGeoJSON(source, geoMessages));
        const options = backupOptions({features, editMap: true}); const put = vi.spyOn(options.guiApi.mowglinext, 'putMowglinext');
        await renderHook(() => useMapFiles(options)).result.current.handleSaveMap();
        expect(put).toHaveBeenCalledOnce();
        const saved = put.mock.calls[0][0].areas?.[0].area;
        if (!saved) throw new Error('No saved area');
        expect(saved.name).toBe('Garden'); expect(saved.obstacles).toHaveLength(2);
        const readback = new MowingAreaFeature('garden-readback', 1); readback.setArea(saved, 0, 0, [0, 0, 0]);
        expect(readback.geometry.coordinates[0]).toHaveLength(outer.length);
        for (const [i, polygon] of (saved.obstacles ?? []).entries()) {
            const obstacle = new ObstacleFeature(`readback-${i}`, readback);
            obstacle.transpose(polygon.points ?? [], 0, 0, [0, 0, 0]);
            for (const [j, position] of obstacle.geometry.coordinates[0].entries()) {
                expect(position[0]).toBeCloseTo([holeA, holeB][i][j][0], 12);
                expect(position[1]).toBeCloseTo([holeA, holeB][i][j][1], 12);
            }
            expect(obstacle.getMowingArea()).toBe(readback);
        }
        expect(options.notification.success).toHaveBeenCalledOnce();
    });
    it('refuses an unparented explicit exclusion instead of saving a map without it', async () => {
        const missing = new MowingAreaFeature('missing', 1);
        const obstacle = new ObstacleFeature('exclusion', missing); obstacle.setGeometry({type: 'Polygon', coordinates: [geometryRing(0, 0, 2)]});
        const options = backupOptions({features: {exclusion: obstacle}, editMap: true});
        const put = vi.spyOn(options.guiApi.mowglinext, 'putMowglinext');
        await renderHook(() => useMapFiles(options)).result.current.handleSaveMap();
        expect(put).not.toHaveBeenCalled(); expect(options.notification.error).toHaveBeenCalledWith({message: 'mapFiles.failedToSaveArea', description: 'mapFiles.obstacleParentMissing'});
        expect(options.setHasUnsavedChanges).not.toHaveBeenCalled(); expect(options.notification.success).not.toHaveBeenCalled();
    });
});

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

        const guiApi = mockApi();
        const putMowglinext = vi.spyOn(guiApi.mowglinext, 'putMowglinext');

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
            notification: notifications(),
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
        const sentAreas = putMowglinext.mock.calls[0][0].areas ?? [];
        const workAreas = sentAreas.filter(a => !a.is_navigation_area);
        const navAreas = sentAreas.filter(a => a.is_navigation_area);
        const byName = new Map(workAreas.map(a => [a.area?.name, a.area]));
        expect(byName.get('Area 1')?.id).toBe(501);
        expect(byName.get('Area 2')?.id).toBeUndefined();
        expect(navAreas).toHaveLength(1);
        expect(navAreas[0].area?.id).toBe(777);
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
        const guiApi = mockApi();
        const putMowglinext = vi.spyOn(guiApi.mowglinext, 'putMowglinext');
        const hook = renderHook(() => useMapFiles(backupOptions({
            features,
            editMap: true,
            guiApi,
        })));
        await act(async () => {
            await hook.result.current.handleSaveMap();
        });
        return (putMowglinext.mock.calls[0][0].areas ?? []).map(entry => ({area: entry.area ?? {}}));
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
        const byName = new Map(sent.map(x => [x.area.name, x.area]));
        expect(byName.get('Area 1')).toMatchObject({has_start_point: true, start_x: 12.5, start_y: -3});
        expect(byName.get('Area 2')).toMatchObject({has_start_point: true, start_x: 0, start_y: 0});
        expect(byName.get('Area 3')?.has_start_point).toBeFalsy();
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
        const byName = new Map(sent.map(x => [x.area.name, x.area]));
        expect(byName.get('Area 1')?.has_mow_angle).toBe(true);
        expect(byName.get('Area 2')?.has_mow_angle).toBeFalsy();
    });
});

// Everything handleBackupMap does NOT care about. Shared by the two backup
// tests below with a complete typed notification stub and an in-memory API.
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
        notification: notifications(),
        guiApi: mockApi(),
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

// An area's own mowing lines are part of the Map message, so "Backup map" carries them, and
// "Restore map" followed by "Save map" must put them back on the same area.
describe('useMapFiles backup and restore keep the mowing lines', () => {
    const lines = {
        has_mow_angle: true, mow_angle_deg: 35,
        has_ring_direction: true, ring_direction: 2,
        has_start_point: true, start_x: 9.5, start_y: 4,
    };
    const ring = {points: [{x: 0, y: 0, z: 0}, {x: 10, y: 0, z: 0}, {x: 10, y: 10, z: 0}, {x: 0, y: 10, z: 0}]};

    it('writes them into map.json', async () => {
        let blob: Blob | undefined;
        Object.assign(window.URL, {createObjectURL: vi.fn((b: Blob) => { blob = b; return 'blob:x'; }), revokeObjectURL: vi.fn()});
        vi.spyOn(HTMLAnchorElement.prototype, 'click').mockImplementation(() => {});

        const hook = renderHook(() => useMapFiles(backupOptions({
            map: {working_area: [{name: 'Voor', id: 7, area: ring, obstacles: [], is_navigation_area: false, ...lines}], navigation_areas: []},
        })));
        hook.result.current.handleBackupMap();

        const saved = JSON.parse(await blob!.text()) as {working_area: Array<Record<string, unknown>>};
        expect(saved.working_area[0]).toMatchObject({id: 7, ...lines});
    });

    it('a restored backup saved again keeps them on the same area', async () => {
        // Backup text -> parse (what Restore map does) -> the area the editor holds -> the save payload.
        const parsed = parseMapBackup(JSON.stringify({
            working_area: [{name: 'Voor', id: 7, area: ring, obstacles: [], ...lines}],
            navigation_areas: [],
        }));
        expect(parsed.ok).toBe(true);
        if (!parsed.ok) return;
        const restored = new MowingAreaFeature('area-0-area-0', 1);
        restored.setArea(parsed.map.working_area![0], 0, 0, [0, 0, 0]);

        const guiApi = mockApi();
        const putMowglinext = vi.spyOn(guiApi.mowglinext, 'putMowglinext');
        const hook = renderHook(() => useMapFiles(backupOptions({
            features: {[restored.id]: restored},
            editMap: true,
            guiApi,
        })));
        await act(async () => {
            await hook.result.current.handleSaveMap();
        });

        const sent = (putMowglinext.mock.calls[0][0].areas ?? []).map(entry => ({area: entry.area ?? {}}));
        expect(sent[0].area).toMatchObject({id: 7, ...lines});
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


describe('map file reader failures', () => {
    it('reports a failed backup read without changing the editor and removes the input', async () => {
        const options = backupOptions({});
        const hook = renderHook(() => useMapFiles(options));
        const log = vi.spyOn(console, 'error').mockImplementation(() => undefined);
        const click = vi.spyOn(HTMLInputElement.prototype, 'click').mockImplementation(() => undefined);
        hook.result.current.handleRestoreMap();
        const input = document.querySelector<HTMLInputElement>('input[type="file"]');
        expect(input).not.toBeNull();
        const file = new File(['unreadable'], 'map.json');
        Object.defineProperty(file, 'text', {value: () => Promise.reject(new Error('Cannot read backup'))});
        if (!input) throw new Error('Missing file input');
        fireEvent.change(input, {target: {files: [file]}});
        await waitFor(() => expect(options.notification.error).toHaveBeenCalledWith({
            message: 'mapFiles.restoreFailed', description: 'Cannot read backup',
        }));
        expect(options.setFeatures).not.toHaveBeenCalled();
        expect(options.setEditMap).not.toHaveBeenCalled();
        expect(options.notification.success).not.toHaveBeenCalled();
        expect(input).not.toBeInTheDocument();
        log.mockRestore();
        click.mockRestore();
    });

    it('removes the temporary input when file selection is cancelled', () => {
        const hook = renderHook(() => useMapFiles(backupOptions({})));
        const click = vi.spyOn(HTMLInputElement.prototype, 'click').mockImplementation(() => undefined);
        hook.result.current.handleImportOpenMower(vi.fn(), vi.fn());
        const input = document.querySelector<HTMLInputElement>('input[type="file"]');
        if (!input) throw new Error('Missing file input');
        fireEvent(input, new Event('cancel'));
        expect(input).not.toBeInTheDocument();
        click.mockRestore();
    });
});


const previewSummary = (): ImportOpenMowerSummary => ({mowing_areas: 1, navigation_areas: 0, obstacles: 0, orphan_obstacles: 0,
    datum_shift_east_m: 0, datum_shift_north_m: 0, applied: false, mn_datum_configured: false,
    warnings: [], dock_pose: {x: 0, y: 0, yaw_rad: 0},
    areas: [{name: 'Ängen', type: 'mow', vertices: 4, obstacles: 0, is_navigation_area: false, approx_area_sqm: 1}]});
const geoMessages = {invalid: 'invalid', empty: 'empty', unknownType: (type: string) => `unknown ${type}`};
const polygon = () => ({type: 'Feature', id: 0, properties: {feature_type: 'workarea', name: '', mowing_order: 0, extra: null},
    geometry: {type: 'Polygon', coordinates: [[[0, 0], [1, 0], [1, 1], [0, 0]]]}});
function selectedFile(action: () => void, text: string | Error, name = 'map.json') {
    vi.spyOn(HTMLInputElement.prototype, 'click').mockImplementation(() => {});
    action();
    const input = document.querySelector<HTMLInputElement>('input[type="file"]');
    if (!input) throw new Error('Missing file picker');
    const file = new File([], name);
    Object.defineProperty(file, 'text', {value: () => text instanceof Error ? Promise.reject(text) : Promise.resolve(text)});
    fireEvent.change(input, {target: {files: [file]}});
    return input;
}

describe('OpenMower response boundary', () => {
    it('accepts exact Go summary fields, origin dock, absent legacy metadata and null dock', () => {
        const raw = previewSummary(); expect(decodeImportOpenMowerSummary(raw, false)).toBe(raw);
        const {mn_datum_configured, ...legacy} = raw;
        expect(mn_datum_configured).toBe(false);
        expect(decodeImportOpenMowerSummary({...legacy, dock_pose: null}, false).dock_pose).toBeNull();
        const {dock_pose, ...noDock} = legacy;
        expect(dock_pose).toBeDefined(); expect(decodeImportOpenMowerSummary(noDock, false).dock_pose).toBeUndefined();
    });
    it.each([null, {}, {warnings: null}, {areas: null}, {applied: true}, {mowing_areas: '1'}, {navigation_areas: -1},
        {datum_shift_east_m: Infinity}, {dock_pose: {x: 0, y: null, yaw_rad: 0}}, {mn_datum_configured: 'false'},
        {areas: [{...previewSummary().areas[0], approx_area_sqm: '1'}]}, {warnings: [42]}])('rejects malformed/contradictory preview %j', patch => {
        const raw = patch === null ? null : {...previewSummary(), ...patch};
        // An empty object alone is also not a producer summary.
        if (patch && Object.keys(patch).length === 0) expect(() => decodeImportOpenMowerSummary(patch, false)).toThrow();
        else expect(() => decodeImportOpenMowerSummary(raw, false)).toThrow();
    });
    it.each(['{"areas":[],"docking_stations":[]}', '{"WorkingArea":[],"NavigationAreas":[],"DockX":0,"DockY":0}'])('previews existing raw formats verbatim without touching editor state', async text => {
        const fetcher = vi.fn<typeof fetch>().mockResolvedValue(new Response(JSON.stringify(previewSummary())));
        vi.stubGlobal('fetch', fetcher);
        const options = backupOptions({}); const hook = renderHook(() => useMapFiles(options));
        const preview = vi.fn(), stash = vi.fn();
        const input = selectedFile(() => hook.result.current.handleImportOpenMower(preview, stash), text);
        await waitFor(() => expect(preview).toHaveBeenCalledWith(previewSummary()));
        expect(stash).toHaveBeenCalledExactlyOnceWith(text);
        expect(fetcher).toHaveBeenCalledExactlyOnceWith('/api/import/openmower', expect.objectContaining({body: text}));
        expect(options.setFeatures).not.toHaveBeenCalled(); expect(options.setEditMap).not.toHaveBeenCalled();
        expect(input).not.toBeInTheDocument();
    });
    it.each(['{', new Error('Cannot read import'), '{"map":{"areas":[]},"apply":true}', '{"Map":{"areas":[]},"Apply":true}'])('rejects unreadable files and write-bearing API envelopes before any request', async text => {
        const fetcher = vi.fn<typeof fetch>(); vi.stubGlobal('fetch', fetcher);
        const options = backupOptions({}); const hook = renderHook(() => useMapFiles(options)); const preview = vi.fn(), stash = vi.fn();
        selectedFile(() => hook.result.current.handleImportOpenMower(preview, stash), text);
        await waitFor(() => expect(options.notification.error).toHaveBeenCalled());
        expect(fetcher).not.toHaveBeenCalled(); expect(preview).not.toHaveBeenCalled(); expect(stash).not.toHaveBeenCalled();
        expect(options.setFeatures).not.toHaveBeenCalled();
    });
    it('keeps previous preview/file and map after malformed response, then permits explicit retry', async () => {
        const fetcher = vi.fn<typeof fetch>().mockResolvedValueOnce(new Response('{}')).mockResolvedValueOnce(new Response(JSON.stringify(previewSummary())));
        vi.stubGlobal('fetch', fetcher); const options = backupOptions({}); const hook = renderHook(() => useMapFiles(options));
        const preview = vi.fn(), stash = vi.fn(); const action = () => hook.result.current.handleImportOpenMower(preview, stash);
        selectedFile(action, '{"areas":[]}'); await waitFor(() => expect(options.notification.error).toHaveBeenCalled());
        expect(preview).not.toHaveBeenCalled(); expect(stash).not.toHaveBeenCalled(); expect(options.setFeatures).not.toHaveBeenCalled();
        selectedFile(action, '{"areas":[]}'); await waitFor(() => expect(preview).toHaveBeenCalledOnce());
        expect(fetcher).toHaveBeenCalledTimes(2);
    });
    it('does not clear edits or write datum after a partial backend dock failure', async () => {
        const fetcher = vi.fn<typeof fetch>().mockResolvedValue(new Response(JSON.stringify({error: 'set docking point: ROS rejected'}), {status: 500}));
        vi.stubGlobal('fetch', fetcher); const options = backupOptions({}); const hook = renderHook(() => useMapFiles(options));
        await expect(hook.result.current.handleApplyOpenMowerImport('{"areas":[]}', 0, 0, true)).rejects.toThrow('mapImportOpenMower.dockUnconfirmed');
        expect(fetcher).toHaveBeenCalledOnce(); expect(options.setFeatures).not.toHaveBeenCalled();
        expect(options.setEditMap).not.toHaveBeenCalled(); expect(options.setHasUnsavedChanges).not.toHaveBeenCalled();
        expect(options.notification.success).not.toHaveBeenCalled();
    });
    it('requires an applied acknowledgment before clearing drafts, including a possibly successful malformed response', async () => {
        const fetcher = vi.fn<typeof fetch>().mockResolvedValueOnce(new Response('{}')).mockResolvedValueOnce(new Response(JSON.stringify({
            ...previewSummary(), applied: true, map_status: 'persisted', dock_status: 'confirmed',
        })));
        vi.stubGlobal('fetch', fetcher); const options = backupOptions({}); const hook = renderHook(() => useMapFiles(options));
        await expect(hook.result.current.handleApplyOpenMowerImport('{"areas":[]}')).rejects.toThrow('mapImportOpenMower.outcomeUnknown');
        expect(options.setEditMap).not.toHaveBeenCalled(); expect(options.setDockDirty).not.toHaveBeenCalled(); expect(fetcher).toHaveBeenCalledOnce();
        await hook.result.current.handleApplyOpenMowerImport('{"areas":[]}');
        expect(options.setEditMap).toHaveBeenCalledWith(false); expect(options.setDockDirty).toHaveBeenCalledWith(false);
        expect(options.notification.success).toHaveBeenCalledOnce();
    });
    it('validates reprojection response without changing projection inputs or applying a map', async () => {
        const fetcher = vi.fn<typeof fetch>().mockResolvedValueOnce(new Response('{}')).mockResolvedValueOnce(new Response(JSON.stringify(previewSummary())));
        vi.stubGlobal('fetch', fetcher); const hook = renderHook(() => useMapFiles(backupOptions({})));
        await expect(hook.result.current.handleReprojectOpenMowerPreview('{"areas":[]}', 0, 0)).rejects.toThrow('Invalid OpenMower');
        await expect(hook.result.current.handleReprojectOpenMowerPreview('{"areas":[]}', 0, 0)).resolves.toEqual(previewSummary());
        expect(fetcher).toHaveBeenLastCalledWith('/api/import/openmower', expect.objectContaining({body: '{"map":{"areas":[]},"om_datum_lat":0,"om_datum_lon":0}'}));
    });
});

describe('structured OpenMower import outcomes', () => {
    it.each([
        ['unchanged', 'not_attempted'], ['restored', 'not_requested'],
        ['unconfirmed', 'not_attempted'], ['active_persistence_unconfirmed', 'not_attempted'],
        ['persisted', 'rejected'], ['persisted', 'unconfirmed'],
        ['persisted', 'not_attempted'], ['persisted', 'active_persistence_unconfirmed'],
        ['persisted', 'confirmed'], ['persisted', 'not_requested'],
    ])('decodes the explicit pair %s / %s without inferring values', (map_status, dock_status) => {
        expect(decodeImportOutcome({map_status, dock_status})).toEqual({map_status, dock_status});
    });
    it.each([
        {map_status: 'persisted'}, {dock_status: 'rejected'},
        {map_status: 'future', dock_status: 'confirmed'}, {map_status: null, dock_status: 'not_attempted'},
        {map_status: 'unchanged', dock_status: 'confirmed'}, {map_status: 'restored', dock_status: 'unconfirmed'},
        {map_status: 'persisted', dock_status: 'future'},
    ])('rejects malformed or contradictory outcome %j', raw => {
        expect(() => decodeImportOutcome(raw)).toThrow('Invalid OpenMower import outcome');
    });
    it('keeps absent legacy outcome distinct from confirmed success', () => {
        expect(decodeImportOutcome(previewSummary())).toBeUndefined();
        expect(decodeImportOpenMowerSummary(previewSummary(), false)).toEqual(previewSummary());
    });
    it.each([
        {map_status: 'persisted', dock_status: 'rejected'},
        {map_status: 'active_persistence_unconfirmed', dock_status: 'not_attempted'},
        {map_status: 'persisted', dock_status: 'not_requested'},
    ])('rejects an applied success inconsistent with its statuses or dock preview: %j', patch => {
        expect(() => decodeImportOpenMowerSummary({...previewSummary(), applied: true, ...patch}, true)).toThrow();
    });
    it.each([
        ['unchanged', 'not_attempted', 'notStarted'],
        ['restored', 'not_attempted', 'mapRestored'],
        ['unconfirmed', 'not_attempted', 'outcomeUnknown'],
        ['active_persistence_unconfirmed', 'not_attempted', 'mapPersistenceUnconfirmed'],
        ['persisted', 'rejected', 'dockRejected'],
        ['persisted', 'unconfirmed', 'dockUnconfirmed'],
        ['persisted', 'not_attempted', 'dockNotAttempted'],
        ['persisted', 'active_persistence_unconfirmed', 'dockPersistenceUnconfirmed'],
    ])('reports %s / %s and preserves every draft without writing datum or retrying', async (map_status, dock_status, key) => {
        // Deliberately no English prefix: structured fields alone determine the message.
        const fetcher = vi.fn<typeof fetch>().mockResolvedValue(new Response(JSON.stringify({error: 'raison locale', map_status, dock_status}), {status: 500}));
        vi.stubGlobal('fetch', fetcher);
        const options = backupOptions({}); const hook = renderHook(() => useMapFiles(options));
        await expect(hook.result.current.handleApplyOpenMowerImport('{"areas":[]}', 0, 0, true)).rejects.toThrow(`mapImportOpenMower.${key}`);
        expect(fetcher).toHaveBeenCalledExactlyOnceWith('/api/import/openmower', expect.objectContaining({method: 'POST'}));
        for (const setter of [options.setFeatures, options.setMap, options.setEditMap, options.setHasUnsavedChanges, options.setDockDirty,
            options.restoreCorridors, options.restoreObstacleOriginals]) expect(setter).not.toHaveBeenCalled();
        expect(options.notification.success).not.toHaveBeenCalled();
    });
    it.each([
        {error: 'legacy map error'},
        {error: 'set docking point: legacy error', map_status: 'future', dock_status: 'rejected'},
    ])('does not infer a safe state from missing/invalid metadata %j', async response => {
        const fetcher = vi.fn<typeof fetch>().mockResolvedValue(new Response(JSON.stringify(response), {status: 500}));
        vi.stubGlobal('fetch', fetcher);
        const options = backupOptions({}); const hook = renderHook(() => useMapFiles(options));
        await expect(hook.result.current.handleApplyOpenMowerImport('{"areas":[]}', 0, 0, true)).rejects.toThrow('mapImportOpenMower.outcomeUnknown');
        expect(fetcher).toHaveBeenCalledOnce(); expect(options.setHasUnsavedChanges).not.toHaveBeenCalled();
        expect(options.notification.success).not.toHaveBeenCalled();
    });
    it('retains a legacy applied acknowledgement without claiming detailed persistence or writing datum', async () => {
        const fetcher = vi.fn<typeof fetch>().mockResolvedValue(new Response(JSON.stringify({...previewSummary(), applied: true})));
        vi.stubGlobal('fetch', fetcher);
        const options = backupOptions({}); const hook = renderHook(() => useMapFiles(options));
        await expect(hook.result.current.handleApplyOpenMowerImport('{"areas":[]}', 0, 0, true)).rejects.toThrow('mapImportOpenMower.legacyApplied');
        expect(fetcher).toHaveBeenCalledOnce(); expect(options.setEditMap).not.toHaveBeenCalled();
        expect(options.setDockDirty).not.toHaveBeenCalled(); expect(options.notification.success).not.toHaveBeenCalled();
    });
    it('does not turn a lost response into success, retry or a datum write', async () => {
        const fetcher = vi.fn<typeof fetch>().mockRejectedValue(new TypeError('Connection lost'));
        vi.stubGlobal('fetch', fetcher);
        const options = backupOptions({}); const hook = renderHook(() => useMapFiles(options));
        await expect(hook.result.current.handleApplyOpenMowerImport('{"areas":[]}', 0, 0, true)).rejects.toThrow('mapImportOpenMower.outcomeUnknown');
        expect(fetcher).toHaveBeenCalledOnce(); expect(options.setEditMap).not.toHaveBeenCalled();
        expect(options.notification.success).not.toHaveBeenCalled();
    });
    it.each([true, false])('clears drafts only after confirmed success; dock requested=%s', async hasDock => {
        const data = {...previewSummary(), applied: true, map_status: 'persisted',
            dock_status: hasDock ? 'confirmed' : 'not_requested', dock_pose: hasDock ? previewSummary().dock_pose : null};
        const fetcher = vi.fn<typeof fetch>().mockResolvedValueOnce(new Response(JSON.stringify(data))).mockResolvedValueOnce(new Response('{}'));
        vi.stubGlobal('fetch', fetcher);
        const options = backupOptions({}); const hook = renderHook(() => useMapFiles(options));
        await hook.result.current.handleApplyOpenMowerImport('{"areas":[]}', 0, 0, true);
        expect(fetcher).toHaveBeenCalledTimes(2);
        expect(fetcher).toHaveBeenNthCalledWith(2, '/api/settings/yaml', expect.objectContaining({body: '{"datum_lat":0,"datum_lon":0}'}));
        expect(options.setEditMap).toHaveBeenCalledExactlyOnceWith(false);
        expect(options.setHasUnsavedChanges).toHaveBeenCalledExactlyOnceWith(false);
        expect(options.setDockDirty).toHaveBeenCalledExactlyOnceWith(false);
        expect(options.notification.success).toHaveBeenCalledExactlyOnceWith({message: 'mapFiles.openMowerMapImported',
            description: hasDock ? 'mapFiles.openMowerMapImportedDescription' : 'mapFiles.openMowerMapImportedWithoutDock'});
    });
});

describe('editable GeoJSON import boundary', () => {
    it('preserves ID zero, zero coordinates/order, optional null metadata and arbitrary extra data', () => {
        const feature = polygon(); const snapshot = decodeEditableGeoJSON({type: 'FeatureCollection', features: [feature]}, geoMessages);
        expect(snapshot['0'].id).toBe('0'); expect(snapshot['0'].geometry).toBe(feature.geometry);
        expect(snapshot['0'].properties).toEqual(feature.properties);
    });
    it.each([null, {}, {type: 'FeatureCollection', features: [null]},
        {type: 'FeatureCollection', features: [{...polygon(), geometry: null}]},
        {type: 'FeatureCollection', features: [{...polygon(), geometry: {type: 'Point', coordinates: [0, 0]}}]},
        {type: 'FeatureCollection', features: [{...polygon(), geometry: {type: 'Polygon', coordinates: [[[0, null], [1, 1], [0, 0]]]}}]},
        {type: 'FeatureCollection', features: [{...polygon(), properties: {...polygon().properties, mowing_order: '0'}}]},
        {type: 'FeatureCollection', features: [{...polygon(), properties: {...polygon().properties, name: null}}]},
        {type: 'FeatureCollection', features: [{...polygon(), properties: {...polygon().properties, mowing_order: null}}]},
        {type: 'FeatureCollection', features: [{...polygon(), properties: {...polygon().properties, feature_type: 'unsupported'}}]},
        {type: 'FeatureCollection', features: [polygon(), polygon()]},
        {type: 'FeatureCollection', features: [{...polygon(), id: '__proto__'}]},
    ])('rejects malformed complete candidates %j', raw => expect(() => decodeEditableGeoJSON(raw, geoMessages)).toThrow());
    it.each(['{', new Error('Cannot read GeoJSON'), JSON.stringify({type: 'FeatureCollection', features: [polygon(), {...polygon(), id: 2, geometry: null}]}),
        JSON.stringify({type: 'FeatureCollection', features: []}),
        JSON.stringify({type: 'FeatureCollection', features: [{...polygon(), properties: {feature_type: 'obstacle'}}]})])('preserves map, dirty state and dock when a file fails %s', async text => {
        const options = backupOptions({features: {dock: new DockFeatureBase([0, 0], 0)}});
        const hook = renderHook(() => useMapFiles(options)); selectedFile(hook.result.current.handleUploadGeoJSON, text, 'map.geojson');
        await waitFor(() => expect(options.notification.error).toHaveBeenCalled());
        expect(options.setFeatures).not.toHaveBeenCalled(); expect(options.setMap).not.toHaveBeenCalled();
        expect(options.setHasUnsavedChanges).not.toHaveBeenCalled(); expect(options.setDockDirty).not.toHaveBeenCalled();
        expect(options.notification.success).not.toHaveBeenCalled();
    });
    it('constructs real area/dock instances only after complete validation, without a save request', async () => {
        const setter = vi.fn<Parameters<typeof useMapFiles>[0]['setFeatures']>();
        const options = backupOptions({setFeatures: setter});
        const put = vi.spyOn(options.guiApi.mowglinext, 'putMowglinext');
        const hook = renderHook(() => useMapFiles(options));
        const point = {type: 'Feature', id: 'dock', properties: {feature_type: 'dock', heading: 0}, geometry: {type: 'Point', coordinates: [0, 0]}};
        const text = JSON.stringify({type: 'FeatureCollection', features: [polygon(), point]});
        selectedFile(hook.result.current.handleUploadGeoJSON, text);
        await waitFor(() => expect(setter).toHaveBeenCalledOnce());
        const next = setter.mock.calls[0][0]; if (typeof next === 'function') throw new Error('Expected prepared feature map');
        expect(next['0']).toBeInstanceOf(MowingAreaFeature); expect(next.dock).toBeInstanceOf(DockFeatureBase);
        expect(next['0'].geometry).toEqual(polygon().geometry);
        expect(options.setHasUnsavedChanges).toHaveBeenCalledWith(true);
        expect(put).not.toHaveBeenCalled();
        expect(options.notification.success).toHaveBeenCalledOnce();
    });
    it('cancels selection without mutation or an implicit write', () => {
        vi.spyOn(HTMLInputElement.prototype, 'click').mockImplementation(() => {});
        const options = backupOptions({}); const hook = renderHook(() => useMapFiles(options)); hook.result.current.handleUploadGeoJSON();
        const input = document.querySelector<HTMLInputElement>('input[type="file"]'); if (!input) throw new Error('Missing file picker');
        fireEvent(input, new Event('cancel')); expect(input).not.toBeInTheDocument();
        expect(options.setFeatures).not.toHaveBeenCalled(); expect(options.setHasUnsavedChanges).not.toHaveBeenCalled();
    });
});


describe('GeoJSON rehydration identity and parent integrity', () => {
    it.each(['toString', 'constructor', '__proto__'])('rejects inherited parent %s without an actual area, preserving all local state', async parent => {
        const options = backupOptions({features: {dock: new DockFeatureBase([0, 0], 0)}});
        const put = vi.spyOn(options.guiApi.mowglinext, 'putMowglinext');
        const hook = renderHook(() => useMapFiles(options));
        const obstacle = {...polygon(), id: 'obstacle', properties: {feature_type: 'obstacle', mowing_area: parent}};
        selectedFile(hook.result.current.handleUploadGeoJSON, JSON.stringify({type: 'FeatureCollection', features: [obstacle]}));
        await waitFor(() => expect(options.notification.error).toHaveBeenCalledOnce());
        expect(options.setFeatures).not.toHaveBeenCalled(); expect(options.setMap).not.toHaveBeenCalled();
        expect(options.setEditMap).not.toHaveBeenCalled(); expect(options.setDockDirty).not.toHaveBeenCalled();
        expect(options.setHasUnsavedChanges).not.toHaveBeenCalled(); expect(options.notification.success).not.toHaveBeenCalled();
        expect(put).not.toHaveBeenCalled();
    });
    it.each(['toString', 'constructor', '__proto__', 'missing', 'navigation'])('retains real-area fallback for an unresolved parent %s', async parent => {
        const setter = vi.fn<Parameters<typeof useMapFiles>[0]['setFeatures']>();
        const options = backupOptions({setFeatures: setter}); const hook = renderHook(() => useMapFiles(options));
        const obstacle = {...polygon(), id: 'obstacle', properties: {feature_type: 'obstacle', mowing_area: parent}};
        const navigation = {...polygon(), id: 'navigation', properties: {feature_type: 'navigation'}};
        selectedFile(hook.result.current.handleUploadGeoJSON, JSON.stringify({type: 'FeatureCollection', features: [polygon(), navigation, obstacle]}));
        await waitFor(() => expect(setter).toHaveBeenCalledOnce());
        const next = setter.mock.calls[0][0]; if (typeof next === 'function') throw new Error('Expected prepared map');
        const restored = next.obstacle; expect(restored).toBeInstanceOf(ObstacleFeature);
        if (!(restored instanceof ObstacleFeature)) throw new Error('Missing obstacle');
        expect(restored.getMowingArea()).toBe(next['0']); expect(restored.getMowingArea()).toBeInstanceOf(MowingAreaFeature);
        expect(restored.properties).toHaveProperty('mowing_area', parent); // Source metadata is not silently rewritten.
        expect(options.notification.error).not.toHaveBeenCalled();
    });
    it.each(['toString', 'constructor'])('accepts a real own workarea named %s instead of using fallback', async id => {
        const setter = vi.fn<Parameters<typeof useMapFiles>[0]['setFeatures']>();
        const options = backupOptions({setFeatures: setter}); const hook = renderHook(() => useMapFiles(options));
        const obstacle = {...polygon(), id: 'obstacle', properties: {feature_type: 'obstacle', mowing_area: id}};
        selectedFile(hook.result.current.handleUploadGeoJSON, JSON.stringify({type: 'FeatureCollection', features: [polygon(), {...polygon(), id}, obstacle]}));
        await waitFor(() => expect(setter).toHaveBeenCalledOnce());
        const next = setter.mock.calls[0][0]; if (typeof next === 'function') throw new Error('Expected prepared map');
        const restored = next.obstacle; if (!(restored instanceof ObstacleFeature)) throw new Error('Missing obstacle');
        expect(restored.getMowingArea()).toBe(next[id]); expect(restored.getMowingArea()).not.toBe(next['0']);
        expect(restored.getMowingArea()).toBeInstanceOf(MowingAreaFeature);
    });
    it.each(['other-dock', 5])('rejects noncanonical dock ID %s without modifying any local state', async id => {
        const options = backupOptions({features: {dock: new DockFeatureBase([0, 0], 0)}});
        const put = vi.spyOn(options.guiApi.mowglinext, 'putMowglinext');
        const hook = renderHook(() => useMapFiles(options));
        const dock = {type: 'Feature', id, properties: {feature_type: 'dock', heading: 0}, geometry: {type: 'Point', coordinates: [0, 0]}};
        selectedFile(hook.result.current.handleUploadGeoJSON, JSON.stringify({type: 'FeatureCollection', features: [polygon(), dock]}));
        await waitFor(() => expect(options.notification.error).toHaveBeenCalledOnce());
        expect(options.setFeatures).not.toHaveBeenCalled(); expect(options.setMap).not.toHaveBeenCalled();
        expect(options.setEditMap).not.toHaveBeenCalled(); expect(options.setDockDirty).not.toHaveBeenCalled();
        expect(options.setHasUnsavedChanges).not.toHaveBeenCalled(); expect(options.notification.success).not.toHaveBeenCalled();
        expect(put).not.toHaveBeenCalled();
    });
});
