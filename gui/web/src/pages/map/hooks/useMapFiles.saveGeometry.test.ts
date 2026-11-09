import {act, renderHook} from '@testing-library/react';
import {describe, expect, it, vi} from 'vitest';
import {useMapFiles} from './useMapFiles';
import {MowingAreaFeature, ObstacleFeature} from '../../../types/map.ts';
import type {Map as MapType, MapArea} from '../../../types/ros.ts';
import {Api} from '../../../api/Api.ts';

// Interpolating mock so the tests can see WHICH polygon a message names.
vi.mock('react-i18next', () => ({
    useTranslation: () => ({t: (key: string, opts?: Record<string, unknown>) => opts ? `${key}:${JSON.stringify(opts)}` : key}),
}));

type Options = Parameters<typeof useMapFiles>[0];
type Pt = {x: number; y: number; z: number};
const text = (value: unknown) => typeof value === 'string' ? value : '';
const pts = (ring: Array<[number, number]>): Pt[] => ring.map(([x, y]) => ({x, y, z: 0}));

// A recorded perimeter that crosses itself (bow-tie), as a driven boundary can.
const BOWTIE = pts([[0, 0], [10, 10], [0, 10], [10, 0]]);
const SQUARE = pts([[20, 0], [30, 0], [30, 10], [20, 10]]);

function options(features: Options['features'], map: MapType): Options {
    const guiApi = new Api<unknown>({customFetch: vi.fn<typeof fetch>().mockImplementation(() => Promise.resolve(new Response('{}')))});
    return {
        features, setFeatures: vi.fn(), map, setMap: vi.fn(), editMap: true, setEditMap: vi.fn(),
        setHasUnsavedChanges: vi.fn(), offsetX: 0, offsetY: 0, datum: [0, 0, 0],
        notification: {success: vi.fn(), warning: vi.fn(), error: vi.fn(), info: vi.fn(), open: vi.fn(), destroy: vi.fn()},
        guiApi, dockDirty: false, setDockDirty: vi.fn(), buildFeaturesFromMap: vi.fn(),
        obstacleOriginals: [], restoreObstacleOriginals: vi.fn(), corridors: [], restoreCorridors: vi.fn(),
    };
}

// Same construction as MapPage.buildFeatures: geometry comes from the ROS area.
function loadedArea(id: string, order: number, area: MapArea) {
    const feature = new MowingAreaFeature(id, order);
    feature.setArea(structuredClone(area), 0, 0, [0, 0, 0]);
    return feature;
}

async function save(opts: Options) {
    const put = vi.spyOn(opts.guiApi.mowglinext, 'putMowglinext');
    const hook = renderHook(() => useMapFiles(opts));
    await act(async () => {await hook.result.current.handleSaveMap();});
    return put;
}

describe('handleSaveMap polygon validity', () => {
    it('refuses a self-intersecting area drawn in this edit and names it', async () => {
        const drawn = new MowingAreaFeature('new-area', 1);
        drawn.setName('Back lawn');
        const fresh = loadedArea('tmp', 1, {name: 'tmp', area: {points: BOWTIE}});
        drawn.setGeometry(fresh.geometry);
        const opts = options({[drawn.id]: drawn}, {working_area: [], navigation_areas: []});

        const put = await save(opts);

        expect(put).not.toHaveBeenCalled();
        expect(opts.notification.error).toHaveBeenCalledOnce();
        const {description} = vi.mocked(opts.notification.error).mock.calls[0][0];
        expect(text(description)).toContain('mapFiles.invalidGeometry');
        expect(text(description)).toContain('mapFiles.affectedPolygons');
        expect(text(description)).toContain('Back lawn');
        expect(opts.setEditMap).not.toHaveBeenCalled();
    });

    it('refuses a loaded invalid perimeter the operator edited, and names an invalid edited obstacle with its area', async () => {
        const original: MapArea = {name: 'Front', id: 1, area: {points: BOWTIE}};
        const front = loadedArea('area-0-area-0', 1, original);
        // Operator drags one vertex: still crossing, no longer the loaded ring.
        const ring = front.geometry.coordinates[0].map(p => [...p]);
        ring[1] = [ring[1][0] * 0.9, ring[1][1] * 0.9];
        front.setGeometry({type: 'Polygon', coordinates: [ring]});
        const garden = loadedArea('area-1-area-0', 2, {name: 'Garden', id: 2, area: {points: SQUARE}});
        const obstacle = new ObstacleFeature('area-1-obstacle-0', garden);
        obstacle.transpose(pts([[22, 2], [28, 8], [22, 8], [28, 2]]), 0, 0, [0, 0, 0]);
        const opts = options({[front.id]: front, [garden.id]: garden, [obstacle.id]: obstacle},
            {working_area: [original, {name: 'Garden', id: 2, area: {points: SQUARE}}]});

        const put = await save(opts);

        expect(put).not.toHaveBeenCalled();
        const description = text(vi.mocked(opts.notification.error).mock.calls[0][0].description);
        expect(description).toContain('Front');
        expect(description).toContain('mapFiles.polygonObstacle');
        expect(description).toContain('\\"index\\":1');
        expect(description).toContain('Garden');
    });

    it('saves an unrelated edit, passing an untouched invalid perimeter through unchanged with a warning', async () => {
        const recorded: MapArea = {name: 'Recorded', id: 7, area: {points: BOWTIE}};
        const map: MapType = {working_area: [recorded, {name: 'Garden', id: 8, area: {points: SQUARE}}], navigation_areas: []};
        const untouched = loadedArea('area-0-area-0', 1, recorded);
        const edited = loadedArea('area-1-area-0', 2, {name: 'Garden', id: 8, area: {points: SQUARE}});
        edited.setGeometry({type: 'Polygon', coordinates: [edited.geometry.coordinates[0].map(([lon, lat]) => [lon, lat * 1.0001])]});
        const opts = options({[untouched.id]: untouched, [edited.id]: edited}, map);

        const put = await save(opts);

        expect(opts.notification.error).not.toHaveBeenCalled();
        expect(put).toHaveBeenCalledOnce();
        const sent = put.mock.calls[0][0].areas ?? [];
        const byName = new Map(sent.map(a => [a.area?.name, a.area]));
        expect(byName.get('Recorded')?.area?.points).toEqual(BOWTIE);
        expect(byName.get('Garden')?.area?.points).not.toEqual(SQUARE);
        expect(opts.notification.success).toHaveBeenCalledOnce();
        expect(opts.notification.warning).toHaveBeenCalledOnce();
        const warning = vi.mocked(opts.notification.warning).mock.calls[0][0];
        expect(warning.message).toBe('mapFiles.preExistingInvalidGeometryTitle');
        expect(text(warning.description)).toContain('Recorded');
        expect(text(warning.description)).not.toContain('Garden');
        expect(opts.setEditMap).toHaveBeenCalledWith(false);
    });

    it('saves a valid map without any warning', async () => {
        const area: MapArea = {name: 'Garden', id: 8, area: {points: SQUARE}};
        const garden = loadedArea('area-0-area-0', 1, area);
        const opts = options({[garden.id]: garden}, {working_area: [area], navigation_areas: []});

        const put = await save(opts);

        expect(put).toHaveBeenCalledOnce();
        expect(opts.notification.error).not.toHaveBeenCalled();
        expect(opts.notification.warning).not.toHaveBeenCalled();
        expect(opts.notification.success).toHaveBeenCalledOnce();
    });
});
