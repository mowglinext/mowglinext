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
