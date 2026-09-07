import {act, renderHook} from '@testing-library/react';
import {beforeEach, describe, expect, it, vi} from 'vitest';
import {useMapEditing, type UseMapEditingOptions} from './useMapEditing';

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
