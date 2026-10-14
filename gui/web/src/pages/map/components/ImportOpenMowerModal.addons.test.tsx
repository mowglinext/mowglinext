import {act, cleanup, fireEvent, render, screen} from '@testing-library/react';
import {afterEach, expect, it, vi} from 'vitest';
import {App} from 'antd';
import {ImportOpenMowerModal} from './ImportOpenMowerModal';
import type {ImportOpenMowerSummary} from '../hooks/useMapFiles';
afterEach(cleanup);
const summary: ImportOpenMowerSummary = {mowing_areas: 1, navigation_areas: 0, obstacles: 0, orphan_obstacles: 0,
    datum_shift_east_m: 0, datum_shift_north_m: 0, warnings: [], areas: [], applied: false};
it('forwards each explicit Enter/blur to one preview reprojection, without applying the map', async () => {
    const reproject = vi.fn<(lat?: number, lon?: number) => Promise<ImportOpenMowerSummary>>().mockResolvedValue(summary);
    const apply = vi.fn(); const error = vi.spyOn(console, 'error');
    try {
        render(<App><ImportOpenMowerModal preview={summary} onApply={apply} onReproject={reproject} onClose={vi.fn()}/></App>);
        const latitude = screen.getByPlaceholderText('OM_DATUM_LAT');
        const longitude = screen.getByPlaceholderText('OM_DATUM_LONG');
        fireEvent.change(latitude, {target: {value: '0'}}); fireEvent.change(longitude, {target: {value: '2'}});
        await act(async () => {fireEvent.keyDown(latitude, {key: 'Enter', code: 'Enter'}); await Promise.resolve();});
        expect(reproject).toHaveBeenCalledExactlyOnceWith(0, 2);
        await act(async () => {fireEvent.blur(latitude); await Promise.resolve();});
        expect(reproject).toHaveBeenCalledTimes(2); // Existing two distinct commit events; no new wrapper listeners.
        expect(apply).not.toHaveBeenCalled();
        expect(error.mock.calls.some(args => args.some(arg => typeof arg === 'string' && arg.includes('[antd: InputNumber]') && /addonBefore|addonAfter/.test(arg)))).toBe(false);
    } finally {error.mockRestore();}
});
