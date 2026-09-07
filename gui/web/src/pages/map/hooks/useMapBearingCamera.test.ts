import {act, renderHook} from '@testing-library/react';
import {describe, expect, it, vi} from 'vitest';
import type {Map as MapboxMap} from 'mapbox-gl';
import {useMapBearingCamera} from './useMapBearingCamera';

function camera() {
    let bearing = 0;
    const map = {
        getBearing: () => bearing,
        setBearing: vi.fn((value: number) => { bearing = value; }),
        easeTo: vi.fn((value: {bearing: number}) => { bearing = value.bearing; }),
        on: vi.fn(), off: vi.fn(),
    };
    return {map, target: map as unknown as MapboxMap};
}

describe('saved map camera bearing', () => {
    it.each([true, false])('restores config loaded before the map (interactive=%s)', interactive => {
        const mapInstanceRef = {current: null as MapboxMap | null};
        const onBearingChange = vi.fn();
        const {result} = renderHook(() => useMapBearingCamera({mapInstanceRef, bearing: 47, interactive, onBearingChange}));
        const {map, target} = camera();
        act(() => result.current({target}));
        expect(map.getBearing()).toBe(47);
        expect(onBearingChange).not.toHaveBeenCalled();
        expect(map.on).toHaveBeenCalledTimes(interactive ? 1 : 0);
    });

    it('applies a saved value arriving after map load', () => {
        const mapInstanceRef = {current: null as MapboxMap | null};
        const {result, rerender} = renderHook(({bearing}) => useMapBearingCamera({mapInstanceRef, bearing, interactive: true, onBearingChange: vi.fn()}), {initialProps: {bearing: 0}});
        const {map, target} = camera();
        act(() => result.current({target}));
        rerender({bearing: -35});
        expect(map.getBearing()).toBe(-35);
    });

    it('restores replacement maps and removes listeners when leaving the page', () => {
        const mapInstanceRef = {current: null as MapboxMap | null};
        const {result, unmount} = renderHook(() => useMapBearingCamera({mapInstanceRef, bearing: 73, interactive: true, onBearingChange: vi.fn()}));
        const first = camera();
        const second = camera();
        act(() => result.current({target: first.target}));
        act(() => result.current({target: second.target}));
        expect(first.map.off).toHaveBeenCalledOnce();
        expect(second.map.getBearing()).toBe(73);
        unmount();
        expect(second.map.off).toHaveBeenCalledOnce();
        expect(mapInstanceRef.current).toBeNull();
    });

    it('saves gestures through the latest callback but ignores programmatic rotation', () => {
        const mapInstanceRef = {current: null as MapboxMap | null};
        const oldCallback = vi.fn();
        const nextCallback = vi.fn();
        const {result, rerender} = renderHook(({onBearingChange}) => useMapBearingCamera({mapInstanceRef, bearing: 20, interactive: true, onBearingChange}), {initialProps: {onBearingChange: oldCallback}});
        const {map, target} = camera();
        act(() => result.current({target}));
        const rotateEnd = map.on.mock.calls[0][1] as (event: object) => void;
        rotateEnd({});
        expect(oldCallback).not.toHaveBeenCalled();
        rerender({onBearingChange: nextCallback});
        map.setBearing(82);
        rotateEnd({originalEvent: {type: 'touchend'}});
        expect(nextCallback).toHaveBeenCalledWith(82);
        expect(oldCallback).not.toHaveBeenCalled();
    });
});
