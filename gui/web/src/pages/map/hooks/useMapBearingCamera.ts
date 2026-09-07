import {useCallback, useEffect, useRef, type RefObject} from 'react';
import type {Map as MapboxMap} from 'mapbox-gl';

interface Options {
    mapInstanceRef: RefObject<MapboxMap | null>;
    bearing: number;
    onBearingChange: (bearing: number) => void;
    interactive: boolean;
}

/** Restore the display camera regardless of whether config or the map loads first. */
export function useMapBearingCamera({mapInstanceRef, bearing, onBearingChange, interactive}: Options) {
    const detachRef = useRef<(() => void) | null>(null);
    const onBearingChangeRef = useRef(onBearingChange);
    useEffect(() => {
        onBearingChangeRef.current = onBearingChange;
    }, [onBearingChange]);

    useEffect(() => {
        const map = mapInstanceRef.current;
        if (map && Math.abs(map.getBearing() - bearing) > 0.5) {
            map.easeTo({bearing, duration: 200});
        }
    }, [bearing, mapInstanceRef]);

    const onLoad = useCallback(({target}: {target: MapboxMap}) => {
        detachRef.current?.();
        mapInstanceRef.current = target;
        // Setting a ref doesn't rerun the effect above. Bounds fitting and
        // reused map instances can also start north-up despite initialViewState.
        target.setBearing(bearing);
        const onRotateEnd = (event: {originalEvent?: unknown}) => {
            // Restoration and slider animations are already represented in
            // config/state. Only gestures should initiate another save.
            if (event.originalEvent) onBearingChangeRef.current(target.getBearing());
        };
        if (interactive) target.on('rotateend', onRotateEnd);
        detachRef.current = () => {
            if (interactive) target.off('rotateend', onRotateEnd);
            if (mapInstanceRef.current === target) mapInstanceRef.current = null;
        };
    }, [bearing, interactive, mapInstanceRef]);

    useEffect(() => () => {
        detachRef.current?.();
        detachRef.current = null;
    }, []);

    return onLoad;
}
