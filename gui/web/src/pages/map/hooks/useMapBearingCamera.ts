import {useCallback, useEffect, useRef, type RefObject} from 'react';
import type {Map as MapboxMap} from 'mapbox-gl';

interface Options {
    mapInstanceRef: RefObject<MapboxMap | null>;
    bearing: number;
    onBearingChange: (bearing: number) => void;
    interactive: boolean;
    rotationLocked: boolean;
}

/** Restore the display camera regardless of whether config or the map loads first. */
export function useMapBearingCamera({
    mapInstanceRef,
    bearing,
    onBearingChange,
    interactive,
    rotationLocked,
}: Options) {
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

    const applyRotationInteraction = useCallback((map: MapboxMap) => {
        if (!interactive) return;
        if (rotationLocked) {
            map.dragRotate.disable();
            map.touchZoomRotate.disableRotation();
        } else {
            map.dragRotate.enable();
            map.touchZoomRotate.enableRotation();
        }
    }, [interactive, rotationLocked]);

    useEffect(() => {
        const map = mapInstanceRef.current;
        if (map) applyRotationInteraction(map);
    }, [applyRotationInteraction, mapInstanceRef]);

    const onLoad = useCallback(({target}: {target: MapboxMap}) => {
        detachRef.current?.();
        mapInstanceRef.current = target;
        // Setting a ref doesn't rerun the effect above. Bounds fitting and
        // reused map instances can also start north-up despite initialViewState.
        target.setBearing(bearing);
        applyRotationInteraction(target);
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
    }, [applyRotationInteraction, bearing, interactive, mapInstanceRef]);

    useEffect(() => () => {
        detachRef.current?.();
        detachRef.current = null;
    }, []);

    return onLoad;
}
