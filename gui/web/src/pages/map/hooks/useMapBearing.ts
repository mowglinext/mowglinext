import {useCallback} from 'react';
import {useMapDisplayPreference, type MapDisplayPreferenceOptions} from './useMapDisplayPreference';

// Display-only Mapbox compass rotation, clockwise degrees from north. This
// changes no robot frame, map datum, ROS yaw or rotation-aware click projection.
export function useMapBearing(options: MapDisplayPreferenceOptions) {
    const {value: bearing, handleValue} = useMapDisplayPreference(options, 'gui.map.display.bearing', 400, 'mapBearing.saveError');
    const handleBearing = useCallback((value: number) => {
        if (!Number.isFinite(value)) return;
        // Preserve the existing normalisation into [-180, 180).
        handleValue(((value + 180) % 360 + 360) % 360 - 180);
    }, [handleValue]);
    return {bearing, handleBearing};
}
