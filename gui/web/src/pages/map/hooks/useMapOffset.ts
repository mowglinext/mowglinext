import {useMapDisplayPreference, type MapDisplayPreferenceOptions} from './useMapDisplayPreference';

export function useMapOffset(options: MapDisplayPreferenceOptions) {
    const x = useMapDisplayPreference(options, 'gui.map.offset.x', 1000, 'mapOffset.saveError');
    const y = useMapDisplayPreference(options, 'gui.map.offset.y', 1000, 'mapOffset.saveError');
    return {offsetX: x.value, offsetY: y.value, handleOffsetX: x.handleValue, handleOffsetY: y.handleValue};
}
