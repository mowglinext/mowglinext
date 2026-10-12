import type {TFunction} from 'i18next';
import type {Map as MowingMap} from '../types/ros.ts';

/** Names are presentation only. Commands must continue to resolve stable IDs. */
export function areaLabel(t: TFunction, oneBasedOrder: number, name?: string): string {
    return name?.trim() || t('mapAreasList.unnamedArea', {index: oneBasedOrder});
}

/** Live ROS indices include navigation areas; resolve the current mowing ordinal. */
export function liveAreaLabel(t: TFunction, map: MowingMap, rosIndex: number): string {
    const position = map.working_area_indices?.indexOf(rosIndex) ?? -1;
    const area = position >= 0 ? map.working_area?.[position] : undefined;
    return areaLabel(t, position >= 0 ? position + 1 : rosIndex + 1, area?.name);
}

export function formatArea(squareMeters: number, language: string): string {
    if (squareMeters > 0 && squareMeters < 1) return '< 1 m²';
    const hectares = squareMeters >= 10000;
    return `${(hectares ? squareMeters / 10000 : squareMeters).toLocaleString(language, {
        maximumFractionDigits: hectares ? 2 : 0,
    })} ${hectares ? 'ha' : 'm²'}`;
}
