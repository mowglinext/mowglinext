import type {Map as MapType, MapArea} from "../../../types/ros.ts";

export interface MetricBounds {
    minX: number;
    minY: number;
    maxX: number;
    maxY: number;
}

const DEFAULT_MAP_SPAN_M = 10;
const MIN_GEOMETRY_SPAN_M = 1;

const finiteOr = (value: number | undefined, fallback: number): number =>
    value !== undefined && Number.isFinite(value) ? value : fallback;

const positiveFiniteOr = (value: number | undefined, fallback: number): number =>
    value !== undefined && Number.isFinite(value) && value > 0 ? value : fallback;

function fallbackBounds(map: MapType | undefined): MetricBounds {
    const centerX = finiteOr(map?.map_center_x, 0);
    const centerY = finiteOr(map?.map_center_y, 0);
    const halfWidth = positiveFiniteOr(map?.map_width, DEFAULT_MAP_SPAN_M) / 2;
    const halfHeight = positiveFiniteOr(map?.map_height, DEFAULT_MAP_SPAN_M) / 2;
    return {
        minX: centerX - halfWidth,
        minY: centerY - halfHeight,
        maxX: centerX + halfWidth,
        maxY: centerY + halfHeight,
    };
}

/**
 * Calculate an initial viewport that contains every configured mowing and
 * navigation area. Coordinates are local map metres (x=east, y=north).
 *
 * Mapbox fits bounds north-up before the saved bearing is restored. Inflate
 * the bounds for that bearing so rotating the fitted camera cannot clip the
 * garden's corners. Pixel padding is applied separately by the Map component.
 */
export function calculateMapViewportBounds(
    map: MapType | undefined,
    bearingDegrees: number,
): MetricBounds {
    const areas: MapArea[] = [
        ...(map?.working_area ?? []),
        ...(map?.navigation_areas ?? []),
    ];

    let minX = Number.POSITIVE_INFINITY;
    let minY = Number.POSITIVE_INFINITY;
    let maxX = Number.NEGATIVE_INFINITY;
    let maxY = Number.NEGATIVE_INFINITY;

    for (const area of areas) {
        for (const point of area.area?.points ?? []) {
            const {x, y} = point;
            if (typeof x !== "number" || !Number.isFinite(x) ||
                typeof y !== "number" || !Number.isFinite(y)) continue;
            minX = Math.min(minX, x);
            minY = Math.min(minY, y);
            maxX = Math.max(maxX, x);
            maxY = Math.max(maxY, y);
        }
    }

    if (!Number.isFinite(minX)) return fallbackBounds(map);

    const centerX = (minX + maxX) / 2;
    const centerY = (minY + maxY) / 2;
    const width = Math.max(maxX - minX, MIN_GEOMETRY_SPAN_M);
    const height = Math.max(maxY - minY, MIN_GEOMETRY_SPAN_M);
    const bearingRadians = finiteOr(bearingDegrees, 0) * Math.PI / 180;
    const cos = Math.abs(Math.cos(bearingRadians));
    const sin = Math.abs(Math.sin(bearingRadians));

    // Axis-aligned screen-space extent after rotating the map around its centre.
    const fittedWidth = width * cos + height * sin;
    const fittedHeight = width * sin + height * cos;

    return {
        minX: centerX - fittedWidth / 2,
        minY: centerY - fittedHeight / 2,
        maxX: centerX + fittedWidth / 2,
        maxY: centerY + fittedHeight / 2,
    };
}
