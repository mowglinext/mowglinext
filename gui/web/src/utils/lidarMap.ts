import {OccupancyGrid} from "../types/ros.ts";
import {RasterizedGrid, Rgba, rasterizeOccupancyGrid} from "./occupancyGrid.ts";

/**
 * The LiDAR anchor map (/fusion_graph/lidar_map, fusion_graph's log-odds grid
 * exported as 0 = free, 100 = occupied, -1 = unknown). Drawn as ink on the
 * satellite tiles: occupied cells are solid, free cells a faint wash so the
 * scanned fans read against unknown ground, unknown stays transparent.
 */
const OCCUPIED_RGBA: Rgba = [18, 26, 34, 235];
const FREE_RGBA: Rgba = [255, 255, 255, 46];
const OCCUPIED_MIN = 50;

export function lidarMapPaint(value: number): Rgba | null {
    if (value < 0) return null; // unknown
    return value >= OCCUPIED_MIN ? OCCUPIED_RGBA : FREE_RGBA;
}

/** True when at least one cell is occupied — an empty grid is "no map yet". */
export function lidarMapHasStructure(grid: OccupancyGrid): boolean {
    if (!grid.data) return false;
    for (let i = 0; i < grid.data.length; i++) {
        if (grid.data[i] >= OCCUPIED_MIN) return true;
    }
    return false;
}

/**
 * Rasterize the anchor map, or null while it has no occupied cell: the map
 * page replaces the raw scan points with the map only once there is a map,
 * otherwise a fresh (empty, latched) grid would blank the points for nothing.
 */
export function rasterizeLidarMap(grid: OccupancyGrid): RasterizedGrid | null {
    if (!lidarMapHasStructure(grid)) return null;
    return rasterizeOccupancyGrid(grid, lidarMapPaint);
}
