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

export function rasterizeLidarMap(grid: OccupancyGrid): RasterizedGrid | null {
    return rasterizeOccupancyGrid(grid, lidarMapPaint);
}
