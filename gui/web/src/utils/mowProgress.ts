import {OccupancyGrid} from "../types/ros.ts";
import {RasterizedGrid, Rgba, rasterizeOccupancyGrid} from "./occupancyGrid.ts";

export type RasterizedMowProgress = RasterizedGrid;

/** Translucent lime overlay used for mowed cells across the GUI. */
const MOWED_RGBA: Rgba = [124, 255, 178, 150];

/** A mow-progress cell is mowed at value >= 100; anything else is transparent. */
export function mowProgressPaint(value: number): Rgba | null {
    return value >= 100 ? MOWED_RGBA : null;
}

/**
 * Rasterize the mow-progress OccupancyGrid into the translucent lime overlay.
 * Single source of truth for the mowed-cell pixel pass (MapPage + dashboard
 * mini-map); the row flip and canvas plumbing live in utils/occupancyGrid.ts.
 */
export function rasterizeMowProgress(grid: OccupancyGrid): RasterizedMowProgress | null {
    return rasterizeOccupancyGrid(grid, mowProgressPaint);
}
