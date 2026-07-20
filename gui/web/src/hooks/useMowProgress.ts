import {OccupancyGrid} from "../types/ros.ts";
import {useTopic} from "./useTopic.ts";

/**
 * Subscribes to the mow-progress OccupancyGrid (`/map_server_node/mow_progress`,
 * 100 = mowed) that MapPage already renders. Factored out so the always-on
 * dashboard mini-map can reuse the same stream. Throttled hard client-side (on
 * top of the backend's 500 ms cap) because the grid is large and the widget is
 * small — one raster per second is plenty for a mini-map.
 */
export const useMowProgress = (): OccupancyGrid =>
    useTopic<OccupancyGrid>("mowProgress", {}, {throttleMs: 1000}).data;
