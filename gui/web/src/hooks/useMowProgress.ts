import {OccupancyGrid} from "../types/ros.ts";
import {useTopic} from "./useTopic.ts";

/**
 * Subscribes to the mow-progress OccupancyGrid (`/map_server_node/mow_progress`,
 * 100 = mowed) that MapPage already renders. Factored out so the always-on
 * dashboard mini-map can reuse the same stream. Throttled hard client-side (on
 * top of the backend's 500 ms cap) because the grid is large and the widget is
 * small — one raster per second is plenty for a mini-map.
 *
 * Pass `enabled=false` whenever nothing on screen is drawing the grid. The
 * throttle only limits React re-renders; the websocket still delivers every
 * message, and this one is large: 351 kB per grid, ~207 kB/s into the browser
 * on the field robot (2026-09-16). `useTopic` drops the upstream subscription
 * entirely while disabled, so the backend stops forwarding it too. Callers
 * normally pass `useDocumentVisible()`.
 */
export const useMowProgress = (enabled = true): OccupancyGrid =>
    useTopic<OccupancyGrid>("mowProgress", {}, {throttleMs: 1000, enabled}).data;
