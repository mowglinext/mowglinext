import {Map as MapType} from "../types/ros.ts";
import {useTopic} from "./useTopic.ts";

/**
 * Subscribes to the merged /map view (working_area + navigation_area + dock
 * pose + obstacles). Mirrors the inline stream that SchedulePage and MapPage
 * both maintain locally -- factored out so the dashboard widget can reuse the
 * same data.
 */
export const useMowingMap = (): MapType => useTopic<MapType>("map", {}).data;
