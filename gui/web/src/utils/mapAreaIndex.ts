import type {Map} from "../types/ros.ts";

// The map topic separates navigation areas; array position is not a ROS ID.
export function mowingAreaIndex(map: Map | undefined, position: number | undefined): number | undefined {
    if (position === undefined || !Number.isInteger(position) || position < 0 || position >= (map?.working_area?.length ?? 0)) return;
    const index = map?.working_area_indices?.[position];
    return index !== undefined && Number.isInteger(index) && index >= 0 ? index : undefined;
}

// Resolve a working area's CURRENT ROS index (map_server's array position —
// what ~/start_in_area and coverage_orientation expect) by its STABLE id
// (MapArea.id, mowglinext#637), instead of a position captured earlier.
//
// The array position of a given area can shift whenever the area list is
// edited and saved (map_server rebuilds it wholesale — clear + re-add every
// surviving area), so a position captured at one point in time and re-resolved
// against a LATER `map` snapshot can silently land on a different area. Any
// caller that captures a selection and acts on it later (a button click,
// after a poll refresh) should resolve by id at the moment it fires the
// action, against the freshest `map` available, rather than carry a stale
// index forward.
export function mowingAreaIndexById(map: Map | undefined, id: number | undefined): number | undefined {
    if (id === undefined || !Number.isInteger(id)) return;
    const position = map?.working_area?.findIndex(area => area.id === id) ?? -1;
    return position >= 0 ? mowingAreaIndex(map, position) : undefined;
}
