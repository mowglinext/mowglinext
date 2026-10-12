import type {LidarIgnoreCorridor, Map as MapType} from "../../../types/ros.ts";
import {parseOriginals, type ObstacleOriginal} from "./obstacleOriginals.ts";

const MIN_POLYGON_POINTS = 3;

export type MapBackupParseResult =
    | {
          ok: true;
          map: MapType;
          hasDock: boolean;
          obstacleOriginals: ObstacleOriginal[] | null;
          corridors: LidarIgnoreCorridor[] | null;
      }
    | {ok: false; reason: string};

/** The outlines obstacles had before the recorded-obstacle shrink (null = field absent). */
export const BACKUP_ORIGINALS_KEY = "obstacle_originals";

const isRecord = (value: unknown): value is Record<string, unknown> =>
    typeof value === "object" && value !== null && !Array.isArray(value);

const isFiniteNumber = (value: unknown): value is number =>
    typeof value === "number" && Number.isFinite(value);

function polygonProblem(polygon: unknown, label: string): string | null {
    if (!isRecord(polygon) || !Array.isArray(polygon.points)) {
        return `${label} has no points list`;
    }
    if (polygon.points.length < MIN_POLYGON_POINTS) {
        return `${label} has fewer than ${MIN_POLYGON_POINTS} points`;
    }
    const isValidPoint = (p: unknown) => isRecord(p) && isFiniteNumber(p.x) && isFiniteNumber(p.y);
    return polygon.points.every(isValidPoint) ? null : `${label} has a non-numeric coordinate`;
}

function areaListProblem(list: unknown, label: string): string | null {
    if (list === undefined) {
        return null;
    }
    if (!Array.isArray(list)) {
        return `${label} is not a list`;
    }
    for (const [index, area] of list.entries()) {
        const areaLabel = `${label}[${index}]`;
        if (!isRecord(area)) {
            return `${areaLabel} is not an object`;
        }
        const obstacles = area.obstacles ?? [];
        if (!Array.isArray(obstacles)) {
            return `${areaLabel}.obstacles is not a list`;
        }
        const problem = polygonProblem(area.area, areaLabel)
            ?? obstacles.map((o, i) => polygonProblem(o, `${areaLabel}.obstacles[${i}]`)).find((p) => p !== null);
        if (problem) {
            return problem;
        }
    }
    return null;
}

/** The backup field the ignore lines travel in (map_server owns them, not the Map message). */
export const BACKUP_CORRIDORS_KEY = "lidar_ignore_corridors";

/** `null` = the backup has no such field (older backups): a restore leaves the current lines alone. */
function corridorsFrom(value: unknown): {corridors: LidarIgnoreCorridor[] | null} | {problem: string} {
    if (value === undefined) {
        return {corridors: null};
    }
    if (!Array.isArray(value)) {
        return {problem: `${BACKUP_CORRIDORS_KEY} is not a list`};
    }
    const corridors: LidarIgnoreCorridor[] = [];
    for (const [index, entry] of value.entries()) {
        const label = `${BACKUP_CORRIDORS_KEY}[${index}]`;
        if (!isRecord(entry) || !isRecord(entry.polyline) || !Array.isArray(entry.polyline.points)) {
            return {problem: `${label} has no polyline`};
        }
        const points: unknown[] = entry.polyline.points;
        if (points.length < 2 || !points.every((p) => isRecord(p) && isFiniteNumber(p.x) && isFiniteNumber(p.y))) {
            return {problem: `${label} needs at least 2 numeric points`};
        }
        if (!isFiniteNumber(entry.width_m)) {
            return {problem: `${label} has no numeric width_m`};
        }
        corridors.push({
            name: typeof entry.name === "string" ? entry.name : "",
            polyline: {points: (points as {x: number; y: number}[]).map((p) => ({x: p.x, y: p.y, z: 0}))},
            width_m: entry.width_m,
            // Fresh ids on restore: the ones in the file belong to the robot it was taken from.
            id: 0,
        });
    }
    return {corridors};
}

/**
 * Validate the text of a map.json backup as a COMPLETE restore candidate
 * before any editor state is touched (#704). A backup with no area at all is
 * refused: restoring it and pressing Save would replace the real map with
 * nothing. `hasDock` is false when the dock fields are absent or not all
 * finite numbers — a missing dock is NOT a dock at (0, 0, 0).
 */
export function parseMapBackup(text: string): MapBackupParseResult {
    let parsed: unknown;
    try {
        parsed = JSON.parse(text);
    } catch (e) {
        return {ok: false, reason: `not valid JSON (${e instanceof Error ? e.message : String(e)})`};
    }
    if (!isRecord(parsed)) {
        return {ok: false, reason: "not a map object"};
    }
    const problem = areaListProblem(parsed.working_area, "working_area")
        ?? areaListProblem(parsed.navigation_areas, "navigation_areas");
    if (problem) {
        return {ok: false, reason: problem};
    }
    const areaCount = [parsed.working_area, parsed.navigation_areas]
        .reduce<number>((count, list) => count + (Array.isArray(list) ? list.length : 0), 0);
    if (areaCount === 0) {
        return {ok: false, reason: "the backup contains no area"};
    }
    const hasDock = [parsed.dock_x, parsed.dock_y, parsed.dock_heading].every(isFiniteNumber);
    if (parsed[BACKUP_ORIGINALS_KEY] !== undefined && !Array.isArray(parsed[BACKUP_ORIGINALS_KEY])) {
        return {ok: false, reason: `${BACKUP_ORIGINALS_KEY} is not a list`};
    }
    const obstacleOriginals = parseOriginals(parsed[BACKUP_ORIGINALS_KEY]);
    const lines = corridorsFrom(parsed[BACKUP_CORRIDORS_KEY]);
    if ("problem" in lines) {
        return {ok: false, reason: lines.problem};
    }
    // Neither side-channel is part of the Map message: keep both out of it.
    const mapOnly = {...parsed};
    delete mapOnly[BACKUP_ORIGINALS_KEY];
    delete mapOnly[BACKUP_CORRIDORS_KEY];
    return {ok: true, map: mapOnly as MapType, hasDock, obstacleOriginals, corridors: lines.corridors};
}
