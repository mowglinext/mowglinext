import type {Map as MapType} from '../../../types/ros.ts';
import {isJsonObject} from '../../../utils/json';
import {MapGeometryError, projectMapPolygon, projectMapPosition} from './mapGeometryContract';

export type RosPoint = {x: number; y: number; z: number};

/** A ring counts as untouched when every vertex matches the loaded map to 1 mm. */
const BASELINE_MATCH_TOLERANCE_M = 1e-3;

const finitePoint = (p: RosPoint) => Number.isFinite(p.x) && Number.isFinite(p.y) && Number.isFinite(p.z);
const near = (a: RosPoint, b: RosPoint) =>
    Math.abs(a.x - b.x) <= BASELINE_MATCH_TOLERANCE_M && Math.abs(a.y - b.y) <= BASELINE_MATCH_TOLERANCE_M;
const openRing = (ring: readonly RosPoint[]) =>
    ring.length > 1 && near(ring[0], ring[ring.length - 1]) ? ring.slice(0, -1) : ring;

/**
 * Every area, navigation area and obstacle ring of the map as last loaded from
 * (or saved to) map_server, in ROS map-frame metres. These rings were already
 * accepted by the backend, whatever the GUI's own validator says about them.
 */
export function baselineRings(map: MapType | undefined): RosPoint[][] {
    const areas = [...(map?.working_area ?? []), ...(map?.navigation_areas ?? [])];
    return areas
        .flatMap(area => [area.area, ...(area.obstacles ?? [])])
        .flatMap(polygon => {
            const ring = (polygon?.points ?? []).map(p => ({x: p.x ?? 0, y: p.y ?? 0, z: p.z ?? 0}));
            return ring.length > 0 && ring.every(finitePoint) ? [ring] : [];
        });
}

/** The loaded ring this GeoJSON polygon still is, vertex for vertex, if any. */
function findUntouchedRing(raw: unknown, baseline: readonly RosPoint[][], offsetX: number, offsetY: number,
                           datum: [number, number, number]): RosPoint[] | undefined {
    if (!isJsonObject(raw) || raw.type !== 'Polygon' || !Array.isArray(raw.coordinates) || raw.coordinates.length !== 1) return undefined;
    const ring: unknown = raw.coordinates[0];
    if (!Array.isArray(ring)) return undefined;
    let projected: RosPoint[];
    try {
        projected = ring.map((position: unknown) => {
            if (!Array.isArray(position)) throw new MapGeometryError('invalid');
            const [x, y] = projectMapPosition(position as number[], offsetX, offsetY, datum);
            return {x, y, z: 0};
        });
    } catch {
        return undefined;
    }
    const candidate = openRing(projected);
    const match = baseline.find(original => {
        const opened = openRing(original);
        return opened.length === candidate.length && opened.every((p, i) => near(p, candidate[i]));
    });
    return match?.map(p => ({...p}));
}

export interface PreparedPolygon {
    points: RosPoint[];
    /** Invalid ring the operator did not change: sent back exactly as loaded. */
    isPreExistingInvalid: boolean;
}

/**
 * Validate a polygon for the ROS save. A ring that fails the simple-ring check
 * is refused when it was created or edited in this session (it would reach
 * GEOS in the coverage planner), but a ring still identical to the loaded map
 * is passed through unchanged so one old recorded perimeter cannot block every
 * other edit. Unsupported shapes (holes, MultiPolygon) are always refused.
 */
export function prepareSavePolygon(raw: unknown, baseline: readonly RosPoint[][], offsetX: number, offsetY: number,
                                   datum: [number, number, number]): PreparedPolygon {
    try {
        return {points: projectMapPolygon(raw, offsetX, offsetY, datum), isPreExistingInvalid: false};
    } catch (error) {
        if (!(error instanceof MapGeometryError) || error.reason !== 'invalid') throw error;
        const untouched = findUntouchedRing(raw, baseline, offsetX, offsetY, datum);
        if (!untouched) throw error;
        return {points: untouched, isPreExistingInvalid: true};
    }
}
