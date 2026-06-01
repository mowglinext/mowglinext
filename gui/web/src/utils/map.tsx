import {Quaternion} from "../types/ros.ts";
import type {RobotGeometry} from "../hooks/useRobotDescription.ts";

export const earth = 6371008.8;  // radius of the earth in meters
export const pi = Math.PI;

// Metres per degree of latitude (WGS84 equatorial radius × deg→rad)
const METERS_PER_DEG = 6378137.0 * Math.PI / 180.0;

export function getQuaternionFromHeading(heading: number): Quaternion {
    const q = {
        x: 0,
        y: 0,
        z: 0,
        w: 0,
    } as Quaternion
    q.w = Math.cos(heading / 2)
    q.z = Math.sin(heading / 2)
    return q
}

export function drawLine(offsetX: number, offsetY: number, datum: [number, number, number], y: number, x: number, orientation: number): [number, number] {
    const endX = x + Math.cos(orientation);
    const endY = y + Math.sin(orientation);
    return transpose(offsetX, offsetY, datum, endY, endX);
}

/**
 * Generate a rotated robot footprint polygon in [lon, lat] coordinates.
 * The footprint is a rectangle defined by front/rear/halfWidth offsets from
 * base_link (at wheel axis centre), rotated by the robot heading.
 */
export function drawRobotFootprint(
    offsetX: number, offsetY: number, datum: [number, number, number],
    posY: number, posX: number, heading: number,
    chassisFront: number, chassisRear: number, halfWidth: number,
): [number, number][] {
    const cos = Math.cos(heading);
    const sin = Math.sin(heading);
    // Corners in local frame relative to base_link: [forward, left]
    const corners = [
        [chassisFront, halfWidth],
        [chassisFront, -halfWidth],
        [chassisRear, -halfWidth],
        [chassisRear, halfWidth],
        [chassisFront, halfWidth], // close the polygon
    ];
    return corners.map(([fx, fy]) => {
        const rx = posX + fx * cos - fy * sin;
        const ry = posY + fx * sin + fy * cos;
        return transpose(offsetX, offsetY, datum, ry, rx);
    });
}

/**
 * Robot silhouette polygons in [lon, lat], derived from the URDF geometry
 * (/robot_description) so the map robot matches the sensors-page model exactly
 * — chassis box + the two drive wheels + the blade disc — instead of a plain
 * settings-sized rectangle. base_link is at the rear wheel axis, so the chassis
 * is offset forward by chassisCenterX; wheels sit at wheelXOffset / ±wheelTrack/2.
 * All points are in the local base_link frame, rotated by heading and projected.
 */
export function drawRobotSilhouette(
    offsetX: number, offsetY: number, datum: [number, number, number],
    posY: number, posX: number, heading: number, g: RobotGeometry,
): { chassis: [number, number][]; wheelL: [number, number][]; wheelR: [number, number][]; blade: [number, number][] } {
    const cos = Math.cos(heading);
    const sin = Math.sin(heading);
    // Local (forward x, left y) -> map [lon, lat].
    const pt = (fx: number, fy: number): [number, number] => {
        const rx = posX + fx * cos - fy * sin;
        const ry = posY + fx * sin + fy * cos;
        return transpose(offsetX, offsetY, datum, ry, rx);
    };
    // Axis-aligned (in robot frame) rectangle centred at (xc, yc), half-extents hx/hy.
    const rect = (xc: number, yc: number, hx: number, hy: number): [number, number][] => [
        pt(xc - hx, yc - hy),
        pt(xc + hx, yc - hy),
        pt(xc + hx, yc + hy),
        pt(xc - hx, yc + hy),
        pt(xc - hx, yc - hy),
    ];

    const chassis = rect(g.chassisCenterX, 0, g.baseLength / 2, g.baseWidth / 2);
    const wheelL = rect(g.wheelXOffset, g.wheelTrack / 2, g.wheelRadius, g.wheelWidth / 2);
    const wheelR = rect(g.wheelXOffset, -g.wheelTrack / 2, g.wheelRadius, g.wheelWidth / 2);

    const blade: [number, number][] = [];
    const N = 20;
    for (let i = 0; i <= N; i++) {
        const a = (2 * Math.PI * i) / N;
        blade.push(pt(g.chassisCenterX + g.bladeRadius * Math.cos(a), g.bladeRadius * Math.sin(a)));
    }

    return { chassis, wheelL, wheelR, blade };
}

/**
 * Convert local map coordinates (x=east, y=north in metres relative to datum)
 * to [longitude, latitude], with an optional metric display offset.
 *
 * `offsetX` / `offsetY` are added to the metric ROS-frame coordinates before
 * the equirectangular projection — this is the user-tunable "Map Offset"
 * panel on the map page, used to nudge the displayed map relative to the
 * satellite/tile basemap when the published datum is slightly off. The
 * offset is purely visual and does not feed back into ROS.
 *
 *   east  = (lon - datum_lon) * cos(datum_lat) * METERS_PER_DEG
 *   north = (lat - datum_lat) * METERS_PER_DEG
 *
 * datum is passed as [datum_lat, datum_lon, _].
 */
export const transpose = (offsetX: number, offsetY: number, datum: [number, number, number], y: number, x: number): [number, number] => {
    const datum_lat = datum[0];
    const datum_lon = datum[1];
    const cos_lat = Math.cos(datum_lat * Math.PI / 180.0);

    const lon = datum_lon + (x + offsetX) / (cos_lat * METERS_PER_DEG);
    const lat = datum_lat + (y + offsetY) / METERS_PER_DEG;
    return [lon, lat];
};

/**
 * Convert [longitude, latitude] to local map coordinates (x=east, y=north).
 * Inverse of transpose — strips the same metric display offset before
 * returning the underlying ROS-frame coordinate.
 */
export const itranspose = (offsetX: number, offsetY: number, datum: [number, number, number], lat: number, lon: number): [number, number] => {
    const datum_lat = datum[0];
    const datum_lon = datum[1];
    const cos_lat = Math.cos(datum_lat * Math.PI / 180.0);

    const x = (lon - datum_lon) * cos_lat * METERS_PER_DEG - offsetX;
    const y = (lat - datum_lat) * METERS_PER_DEG - offsetY;
    return [x, y];
};

/**
 * Remove near-duplicate consecutive points caused by floating-point precision.
 */
export function dedupePoints(points: { x: number; y: number; z: number }[], epsilon = 0.001): { x: number; y: number; z: number }[] {
    if (points.length === 0) return points;
    const result = [points[0]];
    for (let i = 1; i < points.length; i++) {
        const prev = result[result.length - 1];
        const curr = points[i];
        const dx = curr.x - prev.x;
        const dy = curr.y - prev.y;
        if (Math.sqrt(dx * dx + dy * dy) > epsilon) {
            result.push(curr);
        }
    }
    return result;
}
