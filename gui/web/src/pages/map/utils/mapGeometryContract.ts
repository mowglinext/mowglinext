import type {Polygon, Position} from 'geojson';
import {isJsonObject} from '../../../utils/json';
import {dedupePoints, itranspose} from '../../../utils/map';

export class MapGeometryError extends Error {
    constructor(readonly reason: 'unsupported' | 'invalid') {super(reason);}
}

type XY = readonly number[];
const finite = (value: unknown): value is number => typeof value === 'number' && Number.isFinite(value);
const position = (value: unknown): value is Position => Array.isArray(value) && value.length >= 2 && value.every(finite);
const same = (a: XY, b: XY) => a[0] === b[0] && a[1] === b[1];
const cross = (a: XY, b: XY, c: XY) => (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
const between = (a: XY, b: XY, p: XY) => cross(a, b, p) === 0
    && p[0] >= Math.min(a[0], b[0]) && p[0] <= Math.max(a[0], b[0])
    && p[1] >= Math.min(a[1], b[1]) && p[1] <= Math.max(a[1], b[1]);
function intersects(a: XY, b: XY, c: XY, d: XY): boolean {
    const ac = cross(a, b, c), ad = cross(a, b, d), ca = cross(c, d, a), cb = cross(c, d, b);
    return (ac > 0 && ad < 0 || ac < 0 && ad > 0) && (ca > 0 && cb < 0 || ca < 0 && cb > 0)
        || between(a, b, c) || between(a, b, d) || between(c, d, a) || between(c, d, b);
}

/** Accept existing open or closed rings; inspect without changing their points. */
export function assertSimpleRing(ring: readonly XY[]): void {
    const vertices = ring.length > 1 && same(ring[0], ring[ring.length - 1]) ? ring.slice(0, -1) : ring;
    if (vertices.length < 3 || vertices.some(p => p.length < 2 || !p.every(finite))) throw new MapGeometryError('invalid');
    // Translate to the first vertex to avoid cancellation in small geographic rings.
    const origin = vertices[0];
    let area = 0;
    for (let i = 0; i < vertices.length; i++) {
        const a = vertices[i], b = vertices[(i + 1) % vertices.length];
        if (same(a, b)) throw new MapGeometryError('invalid');
        area += (a[0] - origin[0]) * (b[1] - origin[1]) - (b[0] - origin[0]) * (a[1] - origin[1]);
        for (let j = i + 1; j < vertices.length; j++) {
            if (j === i + 1 || i === 0 && j === vertices.length - 1) continue;
            if (intersects(a, b, vertices[j], vertices[(j + 1) % vertices.length])) throw new MapGeometryError('invalid');
        }
    }
    if (!Number.isFinite(area) || area === 0) throw new MapGeometryError('invalid');
}

/** ROS MapArea has one outline plus separately modelled obstacles, not holes. */
export function assertSupportedMapPolygon(raw: unknown): asserts raw is Polygon {
    if (isJsonObject(raw) && (raw.type === 'MultiPolygon'
        || raw.type === 'Polygon' && Array.isArray(raw.coordinates) && raw.coordinates.length > 1)) {
        throw new MapGeometryError('unsupported');
    }
    if (!isJsonObject(raw) || raw.type !== 'Polygon' || !Array.isArray(raw.coordinates) || raw.coordinates.length !== 1) throw new MapGeometryError('invalid');
    const ring: unknown = raw.coordinates[0];
    if (!Array.isArray(ring) || !ring.every(position)
        || ring.some(p => Math.abs(p[0]) > 180 || Math.abs(p[1]) > 90)) throw new MapGeometryError('invalid');
    assertSimpleRing(ring);
}

export function projectMapPosition(coordinates: readonly number[], offsetX: number, offsetY: number, datum: [number, number, number]): [number, number] {
    if (![offsetX, offsetY, ...datum].every(finite) || Math.abs(datum[0]) >= 90 || Math.abs(datum[1]) > 180
        || !position(coordinates) || Math.abs(coordinates[0]) > 180 || Math.abs(coordinates[1]) > 90) throw new MapGeometryError('invalid');
    const result = itranspose(offsetX, offsetY, datum, coordinates[1], coordinates[0]);
    // Same generous local-frame bound as the established OpenMower importer.
    if (!result.every(value => finite(value) && Math.abs(value) <= 100_000)) throw new MapGeometryError('invalid');
    return result;
}

/** Retain the existing projection/deduplication, then validate the actual ROS shape. */
export function projectMapPolygon(raw: unknown, offsetX: number, offsetY: number, datum: [number, number, number]) {
    assertSupportedMapPolygon(raw);
    const points = dedupePoints(raw.coordinates[0].map(position => {
        const [x, y] = projectMapPosition(position, offsetX, offsetY, datum);
        return {x, y, z: 0};
    }));
    assertSimpleRing(points.map(p => [p.x, p.y]));
    // geometry_msgs/Polygon uses float32: quantization must not collapse an edge.
    assertSimpleRing(points.map(p => [Math.fround(p.x), Math.fround(p.y)]));
    return points;
}
