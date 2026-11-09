import type {Feature, FeatureCollection, LineString, Point, Polygon} from "geojson";
import {itranspose, transpose} from "../../utils/map.tsx";

/// Pure helpers behind the Map page's "mowing lines" overlay. The geometry
/// itself always comes from coverage_server's preview_coverage service (the
/// real planner, never reimplemented here); this file only converts between the
/// map frame (x east, y north, metres) and the display frame, and decides where
/// the direction arrows go.

export type RosPoint = {x?: number; y?: number};
export type RosPolygon = {points?: RosPoint[]};

/** Perimeter winding, as coverage_server's ring_direction parameter. */
export const RING_DIRECTION = {planner: 0, clockwise: 1, counterClockwise: 2} as const;
export type RingDirection = typeof RING_DIRECTION[keyof typeof RING_DIRECTION];

/** mow_angle_deg < 0 means "auto" (the planner picks the heading). */
export const MOW_ANGLE_AUTO = -1;

/**
 * What one area does about its swath angle: follow the robot-wide setting, pin
 * itself to auto, or use its own fixed angle. The wire form (MapArea) cannot
 * say "global + a stored number", so this is the model the panel works with.
 */
export type AngleMode = "global" | "auto" | "fixed";
/** The perimeter winding of one area: the robot-wide setting, or its own 0/1/2. */
export type DirectionChoice = "global" | RingDirection;

/** A point in the map frame (x east, y north, metres). */
export interface MapPoint {
    x: number;
    y: number;
}

export interface AreaChoices {
    angleMode: AngleMode;
    /** Meaningful when angleMode is "fixed"; the last fixed value otherwise. */
    angleDeg: number;
    direction: DirectionChoice;
    /**
     * Where the route starts: a point the planner snaps onto the OUTERMOST headland ring.
     * null = the planner's own start.
     */
    start: MapPoint | null;
}

/** The MapArea fields that carry an area's overrides (generated type, all optional). */
export interface AreaOverrideFields {
    has_mow_angle?: boolean;
    mow_angle_deg?: number;
    has_ring_direction?: boolean;
    ring_direction?: number;
    has_start_point?: boolean;
    start_x?: number;
    start_y?: number;
}

const asRingDirection = (v: number | undefined): RingDirection =>
    v === 1 || v === 2 ? v : RING_DIRECTION.planner;

/** Read an area's overrides into the panel's model. A negative stored angle is auto. */
export const choicesFromArea = (area: AreaOverrideFields | undefined): AreaChoices => {
    const angle = area?.mow_angle_deg ?? 0;
    return {
        angleMode: !area?.has_mow_angle ? "global" : angle < 0 ? "auto" : "fixed",
        angleDeg: area?.has_mow_angle && angle >= 0 ? angle : 0,
        direction: area?.has_ring_direction ? asRingDirection(area.ring_direction) : "global",
        start: area?.has_start_point ? {x: area.start_x ?? 0, y: area.start_y ?? 0} : null,
    };
};

/** The angle and winding to ASK the planner for: an area follows the robot-wide value unless it overrides. */
export const requestedValues = (
    choices: AreaChoices,
    globalAngleDeg: number,
    globalDirection: number,
): {mow_angle_deg: number; ring_direction: RingDirection; start: MapPoint | null} => ({
    mow_angle_deg: choices.angleMode === "global"
        ? globalAngleDeg
        : choices.angleMode === "auto" ? MOW_ANGLE_AUTO : choices.angleDeg,
    ring_direction: choices.direction === "global" ? asRingDirection(globalDirection) : choices.direction,
    // The start point has no robot-wide counterpart: it is per area or the planner's own.
    start: choices.start,
});

/** The set_area_coverage_lines body: a flag per value, so "global" clears an override. */
export const overridesFromChoices = (choices: AreaChoices): Required<AreaOverrideFields> => ({
    has_mow_angle: choices.angleMode !== "global",
    mow_angle_deg: choices.angleMode === "global" ? 0 : choices.angleMode === "auto" ? MOW_ANGLE_AUTO : choices.angleDeg,
    has_ring_direction: choices.direction !== "global",
    ring_direction: choices.direction === "global" ? 0 : choices.direction,
    has_start_point: choices.start !== null,
    start_x: choices.start?.x ?? 0,
    start_y: choices.start?.y ?? 0,
});

const sameStart = (a: MapPoint | null, b: MapPoint | null): boolean =>
    a === null || b === null ? a === b : Math.abs(a.x - b.x) < 1e-6 && Math.abs(a.y - b.y) < 1e-6;

/** Two choice sets mean the same thing. A stored angle only counts while the mode is "fixed". */
export const sameChoices = (a: AreaChoices, b: AreaChoices): boolean =>
    a.angleMode === b.angleMode
    && a.direction === b.direction
    && sameStart(a.start, b.start)
    && (a.angleMode !== "fixed" || a.angleDeg === b.angleDeg);

export interface CoveragePreviewResult {
    success?: boolean;
    message?: string;
    rings?: RosPolygon[];
    swaths?: RosPolygon[];
    mow_angle_deg?: number;
    headland_passes?: number;
    ring_direction?: number;
    planned_fraction?: number;
    field_area_m2?: number;
    dropped_pieces?: number;
    /** A headland ring exists, so a start point can be placed (false with the rings off). */
    start_adjustable?: boolean;
    /** Where the route really starts: the operator's start point snapped onto the outer ring. */
    start_x?: number;
    start_y?: number;
}

export interface CoveragePreviewLayers {
    /** Rings and swaths as LineStrings; `kind` is "ring" | "swath". */
    lines: FeatureCollection;
    /**
     * Direction arrowheads, small triangles drawn to scale in metres so they
     * need no font or sprite. `bearing` is degrees clockwise from north.
     */
    arrows: FeatureCollection;
    /** [lon, lat] of the route's start, for the draggable marker; null when there is none. */
    startLonLat: [number, number] | null;
    /** The outermost headland ring as a closed [lon, lat] line, for snapping the start marker to it. */
    outerRingLonLat: [number, number][] | null;
}

const EMPTY: FeatureCollection = {type: "FeatureCollection", features: []};
export const emptyLayers = (): CoveragePreviewLayers => ({
    lines: EMPTY, arrows: EMPTY, startLonLat: null, outerRingLonLat: null,
});

const METERS_PER_DEG = 111319.49;

/**
 * The point on `line` ([lon, lat] vertices) nearest to `point`, measured in metres (longitude
 * scaled by cos(lat)). Used to slide the start marker along the outer ring while it is dragged.
 * An empty line returns the point unchanged.
 */
export const nearestOnPolyline = (
    line: readonly (readonly [number, number])[],
    point: readonly [number, number],
): [number, number] => {
    if (line.length === 0) return [point[0], point[1]];
    if (line.length === 1) return [line[0][0], line[0][1]];
    const kx = Math.cos((point[1] * Math.PI) / 180) * METERS_PER_DEG;
    const ky = METERS_PER_DEG;
    let best: [number, number] = [line[0][0], line[0][1]];
    let bestD = Infinity;
    for (let i = 0; i + 1 < line.length; i++) {
        const ax = line[i][0] * kx;
        const ay = line[i][1] * ky;
        const dx = line[i + 1][0] * kx - ax;
        const dy = line[i + 1][1] * ky - ay;
        const len2 = dx * dx + dy * dy;
        const t = len2 < 1e-12 ? 0 : Math.max(0, Math.min(1, ((point[0] * kx - ax) * dx + (point[1] * ky - ay) * dy) / len2));
        const cx = line[i][0] + t * (line[i + 1][0] - line[i][0]);
        const cy = line[i][1] + t * (line[i + 1][1] - line[i][1]);
        const d = Math.hypot((point[0] - cx) * kx, (point[1] - cy) * ky);
        if (d < bestD) {
            bestD = d;
            best = [cx, cy];
        }
    }
    return best;
};

/** Keep roughly this many metres between arrows along a ring. */
export const RING_ARROW_SPACING_M = 8;
/** Never draw more than about this many swath arrows, however many swaths there are. */
export const MAX_SWATH_ARROWS = 12;
/** Arrowhead length as a fraction of the preview's extent, clamped to a readable size. */
const ARROW_EXTENT_FRACTION = 0.035;
const ARROW_MIN_M = 0.35;
/** Where along a swath successive arrows sit (fraction of its length), cycling. */
const SWATH_ARROW_POSITIONS = [0.25, 0.5, 0.75];
const ARROW_MAX_M = 2.0;

const bearingDeg = (dx: number, dy: number): number => {
    const deg = (Math.atan2(dx, dy) * 180) / Math.PI;
    return (deg + 360) % 360;
};

type XY = {x: number; y: number};

/** A triangle of length `size` metres centred on `mid`, pointing along `bearing`. */
const arrowhead = (mid: XY, bearing: number, size: number): XY[] => {
    const rad = (bearing * Math.PI) / 180;
    const dx = Math.sin(rad); // east
    const dy = Math.cos(rad); // north
    const px = -dy;
    const py = dx;
    const tip = {x: mid.x + dx * size * 0.6, y: mid.y + dy * size * 0.6};
    const baseX = mid.x - dx * size * 0.4;
    const baseY = mid.y - dy * size * 0.4;
    const half = size * 0.3;
    return [tip, {x: baseX + px * half, y: baseY + py * half}, {x: baseX - px * half, y: baseY - py * half}];
};

const usable = (poly: RosPolygon | undefined): {x: number; y: number}[] =>
    (poly?.points ?? []).map((p) => ({x: p.x ?? 0, y: p.y ?? 0}));

/**
 * Convert a drawn area (GeoJSON [lon, lat] ring, closed or not) into the map
 * frame polygon the service takes. The closing duplicate is dropped: ROS
 * polygons list each vertex once.
 */
export const ringToRosPolygon = (
    ring: readonly (readonly number[])[],
    datum: [number, number, number],
    offsetX: number,
    offsetY: number,
): {points: {x: number; y: number}[]} => {
    const pts = ring.map((coord) => {
        const [x, y] = itranspose(offsetX, offsetY, datum, coord[1], coord[0]);
        return {x, y};
    });
    if (pts.length > 1) {
        const first = pts[0];
        const last = pts[pts.length - 1];
        if (Math.hypot(first.x - last.x, first.y - last.y) < 1e-6) pts.pop();
    }
    return {points: pts};
};

/**
 * Where a rotation of the swaths should be reported from the heading the user
 * chose. The service resolves "auto" to a concrete angle; this is the angle to
 * show and to seed the slider with.
 */
export const effectiveAngleDeg = (requested: number, resolved: number | undefined): number => {
    if (requested >= 0) return requested;
    return resolved ?? 0;
};

/**
 * Turn a preview answer into map layers. Arrows sit on every ring (one per
 * RING_ARROW_SPACING_M of path, at least one) and on a thinned selection of
 * swaths, so the serpentine and the perimeter winding both read at a glance
 * without 100 arrows on a big lawn. `startLonLat` is where the robot begins.
 */
export const buildPreviewLayers = (
    res: CoveragePreviewResult | undefined,
    datum: [number, number, number],
    offsetX: number,
    offsetY: number,
): CoveragePreviewLayers => {
    if (!res?.success) return emptyLayers();
    const toLonLat = (p: {x: number; y: number}): [number, number] =>
        transpose(offsetX, offsetY, datum, p.y, p.x);

    const lines: Feature<LineString>[] = [];
    const arrows: Feature<Point | Polygon>[] = [];

    // Arrowhead size follows the extent of what is drawn: a fixed size is a speck
    // on a big lawn and a blob on a small one.
    let minX = Infinity;
    let minY = Infinity;
    let maxX = -Infinity;
    let maxY = -Infinity;
    for (const poly of [...(res.rings ?? []), ...(res.swaths ?? [])]) {
        for (const p of usable(poly)) {
            minX = Math.min(minX, p.x);
            maxX = Math.max(maxX, p.x);
            minY = Math.min(minY, p.y);
            maxY = Math.max(maxY, p.y);
        }
    }
    const extent = Number.isFinite(minX) ? Math.max(maxX - minX, maxY - minY) : 0;
    const arrowSize = Math.min(ARROW_MAX_M, Math.max(ARROW_MIN_M, extent * ARROW_EXTENT_FRACTION));
    const arrow = (kind: string, mid: XY, bearing: number): Feature<Polygon> => {
        const ring = arrowhead(mid, bearing, arrowSize).map(toLonLat);
        return {
            type: "Feature",
            properties: {kind, bearing},
            geometry: {type: "Polygon", coordinates: [[...ring, ring[0]]]},
        };
    };

    (res.rings ?? []).forEach((poly, ringIndex) => {
        const pts = usable(poly);
        if (pts.length < 2) return;
        // A ring is a closed loop: draw the closing edge too.
        const loop = [...pts, pts[0]];
        lines.push({
            type: "Feature",
            properties: {kind: "ring", index: ringIndex},
            geometry: {type: "LineString", coordinates: loop.map(toLonLat)},
        });

        let sinceArrow = RING_ARROW_SPACING_M; // an arrow on the first long-enough edge
        let placed = 0;
        for (let i = 0; i + 1 < loop.length; i++) {
            const a = loop[i];
            const b = loop[i + 1];
            const len = Math.hypot(b.x - a.x, b.y - a.y);
            if (len < 0.3) continue;
            sinceArrow += len;
            if (sinceArrow >= RING_ARROW_SPACING_M) {
                sinceArrow = 0;
                placed++;
                arrows.push(arrow("ring-arrow", {x: (a.x + b.x) / 2, y: (a.y + b.y) / 2}, bearingDeg(b.x - a.x, b.y - a.y)));
            }
        }
        if (placed === 0 && loop.length >= 2) {
            const a = loop[0];
            const b = loop[1];
            arrows.push(arrow("ring-arrow", {x: (a.x + b.x) / 2, y: (a.y + b.y) / 2}, bearingDeg(b.x - a.x, b.y - a.y)));
        }
    });

    const swaths = res.swaths ?? [];
    const step = Math.max(1, Math.ceil(swaths.length / MAX_SWATH_ARROWS));
    swaths.forEach((poly, swathIndex) => {
        const pts = usable(poly);
        if (pts.length < 2) return;
        const a = pts[0];
        const b = pts[pts.length - 1];
        lines.push({
            type: "Feature",
            properties: {kind: "swath", index: swathIndex},
            geometry: {type: "LineString", coordinates: [toLonLat(a), toLonLat(b)]},
        });
        if (swathIndex % step === 0) {
            // Stagger along the swath: arrows all at the midpoint line up in one
            // row across the lawn and overlap each other.
            const t = SWATH_ARROW_POSITIONS[(swathIndex / step) % SWATH_ARROW_POSITIONS.length];
            arrows.push(arrow("swath-arrow",
                {x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t},
                bearingDeg(b.x - a.x, b.y - a.y)));
        }
    });

    // Where the route really starts: the planner reports it (the operator's start point
    // snapped onto the outer ring); older answers fall back to the first ring point, or
    // with no ring the first swath start.
    const firstRing = usable((res.rings ?? [])[0]);
    const firstSwath = usable((res.swaths ?? [])[0]);
    const start: XY | undefined = Number.isFinite(res.start_x) && Number.isFinite(res.start_y)
        ? {x: res.start_x as number, y: res.start_y as number}
        : firstRing[0] ?? firstSwath[0];

    const outerRing = lines.find((f) => f.properties?.kind === "ring" && f.properties?.index === 0);

    return {
        lines: {type: "FeatureCollection", features: lines},
        arrows: {type: "FeatureCollection", features: arrows},
        startLonLat: start ? toLonLat(start) : null,
        outerRingLonLat: outerRing ? (outerRing.geometry.coordinates as [number, number][]) : null,
    };
};
