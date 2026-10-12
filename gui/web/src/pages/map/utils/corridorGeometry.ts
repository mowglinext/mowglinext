/// Pure polyline helpers for editing LiDAR-ignore lines (ROS map frame, metres).
/// A "curve" is stored as a dense polyline: the filter and map_server only ever
/// see straight segments, so smoothing just adds vertices along a spline.

export interface XY {
    x: number;
    y: number;
}

/// Above this many vertices the map stops rendering per-vertex handles (a
/// smoothed long line can get dense); use simplifyPolyline to get handles back.
export const MAX_EDITABLE_VERTICES = 80;

/// Hard cap so a smoothing pass can never produce an absurd polyline.
const MAX_SMOOTHED_VERTICES = 200;

const dist = (a: XY, b: XY): number => Math.hypot(b.x - a.x, b.y - a.y);

export const polylineLengthM = (points: XY[]): number => {
    let total = 0;
    for (let i = 0; i + 1 < points.length; i++) total += dist(points[i], points[i + 1]);
    return total;
};

/// Midpoint of segment i (between points[i] and points[i + 1]).
export const segmentMidpoint = (points: XY[], i: number): XY => ({
    x: (points[i].x + points[i + 1].x) / 2,
    y: (points[i].y + points[i + 1].y) / 2,
});

/// Insert a vertex at the midpoint of segment i.
export const insertMidpoint = (points: XY[], i: number): XY[] => {
    if (i < 0 || i + 1 >= points.length) return points;
    return [...points.slice(0, i + 1), segmentMidpoint(points, i), ...points.slice(i + 1)];
};

/// Remove vertex i; a line always keeps at least 2 vertices.
export const removeVertex = (points: XY[], i: number): XY[] => {
    if (points.length <= 2 || i < 0 || i >= points.length) return points;
    return points.filter((_, idx) => idx !== i);
};

/// Centripetal-free uniform Catmull-Rom spline THROUGH the given vertices,
/// sampled roughly every `stepM` metres. The original vertices are kept, so
/// the line still passes exactly through what the operator placed. Endpoints
/// are duplicated as phantom neighbours, so the curve starts and ends on them.
export const smoothPolyline = (points: XY[], stepM = 0.5): XY[] => {
    // Already at/over the cap: smoothing could only add vertices (and the
    // coarser-step fallback below would never terminate).
    if (points.length < 3 || points.length >= MAX_SMOOTHED_VERTICES) return points;
    const p = (i: number): XY => points[Math.min(Math.max(i, 0), points.length - 1)];
    const out: XY[] = [points[0]];
    for (let i = 0; i + 1 < points.length; i++) {
        const p0 = p(i - 1), p1 = p(i), p2 = p(i + 1), p3 = p(i + 2);
        const samples = Math.max(1, Math.ceil(dist(p1, p2) / stepM));
        for (let s = 1; s <= samples; s++) {
            const t = s / samples;
            const t2 = t * t, t3 = t2 * t;
            const cr = (a: number, b: number, c: number, d: number) =>
                0.5 * ((2 * b) + (-a + c) * t + (2 * a - 5 * b + 4 * c - d) * t2 + (-a + 3 * b - 3 * c + d) * t3);
            out.push({
                x: cr(p0.x, p1.x, p2.x, p3.x),
                y: cr(p0.y, p1.y, p2.y, p3.y),
            });
        }
    }
    if (out.length > MAX_SMOOTHED_VERTICES) {
        // Too dense: fall back to a coarser step rather than return a huge line.
        return smoothPolyline(points, stepM * 2);
    }
    return out;
};

const perpendicularDistance = (pt: XY, a: XY, b: XY): number => {
    const dx = b.x - a.x, dy = b.y - a.y;
    const len2 = dx * dx + dy * dy;
    if (len2 < 1e-12) return dist(pt, a);
    const t = Math.min(1, Math.max(0, ((pt.x - a.x) * dx + (pt.y - a.y) * dy) / len2));
    return dist(pt, {x: a.x + t * dx, y: a.y + t * dy});
};

/// Douglas-Peucker: drop vertices closer than `toleranceM` to the simplified
/// line. Endpoints are always kept.
export const simplifyPolyline = (points: XY[], toleranceM = 0.15): XY[] => {
    if (points.length < 3) return points;
    const keep = new Array<boolean>(points.length).fill(false);
    keep[0] = keep[points.length - 1] = true;
    const stack: [number, number][] = [[0, points.length - 1]];
    while (stack.length > 0) {
        const [lo, hi] = stack.pop()!;
        let maxD = 0, idx = -1;
        for (let i = lo + 1; i < hi; i++) {
            const d = perpendicularDistance(points[i], points[lo], points[hi]);
            if (d > maxD) {
                maxD = d;
                idx = i;
            }
        }
        if (idx !== -1 && maxD > toleranceM) {
            keep[idx] = true;
            stack.push([lo, idx], [idx, hi]);
        }
    }
    return points.filter((_, i) => keep[i]);
};

/// A mitred offset (see buildCorridorBandPolygon) can shoot arbitrarily far
/// out at a very sharp bend (the classic "stroke spike" problem) — cap it at
/// this multiple of the half-width, same idea as an SVG/canvas miterLimit.
const MITER_LIMIT = 4;

/// Left-hand perpendicular unit normal of a direction vector.
const leftNormal = (dx: number, dy: number): XY => {
    const len = Math.hypot(dx, dy);
    return len < 1e-9 ? {x: 0, y: 0} : {x: -dy / len, y: dx / len};
};

/// Per-vertex mitred offset on both sides of a polyline, `widthM / 2` out —
/// the shared geometry behind buildCorridorBandPolygon (the WHOLE band) and
/// buildCorridorSideRuns (one side, only where it overlaps a recorded
/// area). `left`/`right` are index-aligned with `points`: left[i]/right[i]
/// are the two offset points of points[i]. See buildCorridorBandPolygon's
/// doc comment for the mitre/degenerate-neighbour rules.
export const buildCorridorSideOffsets = (points: XY[], widthM: number): {left: XY[]; right: XY[]} => {
    if (points.length < 2) return {left: [], right: []};
    const halfW = Math.max(0.01, widthM) / 2;
    const n = points.length;
    const left: XY[] = [];
    const right: XY[] = [];
    for (let i = 0; i < n; i++) {
        const cur = points[i];
        // nIn/nOut come back {0,0} — "no real direction" — both at a true
        // endpoint (no i-1 / no i+1) AND when the neighbouring point is a
        // duplicate of `cur` (a zero-length segment); either way there is
        // nothing to mitre against on that side.
        const nIn = i > 0 ? leftNormal(cur.x - points[i - 1].x, cur.y - points[i - 1].y) : {x: 0, y: 0};
        const nOut = i < n - 1 ? leftNormal(points[i + 1].x - cur.x, points[i + 1].y - cur.y) : {x: 0, y: 0};
        const inLen = Math.hypot(nIn.x, nIn.y);
        const outLen = Math.hypot(nOut.x, nOut.y);
        let offset: XY;
        if (inLen < 1e-9 && outLen < 1e-9) {
            // Neither neighbour has a real direction (an isolated duplicate
            // point) — nothing sensible to offset; collapse both banks onto
            // the vertex itself.
            offset = {x: 0, y: 0};
        } else if (inLen < 1e-9 || outLen < 1e-9) {
            // Exactly one real neighbouring direction — a true endpoint, or
            // an interior vertex whose OTHER neighbour happens to be a
            // duplicate point. There is no bend to mitre: offset straight
            // along the one direction that exists (this is what a 2-point
            // straight line hits at BOTH its vertices, and must reduce to a
            // plain widthM-wide rectangle, not a mitred spike).
            const single = inLen >= 1e-9 ? nIn : nOut;
            offset = {x: single.x * halfW, y: single.y * halfW};
        } else {
            let bx = nIn.x + nOut.x, by = nIn.y + nOut.y;
            const bLen = Math.hypot(bx, by);
            if (bLen < 1e-9) {
                // Near-180° reversal — the bisector direction is undefined;
                // fall back to a plain perpendicular offset along nIn rather
                // than divide by (near) zero.
                offset = {x: nIn.x * halfW, y: nIn.y * halfW};
            } else {
                bx /= bLen;
                by /= bLen;
                // cos(half the turn angle) = dot(bisector, either normal) —
                // the miter scale factor (1/cos) blows up as the turn
                // approaches 180°, so clamp it (MITER_LIMIT) rather than let
                // a hairpin spike out.
                const cosHalfAngle = Math.max(bx * nIn.x + by * nIn.y, 1 / MITER_LIMIT);
                const miter = halfW / cosHalfAngle;
                offset = {x: bx * miter, y: by * miter};
            }
        }
        left.push({x: cur.x + offset.x, y: cur.y + offset.y});
        right.push({x: cur.x - offset.x, y: cur.y - offset.y});
    }
    return {left, right};
};

/// Build the visible "ignore band" for a corridor: ONE continuous polygon
/// (not a quad per segment — see git history for the earlier, blockier
/// version), offset `widthM / 2` to each side of the polyline with a mitred
/// join at every interior vertex so the band flows smoothly through a bend
/// instead of showing per-segment seams. This is a visual aid only, not a
/// proper geometric buffer (no self-intersection repair for a very tight,
/// sharply doubling-back line) — costmap_scan_filter_node's own point-to-
/// segment test remains the ground truth for what's actually suppressed,
/// and (since the area-side restriction) buildCorridorSideRuns below is the
/// one that actually matches what gets suppressed; this whole-band version
/// is kept for the in-progress draft preview, which has no area to check
/// against yet.
/// Returns a single closed ring wrapped in an array (kept as XY[][] so
/// existing callers — one Feature per ring — don't need to change), or []
/// for a degenerate (<2-point) input.
export const buildCorridorBandPolygon = (points: XY[], widthM: number): XY[][] => {
    const {left, right} = buildCorridorSideOffsets(points, widthM);
    if (left.length < 2) return [];
    const ring = [...left, ...right.reverse()];
    ring.push(ring[0]);
    return [ring];
};

/// Build one strip polygon per contiguous run of vertices on `side` whose
/// OFFSET point (not the centerline point) satisfies `insideArea` — the
/// costmap_scan_filter_node area-side restriction (a beam is only ever
/// suppressed when it lands within reach of the line AND inside a recorded
/// area), reflected in the map so the visualization doesn't show ignore
/// coverage the filter doesn't actually grant. A run's strip is the
/// quadrilateral-chain between that stretch of the centerline and its
/// offset — smooth/mitred within the run, since it reuses the same offset
/// vertices as buildCorridorBandPolygon — and the strip naturally breaks
/// (a real gap, not a rendering artefact) wherever the area boundary
/// interrupts the run. A lone included vertex with excluded neighbours on
/// both sides has no segment to fill and is dropped.
export const buildCorridorSideRuns = (
    points: XY[],
    widthM: number,
    side: 'left' | 'right',
    insideArea: (p: XY) => boolean,
): XY[][] => {
    const {left, right} = buildCorridorSideOffsets(points, widthM);
    if (left.length < 2) return [];
    const offsets = side === 'left' ? left : right;
    const strips: XY[][] = [];
    let runStart = -1;
    const flush = (end: number) => {
        if (runStart >= 0 && end - runStart >= 2) {
            const centerRun = points.slice(runStart, end);
            const offsetRun = offsets.slice(runStart, end).reverse();
            const ring = [...centerRun, ...offsetRun];
            ring.push(ring[0]);
            strips.push(ring);
        }
        runStart = -1;
    };
    for (let i = 0; i < points.length; i++) {
        if (insideArea(offsets[i])) {
            if (runStart < 0) runStart = i;
        } else {
            flush(i);
        }
    }
    flush(points.length);
    return strips;
};

/// gl-draw's `draw_line_string` keeps one extra, LIVE vertex at the end of the
/// feature: the position the cursor was last at (it follows every mouse-move and
/// is only committed by a click). Reading the feature back for the Finish
/// button therefore returns the placed points PLUS the spot the pointer left the
/// map on its way to the button — a stray point right under "Finish" that the
/// operator then had to delete by hand. gl-draw's own finish gestures drop it
/// (`removeCoordinate(currentVertexPosition)`); a button that reads the feature
/// itself must do the same.
export const dropLiveVertex = <T>(coords: T[]): T[] => coords.slice(0, -1);
