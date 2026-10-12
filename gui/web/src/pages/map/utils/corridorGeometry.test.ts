import {describe, expect, it} from "vitest";
import {
    buildCorridorBandPolygon,
    buildCorridorSideRuns,
    dropLiveVertex,
    insertMidpoint,
    polylineLengthM,
    removeVertex,
    simplifyPolyline,
    smoothPolyline,
} from "./corridorGeometry.ts";

describe("corridorGeometry", () => {
    it("measures polyline length", () => {
        expect(polylineLengthM([{x: 0, y: 0}, {x: 3, y: 4}, {x: 3, y: 10}])).toBeCloseTo(11);
        expect(polylineLengthM([{x: 1, y: 1}])).toBe(0);
    });

    it("inserts a vertex at a segment midpoint", () => {
        const out = insertMidpoint([{x: 0, y: 0}, {x: 4, y: 0}], 0);
        expect(out).toEqual([{x: 0, y: 0}, {x: 2, y: 0}, {x: 4, y: 0}]);
        expect(insertMidpoint([{x: 0, y: 0}, {x: 4, y: 0}], 5)).toHaveLength(2);
    });

    it("never removes a vertex below two points", () => {
        const two = [{x: 0, y: 0}, {x: 1, y: 0}];
        expect(removeVertex(two, 0)).toEqual(two);
        expect(removeVertex([{x: 0, y: 0}, {x: 1, y: 0}, {x: 2, y: 0}], 1)).toHaveLength(2);
    });

    it("smoothing passes through the original vertices and keeps endpoints", () => {
        const pts = [{x: 0, y: 0}, {x: 4, y: 3}, {x: 8, y: 0}];
        const out = smoothPolyline(pts, 0.5);
        expect(out.length).toBeGreaterThan(pts.length);
        expect(out[0]).toEqual(pts[0]);
        const last = out[out.length - 1];
        expect(last.x).toBeCloseTo(8);
        expect(last.y).toBeCloseTo(0);
        // The middle control point is still on the curve.
        expect(out.some((p) => Math.abs(p.x - 4) < 1e-6 && Math.abs(p.y - 3) < 1e-6)).toBe(true);
    });

    it("smoothing leaves a two-point line alone and stays bounded", () => {
        const two = [{x: 0, y: 0}, {x: 10, y: 0}];
        expect(smoothPolyline(two)).toEqual(two);
        const long = Array.from({length: 40}, (_, i) => ({x: i * 5, y: (i % 2) * 2}));
        expect(smoothPolyline(long, 0.1).length).toBeLessThanOrEqual(200);
    });

    it("simplify drops collinear vertices but keeps endpoints and real corners", () => {
        const line = [{x: 0, y: 0}, {x: 1, y: 0}, {x: 2, y: 0}, {x: 3, y: 0}];
        expect(simplifyPolyline(line, 0.1)).toEqual([{x: 0, y: 0}, {x: 3, y: 0}]);
        const corner = [{x: 0, y: 0}, {x: 5, y: 0}, {x: 5, y: 5}];
        expect(simplifyPolyline(corner, 0.1)).toEqual(corner);
    });

    it("bands a straight segment as a widthM-wide rectangle, closed", () => {
        const bands = buildCorridorBandPolygon([{x: 0, y: 0}, {x: 10, y: 0}], 1.0);
        expect(bands).toHaveLength(1);
        const [ring] = bands;
        expect(ring[0]).toEqual(ring[ring.length - 1]); // closed
        expect(ring.every((p) => Math.abs(p.y) <= 0.5 + 1e-9)).toBe(true);
        expect(ring.some((p) => Math.abs(p.y - 0.5) < 1e-9)).toBe(true);
        expect(ring.some((p) => Math.abs(p.y + 0.5) < 1e-9)).toBe(true);
    });

    it("bands a bent polyline as ONE continuous, mitred ring — not one piece per segment", () => {
        const bands = buildCorridorBandPolygon([{x: 0, y: 0}, {x: 5, y: 0}, {x: 5, y: 5}], 0.4);
        expect(bands).toHaveLength(1);
        const [ring] = bands;
        // 3 input vertices -> 3 left-bank + 3 right-bank points, ring-closed.
        expect(ring).toHaveLength(3 + 3 + 1);
        // The outer corner (mitred) sits further than half-width from the
        // bend vertex (5, 0) — that's the whole point of mitring instead of
        // leaving a gap/overlap between two independent quads.
        const outerCorner = ring.find((p) => p.x > 5.05 && p.y < -0.05);
        expect(outerCorner).toBeDefined();
    });

    it("does not choke on a degenerate (repeated) point", () => {
        const bands = buildCorridorBandPolygon([{x: 0, y: 0}, {x: 0, y: 0}, {x: 5, y: 0}], 0.4);
        expect(bands).toHaveLength(1);
        expect(bands[0].every((p) => Number.isFinite(p.x) && Number.isFinite(p.y))).toBe(true);
    });

    it("clamps a zero or negative width to a thin sliver rather than collapsing", () => {
        const bands = buildCorridorBandPolygon([{x: 0, y: 0}, {x: 1, y: 0}], 0);
        expect(bands[0].some((p) => p.y !== 0)).toBe(true);
    });

    it("returns nothing for a single point (no line to band)", () => {
        expect(buildCorridorBandPolygon([{x: 0, y: 0}], 0.5)).toEqual([]);
    });

    it("side runs: a side fully inside the area gives one strip spanning the whole line", () => {
        const points = [{x: 0, y: 0}, {x: 10, y: 0}];
        const strips = buildCorridorSideRuns(points, 1.0, 'left', () => true);
        expect(strips).toHaveLength(1);
        // Strip = the 2 centerline points + the 2 (reversed) offset points, closed.
        expect(strips[0]).toHaveLength(2 + 2 + 1);
    });

    it("side runs: a side fully outside the area gives no strips", () => {
        const points = [{x: 0, y: 0}, {x: 10, y: 0}];
        const strips = buildCorridorSideRuns(points, 1.0, 'left', () => false);
        expect(strips).toEqual([]);
    });

    it("side runs: only the side whose offset the predicate accepts produces strips", () => {
        // A predicate that's only true for negative-y offset points — i.e.
        // only the RIGHT bank of a line running along y=0 (right = -y here,
        // since right.push({..., y: cur.y - offset.y}) and offset.y is
        // positive for the left-hand normal of a line pointing +x).
        const points = [{x: 0, y: 0}, {x: 10, y: 0}];
        const insideArea = (p: {x: number; y: number}) => p.y < 0;
        expect(buildCorridorSideRuns(points, 1.0, 'left', insideArea)).toEqual([]);
        expect(buildCorridorSideRuns(points, 1.0, 'right', insideArea)).toHaveLength(1);
    });

    it("side runs: breaks into separate strips where the area is interrupted", () => {
        // 4 points along a line; only the middle two are "inside" -> one
        // strip covering just that middle segment, not the whole line.
        // The predicate is called once per index, in index order, so an
        // explicit allow-list keyed by call count reproduces "only indices
        // 1 and 2 are inside" without having to predict exact mitred
        // offset coordinates.
        const points = [{x: 0, y: 0}, {x: 1, y: 0}, {x: 2, y: 0}, {x: 3, y: 0}];
        const insideByIndex = [false, true, true, false];
        let callCount = 0;
        const strips = buildCorridorSideRuns(points, 0.4, 'left', () => insideByIndex[callCount++]);
        expect(strips).toHaveLength(1);
        // The one strip covers exactly the 2-point middle run: 2 centerline
        // + 2 offset + closing point.
        expect(strips[0]).toHaveLength(2 + 2 + 1);
    });

    it("side runs: a lone included vertex between two excluded ones is dropped", () => {
        const points = [{x: 0, y: 0}, {x: 1, y: 0}, {x: 2, y: 0}];
        const allow = [false, true, false];
        let i = -1;
        const strips = buildCorridorSideRuns(points, 0.4, 'left', () => {
            i++;
            return allow[i];
        });
        expect(strips).toEqual([]);
    });
});

describe("dropLiveVertex", () => {
    it("removes only the trailing cursor vertex", () => {
        expect(dropLiveVertex([[0, 0], [1, 1], [2, 2]])).toEqual([[0, 0], [1, 1]]);
    });

    it("leaves a line with only the live vertex empty, so Finish saves nothing", () => {
        expect(dropLiveVertex([[5, 5]])).toEqual([]);
        expect(dropLiveVertex([])).toEqual([]);
    });
});
