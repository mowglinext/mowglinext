import {describe, expect, it} from "vitest";
import {
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
});
