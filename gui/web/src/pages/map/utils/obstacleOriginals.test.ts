import {describe, expect, it} from "vitest";
import {
    findOriginal,
    forgetOriginal,
    MAX_ORIGINALS,
    originalsForObstacles,
    parseOriginals,
    parseStoredOriginals,
    rememberOriginal,
    sameOutline,
    type XY,
} from "./obstacleOriginals.ts";

const square = (x0: number, y0: number, x1: number, y1: number): XY[] =>
    [{x: x0, y: y0}, {x: x1, y: y0}, {x: x1, y: y1}, {x: x0, y: y1}];

const original = square(4, 4, 6, 6);
const shrunk = square(4.3, 4.3, 5.7, 5.7);

describe("obstacleOriginals", () => {
    it("matches an outline within 1 cm and ignores a duplicated closing point", () => {
        const closed = [...shrunk, shrunk[0]];
        const nudged = shrunk.map((p) => ({x: p.x + 0.004, y: p.y - 0.004}));

        expect(sameOutline(shrunk, closed)).toBe(true);
        expect(sameOutline(shrunk, nudged)).toBe(true);
        expect(sameOutline(shrunk, square(4.3, 4.3, 5.75, 5.7))).toBe(false);
        expect(sameOutline(shrunk, shrunk.slice(0, 3))).toBe(false);
    });

    it("finds the original only for the exact shrunk outline", () => {
        const records = rememberOriginal([], {shrunk, original});

        expect(findOriginal(records, [...shrunk, shrunk[0]])).toEqual(original);
        expect(findOriginal(records, square(4.2, 4.2, 5.8, 5.8))).toBeUndefined();
    });

    it("replaces a record for the same shrunk outline instead of piling up", () => {
        const once = rememberOriginal([], {shrunk, original});
        const twice = rememberOriginal(once, {shrunk, original: square(3, 3, 7, 7)});

        expect(twice).toHaveLength(1);
        expect(twice[0].original).toEqual(square(3, 3, 7, 7));
    });

    it("forgets a record and caps the list", () => {
        expect(forgetOriginal(rememberOriginal([], {shrunk, original}), shrunk)).toEqual([]);

        let records = [] as ReturnType<typeof rememberOriginal>;
        for (let i = 0; i < MAX_ORIGINALS + 20; i++) {
            records = rememberOriginal(records, {shrunk: square(i, 0, i + 1, 1), original});
        }
        expect(records).toHaveLength(MAX_ORIGINALS);
    });

    it("keeps only the records that still describe an obstacle of the map", () => {
        const kept = {shrunk, original};
        const stale = {shrunk: square(9, 9, 10, 10), original: square(8, 8, 11, 11)};

        expect(originalsForObstacles([kept, stale], [shrunk])).toEqual([kept]);
    });

    it("parses stored JSON defensively", () => {
        expect(parseStoredOriginals(undefined)).toEqual([]);
        expect(parseStoredOriginals("not json")).toEqual([]);
        expect(parseStoredOriginals("{}")).toEqual([]);
        expect(parseStoredOriginals(JSON.stringify([{shrunk, original}]))).toEqual([{shrunk, original}]);
        // a record with a ring too short, or a non-numeric vertex, is dropped
        expect(parseOriginals([{shrunk: shrunk.slice(0, 2), original}, {shrunk, original: [{x: "a", y: 1}]}])).toEqual([]);
        expect(parseOriginals("nope")).toBeNull();
    });
});
