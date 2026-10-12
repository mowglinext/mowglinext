import {describe, expect, it} from "vitest";
import {mowingAreaIndex, mowingAreaIndexById} from "./mapAreaIndex.ts";

describe("ROS mowing-area IDs", () => {
    const map = {working_area: [{name: "Front"}, {name: "Back"}], working_area_indices: [0, 2]};
    it("preserves IDs across an interleaved navigation area", () => {
        expect(mowingAreaIndex(map, 1)).toBe(2);
    });
    it("never silently defaults an unknown selection to area zero", () => {
        expect(mowingAreaIndex(map, -1)).toBeUndefined();
        expect(mowingAreaIndex(map, 3)).toBeUndefined();
        expect(mowingAreaIndex({working_area: [{}]}, 0)).toBeUndefined();
    });
});

// mowglinext#637 phase 3: resolving by the stable MapArea.id instead of a
// captured array position must not be fooled when the area list has been
// edited (re-added/reordered) since the position was captured.
describe("mowingAreaIndexById", () => {
    const map = {
        working_area: [{name: "Front", id: 501}, {name: "Back", id: 502}],
        working_area_indices: [0, 2],
    };
    it("resolves the current ROS index for a known id", () => {
        expect(mowingAreaIndexById(map, 502)).toBe(2);
        expect(mowingAreaIndexById(map, 501)).toBe(0);
    });
    it("returns undefined for an id no longer present (area deleted)", () => {
        expect(mowingAreaIndexById(map, 999)).toBeUndefined();
    });
    it("returns undefined for an undefined/non-integer id rather than guessing", () => {
        expect(mowingAreaIndexById(map, undefined)).toBeUndefined();
        expect(mowingAreaIndexById(map, 1.5)).toBeUndefined();
    });
    it("tracks an area that moved to a different position, unlike a stale position lookup", () => {
        // Area 502 ("Back") used to sit at position 1 (ROS index 2). The
        // operator edits the map: it is now at position 0 (ROS index 0).
        const reindexed = {
            working_area: [{name: "Back", id: 502}, {name: "Front", id: 501}],
            working_area_indices: [0, 1],
        };
        // A position captured before the edit (1) now resolves to the WRONG
        // area (Front) — this is exactly the bug id-based resolution avoids.
        expect(mowingAreaIndex(reindexed, 1)).toBe(1);
        // Resolving by the stable id instead still finds "Back" correctly.
        expect(mowingAreaIndexById(reindexed, 502)).toBe(0);
    });
});
