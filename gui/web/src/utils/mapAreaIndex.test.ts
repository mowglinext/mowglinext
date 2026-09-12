import {describe, expect, it} from "vitest";
import {mowingAreaIndex} from "./mapAreaIndex.ts";

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
