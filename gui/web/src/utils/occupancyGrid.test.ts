import {describe, expect, it} from "vitest";
import {paintOccupancyGrid} from "./occupancyGrid.ts";
import {mowProgressPaint} from "./mowProgress.ts";
import {lidarMapPaint} from "./lidarMap.ts";
import {OccupancyGrid} from "../types/ros.ts";

function grid(width: number, height: number, data: number[]): OccupancyGrid {
    return {
        info: {width, height, resolution: 0.1, origin: {position: {x: 1, y: 2, z: 0}}},
        data,
    } as unknown as OccupancyGrid;
}

describe("paintOccupancyGrid", () => {
    it("flips rows so grid row 0 (min map Y) lands at the bottom of the image", () => {
        // 1 column, 2 rows: bottom cell mowed, top cell not.
        const painted = paintOccupancyGrid(grid(1, 2, [100, 0]), mowProgressPaint);
        expect(painted).not.toBeNull();
        const px = painted!.pixels;
        expect(px[3]).toBe(0);      // image row 0 (top) = grid row 1 -> transparent
        expect(px[4 + 3]).toBe(150); // image row 1 (bottom) = grid row 0 -> mowed
    });

    it("returns null for an empty or degenerate grid", () => {
        expect(paintOccupancyGrid(grid(0, 3, []), mowProgressPaint)).toBeNull();
        expect(paintOccupancyGrid({} as OccupancyGrid, mowProgressPaint)).toBeNull();
    });

    it("keeps width = X cells and height = Y cells (Invariant 14)", () => {
        const painted = paintOccupancyGrid(grid(3, 2, [0, 0, 0, 0, 0, 0]), mowProgressPaint);
        expect(painted!.width).toBe(3);
        expect(painted!.height).toBe(2);
        expect(painted!.pixels.length).toBe(3 * 2 * 4);
    });
});

describe("mowProgressPaint", () => {
    it("paints only cells at or above 100", () => {
        expect(mowProgressPaint(100)).not.toBeNull();
        expect(mowProgressPaint(99)).toBeNull();
        expect(mowProgressPaint(-1)).toBeNull();
    });
});

describe("lidarMapPaint", () => {
    it("leaves unknown transparent, washes free, inks occupied", () => {
        expect(lidarMapPaint(-1)).toBeNull();
        const free = lidarMapPaint(0)!;
        const occupied = lidarMapPaint(100)!;
        expect(free[3]).toBeLessThan(occupied[3]);
        expect(occupied[3]).toBeGreaterThan(200);
    });

    it("treats the 50 % mark as occupied", () => {
        expect(lidarMapPaint(50)).toEqual(lidarMapPaint(100));
        expect(lidarMapPaint(49)).toEqual(lidarMapPaint(0));
    });
});
