import {describe, expect, it} from "vitest";
import type {Map as MapType, MapArea, Point32} from "../../../types/ros.ts";
import {calculateMapViewportBounds} from "./mapViewport.ts";

const area = (...points: Point32[]): MapArea => ({area: {points}});

describe("calculateMapViewportBounds", () => {
    it("fits all mowing and navigation areas instead of the fixed map metadata", () => {
        const map: MapType = {
            map_width: 20,
            map_height: 20,
            map_center_x: 0,
            map_center_y: 0,
            working_area: [area({x: 40, y: -5}, {x: 70, y: 15})],
            navigation_areas: [area({x: -30, y: 8}, {x: -10, y: 25})],
        };

        expect(calculateMapViewportBounds(map, 0)).toEqual({
            minX: -30,
            minY: -5,
            maxX: 70,
            maxY: 25,
        });
    });

    it("expands the fitted extents for a saved rotation", () => {
        const map: MapType = {
            working_area: [area({x: -20, y: -5}, {x: 20, y: 5})],
        };

        const quarterTurn = calculateMapViewportBounds(map, 90);
        expect(quarterTurn.minX).toBeCloseTo(-5);
        expect(quarterTurn.minY).toBeCloseTo(-20);
        expect(quarterTurn.maxX).toBeCloseTo(5);
        expect(quarterTurn.maxY).toBeCloseTo(20);

        const diagonal = calculateMapViewportBounds(map, 45);
        expect(diagonal.maxX - diagonal.minX).toBeCloseTo(35.355, 3);
        expect(diagonal.maxY - diagonal.minY).toBeCloseTo(35.355, 3);
    });

    it("ignores incomplete coordinates without losing valid area points", () => {
        const map: MapType = {
            working_area: [area({x: undefined, y: 10}, {x: 2, y: 3}, {x: 8, y: 9})],
        };

        expect(calculateMapViewportBounds(map, 0)).toEqual({
            minX: 2,
            minY: 3,
            maxX: 8,
            maxY: 9,
        });
    });

    it("keeps the datum-centred metadata fallback when no areas exist", () => {
        const map: MapType = {
            map_width: 20,
            map_height: 30,
            map_center_x: 4,
            map_center_y: -6,
            working_area: [],
            navigation_areas: [],
        };

        expect(calculateMapViewportBounds(map, 35)).toEqual({
            minX: -6,
            minY: -21,
            maxX: 14,
            maxY: 9,
        });
    });
});
