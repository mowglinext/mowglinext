import {describe, expect, it} from "vitest";
import {
    calculateMapImageSizePx,
    calculateMapImageDimensionsPx,
    getMapImageAnchorOffsetPx,
    getMowerHeadingRad,
    hasValidMapPosition,
    offsetMapCoordinatesForward,
    rosHeadingToMapboxRotation,
} from "./mapImageMarkerMath.ts";

const radians = Math.PI / 180;

function mercatorProject(scale: number) {
    return ([longitude, latitude]: [number, number]) => ({
        x: scale * (longitude + 180) / 360,
        y: scale * (1 - Math.log(Math.tan(latitude * radians) + 1 / Math.cos(latitude * radians)) / Math.PI) / 2,
    });
}

describe("map image marker geometry", () => {
    it("passes the existing ROS heading line through all four cardinal directions", () => {
        const identity = (x: number, y: number): readonly [number, number] => [x, y];
        for (const [end, expectedYaw, expectedRotation] of [
            [[1, 0], 0, 90],
            [[0, 1], Math.PI / 2, 0],
            [[-1, 0], Math.PI, -90],
            [[0, -1], -Math.PI / 2, 180],
        ] as const) {
            const yaw = getMowerHeadingRad([[0, 0], end], identity);
            expect(yaw).toBeCloseTo(expectedYaw);
            expect(rosHeadingToMapboxRotation(yaw!)).toBeCloseTo(expectedRotation);
        }
    });

    it("rejects absent, degenerate, non-finite heading and invalid mower positions", () => {
        const identity = (x: number, y: number): readonly [number, number] => [x, y];
        expect(getMowerHeadingRad([], identity)).toBeUndefined();
        expect(getMowerHeadingRad([[1, 2], [1, 2]], identity)).toBeUndefined();
        expect(getMowerHeadingRad([[0, 0], [Number.NaN, 1]], identity)).toBeUndefined();
        expect(hasValidMapPosition([18.06, 59.33])).toBe(true);
        expect(hasValidMapPosition([181, 59.33])).toBe(false);
        expect(hasValidMapPosition([18, Number.NaN])).toBe(false);
    });

    it("offsets only the display coordinate along the ROS heading in local ENU", () => {
        const east = offsetMapCoordinatesForward(18, 0, 0, 0.02)!;
        expect(east[0]).toBeCloseTo(18 + (0.02 / 6_378_137) * 180 / Math.PI, 10);
        expect(east[1]).toBe(0);

        const north = offsetMapCoordinatesForward(18, 0, Math.PI / 2, 0.02)!;
        expect(north[0]).toBeCloseTo(18, 10);
        expect(north[1]).toBeCloseTo((0.02 / 6_378_137) * 180 / Math.PI, 10);

        expect(offsetMapCoordinatesForward(18, 59.33, Math.PI, 0.02)![0]).toBeLessThan(18);
        expect(offsetMapCoordinatesForward(18, 90, 0, 0.02)).toBeUndefined();
        expect(offsetMapCoordinatesForward(18, 59.33, Number.NaN, 0.02)).toBeUndefined();
    });

    it("keeps the normalized pose anchor at the image center for every heading", () => {
        const size = 240;
        const anchor = {x: 0.5, y: 0.77};
        const offset = getMapImageAnchorOffsetPx(size, size, anchor);
        const point = {x: size * anchor.x + offset.left, y: size * anchor.y + offset.top};
        expect(point).toEqual({x: size / 2, y: size / 2});
        for (const heading of [0, Math.PI / 2, Math.PI, -Math.PI / 2]) {
            const rotation = rosHeadingToMapboxRotation(heading) * radians;
            const rotatedX = (point.x - size / 2) * Math.cos(rotation) - (point.y - size / 2) * Math.sin(rotation) + size / 2;
            const rotatedY = (point.x - size / 2) * Math.sin(rotation) + (point.y - size / 2) * Math.cos(rotation) + size / 2;
            expect(rotatedX).toBeCloseTo(size / 2);
            expect(rotatedY).toBeCloseTo(size / 2);
        }
    });

    it("applies both normalized anchor axes to a non-square image", () => {
        const offset = getMapImageAnchorOffsetPx(300, 240, {x: 0.2, y: 0.75});
        expect(300 * 0.2 + offset.left).toBeCloseTo(150);
        expect(240 * 0.75 + offset.top).toBeCloseTo(120);
    });

    it("scales calibrated length over zoom levels and at the local realistic latitude", () => {
        const sizes = [512, 2048, 8192].map((worldPixels) => calculateMapImageSizePx(
            mercatorProject(worldPixels), 18.06, 59.33, 0.57, 0.9,
        ));
        expect(sizes.every((size): size is number => size !== undefined)).toBe(true);
        expect(sizes[1]! / sizes[0]!).toBeCloseTo(4);
        expect(sizes[2]! / sizes[1]!).toBeCloseTo(4);

        const equatorScale = calculateMapImageSizePx(mercatorProject(2048), 18.06, 0, 0.57, 0.9);
        const StockholmScale = sizes[1]!;
        expect(StockholmScale / equatorScale!).toBeCloseTo(1 / Math.cos(59.33 * radians), 2);
        expect(calculateMapImageSizePx(mercatorProject(2048), 18, 90, 0.57, 0.9)).toBeUndefined();
    });

    it("supports independent width and length calibration for a dock image", () => {
        const dimensions = calculateMapImageDimensionsPx(mercatorProject(2048), 18.06, 59.33, 0.63, 0.944, 0.46, 0.667);
        expect(dimensions).toBeDefined();
        expect(dimensions!.width / dimensions!.height).toBeCloseTo((0.46 / 0.667) / (0.63 / 0.944), 3);
        expect(calculateMapImageDimensionsPx(mercatorProject(2048), 18.06, 59.33, 0.63, 0.944, 0.46, undefined)).toBeUndefined();
    });
});
