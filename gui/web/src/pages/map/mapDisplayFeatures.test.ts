import {describe, expect, it} from "vitest";
import {DockFeatureBase, MowerFeatureBase, PathFeature, RobotPartFeature} from "../../types/map.ts";
import {buildMapDisplayFeatures} from "./mapDisplayFeatures.ts";

describe("buildMapDisplayFeatures image fallbacks", () => {
    const makeDockHeading = (dock: DockFeatureBase) => ({
        type: "LineString" as const,
        coordinates: [dock.getCoordinates(), [1, 2]],
    });
    const makeFeatures = () => ({
        mower: new MowerFeatureBase([0, 0]),
        "mower-heading": new PathFeature("mower-heading", [[0, 0], [0, 1]], "blue"),
        "mower-body": new RobotPartFeature("mower-body", [[0, 0], [1, 0], [1, 1], [0, 0]], "blue"),
        dock: new DockFeatureBase([1, 1], Math.PI / 2),
        path: new PathFeature("path", [[0, 0], [1, 1]], "green"),
    });

    it("keeps the mower and dock drawings until their images are ready", () => {
        const collection = buildMapDisplayFeatures(makeFeatures(), false, false, makeDockHeading, "orange");
        expect(collection.features.map(({id}) => id)).toEqual([
            "mower", "mower-heading", "mower-body", "dock", "path", "dock-heading",
        ]);
    });

    it("hides the dock marker and generated heading together after image decode", () => {
        const collection = buildMapDisplayFeatures(makeFeatures(), false, true, makeDockHeading, "orange");
        expect(collection.features.map(({id}) => id)).toEqual(["mower", "mower-heading", "mower-body", "path"]);
        expect(collection.features.some(({properties}) => properties?.feature_type === "dock-heading")).toBe(false);
    });

    it("hides the mower, heading, and URDF footprint only after the mower image is ready", () => {
        const collection = buildMapDisplayFeatures(makeFeatures(), true, false, makeDockHeading, "orange");
        expect(collection.features.map(({id}) => id)).toEqual(["dock", "path", "dock-heading"]);
    });

    it("restores both dock fallbacks if the image is no longer ready", () => {
        const features = makeFeatures();
        expect(buildMapDisplayFeatures(features, false, true, makeDockHeading, "orange").features)
            .toHaveLength(4);
        expect(buildMapDisplayFeatures(features, false, false, makeDockHeading, "orange").features)
            .toHaveLength(6);
    });

    it("keeps the old zero-heading fallback but never promotes an image without heading data", () => {
        const features = makeFeatures();
        features.dock = new DockFeatureBase([1, 1]);
        const collection = buildMapDisplayFeatures(features, false, false, makeDockHeading, "orange");
        expect(collection.features.map(({id}) => id)).toContain("dock-heading");
    });
});
