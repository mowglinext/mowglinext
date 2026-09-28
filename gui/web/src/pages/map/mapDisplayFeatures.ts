import type {Feature, FeatureCollection, LineString} from "geojson";
import {DockFeatureBase, MowingFeature, MowingFeatureBase} from "../../types/map.ts";

type DisplayFeature = Feature & {id?: string | number};

/** Build the GeoJSON source for the display-only map layers. Image readiness
 * hides the complete existing representation; until then, marker and heading
 * remain together as the fallback.
 */
export function buildMapDisplayFeatures(
    features: Record<string, MowingFeature>,
    mowerImageReady: boolean,
    dockImageReady: boolean,
    makeDockHeading: (dock: DockFeatureBase) => LineString,
    dockHeadingColor: string,
): FeatureCollection {
    const displayFeatures: DisplayFeature[] = Object.values(features)
        .filter((feature) => !(feature instanceof MowingFeatureBase))
        .filter((feature) => !mowerImageReady || (
            feature.id !== "mower" && feature.id !== "mower-heading" &&
            feature.properties.feature_type !== "mower-footprint"
        ))
        .filter((feature) => !dockImageReady || (feature.id !== "dock" && feature.id !== "dock-heading"))
        .map((feature) => ({
            type: "Feature",
            id: feature.id,
            geometry: feature.geometry,
            properties: feature.properties,
        }));

    const dock = features.dock;
    if (!dockImageReady && dock instanceof DockFeatureBase) {
        displayFeatures.push({
            type: "Feature",
            id: "dock-heading",
            geometry: makeDockHeading(dock),
            properties: {color: dockHeadingColor, width: 3, feature_type: "dock-heading"},
        });
    }

    return {type: "FeatureCollection", features: displayFeatures};
}
