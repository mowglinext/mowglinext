import {APPEARANCE_REGISTRY} from "./appearanceRegistry";
import type {PartId} from "./partAssets";

/** Artwork registration belongs beside the image bounds, not in the renderer.
 * Anchors are fractions of the full cropped footprint. Offsets are metres in
 * robot X/Y, added to the operator's display adjustment (never the dock pose).
 * Mower shells use shellAssets.ts frames for the corresponding registration.
 */
export type DockArtwork = {
    part: PartId;
    height: number;
    origin: "rearAxle" | "chassisCenter";
    offset: {forward: number; left: number};
    topAnchor: readonly [number, number];
    sideAnchor: readonly [number, number];
};
export type LayeredDockStyle = keyof typeof APPEARANCE_REGISTRY.docks;
export const DOCK_ARTWORK: Record<LayeredDockStyle, DockArtwork> = APPEARANCE_REGISTRY.docks;
export const DEFAULT_DOCK_STYLE = APPEARANCE_REGISTRY.defaultDock;
