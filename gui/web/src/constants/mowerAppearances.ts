import {APPEARANCE_REGISTRY} from "../components/robot/appearanceRegistry";
import type {MowerStyle} from "../components/robot/shellAssets";
import type {LayeredDockStyle} from "../components/robot/dockAssets";

/** Display-only dock registry. Hardware appearance pairs reference these IDs;
 * image dimensions are visual metadata, never ROS/collision calibration.
 */

export interface MapImageAppearance {
    src: string;
    altKey: string;
    /** Physical length represented by the source image's long axis. */
    visibleLengthM: number;
    /** Fraction of the source image occupied along its long axis. */
    visibleLengthFraction: number;
    /** Optional calibrated width; omitted for square-scaled mower art. */
    visibleWidthM?: number;
    /** Fraction of the source image occupied along its short axis. */
    visibleWidthFraction?: number;
    /** Pose location in normalized source-image coordinates (0..1). */
    poseAnchor: {x: number; y: number};
    /** Visual correction from the tracked object's heading to this image's forward axis. */
    headingOffsetRad?: number;
    /** Small display-only shift in meters along the tracked heading. */
    forwardOffsetM?: number;
}

export type DockAppearanceId = (typeof APPEARANCE_REGISTRY.docks)[LayeredDockStyle]["appearanceId"] | keyof typeof APPEARANCE_REGISTRY.legacyDocks;

export interface DockAppearance {
    id: DockAppearanceId;
    labelKey: string;
    image?: MapImageAppearance;
    layered?: LayeredDockStyle;
    /** Optional foreground cutout for the dock's center tongue over the mower. */
    foregroundClipPath?: string;
}

export const DOCK_FOREGROUND_CLIP_PATH = "polygon(43% 78%, 57% 78%, 68% 83%, 68% 92%, 59% 96%, 41% 96%, 32% 92%, 32% 83%)";

export const DOCK_APPEARANCES = {
    ...APPEARANCE_REGISTRY.legacyDocks,
    ...Object.fromEntries(Object.entries(APPEARANCE_REGISTRY.docks).map(([layered, dock]) =>
        [dock.appearanceId, {id: dock.appearanceId, labelKey: "mowerVisual.dockStyle", layered}])),
} as Record<DockAppearanceId, DockAppearance>;
export const DEFAULT_DOCK_APPEARANCE = APPEARANCE_REGISTRY.docks[APPEARANCE_REGISTRY.defaultDock].appearanceId;

/** Docks are paired with the hardware appearance; there is no separate map override. */
export function resolveDockAppearance(value: unknown): DockAppearance {
    return Object.values(DOCK_APPEARANCES).find(({id}) => id === value) ?? DOCK_APPEARANCES[DEFAULT_DOCK_APPEARANCE];
}

export function shouldDisplayMapImage(
    image: MapImageAppearance | undefined,
    loadedSrc: string | undefined,
    hasPose: boolean,
    hasHeading: boolean,
): boolean {
    return Boolean(image && loadedSrc === image.src && hasPose && hasHeading);
}

/** Explicit shell/dock pairs. Existing bodies share the basic dock artwork until
 * their own matching station is available; no duplicate bitmap files needed. */
export const MOWER_STYLE_DOCKS = Object.fromEntries(
    Object.entries(APPEARANCE_REGISTRY.shells).map(([id, shell]) => [id, shell.dock]),
) as Record<MowerStyle, DockAppearanceId>;
