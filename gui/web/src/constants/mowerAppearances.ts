/** Display-only mower imagery. Keep this separate from ROS hardware presets:
 * similar dimensions or electronics do not establish the external shell brand.
 */
export type MowerAppearanceId = "urdf" | "biltema-rm1000";

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

export interface MowerAppearance {
    id: MowerAppearanceId;
    labelKey: string;
    mowerImage?: MapImageAppearance;
}

export type DockAppearanceId = "marker" | "generic" | "biltema-rm1000";

export interface DockAppearance {
    id: DockAppearanceId;
    labelKey: string;
    image?: MapImageAppearance;
    /** Optional foreground cutout for the dock's center tongue over the mower. */
    foregroundClipPath?: string;
    onlyForMowerAppearance?: MowerAppearanceId;
}

export const DOCK_FOREGROUND_CLIP_PATH = "polygon(43% 78%, 57% 78%, 68% 83%, 68% 92%, 59% 96%, 41% 96%, 32% 92%, 32% 83%)";

export const MOWER_APPEARANCES: Record<MowerAppearanceId, MowerAppearance> = {
    urdf: {id: "urdf", labelKey: "mapToolbar.mowerAppearanceUrdf"},
    "biltema-rm1000": {
        id: "biltema-rm1000",
        labelKey: "mapToolbar.mowerAppearanceBiltemaRm1000",
        mowerImage: {
            src: "/assets/robots/biltema-rm1000/mower.webp",
            altKey: "mapToolbar.mowerAppearanceBiltemaRm1000Alt",
            visibleLengthM: 0.57,
            visibleLengthFraction: 0.9,
            // The source's long axis runs front-to-back; the image pose point
            // is near the rear axle, not at the visual center of the body.
            poseAnchor: {x: 0.5, y: 0.77},
            // The docked-photo review favored this slight display-only forward tuck.
            forwardOffsetM: 0.02,
        },
    },
};

export const DOCK_APPEARANCES: Record<DockAppearanceId, DockAppearance> = {
    marker: {id: "marker", labelKey: "mapToolbar.dockAppearanceMarker"},
    generic: {
        id: "generic",
        labelKey: "mapToolbar.dockAppearanceGeneric",
        foregroundClipPath: DOCK_FOREGROUND_CLIP_PATH,
        image: {
            src: "/assets/robots/generic/dock.webp",
            altKey: "mapToolbar.dockAppearanceGenericAlt",
            // Nominal visual footprint inherited from the RM1000 source photo;
            // it is only a sizing approximation for other charging stations.
            visibleLengthM: 0.63,
            visibleLengthFraction: 0.951,
            visibleWidthM: 0.46,
            visibleWidthFraction: 0.678,
            // Visually calibrated against the docked RM1000 photo; the docking
            // tongue is layered over the mower while the outer station stays behind it.
            poseAnchor: {x: 0.5, y: 0.16},
            headingOffsetRad: Math.PI,
        },
    },
    "biltema-rm1000": {
        id: "biltema-rm1000",
        labelKey: "mapToolbar.dockAppearanceBiltemaRm1000",
        foregroundClipPath: DOCK_FOREGROUND_CLIP_PATH,
        image: {
            src: "/assets/robots/biltema-rm1000/dock.webp",
            altKey: "mapToolbar.dockAppearanceBiltemaRm1000Alt",
            visibleLengthM: 0.63,
            visibleLengthFraction: 0.944,
            visibleWidthM: 0.46,
            visibleWidthFraction: 0.667,
            // Visually calibrated against the docked RM1000 photo; the docking
            // tongue is layered over the mower while the outer station stays behind it.
            poseAnchor: {x: 0.5, y: 0.16},
            headingOffsetRad: Math.PI,
        },
        onlyForMowerAppearance: "biltema-rm1000",
    },
};

export function getAvailableDockAppearances(mowerAppearanceId: MowerAppearanceId): DockAppearance[] {
    return Object.values(DOCK_APPEARANCES)
        .filter((appearance) => !appearance.onlyForMowerAppearance || appearance.onlyForMowerAppearance === mowerAppearanceId);
}

export function resolveDockAppearance(value: unknown, mowerAppearanceId: MowerAppearanceId): DockAppearance {
    if (value !== "generic" && value !== "biltema-rm1000") return DOCK_APPEARANCES.marker;
    const appearance = DOCK_APPEARANCES[value];
    if (appearance.onlyForMowerAppearance && appearance.onlyForMowerAppearance !== mowerAppearanceId) {
        return DOCK_APPEARANCES.marker;
    }
    return appearance;
}

/** Reset only model-specific dock choices made incompatible by a mower change. */
export function getDockAppearanceResetForMowerChange(
    selectedDockAppearance: DockAppearance,
    nextMowerAppearanceId: MowerAppearanceId,
): DockAppearanceId | undefined {
    return selectedDockAppearance.onlyForMowerAppearance &&
        selectedDockAppearance.onlyForMowerAppearance !== nextMowerAppearanceId
        ? "marker"
        : undefined;
}

/** Unknown/stale GUI values fail closed to the existing URDF drawing. */
export function resolveMowerAppearance(value: unknown): MowerAppearance {
    if (value === "biltema-rm1000") return MOWER_APPEARANCES["biltema-rm1000"];
    return MOWER_APPEARANCES.urdf;
}

export function shouldDisplayMowerImage(
    appearance: MowerAppearance,
    loadedSrc: string | undefined,
    hasPose: boolean,
    hasHeading: boolean,
): boolean {
    return shouldDisplayMapImage(appearance.mowerImage, loadedSrc, hasPose, hasHeading);
}

export function shouldDisplayMapImage(
    image: MapImageAppearance | undefined,
    loadedSrc: string | undefined,
    hasPose: boolean,
    hasHeading: boolean,
): boolean {
    return Boolean(image && loadedSrc === image.src && hasPose && hasHeading);
}
