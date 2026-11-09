import {APPEARANCE_REGISTRY} from "./appearanceRegistry";
export type MowerStyle = keyof typeof APPEARANCE_REGISTRY.shells;
export const MOWER_STYLES = Object.keys(APPEARANCE_REGISTRY.shells) as MowerStyle[];

export type Shell = {
    /** Optional registered paint mask per view; omit to retain original artwork only. */
    paint?: {masks: Record<"top" | "side", string>; suggestedColour: string};
    label: {en: string; fr: string};
    dock: string;
    mapMaxSize: number;
    wheelsOverShell?: Partial<Record<"top" | "side", boolean>>;
    files?: Partial<Record<"top" | "side", string>>;
    /** Source atlas cells, including padding, used to validate independent alpha bounds. */
    cells: Record<"top" | "side", readonly [number, number, number, number]>;
    size: readonly [number, number];
    sizes?: Partial<Record<"top" | "side", readonly [number, number]>>;
    /** Physical projection frame, including real chassis hidden by a wheel.
     * Distinct from alpha bounds: never stretch a remaining panel to full length. */
    frames?: Partial<Record<"top" | "side", readonly [number, number, number, number]>>;
    top: readonly [number, number, number, number];
    side: readonly [number, number, number, number];
};
/** Typed view of the shared registry; no independent style list. */
export const SHELLS: Record<MowerStyle, Shell> = APPEARANCE_REGISTRY.shells;

export function shellProjectionFrame(style: MowerStyle, view: "top" | "side") {
    return SHELLS[style].frames?.[view] ?? SHELLS[style][view];
}
