import {normalizeShellColour} from "../components/robot/shellPaint";
import {useSyncExternalStore} from "react";
import {MOWER_MODELS} from "../constants/mowerModels";
import {MOWER_STYLE_DOCKS, type DockAppearanceId} from "../constants/mowerAppearances";

import {APPEARANCE_REGISTRY} from "../components/robot/appearanceRegistry";
import {MOWER_STYLES, type MowerStyle} from "../components/robot/shellAssets";
export {MOWER_STYLES, type MowerStyle} from "../components/robot/shellAssets";
/** Display-only pair, independent of physical geometry. */
export type MowerVisualPreset = {style: MowerStyle; dockAppearance: DockAppearanceId};
export const DEFAULT_MOWER_VISUAL: MowerVisualPreset = {style: APPEARANCE_REGISTRY.defaultShell, dockAppearance: MOWER_STYLE_DOCKS[APPEARANCE_REGISTRY.defaultShell]};

/** Blank/unrecognised settings follow the model, without writing a browser's old
 * preference to the mower. Explicit selections persist through Settings Save. */
export function resolveMowerVisual(values: Record<string, unknown>): MowerVisualPreset {
    const style = MOWER_STYLES.find(s => s === values.mower_visual_style);
    return style ? {style, dockAppearance: MOWER_STYLE_DOCKS[style]}
        : MOWER_MODELS.find(model => model.value === values.mower_model)?.appearance ?? DEFAULT_MOWER_VISUAL;
}

// Transparency is a local editing aid. Legacy stored shell/dock fields are ignored.
const KEY = "mowgli.robot-visual.v1";
const EVENT = "mowgli-robot-visual";
function snapshot(): boolean {
    try {
        const data: unknown = JSON.parse(localStorage.getItem(KEY) ?? "null");
        return !!data && typeof data === "object" && "transparent" in data && data.transparent === true;
    }
    catch { return false; }
}
function subscribe(listener: () => void) {
    window.addEventListener(EVENT, listener);
    window.addEventListener("storage", listener);
    return () => {window.removeEventListener(EVENT, listener); window.removeEventListener("storage", listener);};
}
export function useMowerVisual(values: Record<string, unknown> = {}) {
    const transparent = useSyncExternalStore(subscribe, snapshot, () => false);
    return [{...resolveMowerVisual(values), colour: normalizeShellColour(values.mower_visual_colour), transparent}, (patch: {transparent: boolean}) => {
        try {localStorage.setItem(KEY, JSON.stringify(patch));} catch { /* Storage unavailable. */ }
        window.dispatchEvent(new Event(EVENT));
    }] as const;
}
