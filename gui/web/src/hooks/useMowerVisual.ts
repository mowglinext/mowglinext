import {useSyncExternalStore} from "react";

export const MOWER_STYLES = ["rounded", "sculpted", "utility", "yardforce"] as const;
export type MowerStyle = typeof MOWER_STYLES[number];
type Preference = {style: MowerStyle; transparent: boolean};
const KEY = "mowgli.robot-visual.v1";
const EVENT = "mowgli-robot-visual";
const fallback: Preference = {style: "sculpted", transparent: false};
let cachedRaw: string | null | undefined;
let cached = fallback;
function snapshot(): Preference {
    let raw: string | null = null;
    try { raw = localStorage.getItem(KEY); } catch { /* private browsing */ }
    if (raw !== cachedRaw) {
        cachedRaw = raw;
        try {
            const parsed: unknown = JSON.parse(raw ?? "null");
            const data = parsed && typeof parsed === "object" ? parsed as Record<string, unknown> : {};
            const style = MOWER_STYLES.find(s => s === data.style) ?? fallback.style;
            cached = {style, transparent: data.transparent === true};
        } catch { cached = fallback; }
    }
    return cached;
}
function subscribe(listener: () => void) {
    window.addEventListener(EVENT, listener);
    window.addEventListener("storage", listener);
    return () => {window.removeEventListener(EVENT, listener); window.removeEventListener("storage", listener);};
}
export function useMowerVisual() {
    const preference = useSyncExternalStore(subscribe, snapshot, () => fallback);
    return [preference, (patch: Partial<Preference>) => {
        const next = {...snapshot(), ...patch};
        try {localStorage.setItem(KEY, JSON.stringify(next));} catch {cached = next;}
        window.dispatchEvent(new Event(EVENT));
    }] as const;
}
