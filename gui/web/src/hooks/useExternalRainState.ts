import {useTopic} from "./useTopic.ts";
import {isJsonObject} from "../utils/json.ts";

/** What the mower makes of the external rain verdict (behavior_tree_node, JSON). */
export interface ExternalRainState {
    /** "disabled" | "no_data" | "dry" | "wet" | "expired" */
    state: string;
    enabled?: unknown;
    received?: boolean | null;
    /** The last verdict received, whatever its age. */
    wet?: unknown;
    /** A fresh wet verdict that the rain guard counts right now. */
    active?: boolean | null;
    /** The mower's own rain sensor, shown beside it. */
    onboard_wet?: boolean | null;
    /** Seconds since the last verdict; null when none was received. */
    age_s?: number | null;
    /** Seconds until a wet verdict expires; null when not applicable. */
    expires_in_s?: number | null;
    /** Configured expiry in seconds; 0 = never expires. */
    max_age_s?: unknown;
    /** Additional producer fields are preserved, without interpreting their types. */
    [key: string]: unknown;
}

function isExternalRainState(value: unknown): value is ExternalRainState {
    return isJsonObject(value) && typeof value.state === "string"
        && ["received", "active", "onboard_wet"].every(key =>
            !(key in value) || value[key] === null || typeof value[key] === "boolean")
        && ["age_s", "expires_in_s"].every(key =>
            !(key in value) || value[key] === null
            || typeof value[key] === "number" && Number.isFinite(value[key]));
}

/** Invalid frames return undefined so useTopic retains its last valid value. */
export const parseExternalRainState = (raw: unknown): ExternalRainState | undefined => {
    if (!isJsonObject(raw) || typeof raw.data !== "string" || raw.data === "") return undefined;
    try {
        const parsed: unknown = JSON.parse(raw.data);
        return isExternalRainState(parsed) ? parsed : undefined;
    } catch {
        return undefined;
    }
};

// Latched JSON from behavior_tree_node: feedback for the Weather settings page and any other
// surface that wants to show whether an external rain verdict arrived and is being counted.
export const useExternalRainState = (): ExternalRainState | null =>
    useTopic<ExternalRainState | null>("externalRainState", null, {select: parseExternalRainState}).data;
