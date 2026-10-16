import {useTopic} from "./useTopic.ts";

/** What the mower makes of the external rain verdict (behavior_tree_node, JSON). */
export interface ExternalRainState {
    /** "disabled" | "no_data" | "dry" | "wet" | "expired" */
    state: string;
    enabled: boolean;
    received: boolean;
    /** The last verdict received, whatever its age. */
    wet: boolean;
    /** A fresh wet verdict that the rain guard counts right now. */
    active: boolean;
    /** The mower's own rain sensor, shown beside it. */
    onboard_wet: boolean;
    /** Seconds since the last verdict; null when none was received. */
    age_s: number | null;
    /** Seconds until a wet verdict expires; null when not applicable. */
    expires_in_s: number | null;
    /** Configured expiry in seconds; 0 = never expires. */
    max_age_s: number;
}

/** std_msgs/String arrives as {data: "<json>"}; anything unparseable reads as "no information". */
export const parseExternalRainState = (raw: unknown): ExternalRainState | null | undefined => {
    const data = (raw as { data?: unknown } | null | undefined)?.data;
    if (typeof data !== "string" || data === "") return undefined;
    try {
        const parsed = JSON.parse(data) as ExternalRainState;
        return typeof parsed?.state === "string" ? parsed : undefined;
    } catch {
        return undefined;
    }
};

// Latched JSON from behavior_tree_node: feedback for the Weather settings page and any other
// surface that wants to show whether an external rain verdict arrived and is being counted.
export const useExternalRainState = (): ExternalRainState | null =>
    useTopic<ExternalRainState | null>("externalRainState", null, {select: parseExternalRainState}).data;
