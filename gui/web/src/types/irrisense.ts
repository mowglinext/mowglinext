/** Mirrors gui/pkg/types/soil.go — the cached IrriSense verdict. */
export interface SoilZoneStatus {
    id: string;
    label: string;
    enabled: boolean;
    selected: boolean;
    deficitMm: number;
    lastWateredAt?: string;
    wet: boolean;
    reason?: string;
}

export interface SoilStatus {
    enabled: boolean;
    configured: boolean;
    gateScheduler: boolean;
    fresh: boolean;
    wet: boolean;
    unknown: boolean;
    reason: string;
    gardenName?: string;
    fetchedAt?: string;
    zones: SoilZoneStatus[];
    error?: string;
}

/** Mirrors api.IrriSenseSettingsResponse — the token is never in here. */
export interface IrriSenseSettings {
    enabled: boolean;
    baseUrl: string;
    tokenSet: boolean;
    tokenMasked: string;
    gardenId: string;
    zoneIds: string[];
    wetDeficitMm: number;
    dryAfterWateringHours: number;
    maxStaleMinutes: number;
    gateScheduler: boolean;
}

/** Mirrors api.IrriSenseSettingsUpdate — absent fields keep their value. */
export interface IrriSenseSettingsUpdate extends Partial<Omit<IrriSenseSettings, "tokenSet" | "tokenMasked">> {
    token?: string;
    clearToken?: boolean;
}

export interface IrriSenseZoneSummary {
    id: string;
    label: string;
    enabled: boolean;
}

export interface IrriSenseGardenSummary {
    id: string;
    name: string;
    zones: IrriSenseZoneSummary[];
}

export type SoilVerdict = "wet" | "dry" | "unknown";

/** One place to collapse the status flags into what the UI shows. */
export function soilVerdict(status: SoilStatus | null | undefined): SoilVerdict {
    if (!status || status.unknown || !status.fresh) return "unknown";
    return status.wet ? "wet" : "dry";
}

/** Pull a human-readable message out of an unknown thrown API error. */
export function apiErrorMessage(e: unknown): string {
    if (e instanceof Error && e.message) return e.message;
    if (typeof e === "object" && e !== null) {
        const err = e as { error?: { error?: string } | string; message?: string };
        if (typeof err.error === "string") return err.error;
        return err.error?.error ?? err.message ?? "Unexpected error";
    }
    return "Unexpected error";
}
