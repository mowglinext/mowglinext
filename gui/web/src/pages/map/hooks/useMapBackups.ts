import {useCallback} from "react";
import {App} from "antd";
import {useTranslation} from "react-i18next";
import {useApi} from "../../../hooks/useApi.ts";

/// One stored copy of the robot's areas.dat (see gui/pkg/api/map_backups.go).
export interface MapBackup {
    id: string;
    created_at: string;
    size_bytes: number;
    areas: number;
    obstacles: number;
    ignore_lines: number;
    area_names: string[];
}

// The generated HttpClient rejects with the parsed error body ({error: "..."})
// rather than an Error: turn it into one whose message the UI can show.
function toError(e: unknown, fallback: string): Error {
    if (e instanceof Error) return e;
    const body = e as {error?: {error?: string} | string; message?: string} | null;
    const inner = typeof body?.error === "object" ? body.error?.error : body?.error;
    const msg = inner ?? body?.message;
    return new Error(typeof msg === "string" ? msg : fallback);
}

/// Server-side copies of the map, taken before every edit session so a map that
/// goes wrong (a save that stops half way, a bad import) can be put back.
export const useMapBackups = () => {
    const api = useApi();
    const {notification} = App.useApp();
    const {t} = useTranslation();

    const list = useCallback(async (): Promise<{backups: MapBackup[]; keep: number}> => {
        try {
            const res = await api.request<{backups?: MapBackup[]; keep?: number}, {error?: string}>({
                path: "/map-backups", method: "GET", format: "json",
            });
            return {backups: res.data.backups ?? [], keep: res.data.keep ?? 20};
        } catch (e) {
            throw toError(e, "could not list the map backups");
        }
    }, [api]);

    /// Take a copy of the map NOW. Resolves true when it is safe to open the editor
    /// (a copy exists, or there was no map worth copying); false when the copy
    /// could not be made, in which case the editor must stay closed.
    const backupBeforeEdit = useCallback(async (): Promise<boolean> => {
        try {
            await api.request({path: "/map-backups", method: "POST", format: "json"});
            return true;
        } catch (e) {
            notification.error({
                message: t("mapBackups.backupFailed"),
                description: toError(e, "").message || undefined,
            });
            return false;
        }
    }, [api, notification, t]);

    const restore = useCallback(async (id: string): Promise<void> => {
        try {
            await api.request({path: `/map-backups/${encodeURIComponent(id)}/restore`, method: "POST", format: "json"});
        } catch (e) {
            throw toError(e, "could not restore the map backup");
        }
    }, [api]);

    return {list, backupBeforeEdit, restore};
};
