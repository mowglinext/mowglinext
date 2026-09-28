import { useCallback, useEffect, useState } from "react";

/** Mirrors types.FirmwareAvailability (GET /api/setup/firmware/available). */
export interface AvailableFirmware {
    board: string;
    panel: string;
    available: boolean;
    fw_version?: string;
    protocol_version?: number;
    release?: string;
    own_release: boolean;
}

/**
 * useAvailableFirmware — the prebuilt firmware a flash would install for the
 * saved board, from this installation's release (the backend falls back to the
 * latest stable release when that release carries no firmware).
 */
export const useAvailableFirmware = () => {
    const [data, setData] = useState<AvailableFirmware | null>(null);
    const [error, setError] = useState<string | null>(null);
    const [loading, setLoading] = useState(false);

    const refresh = useCallback(async () => {
        setLoading(true);
        try {
            const response = await fetch("/api/setup/firmware/available");
            const body = (await response.json()) as AvailableFirmware & { error?: string };
            if (!response.ok) {
                setError(body.error ?? `HTTP ${response.status}`);
                return;
            }
            setData(body);
            setError(null);
        } catch (e) {
            setError(e instanceof Error ? e.message : String(e));
        } finally {
            setLoading(false);
        }
    }, []);

    useEffect(() => {
        void refresh();
    }, [refresh]);

    return { data, error, loading, refresh };
};
