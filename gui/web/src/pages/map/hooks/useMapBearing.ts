import {useCallback, useEffect, useRef, useState} from "react";
import type {NotificationInstance} from "antd/es/notification/interface";

// Display-only Mapbox bearing (compass rotation in degrees, clockwise from
// north). Persists in GUI config under `gui.map.display.bearing`. The bearing
// is the visual rotation of the basemap + overlays — it does NOT change the
// robot's frame, and the lat/lon emitted by Mapbox click events is already
// rotation-aware, so no inverse-rotation math is required for click handlers.

interface UseMapBearingOptions {
    config: Record<string, string>;
    setConfig: (cfg: Record<string, string>) => Promise<void>;
    notification: NotificationInstance;
}

const DEBOUNCE_MS = 400;

export function useMapBearing({config, setConfig, notification}: UseMapBearingOptions) {
    const [bearing, setBearing] = useState(0);
    // Instance-scoped timeout: a previous module-scoped timer leaked across
    // remounts and was the root cause of issue #153 (rotation reset to 0
    // after navigating away). Pending timer also gets flushed on unmount so
    // a quick rotate-then-navigate still persists.
    const timeoutRef = useRef<ReturnType<typeof setTimeout> | null>(null);
    const pendingValueRef = useRef<number | null>(null);

    useEffect(() => {
        const b = parseFloat(config["gui.map.display.bearing"] ?? "0");
        if (!isNaN(b)) setBearing(b);
    }, [config]);

    const flush = useCallback(async () => {
        if (timeoutRef.current != null) {
            clearTimeout(timeoutRef.current);
            timeoutRef.current = null;
        }
        const value = pendingValueRef.current;
        if (value == null) return;
        pendingValueRef.current = null;
        try {
            await setConfig({"gui.map.display.bearing": value.toString()});
        } catch (e: any) {
            notification.error({message: "Failed to save bearing", description: e.message});
        }
    }, [setConfig, notification]);

    const handleBearing = useCallback((value: number) => {
        // Normalise into [-180, 180) so the persisted value stays compact.
        const normalised = ((value + 180) % 360 + 360) % 360 - 180;
        setBearing(normalised);
        pendingValueRef.current = normalised;
        if (timeoutRef.current != null) clearTimeout(timeoutRef.current);
        timeoutRef.current = setTimeout(() => {
            timeoutRef.current = null;
            void flush();
        }, DEBOUNCE_MS);
    }, [flush]);

    useEffect(() => {
        // On unmount, flush any pending change synchronously so the user does
        // not lose a rotation they made within the debounce window before
        // leaving the page (issue #153).
        return () => {
            void flush();
        };
    }, [flush]);

    return {bearing, handleBearing};
}
