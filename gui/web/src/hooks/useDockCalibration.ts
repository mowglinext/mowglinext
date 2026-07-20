import { useCallback, useEffect, useRef, useState } from "react";
import { useWS } from "./useWS.ts";

// Mirrors mowgli_interfaces/msg/DockCalibrationStatus (phase codes + retry_reason
// mirror CalibrateDock.action). The GUI has no ROS-action support (foxglove
// transport), so the one-click dock calibration is driven via a non-blocking
// start service + this live status topic.
export type DockCalibrationPhase = 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7;

export interface DockCalibrationStatus {
    phase: DockCalibrationPhase;
    progress: number;
    cog_std_deg: number;
    displacement_m: number;
    charging: boolean;
    running: boolean;
    success: boolean;
    retry_reason: number;
    message: string;
}

export const PHASE_LABELS: Record<number, string> = {
    0: "Waiting for RTK-Fixed",
    1: "Reversing off the dock",
    2: "Checking COG coherence",
    3: "Re-docking",
    4: "Verifying charge",
    5: "Saving dock pose",
    6: "Idle",
    7: "Done",
};

export const RETRY_LABELS: Record<number, string> = {
    0: "",
    1: "No RTK-Fixed — wait for a fix and retry.",
    2: "COG incoherent (RTK not truly fixed / GPS noisy) — retry.",
    3: "Reverse leg too short for a heading fit — retry.",
    4: "Re-dock did not re-engage the charger — retry.",
    5: "Emergency active — clear it and retry.",
    6: "Robot not on the dock, or mowing — dock it / send HOME, then retry.",
    7: "Could not save the dock pose (RTK/charging gate) — retry.",
};

interface StartResponse {
    success: boolean;
    message: string;
}

/**
 * useDockCalibration — drive the one-click dock calibration.
 * `start()` POSTs to the non-blocking start service; `status` streams live
 * phase/progress and the terminal outcome from ~/dock_calibration/status.
 */
export const useDockCalibration = () => {
    const [status, setStatus] = useState<DockCalibrationStatus | null>(null);
    const [starting, setStarting] = useState(false);
    const [startError, setStartError] = useState<string | null>(null);
    const startedRef = useRef(false);

    const stream = useWS<string>(
        () => {},
        () => {},
        (e) => {
            setStatus(e as unknown as DockCalibrationStatus);
        },
    );

    useEffect(() => {
        stream.start("/api/mowglinext/subscribe/dockCalibrationStatus");
        return () => stream.stop();
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, []);

    const start = useCallback(async () => {
        setStarting(true);
        setStartError(null);
        startedRef.current = true;
        try {
            const resp = await fetch("/api/calibration/dock/start", { method: "POST" });
            const body = (await resp.json()) as StartResponse;
            if (!resp.ok || !body.success) {
                setStartError(body?.message || `Start rejected (HTTP ${resp.status}).`);
            }
        } catch (err: unknown) {
            setStartError(err instanceof Error ? err.message : "Failed to start dock calibration.");
        } finally {
            setStarting(false);
        }
    }, []);

    const running = status?.running ?? false;
    const done = !!status && status.phase === 7 && !status.running;

    return { status, start, starting, startError, running, done };
};
