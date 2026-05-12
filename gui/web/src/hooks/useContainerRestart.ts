import { useCallback, useRef, useState } from "react";
import { App } from "antd";

/**
 * Wait for ROS2 to be reachable again after a container restart.
 *
 * Opens a short-lived probe WebSocket to a known mowgli stream. The Go
 * backend only forwards a message once it has reconnected to rosbridge
 * AND received a fresh sample, so the first MessageEvent is a reliable
 * "ROS2 is back" signal. Resolves true on first message, false on timeout.
 */
const waitForRos2 = (timeoutMs: number): Promise<boolean> =>
    new Promise((resolve) => {
        const protocol = window.location.protocol === "https:" ? "wss" : "ws";
        const host = import.meta.env.DEV ? "localhost:4006" : window.location.host;
        const url = `${protocol}://${host}/api/mowglinext/subscribe/highLevelStatus`;

        let settled = false;
        const finish = (ok: boolean) => {
            if (settled) return;
            settled = true;
            try {
                ws.close();
            } catch {
                /* ignore */
            }
            clearTimeout(timer);
            resolve(ok);
        };

        const ws = new WebSocket(url);
        const timer = setTimeout(() => finish(false), timeoutMs);
        ws.onmessage = () => finish(true);
        ws.onerror = () => {
            /* keep waiting — backend will reconnect */
        };
    });

export type ContainerRestartOptions = {
    /** Label shown on the disabled button while restarting. */
    pendingLabel?: string;
    /** Toast message on success. */
    successMessage?: string;
    /** Toast message on failure (timeout or error). */
    errorMessage?: string;
    /** How long to wait for ROS2 to come back before giving up. */
    timeoutMs?: number;
    /** If true, skip the readiness probe and resolve as soon as the container API call returns. */
    skipReadinessProbe?: boolean;
};

/**
 * Manages a long-running container restart with proper pending-state UX.
 *
 * Returns `{pending, run}`:
 * - `pending`: true while the restart is in flight (button should be disabled).
 * - `run(action)`: kicks off `action()` (the actual restart call) then waits
 *   for ROS2 to come back. Idempotent: re-entrant calls while pending are no-ops.
 */
export const useContainerRestart = (options: ContainerRestartOptions = {}) => {
    const {
        pendingLabel = "Redémarrage…",
        successMessage = "Container redémarré",
        errorMessage = "Échec du redémarrage",
        timeoutMs = 60_000,
        skipReadinessProbe = false,
    } = options;

    const { notification } = App.useApp();
    const [pending, setPending] = useState(false);
    const pendingRef = useRef(false);

    const run = useCallback(
        async (action: () => Promise<void>) => {
            if (pendingRef.current) return;
            pendingRef.current = true;
            setPending(true);
            try {
                await action();
                if (!skipReadinessProbe) {
                    const ok = await waitForRos2(timeoutMs);
                    if (!ok) {
                        notification.warning({
                            message: errorMessage,
                            description: `ROS2 n'a pas redémarré dans les ${Math.round(timeoutMs / 1000)} s.`,
                        });
                        return;
                    }
                }
                notification.success({ message: successMessage });
            } catch (e: any) {
                notification.error({ message: errorMessage, description: e.message });
            } finally {
                pendingRef.current = false;
                setPending(false);
            }
        },
        [notification, successMessage, errorMessage, timeoutMs, skipReadinessProbe],
    );

    return { pending, pendingLabel, run };
};
