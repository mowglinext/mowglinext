import { useCallback, useRef, useState } from "react";
import { App } from "antd";
import { useTranslation } from "react-i18next";
import {isHttpFailure} from "../utils/containers";
import {requestErrorMessage} from "../utils/errors";
import { wsBase } from "../utils/apiHost";

/**
 * Wait for ROS2 to be reachable again after a container restart.
 *
 * Opens a short-lived probe WebSocket to a known mowgli stream. The Go
 * backend forwards status frames from ROS. Receipt proves stream delivery,
 * not a new physical observation or that a particular restart occurred.
 * Resolves true on first message, false on timeout.
 */
const waitForRos2 = (timeoutMs: number): Promise<boolean> =>
    new Promise((resolve) => {
        const url = `${wsBase()}/api/mowglinext/subscribe/highLevelStatus`;

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
    /** Called only after the restart and readiness check have succeeded. */
    onSuccess?: () => void;
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
    const { t } = useTranslation();
    const {
        pendingLabel,
        successMessage,
        errorMessage,
        timeoutMs = 60_000,
        onSuccess,
        skipReadinessProbe = false,
    } = options;
    // Callers may override the copy; otherwise fall back to the translated defaults.
    const pendingText = pendingLabel ?? t("containerRestart.pending");
    const successText = successMessage ?? t("containerRestart.success");
    const errorText = errorMessage ?? t("containerRestart.error");

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
                            message: t("telemetry.statusUnknown"),
                        });
                        return;
                    }
                }
                onSuccess?.();
                notification.success({ message: successText });
            } catch (e) {
                const description = requestErrorMessage(e) ?? t("diagnosticsPage.unknownError");
                if (isHttpFailure(e)) notification.error({message: errorText, description});
                else notification.warning({message: t("telemetry.statusUnknown"), description});
            } finally {
                pendingRef.current = false;
                setPending(false);
            }
        },
        [notification, successText, errorText, timeoutMs, skipReadinessProbe, t, onSuccess],
    );

    return { pending, pendingLabel: pendingText, run };
};
