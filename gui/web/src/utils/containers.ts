import { Api, ApiContainer } from "../api/Api.ts";

type GuiApi = Api<unknown>;

type ContainerMatch = {
    /** Match by container name substring (checked against all names in the names array) */
    name?: string;
    /** Match by Docker label key=value */
    label?: { key: string; value: string };
};

/**
 * Find a running container by name or label and execute a command on it.
 * Throws on failure so callers can catch and show notifications.
 */
export const containerAction = async (
    api: GuiApi,
    match: ContainerMatch,
    action: "restart" | "start" | "stop",
): Promise<void> => {
    const res = await api.containers.containersList();
    if (res.error) throw new Error(res.error.error);

    const container = res.data.containers?.find((c: any) => {
        if (match.name && c.names?.some((n: string) => n.includes(match.name!))) return true;
        if (match.label && c.labels?.[match.label.key] === match.label.value) return true;
        return false;
    });

    if (!container?.id) {
        throw new Error(`Container not found (match: ${match.name ?? match.label?.value})`);
    }

    const cmdRes = await api.containers.containersCreate(container.id, action);
    if (cmdRes.error) throw new Error(cmdRes.error.error);
};

/** Restart the ROS2 container */
export const restartRos2 = (api: GuiApi) =>
    containerAction(api, { name: "ros2" }, "restart");

/** Restart the GUI container */
export const restartGui = (api: GuiApi) =>
    containerAction(api, { name: "gui" }, "restart");

/**
 * Restart the entire Mowgli stack — the GUI equivalent of the `mowgli-restart`
 * CLI command (`docker compose restart`). Restarts every running `mowgli-*`
 * container.
 *
 * The GUI container (`mowgli-gui`) is restarted LAST and fire-and-forget:
 * restarting it kills the backend serving this very request, so the response
 * never returns. The caller is responsible for reconnecting/reloading the
 * browser once the GUI comes back.
 *
 * (The old `restartMowgliNext` matched name "mowglinext"/label app=mowglinext,
 * which matched nothing — the main container is named `mowgli-ros2` with no
 * labels — so "Restart Mowgli" always failed with "Container not found".)
 */
export const restartMowgliStack = async (api: GuiApi): Promise<void> => {
    const res = await api.containers.containersList();
    if (res.error) throw new Error(res.error.error);

    const bare = (n: string) => n.replace(/^\//, "");
    const inStack = (c: ApiContainer) =>
        !!c.id && (c.names ?? []).some((n) => bare(n).startsWith("mowgli-"));
    const isGui = (c: ApiContainer) =>
        (c.names ?? []).some((n) => bare(n).startsWith("mowgli-gui"));

    const stack = (res.data.containers ?? []).filter(inStack);
    if (stack.length === 0) throw new Error("No mowgli-* containers found");

    // Restart everything except the GUI first (in parallel), so the whole
    // stack is already bouncing before we take our own backend down.
    await Promise.all(
        stack
            .filter((c) => !isGui(c))
            .map((c) => api.containers.containersCreate(c.id!, "restart")),
    );

    // Restart the GUI last. This stops the container serving this request, so
    // the promise never resolves — fire it and don't await.
    const gui = stack.find(isGui);
    if (gui?.id) void api.containers.containersCreate(gui.id, "restart");
};

/** Reconcile the GNSS sidecar after serial/NTRIP configuration changes. */
export const restartGps = async (api: GuiApi): Promise<void> => {
    const res = await api.request({
        path: "/settings/gnss/restart",
        method: "POST",
        format: "json",
    });

    if (res.error) {
        const apiError: unknown = res.error;
        const message =
            typeof apiError === "object" &&
            apiError !== null &&
            "error" in apiError &&
            typeof apiError.error === "string"
                ? apiError.error
                : "Failed to reconcile GNSS service";

        throw new Error(message);
    }
};

/**
 * Settings keys whose changes require the GNSS sidecar to be reconciled.
 * Reconciliation regenerates the derived Universal GNSS runtime config and
 * recreates mowgli-gps so serial mappings, environment and NTRIP settings
 * match the newly persisted configuration.
 *
 * Profile/signal-profile keys stay excluded: those require the explicit
 * Expert-mode Plan & Apply flow because they modify receiver configuration.
 */
export const GPS_RESTART_KEYS = new Set<string>([
    "gnss_receiver_family",
    "gnss_serial_device",
    "gnss_serial_baud",
    "ntrip_enabled",
    "ntrip_host",
    "ntrip_port",
    "ntrip_user",
    "ntrip_password",
    "ntrip_mountpoint",
]);

/** True if any dirty key affects the GPS container. */
export const dirtyKeysRequireGpsRestart = (dirty: Iterable<string>): boolean => {
    for (const k of dirty) if (GPS_RESTART_KEYS.has(k)) return true;
    return false;
};
