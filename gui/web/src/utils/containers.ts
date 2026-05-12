import { Api } from "../api/Api.ts";

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

/** Restart the MowgliNext container */
export const restartMowgliNext = (api: GuiApi) =>
    containerAction(api, { name: "mowglinext", label: { key: "app", value: "mowglinext" } }, "restart");

/** Restart the GPS container (picks up new NTRIP / serial / protocol config) */
export const restartGps = (api: GuiApi) =>
    containerAction(api, { name: "gps" }, "restart");

/**
 * Settings keys whose values are consumed by the GPS docker container
 * (NTRIP credentials, mountpoint, serial port, protocol). Saving any of
 * these requires bouncing mowgli-gps to actually apply the change.
 * Datum lat/lon/alt are read by ROS2 (navsat_to_absolute_pose_node), not
 * by the GPS container itself, so they are deliberately not in this list.
 */
export const GPS_RESTART_KEYS = new Set<string>([
    "gps_protocol",
    "gps_port",
    "gps_baudrate",
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
