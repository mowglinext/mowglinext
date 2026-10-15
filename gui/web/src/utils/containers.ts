import { Api } from "../api/Api.ts";

import {requestErrorMessage} from './errors';
import {isJsonObject} from './json';
import {reportAsyncError} from './async';

class CommandRequestError extends Error {
    readonly status: number;
    constructor(response: {status: number; error: unknown}) {
        super(requestErrorMessage(response) ?? 'Command request was not acknowledged');
        this.status = response.status;
    }
}

type GuiApi = Api<unknown>;
type Container = {id?: string; names: string[]; labels: Record<string, string>};

/** Only metadata used to choose a command target is trusted after decoding. */
async function listContainers(api: GuiApi): Promise<Container[]> {
    const response = await api.containers.containersList();
    if (response.error) throw new CommandRequestError(response);
    const raw: unknown = response.data;
    if (!isJsonObject(raw)) throw new Error('Invalid container list');
    if (raw.containers === undefined || raw.containers === null) return [];
    if (!Array.isArray(raw.containers)) throw new Error('Invalid container list');
    return raw.containers.map((item: unknown) => {
        if (!isJsonObject(item) || (item.id != null && typeof item.id !== 'string')) throw new Error('Invalid container ID');
        const names: unknown = item.names ?? [];
        const labels: unknown = item.labels ?? {};
        if (!Array.isArray(names) || !names.every((name: unknown): name is string => typeof name === 'string')) throw new Error('Invalid container names');
        if (!isJsonObject(labels)) throw new Error('Invalid container labels');
        const entries: Array<[string, string]> = [];
        for (const [key, value] of Object.entries(labels)) {
            if (typeof value !== 'string') throw new Error('Invalid container label');
            entries.push([key, value]);
        }
        return {id: typeof item.id === 'string' ? item.id : undefined, names, labels: Object.fromEntries(entries)};
    });
}

/** An HTTP error reports rejection; transport loss is an unknown outcome. */
export function isHttpFailure(error: unknown): boolean {
    return typeof error === 'object' && error !== null && 'status' in error
        && typeof error.status === 'number' && error.status >= 400 && error.status <= 599;
}

/** The Go command handlers acknowledge acceptance with OkResponse ({} or ok text). */
export function assertCommandAccepted(response: {ok: boolean; status: number; error: unknown; data: unknown}): void {
    if (!response.ok || response.error) throw new CommandRequestError(response);
    if (!isJsonObject(response.data) || ('ok' in response.data && typeof response.data.ok !== 'string')
        || 'error' in response.data) throw new Error('Invalid command acknowledgement');
}

async function command(api: GuiApi, id: string, action: 'start' | 'stop' | 'restart'): Promise<void> {
    const response = await api.containers.containersCreate(id, action);
    assertCommandAccepted(response);
}

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
    const containers = await listContainers(api);
    const container = containers.find(c => {
        const {name, label} = match;
        return (!!name && c.names.some(n => n.includes(name)))
            || (label !== undefined && Object.prototype.hasOwnProperty.call(c.labels, label.key) && c.labels[label.key] === label.value);
    });
    if (!container?.id) throw new Error(`Container not found (match: ${match.name ?? match.label?.value})`);
    await command(api, container.id, action);
};

/** Hardware-bridge sidecars that read mowgli_robot.yaml at start-up. */
const HARDWARE_BRIDGE_SIDECARS = ["mowgli-openmower"];

/**
 * Restart the ROS2 container, and the hardware-bridge sidecar when the robot
 * runs one: it reads the same mowgli_robot.yaml at start-up, so a saved
 * setting (ticks per metre, a serial port, a lift delay) only reaches the
 * OpenMower hardware once it restarts too.
 */
export const restartRos2 = async (api: GuiApi): Promise<void> => {
    await containerAction(api, { name: "ros2" }, "restart");
    const containers = await listContainers(api);
    const bare = (n: string) => n.replace(/^\//, "");
    const sidecars = containers.filter(c => c.names.some(n => HARDWARE_BRIDGE_SIDECARS.includes(bare(n))));
    for (const sidecar of sidecars) {
        if (sidecar.id) await command(api, sidecar.id, 'restart');
    }
};

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
 * may be interrupted or never return. The caller reconnects/reloads the
 * browser once the GUI comes back.
 *
 * (The old `restartMowgliNext` matched name "mowglinext"/label app=mowglinext,
 * which matched nothing — the main container is named `mowgli-ros2` with no
 * labels — so "Restart Mowgli" always failed with "Container not found".)
 */
export const restartMowgliStack = async (
    api: GuiApi,
    onGuiError: (error: unknown) => void = reportAsyncError,
): Promise<void> => {
    const containers = await listContainers(api);
    const bare = (n: string) => n.replace(/^\//, "");
    const isGui = (c: Container) => c.names.some(n => bare(n).startsWith('mowgli-gui'));
    const stack = containers.filter(c => !!c.id && c.names.some(n => bare(n).startsWith('mowgli-')));
    if (stack.length === 0) throw new Error('No mowgli-* containers found');

    // Independent non-GUI restarts remain parallel. All acknowledgements must
    // succeed before dispatching the GUI command; earlier commands may already
    // have taken effect if another fails. There is no rollback or implicit retry.
    await Promise.all(stack.filter(c => !isGui(c)).map(async c => {
        if (c.id) await command(api, c.id, 'restart');
    }));
    const gui = stack.find(isGui);
    // Do not await our own shutdown, but observe both HTTP and transport errors.
    if (gui?.id) void command(api, gui.id, 'restart').catch(onGuiError).catch(reportAsyncError);
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
