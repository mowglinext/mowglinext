/** Mirrors api.RemoteAccessSettingsResponse — the auth key is never in here. */
export interface RemoteAccessSettings {
    enabled: boolean;
    hostname: string;
    authKeySet: boolean;
    authKeyMasked: string;
    serveHttps: boolean;
    image: string;
    defaultImage: string;
    containerName: string;
}

/** Mirrors api.RemoteAccessSettingsUpdate — absent fields keep their value. */
export interface RemoteAccessSettingsUpdate
    extends Partial<Omit<RemoteAccessSettings, "authKeySet" | "authKeyMasked" | "defaultImage" | "containerName">> {
    authKey?: string;
    clearAuthKey?: boolean;
}

export type RemoteAccessPhase = "disabled" | "pulling" | "starting" | "running" | "error";

/** tailscaled's own state machine, as reported by `tailscale status`. */
export type RemoteAccessBackendState =
    | "NoState"
    | "NeedsLogin"
    | "NeedsMachineAuth"
    | "Stopped"
    | "Starting"
    | "Running"
    | "";

/** Mirrors providers.RemoteAccessStatus. */
export interface RemoteAccessStatus {
    enabled: boolean;
    phase: RemoteAccessPhase;
    error?: string;
    containerState?: string;
    backendState?: RemoteAccessBackendState;
    loginUrl?: string;
    hostname: string;
    dnsName?: string;
    tailscaleIps: string[];
    httpsUrl?: string;
    httpUrls: string[];
    magicDnsEnabled: boolean;
    health: string[];
    version?: string;
    checkedAt: string;
}

export type RemoteAccessVerdict = "off" | "working" | "needsLogin" | "connected" | "error";

/** One place to collapse the status into what the UI shows. */
export function remoteAccessVerdict(status: RemoteAccessStatus | null | undefined): RemoteAccessVerdict {
    if (!status || !status.enabled || status.phase === "disabled") return "off";
    if (status.phase === "error" || status.error) return "error";
    if (status.backendState === "NeedsLogin" && status.loginUrl) return "needsLogin";
    if (status.backendState === "Running") return "connected";
    return "working";
}
