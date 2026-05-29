import type { GnssCrossCheck } from "../hooks/useDiagnosticsSnapshot.ts";

type GnssRuntime = GnssCrossCheck | null | undefined;

const normalizeBackend = (backend: string | undefined | null) =>
    (backend ?? "").trim().toLowerCase();

const normalizeProtocol = (protocol: string | undefined | null) =>
    (protocol ?? "").trim().toUpperCase();

export const GNSS_BACKEND_OPTIONS = [
    { value: "gps", label: "Generic GPS driver" },
    { value: "ublox", label: "u-blox driver" },
    { value: "unicore", label: "Unicore UM96x/UM98x driver" },
] as const;

export const EDITABLE_PROTOCOL_OPTIONS = [
    { value: "UBX", label: "UBX (u-blox / compatible)" },
    { value: "NMEA", label: "NMEA (generic)" },
    { value: "UNICORE", label: "UNICORE (selected by installer/runtime)", disabled: true },
] as const;

export function hasRuntimeGnss(gnss: GnssRuntime): gnss is GnssCrossCheck {
    return Boolean(gnss && gnss.has_config);
}

export function isRuntimeUnicore(gnss: GnssRuntime): boolean {
    return normalizeBackend(gnss?.backend) === "unicore" || normalizeProtocol(gnss?.protocol) === "UNICORE";
}

export function resolveDisplayedGnssBackend(gnss: GnssRuntime): string {
    const backend = normalizeBackend(gnss?.backend);
    return backend || "gps";
}

export function resolveDisplayedGpsProtocol(
    yamlProtocol: string | undefined | null,
    gnss: GnssRuntime,
): string {
    if (isRuntimeUnicore(gnss)) {
        return "UNICORE";
    }
    const protocol = normalizeProtocol(gnss?.protocol || yamlProtocol);
    return protocol || "UBX";
}

export function resolveDisplayedGpsPort(
    yamlPort: string | undefined | null,
    gnss: GnssRuntime,
): string {
    return gnss?.port || yamlPort || "/dev/gps";
}

export function resolveDisplayedGpsBaud(
    yamlBaud: string | number | undefined | null,
    gnss: GnssRuntime,
): string | number {
    if (gnss?.baud) {
        const parsed = Number(gnss.baud);
        return Number.isFinite(parsed) ? parsed : gnss.baud;
    }
    return yamlBaud || 460800;
}

export function sanitizeYamlGnssValues(
    values: Record<string, any>,
    gnss: GnssRuntime,
): Record<string, any> {
    if (!isRuntimeUnicore(gnss)) {
        return values;
    }

    const sanitized = { ...values, gps_protocol: "UNICORE" };
    if (gnss?.port) {
        sanitized.gps_port = gnss.port;
    }
    if (gnss?.baud) {
        const parsed = Number(gnss.baud);
        sanitized.gps_baudrate = Number.isFinite(parsed) ? parsed : gnss.baud;
    }
    return sanitized;
}
