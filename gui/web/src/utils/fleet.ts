import {HighLevelStatusConstants} from "../types/ros.ts";
import type {AbsolutePose, GnssStatus, HighLevelStatus, Power, Status} from "../types/ros.ts";
import {computeBatteryPercent} from "./battery.ts";
import {deriveGpsStatus} from "./gpsStatus.ts";

/** Wire shape of GET /api/fleet/robots (gui/pkg/providers/fleet.go). */
export interface FleetIdentity {
    id: string;
    name: string;
    version?: string;
    revision?: string;
    datum_lat?: number;
    datum_lon?: number;
    api_version?: number;
}

export interface FleetRobotWire {
    identity: FleetIdentity;
    self: boolean;
    address?: string;
    online: boolean;
    last_seen?: string | null;
    topics?: Record<string, unknown>;
}

export interface FleetPeerWire {
    id: string;
    name: string;
    address: string;
    api_version?: number;
}

export type FleetPhase = "offline" | "idle" | "mowing" | "returning" | "recording" | "manual" | "charging" | "emergency";

/** One robot as the Fleet page renders it: derived once from the wire row. */
export interface FleetRow {
    id: string;
    name: string;
    self: boolean;
    address?: string;
    online: boolean;
    phase: FleetPhase;
    stateName: string;
    subStateName: string;
    currentArea: number;
    coveragePercent: number;
    batteryPercent: number;
    isCharging: boolean;
    emergency: boolean;
    gpsLabel: string;
    gpsPercent: number;
    position?: {lat: number; lon: number};
    lastSeen?: Date;
    apiVersionMismatch: boolean;
}

/** The fleet API version this build speaks (gui/pkg/providers/fleet_identity.go). */
export const FLEET_API_VERSION = 1;

/** Accept only the array shape; the e2e mock answers `{}` for unknown routes. */
export function parseFleetRobots(payload: unknown): FleetRobotWire[] {
    if (!Array.isArray(payload)) return [];
    return payload.filter((r): r is FleetRobotWire =>
        !!r && typeof r === "object" && typeof (r as FleetRobotWire).identity?.id === "string");
}

function topic<T>(row: FleetRobotWire, key: string): T | undefined {
    const value = row.topics?.[key];
    return value && typeof value === "object" ? (value as T) : undefined;
}

export function derivePhase(row: FleetRobotWire, hl: HighLevelStatus | undefined): FleetPhase {
    if (!row.online) return "offline";
    if (hl?.emergency) return "emergency";
    switch (hl?.state) {
        case HighLevelStatusConstants.HIGH_LEVEL_STATE_AUTONOMOUS:
            return hl.state_name === "RETURNING_HOME" ? "returning" : "mowing";
        case HighLevelStatusConstants.HIGH_LEVEL_STATE_RECORDING:
            return "recording";
        case HighLevelStatusConstants.HIGH_LEVEL_STATE_MANUAL_MOWING:
            return "manual";
        default:
            return hl?.is_charging ? "charging" : "idle";
    }
}

/** A usable fix is finite and not the 0/0 placeholder an unfixed receiver reports. */
function readPosition(lat: unknown, lon: unknown): {lat: number; lon: number} | undefined {
    if (typeof lat !== "number" || typeof lon !== "number") return undefined;
    if (!Number.isFinite(lat) || !Number.isFinite(lon) || (lat === 0 && lon === 0)) return undefined;
    return {lat, lon};
}

export function deriveFleetRow(row: FleetRobotWire, settings: Record<string, unknown> = {}): FleetRow {
    const hl = topic<HighLevelStatus>(row, "highLevelStatus");
    const power = topic<Power>(row, "power");
    const status = topic<Status>(row, "status");
    const gnss = topic<GnssStatus>(row, "gnssStatus");
    // The "gps" key is the backend's adapted AbsolutePose: latitude rides in
    // pose.position.x and longitude in pose.position.y (transform.go adaptGPS).
    const fix = topic<AbsolutePose>(row, "gps")?.pose?.pose?.position;
    const gps = deriveGpsStatus(gnss);
    const position = readPosition(fix?.x, fix?.y);
    return {
        id: row.identity.id,
        name: row.identity.name || row.identity.id,
        self: row.self,
        address: row.address,
        online: row.online,
        phase: derivePhase(row, hl),
        stateName: hl?.state_name ?? "",
        subStateName: hl?.sub_state_name ?? "",
        currentArea: hl?.current_area ?? -1,
        coveragePercent: hl?.coverage_percent ?? 0,
        batteryPercent: computeBatteryPercent(hl?.battery_percent, power?.v_battery, settings),
        isCharging: hl?.is_charging ?? status?.is_charging ?? false,
        emergency: hl?.emergency ?? false,
        gpsLabel: gps.label,
        gpsPercent: gps.percent,
        position,
        lastSeen: row.last_seen ? new Date(row.last_seen) : undefined,
        apiVersionMismatch: row.self ? false : (row.identity.api_version != null && row.identity.api_version !== FLEET_API_VERSION),
    };
}

const METERS_PER_DEG_LAT = 111_320;

/**
 * Project every positioned robot into a shared local ENU frame (metres) centred
 * on the first positioned robot, then into the 0..1 unit square with a 10 %
 * inset — the same convention LiveMapMini uses. Robots without a fix are left
 * out. A single robot lands in the centre.
 */
export function projectFleetPositions(rows: FleetRow[]): {id: string; x: number; y: number}[] {
    const fixed = rows.filter(r => r.position);
    if (fixed.length === 0) return [];
    const ref = fixed[0].position!;
    const cosLat = Math.cos(ref.lat * Math.PI / 180);
    const enu = fixed.map(r => ({
        id: r.id,
        e: (r.position!.lon - ref.lon) * METERS_PER_DEG_LAT * cosLat,
        n: (r.position!.lat - ref.lat) * METERS_PER_DEG_LAT,
    }));
    const xs = enu.map(p => p.e), ys = enu.map(p => p.n);
    const x0 = Math.min(...xs), x1 = Math.max(...xs), y0 = Math.min(...ys), y1 = Math.max(...ys);
    // Keep the scale isotropic and at least 10 m wide so two close robots do
    // not get stretched to opposite corners.
    const span = Math.max(x1 - x0, y1 - y0, 10);
    const cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
    return enu.map(p => ({
        id: p.id,
        x: 0.5 + ((p.e - cx) / span) * 0.8,
        y: 0.5 - ((p.n - cy) / span) * 0.8,
    }));
}

/** Robots the operator can command from the fleet page: online, not in emergency. */
export function canCommand(row: FleetRow): boolean {
    return row.online && !row.apiVersionMismatch;
}
