import {useCallback, useEffect, useRef, useState} from "react";
import {httpBase} from "../utils/apiHost.ts";
import {parseFleetRobots, partitionFleetRobots} from "../utils/fleet.ts";
import type {FleetPeerWire, FleetRobotWire, RejectedFleetRobot} from "../utils/fleet.ts";
import {isJsonObject} from "../utils/json.ts";
import {requestErrorMessage} from "../utils/errors.ts";

export interface CoordinatorSettings {
    enabled: boolean;
    yield_distance_m: number;
    resume_distance_m: number;
    completed_ttl_h: number;
    yield_max_hold_s: number;
    yield_cooldown_s: number;
}

export interface CoordinatorStatus {
    enabled: boolean;
    excluded_areas: number[];
    preferred_start: number;
    yielded: boolean;
    completed_areas: number[];
    last_push_at?: string;
    last_error?: string;
}

export interface PeerFeedStatus {
    active: boolean;
    peers_published: number;
    rate_hz: number;
    publishes: number;
    last_publish_at?: string;
    last_error?: string;
}

export interface FleetCoordination {
    settings: CoordinatorSettings;
    status: CoordinatorStatus;
    /** The /fleet/peers → costmap feed; live whenever a peer is registered. */
    peer_feed: PeerFeedStatus;
}

export interface MapPushPeerResult {
    id: string;
    name: string;
    address: string;
    ok: boolean;
    error?: string;
}

export interface MapPushResult {
    areas: number;
    peers: MapPushPeerResult[];
}

const EMPTY_COORDINATION: FleetCoordination = {
    settings: {enabled: false, yield_distance_m: 3, resume_distance_m: 5, completed_ttl_h: 12, yield_max_hold_s: 45, yield_cooldown_s: 20},
    status: {enabled: false, excluded_areas: [], preferred_start: -1, yielded: false, completed_areas: []},
    peer_feed: {active: false, peers_published: 0, rate_hz: 5, publishes: 0},
};

function isCoordinationPayload(payload: unknown): payload is {
    settings?: Partial<CoordinatorSettings>;
    status?: Partial<CoordinatorStatus>;
    peer_feed?: Partial<PeerFeedStatus>;
} {
    const part = (value: unknown, booleans: string[], numbers: string[], arrays: string[] = [], texts: string[] = []) => {
        if (value === undefined) return true;
        if (!isJsonObject(value)) return false;
        const finite = (item: unknown) => typeof item === "number" && Number.isFinite(item);
        return booleans.every(key => !(key in value) || typeof value[key] === "boolean")
            && numbers.every(key => !(key in value) || finite(value[key]))
            && arrays.every(key => !(key in value) || Array.isArray(value[key]) && value[key].every(finite))
            && texts.every(key => !(key in value) || typeof value[key] === "string");
    };
    return isJsonObject(payload)
        && part(payload.settings, ["enabled"], ["yield_distance_m", "resume_distance_m", "completed_ttl_h", "yield_max_hold_s", "yield_cooldown_s"])
        && part(payload.status, ["enabled", "yielded"], ["preferred_start"], ["excluded_areas", "completed_areas"], ["last_push_at", "last_error"])
        && part(payload.peer_feed, ["active"], ["peers_published", "rate_hz", "publishes"], [], ["last_publish_at", "last_error"]);
}

/** Keep the existing defaults for absent sections/fields and the e2e `{}` response. */
export function parseCoordination(payload: unknown): FleetCoordination {
    if (!isCoordinationPayload(payload)) throw new Error("Invalid fleet coordination response");
    if (!payload.settings || !payload.status) return {...payload, ...EMPTY_COORDINATION};
    return {
        ...payload,
        settings: {...EMPTY_COORDINATION.settings, ...payload.settings},
        status: {...EMPTY_COORDINATION.status, ...payload.status},
        peer_feed: {...EMPTY_COORDINATION.peer_feed, ...payload.peer_feed},
    };
}

export const FLEET_POLL_MS = 2000;

interface FleetState {
    robots: FleetRobotWire[];
    coordination: FleetCoordination;
    loading: boolean;
    error?: string;
    /** Names of robots left out of the last snapshot for malformed status. */
    rejectedRobots: string[];
}

/**
 * Log each newly malformed robot once; returns the ids now malformed so a
 * robot that recovers and breaks again is reported again.
 */
function reportRejections(reported: ReadonlySet<string>, rejected: RejectedFleetRobot[]): Set<string> {
    for (const {robot, reason} of rejected) {
        if (!reported.has(robot.identity.id)) console.warn(`Fleet: hiding robot "${robot.identity.id}": ${reason}`, robot.topics);
    }
    return new Set(rejected.map(({robot}) => robot.identity.id));
}

async function readJson(res: Response, strict = false): Promise<unknown> {
    const text = await res.text();
    try {
        if (strict) return JSON.parse(text);
        return text ? JSON.parse(text) : {};
    } catch {
        if (strict) throw new Error("Invalid fleet snapshot JSON response");
        return {};
    }
}

async function request(path: string, init?: RequestInit, strictJson = false): Promise<unknown> {
    const res = await fetch(`${httpBase()}/api/fleet${path}`, {
        headers: {"Content-Type": "application/json"},
        ...init,
    });
    const body = res.ok
        ? await readJson(res, strictJson)
        : await readJson(res).catch(() => undefined);
    if (!res.ok) {
        const detail = requestErrorMessage(body);
        throw new Error(detail?.trim() ? detail : `HTTP ${res.status}`);
    }
    return body;
}

/**
 * Fleet snapshot polled from this robot's backend, plus the peer-management
 * and command actions. Every action refreshes the snapshot when it returns so
 * the UI never waits a full poll interval to reflect what it just did.
 */
export function useFleet(pollMs: number = FLEET_POLL_MS) {
    const [state, setState] = useState<FleetState>({robots: [], coordination: EMPTY_COORDINATION, loading: true, rejectedRobots: []});
    const nextCycle = useRef(0);
    // Robot ids already reported, so a peer stuck sending bad data logs once.
    const reportedRejections = useRef(new Set<string>());

    const activeLifetime = useRef<{lastPublished: number} | null>(null);

    const refresh = useCallback(async () => {
        const lifetime = activeLifetime.current;
        const cycle = ++nextCycle.current;
        try {
            const [robots, coordination] = await Promise.all([
                request("/robots", undefined, true),
                request("/coordination", undefined, true).then(parseCoordination).then(
                    value => ({value}),
                    (e: unknown) => ({error: e instanceof Error ? e.message : String(e)}),
                ),
            ]);
            if (!lifetime || activeLifetime.current !== lifetime || cycle <= lifetime.lastPublished) return;
            const {robots: nextRobots, rejected} = partitionFleetRobots(parseFleetRobots(robots));
            reportedRejections.current = reportRejections(reportedRejections.current, rejected);
            const rejectedRobots = rejected.map(({robot}) => robot.identity.name || robot.identity.id);
            // Advance outside the React updater, which may be replayed. A
            // pending newer cycle alone must not invalidate this publication.
            lifetime.lastPublished = cycle;
            if ("error" in coordination) {
                setState(prev => ({...prev, robots: nextRobots, rejectedRobots, loading: false, error: coordination.error}));
            } else {
                setState({robots: nextRobots, coordination: coordination.value, loading: false, rejectedRobots});
            }
        } catch (e: unknown) {
            if (!lifetime || activeLifetime.current !== lifetime || cycle <= lifetime.lastPublished) return;
            const message = e instanceof Error ? e.message : String(e);
            lifetime.lastPublished = cycle;
            setState(prev => ({...prev, loading: false, error: message}));
        }
    }, []);

    useEffect(() => {
        activeLifetime.current = {lastPublished: 0};
        void refresh();
        const timer = setInterval(() => void refresh(), pollMs);
        return () => {
            activeLifetime.current = null;
            clearInterval(timer);
        };
    }, [refresh, pollMs]);

    const addPeer = useCallback(async (address: string): Promise<string | null | undefined> => {
        const res: unknown = await request("/peers", {method: "POST", body: JSON.stringify({address})});
        await refresh();
        if (!isJsonObject(res) || !("warning" in res)) return undefined;
        const warning = res.warning;
        return typeof warning === "string" && (warning === "" || warning.trim().length > 0) ? warning : null;
    }, [refresh]);

    const removePeer = useCallback(async (id: string) => {
        await request(`/peers/${encodeURIComponent(id)}`, {method: "DELETE"});
        await refresh();
    }, [refresh]);

    const call = useCallback(async (id: string, command: string, args: Record<string, unknown> = {}) => {
        await request(`/robots/${encodeURIComponent(id)}/call/${command}`, {method: "POST", body: JSON.stringify(args)});
        await refresh();
    }, [refresh]);

    const listPeers = useCallback(async (): Promise<FleetPeerWire[]> => {
        const body = await request("/peers");
        return Array.isArray(body) ? body as FleetPeerWire[] : [];
    }, []);

    const setCoordination = useCallback(async (settings: CoordinatorSettings) => {
        await request("/coordination", {method: "PUT", body: JSON.stringify(settings)});
        await refresh();
    }, [refresh]);

    const startFresh = useCallback(async () => {
        await request("/coordination/reset", {method: "POST", body: "{}"});
        await refresh();
    }, [refresh]);

    const pushMap = useCallback(async (): Promise<MapPushResult> => {
        const res: unknown = await request("/map/push", {method: "POST", body: "{}"});
        await refresh();
        if (!isMapPushResult(res)) {
            throw new Error("Unusable map sharing result: the response is invalid; remote delivery is not determined.");
        }
        return res;
    }, [refresh]);

    return {...state, refresh, addPeer, removePeer, call, listPeers, setCoordination, startFresh, pushMap};
}

function isMapPushResult(value: unknown): value is MapPushResult {
    if (!isJsonObject(value) || typeof value.areas !== "number"
        || !Number.isFinite(value.areas) || !Number.isInteger(value.areas) || !Array.isArray(value.peers)) return false;
    return value.peers.every((peer: unknown) => isJsonObject(peer)
        && typeof peer.id === "string" && typeof peer.name === "string" && typeof peer.address === "string"
        && typeof peer.ok === "boolean" && (!("error" in peer) || typeof peer.error === "string"));
}
