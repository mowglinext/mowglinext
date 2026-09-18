import {useCallback, useEffect, useRef, useState} from "react";
import {httpBase} from "../utils/apiHost.ts";
import {parseFleetRobots} from "../utils/fleet.ts";
import type {FleetPeerWire, FleetRobotWire} from "../utils/fleet.ts";

export interface CoordinatorSettings {
    enabled: boolean;
    yield_distance_m: number;
    resume_distance_m: number;
    completed_ttl_h: number;
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

export interface FleetCoordination {
    settings: CoordinatorSettings;
    status: CoordinatorStatus;
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
    settings: {enabled: false, yield_distance_m: 3, resume_distance_m: 5, completed_ttl_h: 12},
    status: {enabled: false, excluded_areas: [], preferred_start: -1, yielded: false, completed_areas: []},
};

/** The e2e mock answers `{}` for unknown routes: fall back to the defaults. */
export function parseCoordination(payload: unknown): FleetCoordination {
    const p = payload as Partial<FleetCoordination> | null;
    if (!p || typeof p !== "object" || !p.settings || !p.status) return EMPTY_COORDINATION;
    return {
        settings: {...EMPTY_COORDINATION.settings, ...p.settings},
        status: {...EMPTY_COORDINATION.status, ...p.status},
    };
}

export const FLEET_POLL_MS = 2000;

interface FleetState {
    robots: FleetRobotWire[];
    coordination: FleetCoordination;
    loading: boolean;
    error?: string;
}

async function readJson(res: Response): Promise<unknown> {
    const text = await res.text();
    try {
        return text ? JSON.parse(text) : {};
    } catch {
        return {};
    }
}

async function request(path: string, init?: RequestInit): Promise<unknown> {
    const res = await fetch(`${httpBase()}/api/fleet${path}`, {
        headers: {"Content-Type": "application/json"},
        ...init,
    });
    const body = await readJson(res);
    if (!res.ok) {
        const msg = (body as {error?: string})?.error ?? `HTTP ${res.status}`;
        throw new Error(msg);
    }
    return body;
}

/**
 * Fleet snapshot polled from this robot's backend, plus the peer-management
 * and command actions. Every action refreshes the snapshot when it returns so
 * the UI never waits a full poll interval to reflect what it just did.
 */
export function useFleet(pollMs: number = FLEET_POLL_MS) {
    const [state, setState] = useState<FleetState>({robots: [], coordination: EMPTY_COORDINATION, loading: true});
    const alive = useRef(true);

    const refresh = useCallback(async () => {
        try {
            const [robots, coordination] = await Promise.all([request("/robots"), request("/coordination")]);
            if (!alive.current) return;
            setState({robots: parseFleetRobots(robots), coordination: parseCoordination(coordination), loading: false});
        } catch (e: unknown) {
            if (!alive.current) return;
            const message = e instanceof Error ? e.message : String(e);
            setState(prev => ({...prev, loading: false, error: message}));
        }
    }, []);

    useEffect(() => {
        alive.current = true;
        void refresh();
        const timer = setInterval(() => void refresh(), pollMs);
        return () => {
            alive.current = false;
            clearInterval(timer);
        };
    }, [refresh, pollMs]);

    const addPeer = useCallback(async (address: string) => {
        const res = await request("/peers", {method: "POST", body: JSON.stringify({address})}) as {warning?: string};
        await refresh();
        return res?.warning;
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
        const res = await request("/map/push", {method: "POST", body: "{}"}) as MapPushResult;
        await refresh();
        return res;
    }, [refresh]);

    return {...state, refresh, addPeer, removePeer, call, listPeers, setCoordination, startFresh, pushMap};
}
