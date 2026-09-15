import {useCallback, useEffect, useRef, useState} from "react";
import {httpBase} from "../utils/apiHost.ts";
import {parseFleetRobots} from "../utils/fleet.ts";
import type {FleetPeerWire, FleetRobotWire} from "../utils/fleet.ts";

export const FLEET_POLL_MS = 2000;

interface FleetState {
    robots: FleetRobotWire[];
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
    const [state, setState] = useState<FleetState>({robots: [], loading: true});
    const alive = useRef(true);

    const refresh = useCallback(async () => {
        try {
            const body = await request("/robots");
            if (!alive.current) return;
            setState({robots: parseFleetRobots(body), loading: false});
        } catch (e: any) {
            if (!alive.current) return;
            setState(prev => ({robots: prev.robots, loading: false, error: e?.message ?? String(e)}));
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

    return {...state, refresh, addPeer, removePeer, call, listPeers};
}
