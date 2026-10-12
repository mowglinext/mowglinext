import {useEffect, useState} from "react";
import {httpBase} from "../utils/apiHost.ts";
import {parseFleetRobots} from "../utils/fleet.ts";
import type {FleetRobotWire} from "../utils/fleet.ts";

/** Peer poses move at mowing speed: the Fleet page's 2 s cadence is enough. */
export const FLEET_PEERS_POLL_MS = 2000;
/** A robot with no peer (most installs) only checks now and then for one. */
export const FLEET_PEERS_IDLE_POLL_MS = 15000;

/**
 * The fleet snapshot (GET /api/fleet/robots) for drawing the other robots on
 * the map. Lighter than useFleet: no coordination request, and it slows down
 * to FLEET_PEERS_IDLE_POLL_MS while this robot is alone. A failed poll keeps
 * the last snapshot.
 */
export function useFleetPeers(): FleetRobotWire[] {
    const [robots, setRobots] = useState<FleetRobotWire[]>([]);
    useEffect(() => {
        let alive = true;
        let timer: ReturnType<typeof setTimeout> | undefined;
        const poll = async () => {
            let hasPeers = false;
            try {
                const res = await fetch(`${httpBase()}/api/fleet/robots`);
                if (res.ok) {
                    const next = parseFleetRobots(await res.json());
                    hasPeers = next.some(r => !r.self);
                    if (alive) setRobots(next);
                }
            } catch {
                // Offline or an older backend without the fleet routes: keep the last snapshot.
            }
            if (alive) timer = setTimeout(() => void poll(), hasPeers ? FLEET_PEERS_POLL_MS : FLEET_PEERS_IDLE_POLL_MS);
        };
        void poll();
        return () => {
            alive = false;
            if (timer) clearTimeout(timer);
        };
    }, []);
    return robots;
}
