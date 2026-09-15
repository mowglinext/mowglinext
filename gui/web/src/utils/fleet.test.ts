import {describe, expect, it} from "vitest";
import {canCommand, deriveFleetRow, derivePhase, parseFleetRobots, projectFleetPositions} from "./fleet.ts";
import type {FleetRobotWire} from "./fleet.ts";
import type {HighLevelStatus} from "../types/ros.ts";

/** Backend-adapted "gps" payload: AbsolutePose with lat/lon in pose.position.x/y. */
const hl = (over: Partial<HighLevelStatus>): HighLevelStatus => ({state: 1, state_name: "", sub_state_name: "", current_area: -1, current_path: -1, current_path_index: -1, total_swaths: 0, completed_swaths: 0, skipped_swaths: 0, coverage_percent: 0, gps_quality_percent: 0, battery_percent: 0, is_charging: false, emergency: false, ...over});

const gpsAt = (lat: number, lon: number) => ({pose: {pose: {position: {x: lat, y: lon, z: 0}}}});

const wire = (over: Partial<FleetRobotWire> = {}, topics: Record<string, unknown> = {}): FleetRobotWire => ({
    identity: {id: "r1", name: "alpha", api_version: 1},
    self: false,
    online: true,
    topics,
    ...over,
});

describe("parseFleetRobots", () => {
    it("returns an empty list for the e2e mock's `{}` answer and for garbage rows", () => {
        expect(parseFleetRobots({})).toEqual([]);
        expect(parseFleetRobots(null)).toEqual([]);
        expect(parseFleetRobots([null, {self: true}, wire()])).toHaveLength(1);
    });
});

describe("derivePhase", () => {
    it("is offline whatever the cached status says", () => {
        expect(derivePhase(wire({online: false}), hl({state: 2}))).toBe("offline");
    });

    it("maps the high-level state to a fleet phase", () => {
        expect(derivePhase(wire(), hl({state: 2, state_name: "MOWING"}))).toBe("mowing");
        expect(derivePhase(wire(), hl({state: 2, state_name: "RETURNING_HOME"}))).toBe("returning");
        expect(derivePhase(wire(), hl({state: 3}))).toBe("recording");
        expect(derivePhase(wire(), hl({state: 4}))).toBe("manual");
        expect(derivePhase(wire(), hl({state: 1, is_charging: true}))).toBe("charging");
        expect(derivePhase(wire(), hl({state: 1}))).toBe("idle");
        expect(derivePhase(wire(), undefined)).toBe("idle");
    });

    it("emergency wins over every state", () => {
        expect(derivePhase(wire(), hl({state: 2, emergency: true}))).toBe("emergency");
    });
});

describe("deriveFleetRow", () => {
    it("reads battery, area, GPS and position from the mirrored topics", () => {
        const row = deriveFleetRow(wire({}, {
            highLevelStatus: {state: 2, state_name: "MOWING", sub_state_name: "", current_area: 3, coverage_percent: 41.5, battery_percent: 72, is_charging: false, emergency: false},
            gnssStatus: {rtk_mode: 3, fix_valid: true},
            gps: gpsAt(48.1, 2.2),
        }));
        expect(row.name).toBe("alpha");
        expect(row.phase).toBe("mowing");
        expect(row.currentArea).toBe(3);
        expect(row.coveragePercent).toBe(41.5);
        expect(row.batteryPercent).toBe(72);
        expect(row.gpsPercent).toBe(100);
        expect(row.position).toEqual({lat: 48.1, lon: 2.2});
        expect(row.apiVersionMismatch).toBe(false);
    });

    it("falls back to the voltage when the BT has not published a percentage", () => {
        const row = deriveFleetRow(wire({}, {power: {v_battery: 29.0}}), {battery_full_voltage: "29.0", battery_empty_voltage: "22.0"});
        expect(row.batteryPercent).toBe(100);
    });

    it("treats a 0/0 fix as no position and flags a foreign API version", () => {
        const row = deriveFleetRow(wire({identity: {id: "r2", name: "", api_version: 2}}, {gps: gpsAt(0, 0)}));
        expect(row.position).toBeUndefined();
        expect(row.name).toBe("r2");
        expect(row.apiVersionMismatch).toBe(true);
        expect(canCommand(row)).toBe(false);
    });

    it("never flags self for an API mismatch and parses last_seen", () => {
        const row = deriveFleetRow(wire({self: true, identity: {id: "me", name: "me", api_version: 99}, last_seen: "2026-09-15T10:00:00Z"}));
        expect(row.apiVersionMismatch).toBe(false);
        expect(row.lastSeen?.toISOString()).toBe("2026-09-15T10:00:00.000Z");
    });
});

describe("projectFleetPositions", () => {
    const at = (id: string, lat: number, lon: number) =>
        deriveFleetRow(wire({identity: {id, name: id}}, {gps: gpsAt(lat, lon)}));

    it("puts a lone robot in the centre and drops robots without a fix", () => {
        const pts = projectFleetPositions([at("a", 48, 2), deriveFleetRow(wire({identity: {id: "b", name: "b"}}))]);
        expect(pts).toEqual([{id: "a", x: 0.5, y: 0.5}]);
    });

    it("keeps north up and east right with an isotropic scale", () => {
        const pts = projectFleetPositions([at("w", 48, 2), at("e", 48, 2.001)]);
        const w = pts.find(p => p.id === "w")!, e = pts.find(p => p.id === "e")!;
        expect(e.x).toBeGreaterThan(w.x);
        expect(e.y).toBeCloseTo(w.y, 6);
        expect(w.x).toBeCloseTo(0.1, 6);
        expect(e.x).toBeCloseTo(0.9, 6);

        const ns = projectFleetPositions([at("s", 48, 2), at("n", 48.0005, 2)]);
        const s = ns.find(p => p.id === "s")!, n = ns.find(p => p.id === "n")!;
        expect(n.y).toBeLessThan(s.y);
    });

    it("does not stretch two robots a metre apart to opposite corners", () => {
        const pts = projectFleetPositions([at("a", 48, 2), at("b", 48.000009, 2)]);
        const [a, b] = pts;
        expect(Math.abs(a.y - b.y)).toBeLessThan(0.1);
    });
});
