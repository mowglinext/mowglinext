import {describe, expect, it} from "vitest";
import {canCommand, deriveFleetRow, derivePhase, parseFleetRobots, projectFleetPositions, validateFleetHighLevelStatus} from "./fleet.ts";
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
        expect(row.batteryKnown).toBe(false); // cached status without pack voltage
        expect(row.gpsPercent).toBe(100);
        expect(row.position).toEqual({lat: 48.1, lon: 2.2});
        expect(row.apiVersionMismatch).toBe(false);
        expect(canCommand(row)).toBe(true);
    });

    it("falls back to the voltage when the BT has not published a percentage", () => {
        const row = deriveFleetRow(wire({}, {power: {v_battery: 29.0}}), {battery_full_voltage: "29.0", battery_empty_voltage: "22.0"});
        expect(row.batteryPercent).toBe(100);
        expect(row.batteryKnown).toBe(true);
    });

    it('keeps invalid battery interpolation unknown instead of marking a measured percentage', () => {
        const row = deriveFleetRow(wire({}, {power: {v_battery: 26}}), {battery_full_voltage: '28 V'});
        expect(row.batteryKnown).toBe(false);
        expect(row.batteryPercent).toBeNaN();
    });

    it("keeps a measured 0 % distinct from a missing voltage", () => {
        const row = deriveFleetRow(wire({}, {power: {v_battery: 23.0}}));
        expect(row.batteryPercent).toBe(0);
        expect(row.batteryKnown).toBe(true);
    });

    it("treats a 0/0 fix as no position and flags a foreign API version", () => {
        const row = deriveFleetRow(wire({identity: {id: "r2", name: "", api_version: 2}}, {gps: gpsAt(0, 0)}));
        expect(row.position).toBeUndefined();
        expect(row.name).toBe("r2");
        expect(row.apiVersionMismatch).toBe(true);
        expect(canCommand(row)).toBe(false);
    });

    it("fails closed for a persisted peer with no API version", () => {
        const row = deriveFleetRow(wire({identity: {id: "r2", name: "bravo", api_version: 0}}));
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

describe('Fleet robot identity boundary', () => {
    const invalidRows = [
        {name: 'name object', value: {...wire(), identity: {id: 'a', name: {label: 'bad'}}}},
        {name: 'name null', value: {...wire(), identity: {id: 'a', name: null}}},
        {name: 'address object', value: {...wire(), address: {host: 'bad'}}},
        {name: 'online text', value: {...wire(), online: 'false'}},
        {name: 'self text', value: {...wire(), self: 'false'}},
        {name: 'online null', value: {...wire(), online: null}},
        {name: 'self number', value: {...wire(), self: 0}},
        {name: 'last seen number', value: {...wire(), last_seen: 123}},
        {name: 'api text', value: {...wire(), identity: {id: 'a', api_version: '1'}}},
        {name: 'api nonfinite', value: {...wire(), identity: {id: 'a', api_version: Infinity}}},
    ];
    it.each(invalidRows)('rejects the complete snapshot for $name', ({value}) => {
        expect(() => parseFleetRobots([wire(), value])).toThrow('Invalid fleet robots response');
    });
    it('rejects duplicate retained ids including empty ids without choosing a winner', () => {
        for (const id of ['same', '']) {
            expect(() => parseFleetRobots([wire({identity: {id, name: 'Alpha'}}), wire({identity: {id, name: 'Beta'}})]))
                .toThrow('Invalid fleet robots response');
        }
    });
    it('preserves absent fields, an empty unique id, name fallback and additional properties by reference', () => {
        const absent = {identity: {id: '', extra: {keep: true}}, extra: [1], topics: {unknown: {keep: true}}};
        const named = wire({identity: {id: 'b', name: ''}, last_seen: null});
        const parsed = parseFleetRobots([absent, named]);
        expect(parsed).toEqual([absent, named]);
        expect(parsed[0]).toBe(absent);
        expect(parsed[1]).toBe(named);
        expect(deriveFleetRow(parsed[1]).name).toBe('b');
    });
    it.each([undefined, null, 0, 1, 99])('preserves existing API compatibility for %s', version => {
        const raw = {...wire(), identity: {id: 'a', name: 'Alpha', ...(version === undefined ? {} : {api_version: version})}};
        const [parsed] = parseFleetRobots([raw]);
        expect(parsed).toBe(raw);
        expect(deriveFleetRow(parsed).apiVersionMismatch).toBe(version != null && version !== 1);
    });
    it('keeps valid text fields, booleans and timestamps without transforming them', () => {
        const raw = wire({address: '', online: false, self: true, last_seen: '2026-10-18T12:00:00Z'});
        expect(parseFleetRobots([raw])[0]).toBe(raw);
        expect(parseFleetRobots([wire({last_seen: null})])[0].last_seen).toBeNull();
    });
    it('keeps legacy filtering and non-array compatibility and does not validate topics', () => {
        const raw = {...wire(), topics: {highLevelStatus: {state_name: {not: 'in this lot'}}}};
        expect(parseFleetRobots({})).toEqual([]);
        expect(parseFleetRobots([null, {identity: {id: 4}, online: 'bad'}, raw])).toEqual([raw]);
    });
});

describe('Fleet HighLevelStatus targeted validator', () => {
    it('keeps arbitrary finite states, precision, extra properties and object references unchanged', () => {
        const status = {state: 999, state_name: 'FUTURE', sub_state_name: '', emergency: false, is_charging: true,
            current_area: -12.5, coverage_percent: 140.123456, extra: {keep: true}};
        const raw = wire({}, {highLevelStatus: status, other: {keep: true}});
        const robots = parseFleetRobots([raw]);
        const before = JSON.stringify(robots);
        validateFleetHighLevelStatus(robots);
        expect(robots[0]).toBe(raw);
        expect(robots[0].topics?.highLevelStatus).toBe(status);
        expect(JSON.stringify(robots)).toBe(before);
        expect(deriveFleetRow(raw).coveragePercent).toBe(140.123456);
        expect(deriveFleetRow(raw).phase).toBe('charging');
        status.state = -0.5;
        expect(() => validateFleetHighLevelStatus(robots)).not.toThrow();
    });
    it.each(['state', 'current_area', 'coverage_percent'])('rejects NaN and infinities for %s', field => {
        for (const value of [NaN, Infinity, -Infinity]) {
            expect(() => validateFleetHighLevelStatus([wire({}, {highLevelStatus: {[field]: value}})]))
                .toThrow('Invalid fleet highLevelStatus response');
        }
    });
    it('keeps the shared parser independent and leaves other topics outside this validator', () => {
        const raw = wire({}, {highLevelStatus: {state_name: {bad: true}}, pose: {keep: true}});
        expect(parseFleetRobots([raw])[0]).toBe(raw);
        const otherTopics = wire({}, {status: {is_charging: 'false'}, gnssStatus: {fix_valid: 'false'}, power: {v_battery: 'bad'}});
        expect(() => validateFleetHighLevelStatus([otherTopics])).not.toThrow();
        expect(otherTopics.topics?.gnssStatus).toEqual({fix_valid: 'false'});
    });
});
