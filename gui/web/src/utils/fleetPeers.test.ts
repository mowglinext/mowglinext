import {describe, expect, it} from "vitest";
import {DEFAULT_GEOMETRY} from "../hooks/useRobotDescription.ts";
import type {FleetRobotWire} from "./fleet.ts";
import {FLEET_PEER_COLOR, peerDisplayFeatures, peerMapPoses} from "./fleetPeers.ts";

const DATUM: [number, number, number] = [48.0, 2.0, 0];

function robot(id: string, opts: {
    self?: boolean; online?: boolean; x?: number; y?: number; heading?: number; datum?: [number, number];
} = {}): FleetRobotWire {
    const {self = false, online = true, x = 3, y = 4, heading = 0.5, datum} = opts;
    return {
        identity: {id, name: `Robot ${id}`, datum_lat: datum?.[0], datum_lon: datum?.[1]},
        self,
        online,
        topics: {pose: {pose: {pose: {position: {x, y, z: 0}}}, motion_heading: heading}},
    };
}

describe("peerMapPoses", () => {
    it("places an online peer sharing our map at its own map pose", () => {
        const [peer] = peerMapPoses([robot("a", {datum: [48.0, 2.0]})], DATUM);
        expect(peer.id).toBe("a");
        expect(peer.name).toBe("Robot a");
        expect(peer.x).toBeCloseTo(3, 6);
        expect(peer.y).toBeCloseTo(4, 6);
        expect(peer.heading).toBe(0.5);
    });

    it("re-projects a peer anchored at another datum into our frame", () => {
        // 0.0001° further north is ~11.1 m: the peer's origin sits north of ours.
        const [peer] = peerMapPoses([robot("a", {x: 0, y: 0, datum: [48.0001, 2.0]})], DATUM);
        expect(peer.x).toBeCloseTo(0, 3);
        expect(peer.y).toBeGreaterThan(11);
        expect(peer.y).toBeLessThan(11.2);
    });

    it("assumes our datum for a peer that does not publish one", () => {
        const [peer] = peerMapPoses([robot("a")], DATUM);
        expect(peer.x).toBeCloseTo(3, 6);
        expect(peer.y).toBeCloseTo(4, 6);
    });

    it("leaves out this robot, offline peers and peers without a pose", () => {
        const noPose: FleetRobotWire = {identity: {id: "c", name: "C"}, self: false, online: true, topics: {}};
        const robots = [robot("self", {self: true}), robot("b", {online: false}), noPose, robot("d", {x: Number.NaN})];
        expect(peerMapPoses(robots, DATUM)).toEqual([]);
    });

    it("places nothing while our own datum is unknown", () => {
        expect(peerMapPoses([robot("a")], [0, 0, 0])).toEqual([]);
    });
});

describe("peerDisplayFeatures", () => {
    it("draws a silhouette and a named point per peer in the peer colour", () => {
        const features = peerDisplayFeatures(
            [{id: "a", name: "Garden-West", x: 3, y: 4, heading: 0}], 0, 0, DATUM, DEFAULT_GEOMETRY);
        expect(features.map(f => f.id)).toEqual(["fleet-peer-a-footprint", "fleet-peer-a"]);
        expect(features[0].properties).toEqual({feature_type: "mower-footprint", color: FLEET_PEER_COLOR});
        expect(features[1].properties).toEqual({feature_type: "fleet-peer", color: FLEET_PEER_COLOR, name: "Garden-West"});
        expect(features[0].geometry.type).toBe("Polygon");
    });
});
