import type {Feature} from "geojson";
import type {AbsolutePose} from "../types/ros.ts";
import type {RobotGeometry} from "../hooks/useRobotDescription.ts";
import type {FleetRobotWire} from "./fleet.ts";
import {drawRobotSilhouette, itranspose, transpose} from "./map.tsx";

/** Fill of a peer's silhouette and its label: distinct from this robot's blue. */
export const FLEET_PEER_COLOR = "#b37feb";

/** Another fleet robot, placed in THIS robot's map frame (metres, x=east, y=north). */
export interface PeerMapPose {
    id: string;
    name: string;
    x: number;
    y: number;
    heading: number;
}

type Datum = [number, number, number];

function finite(...values: unknown[]): boolean {
    return values.every(v => typeof v === "number" && Number.isFinite(v));
}

/**
 * Online peers with a fused pose, re-projected into our map frame. A peer's
 * pose is in ITS map frame, anchored at ITS datum (identity.datum_lat/lon);
 * a fleet sharing one map has the same datum and this is the identity. A peer
 * that does not publish its datum is assumed to share ours. Nothing is placed
 * while our own datum is unknown.
 */
export function peerMapPoses(robots: FleetRobotWire[], datum: Datum): PeerMapPose[] {
    if (!finite(datum[0], datum[1]) || (datum[0] === 0 && datum[1] === 0)) return [];
    const poses: PeerMapPose[] = [];
    for (const robot of robots) {
        if (robot.self || !robot.online) continue;
        const pose = robot.topics?.pose as AbsolutePose | undefined;
        const position = pose?.pose?.pose?.position;
        const heading = pose?.motion_heading ?? 0;
        if (!position || !finite(position.x, position.y, heading)) continue;
        const peerLat = robot.identity.datum_lat;
        const peerLon = robot.identity.datum_lon;
        const peerDatum: Datum = finite(peerLat, peerLon) && !(peerLat === 0 && peerLon === 0)
            ? [peerLat as number, peerLon as number, 0]
            : datum;
        const [lon, lat] = transpose(0, 0, peerDatum, position.y as number, position.x as number);
        const [x, y] = itranspose(0, 0, datum, lat, lon);
        poses.push({id: robot.identity.id, name: robot.identity.name || robot.identity.id, x, y, heading});
    }
    return poses;
}

/**
 * GeoJSON for the display-features source: each peer's URDF silhouette (the
 * same shape as this robot, in FLEET_PEER_COLOR) and a labelled centre point.
 * Peers run the same software, so our robot geometry stands in for theirs.
 */
export function peerDisplayFeatures(
    peers: PeerMapPose[], offsetX: number, offsetY: number, datum: Datum, robot: RobotGeometry,
): Feature[] {
    return peers.flatMap((peer): Feature[] => {
        const silhouette = drawRobotSilhouette(offsetX, offsetY, datum, peer.y, peer.x, peer.heading, robot);
        return [
            {
                type: "Feature",
                id: `fleet-peer-${peer.id}-footprint`,
                geometry: {type: "Polygon", coordinates: [silhouette.chassis]},
                properties: {feature_type: "mower-footprint", color: FLEET_PEER_COLOR},
            },
            {
                type: "Feature",
                id: `fleet-peer-${peer.id}`,
                geometry: {type: "Point", coordinates: transpose(offsetX, offsetY, datum, peer.y, peer.x)},
                properties: {feature_type: "fleet-peer", color: FLEET_PEER_COLOR, name: peer.name},
            },
        ];
    });
}
