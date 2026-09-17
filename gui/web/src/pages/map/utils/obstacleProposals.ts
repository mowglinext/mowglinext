import type {Map, Polygon} from "../../../types/ros.ts";
import {MapObstacleInfoConstants} from "../../../types/ros.ts";
import {mowingAreaIndex} from "../../../utils/mapAreaIndex.ts";

/// One PENDING obstacle proposal published by map_server
/// (MapArea.proposed_obstacles / proposed_obstacle_info). Today the only
/// producer is the wheel-slip dig detector (SOURCE_DIG).
///
/// A proposal is inert on the robot: no keepout, no coverage hole, nothing
/// saved. It exists so the operator can accept it (promote_obstacle with
/// pending_id → applied + persisted) or reject it (discard_obstacle).
export interface ObstacleProposal {
    /// Session-scoped handle for promote_obstacle{pending_id} / discard_obstacle.
    id: number;
    /// Evidence line written by map_server ("Dig at (x, y): wheels … vs pose …").
    name: string;
    source: number;
    /// map_server area index (the ROS id promote_obstacle expects), NOT the
    /// position in map.working_area.
    areaIndex: number;
    areaName: string;
    polygon: Polygon;
}

/// Flatten the proposals of every working area of the live map. Entries
/// without a usable id or polygon, or whose area has no ROS index, are
/// dropped: they could not be accepted or rejected anyway.
export function extractObstacleProposals(map: Map | undefined): ObstacleProposal[] {
    const proposals: ObstacleProposal[] = [];
    (map?.working_area ?? []).forEach((area, position) => {
        const areaIndex = mowingAreaIndex(map, position);
        if (areaIndex === undefined) return;
        (area.proposed_obstacles ?? []).forEach((polygon, j) => {
            const info = area.proposed_obstacle_info?.[j];
            const id = info?.id ?? 0;
            if (id <= 0 || (polygon.points?.length ?? 0) < 3) return;
            proposals.push({
                id,
                name: info?.name ?? "",
                source: info?.source ?? MapObstacleInfoConstants.SOURCE_USER,
                areaIndex,
                areaName: area.name ?? "",
                polygon,
            });
        });
    });
    return proposals;
}

// The generated constants are a const enum while the wire field is a plain
// number; compare on the number.
const SOURCE_DIG: number = MapObstacleInfoConstants.SOURCE_DIG;

export const isDigProposal = (proposal: ObstacleProposal): boolean => proposal.source === SOURCE_DIG;
