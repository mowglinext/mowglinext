import {useTopic} from "./useTopic.ts";
import type {FusionOdom} from "./useFusionOdom.ts";

/**
 * Subscribes to /fusion_graph/icp_odometry — the LiDAR-only (scan-match
 * integrated) pose. It is seeded from the graph pose at the first accepted
 * scan match, then drifts; published only while use_scan_matching is on.
 * Used to compare ICP heading/pose against the fused/GPS estimate in the
 * Diagnostics ICP panel. Same Odometry shape as useFusionOdom.
 */
export const useIcpOdom = (): FusionOdom => useTopic<FusionOdom>("icpOdom", {}).data;
