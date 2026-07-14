import {useTopic} from "./useTopic.ts";

export interface FusionOdom {
    header?: { stamp?: { sec: number; nanosec: number }; frame_id?: string };
    child_frame_id?: string;
    pose?: {
        pose?: {
            position?: { x: number; y: number; z: number };
            orientation?: { x: number; y: number; z: number; w: number };
        };
        covariance?: number[];
    };
    twist?: {
        twist?: {
            linear?: { x: number; y: number; z: number };
            angular?: { x: number; y: number; z: number };
        };
    };
}

/** Odometry arrives fast; coalesce updates so dashboards re-render at ~5 Hz. */
const FUSION_ODOM_THROTTLE_MS = 200;

/**
 * Subscribes to the global filtered odometry published by `fusion_graph_node`,
 * the sole map-frame localizer (it owns both map→odom and odom→base_footprint;
 * the dual-EKF it replaced is gone). The backend exposes it under the topic
 * alias "fusionRaw" for backward compatibility with existing consumers.
 */
export const useFusionOdom = (): FusionOdom =>
    useTopic<FusionOdom>("fusionRaw", {}, {throttleMs: FUSION_ODOM_THROTTLE_MS}).data;
