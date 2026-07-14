import {Imu} from "../types/ros.ts";
import {useTopic} from "./useTopic.ts";

/**
 * Subscribes to `/imu/cog_heading` — a synthetic absolute-yaw source
 * published by `cog_to_imu.py` when the GPS fix is RTK Fixed and the
 * wheel odometry indicates forward motion. The message carries only
 * `orientation` (quaternion with yaw set from successive GPS samples)
 * and `orientation_covariance[8]` (yaw variance). Consumed by
 * `fusion_graph_node` as a COG-yaw unary factor.
 */
export const useCogHeading = (): { imu: Imu; lastMessageAt: number | null } => {
    const {data: imu, lastMessageAt} = useTopic<Imu>("cogHeading", {}, {withTimestamp: true});
    return {imu, lastMessageAt};
};
