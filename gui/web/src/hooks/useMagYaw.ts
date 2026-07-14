import {Imu} from "../types/ros.ts";
import {useTopic} from "./useTopic.ts";

/**
 * Subscribes to `/imu/mag_yaw` — a tilt-compensated magnetometer yaw
 * published by `mag_yaw_publisher.py` when `mag_calibration.yaml`
 * exists on disk (default: off). The message carries only `orientation`
 * (quaternion with yaw set) and `orientation_covariance[8]` (yaw
 * variance). Consumed by `fusion_graph_node` as a unary yaw factor.
 */
export const useMagYaw = (): { imu: Imu; lastMessageAt: number | null } => {
    const {data: imu, lastMessageAt} = useTopic<Imu>("magYaw", {}, {withTimestamp: true});
    return {imu, lastMessageAt};
};
