import {useEffect, useState} from "react";
import {Imu} from "../types/ros.ts";
import {useWS} from "./useWS.ts";

/**
 * Subscribes to `/imu/cog_heading` — a synthetic absolute-yaw source
 * published by `cog_to_imu.py` when the GPS fix is RTK Fixed and the
 * wheel odometry indicates forward motion. The message carries only
 * `orientation` (quaternion with yaw set from successive GPS samples)
 * and `orientation_covariance[8]` (yaw variance). Fused by ekf_map
 * in robot_localization's dual-EKF setup.
 */
export const useCogHeading = (): { imu: Imu; lastMessageAt: number | null } => {
    const [imu, setImu] = useState<Imu>({});
    const [lastMessageAt, setLastMessageAt] = useState<number | null>(null);
    const stream = useWS<string>(() => {
            console.log({message: "COG heading stream closed"});
        }, () => {
            console.log({message: "COG heading stream connected"});
        },
        (e) => {
            setImu((e as any));
            setLastMessageAt(Date.now());
        });
    useEffect(() => {
        stream.start("/api/mowglinext/subscribe/cogHeading");
        return () => { stream.stop(); };
    }, []);
    return {imu, lastMessageAt};
};
