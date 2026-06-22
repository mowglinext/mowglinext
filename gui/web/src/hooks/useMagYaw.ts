import {useEffect, useState} from "react";
import {Imu} from "../types/ros.ts";
import {useWS} from "./useWS.ts";

/**
 * Subscribes to `/imu/mag_yaw` — a tilt-compensated magnetometer yaw
 * published by `mag_yaw_publisher.py` when `mag_calibration.yaml`
 * exists on disk (default: off). The message carries only `orientation`
 * (quaternion with yaw set) and `orientation_covariance[8]` (yaw
 * variance). Fused by ekf_map in robot_localization's dual-EKF setup.
 */
export const useMagYaw = (): { imu: Imu; lastMessageAt: number | null } => {
    const [imu, setImu] = useState<Imu>({});
    const [lastMessageAt, setLastMessageAt] = useState<number | null>(null);
    const stream = useWS<string>(() => {
            console.log({message: "Mag yaw stream closed"});
        }, () => {
            console.log({message: "Mag yaw stream connected"});
        },
        (e) => {
            setImu((e as any));
            setLastMessageAt(Date.now());
        });
    useEffect(() => {
        stream.start("/api/mowglinext/subscribe/magYaw");
        return () => { stream.stop(); };
    }, []);
    return {imu, lastMessageAt};
};
