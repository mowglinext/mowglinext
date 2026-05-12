import {useEffect, useState} from "react";
import {useWS} from "./useWS.ts";

export interface WheelOdom {
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
        covariance?: number[];
    };
}

/**
 * Subscribes to /wheel_odom — raw wheel-derived odometry from the hardware
 * bridge (pre-EKF). Useful in Diagnostics to verify encoder integration
 * independently of the fused map-frame estimate.
 */
export const useWheelOdom = () => {
    const [odom, setOdom] = useState<WheelOdom>({})
    const stream = useWS<string>(() => {
        }, () => {
        },
        (e) => {
            setOdom(JSON.parse(e))
        })
    useEffect(() => {
        stream.start("/api/mowglinext/subscribe/wheelOdom")
        return () => { stream.stop() }
    }, []);
    return odom;
};
