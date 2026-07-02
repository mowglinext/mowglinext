import {useEffect, useState} from "react";
import {useWS} from "./useWS.ts";

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

/**
 * Subscribes to the global filtered odometry published by the active
 * map-frame localizer — `ekf_map_node` by default, or `fusion_graph_node`
 * when `use_fusion_graph` is true. The backend exposes it under the topic
 * alias "fusionRaw" for backward compatibility with existing consumers.
 */
export const useFusionOdom = () => {
    const [odom, setOdom] = useState<FusionOdom>({})
    const stream = useWS<string>(() => {
            console.log({ message: "MapOdometry Stream closed" })
        }, () => {
            console.log({ message: "MapOdometry Stream connected" })
        },
        (e) => {
            setOdom((e as any))
        })
    useEffect(() => {
        stream.start("/api/mowglinext/subscribe/fusionRaw")
        return () => { stream.stop() }
    }, []);
    return odom;
};
