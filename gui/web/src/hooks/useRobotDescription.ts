import { useEffect, useRef, useState } from "react";
import {parseRobotUrdf, type SensorGeometry} from "../utils/robotModel";
import { useWS } from "./useWS.ts";

export type RobotGeometry = {
    baseLength: number;
    baseWidth: number;
    baseHeight: number;
    chassisCenterX: number;
    wheelRadius: number;
    wheelWidth: number;
    wheelTrack: number;
    wheelXOffset: number;
    casterRadius: number;
    casterXOffset: number;
    casterTrack: number;
    bladeRadius: number;
    fromUrdf?: boolean;
    chassisCenterZ?: number;
    wheelZ?: number;
    casterZ?: number;
    casterWidth?: number;
    bladeX?: number;
    bladeY?: number;
    bladeZ?: number;
    bladeThickness?: number;
    sensors?: SensorGeometry[];
};

const DEFAULTS: RobotGeometry = {
    baseLength: 0.54,
    baseWidth: 0.40,
    baseHeight: 0.19,
    chassisCenterX: 0.18,
    wheelRadius: 0.04475,
    wheelWidth: 0.04,
    wheelTrack: 0.325,
    wheelXOffset: 0.0,
    casterRadius: 0.03,
    casterXOffset: 0.40,
    casterTrack: 0.36,
    bladeRadius: 0.09,
};

/**
 * Subscribe to /robot_description and parse robot geometry from the URDF.
 * Returns DEFAULTS immediately and updates when the URDF is received.
 */
export const useRobotDescription = (): RobotGeometry => {
    const [geometry, setGeometry] = useState<RobotGeometry>(DEFAULTS);
    const receivedXml = useRef("");

    const stream = useWS<string>(
        () => { /* error */ },
        () => { /* info */ },
        (raw) => {
            try {
                const msg = raw as { data?: string; Data?: string };
                // rosbridge delivers std_msgs/String as {"data": "..."}
                // After snakeToPascal it may be {"Data": "..."} or {"data": "..."}
                const urdfXml: string = msg.Data ?? msg.data ?? "";
                if (typeof urdfXml === "string" && urdfXml !== receivedXml.current) {
                    receivedXml.current = urdfXml;
                    // Invalid/unsupported descriptions must not masquerade as a
                    // previously valid running assembly. Legacy consumers retain
                    // their fallback; the assembled map marker requires fromUrdf.
                    setGeometry(parseRobotUrdf(urdfXml) ?? DEFAULTS);
                }
            } catch {
                // ignore parse errors
            }
        }
    );

    useEffect(() => {
        stream.start("/api/mowglinext/subscribe/robotDescription");
        return () => {
            stream.stop();
        };
    }, []);

    return geometry;
};

export { DEFAULTS as DEFAULT_GEOMETRY };
