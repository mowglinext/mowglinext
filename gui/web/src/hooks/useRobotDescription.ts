import { useEffect, useRef, useState } from "react";
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
 * Parse the processed URDF XML to extract robot geometry.
 *
 * The URDF is the output of xacro processing — all property substitutions
 * are resolved. We extract geometry from the actual XML elements:
 *   - base_link visual <box size="L W H"/> for chassis dimensions
 *   - base_link visual <origin xyz="cx 0 ..."/> for chassis center offset
 *   - left_wheel_joint <origin xyz="xoff y 0"/> for wheel offset and track
 *   - left_wheel_link <cylinder radius="R" length="W"/> for wheel size
 *   - blade_link <cylinder radius="R"/> for blade radius
 *   - front_left_caster_joint <origin xyz="x y z"/> for caster position
 *   - front_left_caster_link <cylinder radius="R"/> for caster radius
 */
const parseUrdf = (xml: string): RobotGeometry => {
    const result = { ...DEFAULTS };

    try {
        const parser = new DOMParser();
        const doc = parser.parseFromString(xml, "text/xml");

        // Helper: find a <joint name="..."> and get its <origin xyz="...">
        const jointOrigin = (jointName: string): number[] | null => {
            const joints = doc.getElementsByTagName("joint");
            for (let i = 0; i < joints.length; i++) {
                if (joints[i].getAttribute("name") === jointName) {
                    const origin = joints[i].getElementsByTagName("origin")[0];
                    if (origin) {
                        const xyz = origin.getAttribute("xyz")?.split(/\s+/).map(Number);
                        if (xyz && xyz.length >= 3) return xyz;
                    }
                }
            }
            return null;
        };

        // Helper: find first <cylinder> inside a <link name="..."> visual
        const linkCylinder = (linkName: string): { radius: number; length: number } | null => {
            const links = doc.getElementsByTagName("link");
            for (let i = 0; i < links.length; i++) {
                if (links[i].getAttribute("name") === linkName) {
                    const cyl = links[i].getElementsByTagName("cylinder")[0];
                    if (cyl) {
                        return {
                            radius: parseFloat(cyl.getAttribute("radius") || "0"),
                            length: parseFloat(cyl.getAttribute("length") || "0"),
                        };
                    }
                }
            }
            return null;
        };

        // base_link: first <box> gives chassis dimensions, first <origin> gives offset
        const links = doc.getElementsByTagName("link");
        for (let i = 0; i < links.length; i++) {
            if (links[i].getAttribute("name") === "base_link") {
                const box = links[i].getElementsByTagName("box")[0];
                if (box) {
                    const size = box.getAttribute("size")?.split(/\s+/).map(Number);
                    if (size && size.length >= 3) {
                        result.baseLength = size[0];
                        result.baseWidth = size[1];
                        result.baseHeight = size[2];
                    }
                }
                // The visual origin x gives the chassis center offset from base_link
                const visual = links[i].getElementsByTagName("visual")[0];
                if (visual) {
                    const origin = visual.getElementsByTagName("origin")[0];
                    if (origin) {
                        const xyz = origin.getAttribute("xyz")?.split(/\s+/).map(Number);
                        if (xyz && xyz.length >= 1) {
                            result.chassisCenterX = xyz[0];
                        }
                    }
                }
                break;
            }
        }

        // Wheels: left_wheel_joint origin gives wheelXOffset (x) and half wheelTrack (y)
        const lwOrigin = jointOrigin("left_wheel_joint");
        if (lwOrigin) {
            result.wheelXOffset = lwOrigin[0];
            result.wheelTrack = Math.abs(lwOrigin[1]) * 2;
        }

        // Wheel dimensions from link cylinder
        const wheelCyl = linkCylinder("left_wheel_link");
        if (wheelCyl) {
            result.wheelRadius = wheelCyl.radius;
            result.wheelWidth = wheelCyl.length;
        }

        // Blade radius
        const bladeCyl = linkCylinder("blade_link");
        if (bladeCyl) {
            result.bladeRadius = bladeCyl.radius;
        }

        // Caster position and size
        const casterOrigin = jointOrigin("front_left_caster_joint");
        if (casterOrigin) {
            result.casterXOffset = casterOrigin[0];
            result.casterTrack = Math.abs(casterOrigin[1]) * 2;
        }
        const casterCyl = linkCylinder("front_left_caster_link");
        if (casterCyl) {
            result.casterRadius = casterCyl.radius;
        }
    } catch {
        // Parse error — return defaults
    }

    return result;
};

/**
 * Subscribe to /robot_description and parse robot geometry from the URDF.
 * Returns DEFAULTS immediately and updates when the URDF is received.
 */
export const useRobotDescription = (): RobotGeometry => {
    const [geometry, setGeometry] = useState<RobotGeometry>(DEFAULTS);
    const receivedRef = useRef(false);

    const stream = useWS<string>(
        () => { /* error */ },
        () => { /* info */ },
        (raw) => {
            if (receivedRef.current) return; // only need the first message
            try {
                const msg = raw as { data?: string; Data?: string };
                // rosbridge delivers std_msgs/String as {"data": "..."}
                // After snakeToPascal it may be {"Data": "..."} or {"data": "..."}
                const urdfXml: string = msg.Data ?? msg.data ?? "";
                if (urdfXml.length > 0) {
                    receivedRef.current = true;
                    setGeometry(parseUrdf(urdfXml));
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
