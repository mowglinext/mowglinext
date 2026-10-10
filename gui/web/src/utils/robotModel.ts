import type {RobotGeometry} from "../hooks/useRobotDescription";

export type SensorId = "gps" | "lidar" | "imu";
export type SensorGeometry = {
    id: SensorId;
    x: number; y: number; z: number;
    roll: number; pitch: number; yaw: number;
    length: number; width: number; height: number;
};
export type RobotView = "top" | "side";
export type Bounds = {x: number; y: number; width: number; height: number};

const vector = (value: string | null, fallback: number[] = [0, 0, 0]): number[] => {
    if (value === null) return fallback;
    const parts = value.trim().split(/\s+/).map(Number);
    if (parts.length !== 3 || !parts.every(Number.isFinite)) throw new Error("Invalid URDF vector");
    return parts;
};
const positive = (value: string | null): number => {
    const n = Number(value);
    if (!Number.isFinite(n) || n <= 0) throw new Error("Invalid URDF dimension");
    return n;
};

/** Read visual geometry, never collision/inertial origins. Unsupported geometry
 * is rejected rather than presenting guessed dimensions as the running robot. */
export function parseRobotUrdf(xml: string): RobotGeometry | null {
    try {
        const doc = new DOMParser().parseFromString(xml, "text/xml");
        if (doc.querySelector("parsererror") || !doc.querySelector("robot")) return null;
        const link = (name: string) => Array.from(doc.querySelectorAll("robot > link")).find(e => e.getAttribute("name") === name);
        const joint = (name: string) => Array.from(doc.querySelectorAll("robot > joint")).find(e => e.getAttribute("name") === name);
        const origin = (node: Element | null | undefined) => vector(node?.querySelector(":scope > origin")?.getAttribute("xyz") ?? null);
        const directJoint = (name: string) => {
            const j = joint(name);
            if (!j || j.querySelector("parent")?.getAttribute("link") !== "base_link") throw new Error("Unsupported joint");
            return j;
        };
        const cylinder = (name: string) => {
            const c = link(name)?.querySelector("visual > geometry > cylinder");
            if (!c) throw new Error("Missing cylinder");
            return {radius: positive(c.getAttribute("radius")), length: positive(c.getAttribute("length"))};
        };
        const visual = link("base_link")?.querySelector("visual");
        const box = visual?.querySelector("geometry > box");
        if (!box) return null;
        const size = vector(box.getAttribute("size"), []);
        if (size.length !== 3 || size.some(n => n <= 0)) return null;
        const body = origin(visual);
        // Our shell is axis-aligned in base_link. Don't silently misrepresent a
        // custom rotated/off-centre body as an ordinary Mowgli chassis.
        if (body[1] !== 0 || vector(visual?.querySelector("origin")?.getAttribute("rpy") ?? null).some(n => n !== 0)) return null;
        const wheel = cylinder("left_wheel_link");
        const wheelOrigin = origin(directJoint("left_wheel_joint"));
        const caster = cylinder("front_left_caster_link");
        const casterOrigin = origin(directJoint("front_left_caster_joint"));
        const blade = cylinder("blade_link");
        const bladeOrigin = origin(directJoint("blade_joint"));
        const sensors: SensorGeometry[] = [];
        for (const id of ["gps", "lidar", "imu"] as const) {
            const j = joint(id + "_joint");
            const v = link(id + "_link")?.querySelector("visual");
            if (!j || !v) continue; // A missing sensor is not an invented sensor.
            directJoint(id + "_joint");
            const p = origin(j);
            const rpy = vector(j.querySelector("origin")?.getAttribute("rpy") ?? null);
            const offset = origin(v);
            // Apply the joint rotation to the visual origin.
            const [ox, oy, oz] = rotatePoint(offset, rpy);
            const visualRpy = vector(v.querySelector("origin")?.getAttribute("rpy") ?? null);
            if (visualRpy.some(n => n !== 0)) return null;
            const b = v.querySelector("geometry > box");
            const c = v.querySelector("geometry > cylinder");
            const dims = b ? vector(b.getAttribute("size"), []) : c
                ? [2 * positive(c.getAttribute("radius")), 2 * positive(c.getAttribute("radius")), positive(c.getAttribute("length"))] : [];
            if (dims.length !== 3 || dims.some(n => n <= 0)) return null;
            sensors.push({id, x: p[0]+ox, y: p[1]+oy, z: p[2]+oz,
                roll: rpy[0], pitch: rpy[1], yaw: rpy[2], length: dims[0], width: dims[1], height: dims[2]});
        }
        return {
            baseLength: size[0], baseWidth: size[1], baseHeight: size[2],
            chassisCenterX: body[0], chassisCenterZ: body[2],
            wheelRadius: wheel.radius, wheelWidth: wheel.length,
            wheelXOffset: wheelOrigin[0], wheelTrack: Math.abs(wheelOrigin[1])*2, wheelZ: wheelOrigin[2],
            casterRadius: caster.radius, casterWidth: caster.length,
            casterXOffset: casterOrigin[0], casterTrack: Math.abs(casterOrigin[1])*2, casterZ: casterOrigin[2],
            bladeRadius: blade.radius, bladeX: bladeOrigin[0], bladeY: bladeOrigin[1], bladeZ: bladeOrigin[2] + origin(link("blade_link")?.querySelector("visual"))[2], bladeThickness: blade.length,
            sensors, fromUrdf: true,
        };
    } catch { return null; }
}

export function rotatePoint([x, y, z]: number[], [roll, pitch, yaw]: number[]): number[] {
    const a = y*Math.cos(roll)-z*Math.sin(roll), b = y*Math.sin(roll)+z*Math.cos(roll);
    const c = x*Math.cos(pitch)+b*Math.sin(pitch), d = -x*Math.sin(pitch)+b*Math.cos(pitch);
    return [c*Math.cos(yaw)-a*Math.sin(yaw), c*Math.sin(yaw)+a*Math.cos(yaw), d];
}

/** Values supplied by the settings API already include backend/model defaults.
 * Override only finite values; no second table of physical defaults here. */
export function previewRobotGeometry(live: RobotGeometry, values: Record<string, unknown>): RobotGeometry {
    const next = {...live};
    const keys = {
        baseLength: "chassis_length", baseWidth: "chassis_width", baseHeight: "chassis_height",
        chassisCenterX: "chassis_center_x", wheelRadius: "wheel_radius", wheelWidth: "wheel_width",
        wheelTrack: "wheel_track", wheelXOffset: "wheel_x_offset", casterRadius: "caster_radius",
        casterTrack: "caster_track", bladeRadius: "blade_radius",
    } as const;
    const read = (key: string) => {
        const value = values[key];
        return (typeof value === "number" || (typeof value === "string" && value.trim() !== "")) && Number.isFinite(Number(value)) ? Number(value) : undefined;
    };
    for (const [field, key] of Object.entries(keys) as [keyof typeof keys, string][]) {
        const n = read(key);
        if (n !== undefined && (field === "chassisCenterX" || field === "wheelXOffset" || n > 0)) next[field] = n;
    }
    // These derived relationships are the ones in mowgli.urdf.xacro.
    const configuredCasterX = read("caster_x_offset");
    if (configuredCasterX !== undefined && configuredCasterX !== -1) next.casterXOffset = configuredCasterX;
    else if (configuredCasterX === -1 || next.baseLength !== live.baseLength || next.chassisCenterX !== live.chassisCenterX || next.casterRadius !== live.casterRadius)
        next.casterXOffset = next.chassisCenterX + next.baseLength/2 - next.casterRadius;
    if (next.baseHeight !== live.baseHeight) next.chassisCenterZ = next.baseHeight/2;
    if (next.wheelRadius !== live.wheelRadius || next.casterRadius !== live.casterRadius)
        next.casterZ = -next.wheelRadius + next.casterRadius;
    if (next.chassisCenterX !== live.chassisCenterX) next.bladeX = next.chassisCenterX;
    if (next.wheelRadius !== live.wheelRadius) next.bladeZ = (live.bladeZ ?? 0) + live.wheelRadius - next.wheelRadius;
    next.sensors = live.sensors?.map(sensor => {
        const s = {...sensor};
        for (const axis of ["x", "y", "z", "roll", "pitch", "yaw"] as const) {
            const n = read(sensor.id + "_" + axis);
            if (n !== undefined) s[axis] = n;
        }
        return s;
    });
    return next;
}

export function projectPoint(x: number, y: number, z: number, view: RobotView): [number, number] {
    return view === "top" ? [-y, -x] : [-x, -z];
}

export function sensorCorners(sensor: SensorGeometry, view: RobotView): [number, number][] {
    const points: [number, number][] = [];
    for (const x of [-1, 1]) for (const y of [-1, 1]) for (const z of [-1, 1]) {
        const p = rotatePoint([x*sensor.length/2, y*sensor.width/2, z*sensor.height/2], [sensor.roll, sensor.pitch, sensor.yaw]);
        points.push(projectPoint(sensor.x+p[0], sensor.y+p[1], sensor.z+p[2], view));
    }
    return points;
}

/** Metre bounds include sensors and wheels. Padding is a viewport margin only,
 * never part of the chassis dimensions or pose anchor. */
export function robotBounds(r: RobotGeometry, view: RobotView, padding = 0.04): Bounds {
    const points: number[][] = [];
    const box = (x: number, y: number, w: number, h: number) => { points.push([x-w/2, y-h/2], [x+w/2, y+h/2]); };
    if (view === "top") {
        box(0, -r.chassisCenterX, r.baseWidth, r.baseLength);
        for (const sign of [-1, 1]) {
            box(sign*r.wheelTrack/2, -r.wheelXOffset, r.wheelWidth, r.wheelRadius*2);
            box(sign*r.casterTrack/2, -r.casterXOffset, r.casterWidth ?? r.casterRadius, r.casterRadius*2);
        }
    } else {
        box(-r.chassisCenterX, -(r.chassisCenterZ ?? r.baseHeight/2), r.baseLength, r.baseHeight);
        box(-r.wheelXOffset, -(r.wheelZ ?? 0), r.wheelRadius*2, r.wheelRadius*2);
        box(-r.casterXOffset, -(r.casterZ ?? (-r.wheelRadius+r.casterRadius)), r.casterRadius*2, r.casterRadius*2);
    }
    if (view === "top") box(-(r.bladeY ?? 0), -(r.bladeX ?? r.chassisCenterX), r.bladeRadius*2, r.bladeRadius*2);
    else box(-(r.bladeX ?? r.chassisCenterX), -(r.bladeZ ?? 0), r.bladeRadius*2, r.bladeThickness ?? .01);
    for (const sensor of r.sensors ?? []) points.push(...sensorCorners(sensor, view));
    const x = Math.min(...points.map(p=>p[0]))-padding, y = Math.min(...points.map(p=>p[1]))-padding;
    return {x, y, width: Math.max(...points.map(p=>p[0]))+padding-x, height: Math.max(...points.map(p=>p[1]))+padding-y};
}
