import type {RobotGeometry} from "../../hooks/useRobotDescription";
import type {RobotView, Bounds} from "../../utils/robotModel";
import {dockBounds, dockVisualGeometry, type LayeredDockStyle} from "./dockGeometry";

type Definition = {label: string; view: RobotView; step: number; precision: number; min?: number; max?: number};
const dimension = (label: string, view: RobotView = "top", step = .01): Definition =>
    ({label, view, step, precision: 3, min: .001});
export const MEASUREMENTS = {
    wheel_radius: {...dimension("wheelRadius", "side", .001), precision: 5, min: .00001},
    wheel_track: dimension("wheelTrack", "top", .005),
    blade_radius: dimension("bladeRadius"),
    blade_x_offset: {...dimension("bladeXOffset", "top", .001), precision: 4, min: undefined},
    blade_y_offset: {...dimension("bladeYOffset", "top", .001), precision: 4, min: undefined},
    tool_width: dimension("toolWidth"),
    chassis_length: dimension("chassisLength"),
    chassis_width: dimension("chassisWidth"),
    chassis_height: dimension("chassisHeight", "side"),
    chassis_z_offset: {...dimension("chassisZOffset", "side", .005), min: -.3, max: .3},
    chassis_center_x: {...dimension("chassisCenterX"), min: undefined},
    wheel_width: dimension("wheelWidth", "top", .005),
    wheel_x_offset: {...dimension("wheelXOffset"), min: undefined},
    caster_radius: dimension("casterRadius", "side", .005),
    caster_track: {...dimension("casterTrack"), min: 0},
    caster_x_offset: {...dimension("casterXOffset"), min: -.99, max: 2},
    dock_visual_length: {...dimension("dock_visual_length"), min: .1, max: 3},
    dock_visual_width: {...dimension("dock_visual_width"), min: .1, max: 3},
    dock_visual_x_offset: {...dimension("dock_visual_x_offset"), min: -2, max: 2},
    dock_visual_y_offset: {...dimension("dock_visual_y_offset"), min: -2, max: 2},
} satisfies Record<string, Definition>;
export type MeasurementKey = keyof typeof MEASUREMENTS;
export type Point = [number, number];
export type Measurement = {key: MeasurementKey; view: RobotView; from: Point; to: Point;
    fromLabel: string; toLabel: string; value: number; axis: "x" | "y"};
export function validMeasurement(key: MeasurementKey, raw: unknown): raw is number {
    const spec: Definition = MEASUREMENTS[key];
    return typeof raw === "number" && Number.isFinite(raw)
        && (spec.min === undefined || raw >= spec.min) && (spec.max === undefined || raw <= spec.max);
}

/** Endpoints use the physical geometry, never the artwork's alpha envelope.
 * Coordinate offsets are relative to the robot origin, even with a nonzero wheel X.
 * Dock offsets instead start at their registered built-in alignment. */
export function measurementGeometry(key: MeasurementKey, r: RobotGeometry, values: Record<string, unknown>,
    dockStyle: LayeredDockStyle): Measurement | null {
    const raw = values[key];
    const value = typeof raw === "string" && raw.trim() ? Number(raw) : raw;
    if (!validMeasurement(key, value)) return null;
    const front = -r.chassisCenterX-r.baseLength/2, rear = -r.chassisCenterX+r.baseLength/2;
    const bottom = -(r.chassisCenterZ ?? r.baseHeight/2)+r.baseHeight/2;
    const wheel: Point = [-r.wheelXOffset, -(r.wheelZ ?? 0)];
    const caster: Point = [-r.casterXOffset, -(r.casterZ ?? (-r.wheelRadius+r.casterRadius))];
    const dock = dockVisualGeometry(values, r.wheelXOffset, dockStyle, r.chassisCenterX);
    const nominal = dockVisualGeometry({...values, dock_visual_x_offset: 0, dock_visual_y_offset: 0},
        r.wheelXOffset, dockStyle, r.chassisCenterX);
    const db = dockBounds("top", dockStyle, dock);
    const result = (from: Point, to: Point, fromLabel: string, toLabel: string, axis: "x" | "y"): Measurement =>
        ({key, view: MEASUREMENTS[key].view, from, to, fromLabel, toLabel, value, axis});
    switch (key) {
        case "chassis_length": return result([0, rear], [0, front], "rearEdge", "frontEdge", "y");
        case "chassis_width": return result([-r.baseWidth/2, -r.chassisCenterX], [r.baseWidth/2, -r.chassisCenterX], "leftEdge", "rightEdge", "x");
        case "chassis_height": return result([-r.chassisCenterX, bottom], [-r.chassisCenterX, bottom-r.baseHeight], "bodyBottom", "bodyTop", "y");
        case "chassis_z_offset": return result([-r.chassisCenterX, 0], [-r.chassisCenterX, bottom], "axleHeight", "bodyBottom", "y");
        case "chassis_center_x": return result([0, 0], [0, -r.chassisCenterX], "robotOrigin", "bodyCentre", "y");
        case "wheel_x_offset": return result([0, 0], [0, -r.wheelXOffset], "robotOrigin", "driveAxle", "y");
        case "caster_x_offset": return result([-r.casterTrack/2, 0], [-r.casterTrack/2, -r.casterXOffset], "robotOrigin", "casterAxle", "y");
        case "wheel_track": return result([-r.wheelTrack/2, -r.wheelXOffset], [r.wheelTrack/2, -r.wheelXOffset], "leftWheelCentre", "rightWheelCentre", "x");
        case "caster_track": return result([-r.casterTrack/2, -r.casterXOffset], [r.casterTrack/2, -r.casterXOffset], "leftCasterCentre", "rightCasterCentre", "x");
        case "wheel_width": return result([-r.wheelTrack/2+r.wheelWidth/2, -r.wheelXOffset], [-r.wheelTrack/2-r.wheelWidth/2, -r.wheelXOffset], "innerTyre", "outerTyre", "x");
        case "wheel_radius": return result(wheel, [wheel[0], wheel[1]+r.wheelRadius], "driveAxle", "tyreEdge", "y");
        case "caster_radius": return result(caster, [caster[0], caster[1]+r.casterRadius], "casterAxle", "tyreEdge", "y");
        case "blade_x_offset": return result([-(r.bladeY ?? 0), -r.chassisCenterX],
            [-(r.bladeY ?? 0), -(r.bladeX ?? r.chassisCenterX)], "bodyCentreLine", "bladeCentre", "y");
        case "blade_y_offset": return result([0, -(r.bladeX ?? r.chassisCenterX)],
            [-(r.bladeY ?? 0), -(r.bladeX ?? r.chassisCenterX)], "bodyCentreLine", "bladeCentre", "x");
        case "blade_radius": return result([-(r.bladeY ?? 0), -(r.bladeX ?? r.chassisCenterX)],
            [-(r.bladeY ?? 0)+r.bladeRadius, -(r.bladeX ?? r.chassisCenterX)], "bladeCentre", "cuttingEdge", "x");
        case "tool_width": return result([-(r.bladeY ?? 0)-value/2, -(r.bladeX ?? r.chassisCenterX)], [-(r.bladeY ?? 0)+value/2, -(r.bladeX ?? r.chassisCenterX)], "swathLeft", "swathRight", "x");
        case "dock_visual_length": return result([db.x+db.width/2, db.y+db.height], [db.x+db.width/2, db.y], "dockRear", "dockFront", "y");
        case "dock_visual_width": return result([db.x, db.y+db.height/2], [db.x+db.width, db.y+db.height/2], "dockLeft", "dockRight", "x");
        case "dock_visual_x_offset": return result([-dock.centerY, -nominal.centerX], [-dock.centerY, -dock.centerX], "builtInAlignment", "adjustedAlignment", "y");
        case "dock_visual_y_offset": return result([-nominal.centerY, -dock.centerX], [-dock.centerY, -dock.centerX], "builtInAlignment", "adjustedAlignment", "x");
    }
}

export function measurementLayout(m: Measurement, bounds: Bounds) {
    const x = Math.min(bounds.x, m.from[0], m.to[0]);
    const y = Math.min(bounds.y, m.from[1], m.to[1]);
    const right = Math.max(bounds.x+bounds.width, m.from[0], m.to[0]);
    const bottom = Math.max(bounds.y+bounds.height, m.from[1], m.to[1]);
    const unit = Math.max(right-x, bottom-y)/24;
    const line = m.axis === "x" ? bottom+unit*2 : right+unit*2;
    const from: Point = m.axis === "x" ? [m.from[0], line] : [line, m.from[1]];
    const to: Point = m.axis === "x" ? [m.to[0], line] : [line, m.to[1]];
    return {unit, from, to, bounds: {x: x-unit, y: y-unit,
        width: right-x+unit*(m.axis === "y" ? 7 : 2),
        height: bottom-y+unit*(m.axis === "x" ? 6 : 2)}};
}
