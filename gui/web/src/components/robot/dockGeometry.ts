import {DEFAULT_DOCK_STYLE} from "./dockAssets";
import {DOCK_ARTWORK, type LayeredDockStyle} from "./dockAssets";
export type {LayeredDockStyle} from "./dockAssets";
import type {RobotGeometry} from "../../hooks/useRobotDescription";
import {assemblyBounds} from "./assemblyBounds";
import type {Bounds, RobotView} from "../../utils/robotModel";

// Display geometry only: never used to move/calibrate the saved docking pose.
// Nominal defaults are illustration dimensions, not surveyed hardware values.
export const DOCK_VISUAL_DEFAULTS = {
    dock_visual_length: .67,
    dock_visual_width: .46,
    dock_visual_x_offset: 0,
    dock_visual_y_offset: 0,
};
export const DOCK_VISUAL_KEYS = Object.keys(DOCK_VISUAL_DEFAULTS);
export type DockVisualGeometry = {length: number; width: number; centerX: number; centerY: number};
export const setupDockOpacity = (transparent: boolean) => transparent ? .18 : .7;

/** Standard docks use the parked rear axle. Worx offsets locate the slatted
 * platform relative to the chassis centre; zero centres the mower on its tray.
 * This is artwork registration only, never a change to the calibrated dock pose. */
export function dockVisualGeometry(values: Record<string, unknown> = {}, wheelXOffset = 0, style: LayeredDockStyle = DEFAULT_DOCK_STYLE, chassisCenterX = wheelXOffset): DockVisualGeometry {
    const read = (key: keyof typeof DOCK_VISUAL_DEFAULTS, min: number, max: number) => {
        const raw = values[key];
        const n = typeof raw === "number" ? raw : typeof raw === "string" && raw.trim() ? Number(raw) : NaN;
        return Number.isFinite(n) && n >= min && n <= max ? n : DOCK_VISUAL_DEFAULTS[key];
    };
    const artwork = DOCK_ARTWORK[style];
    const origins = {rearAxle: wheelXOffset, chassisCenter: chassisCenterX};
    return {length: read("dock_visual_length", .1, 3), width: read("dock_visual_width", .1, 3),
        centerX: read("dock_visual_x_offset", -2, 2) + origins[artwork.origin] + artwork.offset.forward,
        centerY: read("dock_visual_y_offset", -2, 2) + artwork.offset.left};
}

export function dockBounds(view: RobotView, style: LayeredDockStyle = DEFAULT_DOCK_STYLE, geometry = dockVisualGeometry()): Bounds {
    const {length, width, centerX, centerY} = geometry;
    const artwork = DOCK_ARTWORK[style];
    const anchor = view === "top" ? artwork.topAnchor : artwork.sideAnchor;
    return view === "top"
        ? {x: -centerY - width * anchor[0], y: -centerX - length * anchor[1], width, height: length}
        : {x: -centerX - length * anchor[0], y: -artwork.height * anchor[1], width: length, height: artwork.height};
}

const DOCK_TRAY_HEIGHT = .02;
export function previewDockOffsetY(robot: RobotGeometry, view: RobotView): number {
    return view === "top" ? 0 : robot.wheelRadius - (robot.wheelZ ?? 0) + DOCK_TRAY_HEIGHT;
}

/** Include the dock in framing only; robot scale and origin stay fixed. */
export function dockedAssemblyBounds(robot: RobotGeometry, view: RobotView, padding = .05,
    style: LayeredDockStyle = DEFAULT_DOCK_STYLE, geometry = dockVisualGeometry({}, robot.wheelXOffset, style, robot.chassisCenterX)): Bounds {
    const mower = assemblyBounds(robot, view, 0);
    const dock = dockBounds(view, style, geometry);
    const dockY = dock.y + previewDockOffsetY(robot, view);
    const x = Math.min(mower.x, dock.x) - padding;
    const y = Math.min(mower.y, dockY) - padding;
    return {x, y,
        width: Math.max(mower.x + mower.width, dock.x + dock.width) + padding - x,
        height: Math.max(mower.y + mower.height, dockY + dock.height) + padding - y};
}
