import {DOCK_ARTWORK, type LayeredDockStyle} from "./dockAssets";
import {describe, expect, it} from "vitest";
import {dockBounds, dockedAssemblyBounds, dockVisualGeometry} from "./dockGeometry";
import {parseRobotUrdf} from "../../utils/robotModel";
import {ROBOT_URDF} from "../../test/robotUrdf";

describe("dock display geometry", () => {
    it("uses the full footprint and the parked rear axle in both projections", () => {
        const geometry = dockVisualGeometry({dock_visual_length: .8, dock_visual_width: .6, dock_visual_x_offset: .2}, .05);
        const top = dockBounds("top", "standard", geometry);
        const side = dockBounds("side", "standard", geometry);
        expect(top.width).toBe(.6);
        expect(top.height).toBe(.8);
        expect(top.y).toBeCloseTo(-.87);
        expect(side.x).toBe(top.y);
        expect(side.width).toBe(top.height);
    });
    it("positions the full side-charging footprint using explicit centre coordinates", () => {
        const bounds = dockBounds("top", "worx", {length: .68, width: .56, centerX: .17, centerY: -.04});
        expect(bounds.x + .56 * (355 - 47) / 721).toBeCloseTo(.04);
        expect(bounds.y).toBeCloseTo(-.51);
        expect(bounds.width).toBe(.56);
    });
    it("rejects invalid saved dimensions without collapsing the renderer", () => {
        expect(dockVisualGeometry({dock_visual_length: 0, dock_visual_width: "", dock_visual_x_offset: Infinity})).toEqual(dockVisualGeometry());
        expect(dockVisualGeometry({dock_visual_x_offset: 0}).centerX).toBe(.220);
        expect(dockVisualGeometry().centerX).toBe(.220);
        expect(dockVisualGeometry({dock_visual_width: "0.7"}).width).toBe(.7);
    });
    it("frames an oversized, rearward dock without moving any robot component", () => {
        const robot = parseRobotUrdf(ROBOT_URDF)!;
        const original = structuredClone(robot);
        const geometry = {length: 1.2, width: .9, centerX: -.5, centerY: .2};
        for (const view of ["top", "side"] as const) {
            const bounds = dockedAssemblyBounds(robot, view, .05, "standard", geometry);
            const dock = dockBounds(view, "standard", geometry);
            expect(bounds.x).toBeLessThanOrEqual(dock.x);
            expect(bounds.x + bounds.width).toBeGreaterThanOrEqual(dock.x + dock.width);
            if (view === "top") {
                expect(bounds.y).toBeLessThan(dock.y);
                expect(bounds.y + bounds.height).toBeGreaterThan(dock.y + dock.height);
            }
        }
        expect(robot).toEqual(original);
    });
});

it("moves the top/map dock left for positive Y without altering the side projection", () => {
    const baseline = dockVisualGeometry();
    const shifted = dockVisualGeometry({dock_visual_y_offset: .12});
    for (const style of Object.keys(DOCK_ARTWORK) as LayeredDockStyle[]) {
        const before = dockBounds("top", style, baseline);
        const after = dockBounds("top", style, shifted);
        expect(after.x - before.x).toBeCloseTo(-.12);
        expect(after.y).toBe(before.y);
        expect(after.width).toBe(before.width);
        expect(dockBounds("side", style, shifted)).toEqual(dockBounds("side", style, baseline));
        expect(before.x + before.width * DOCK_ARTWORK[style].topAnchor[0]).toBeCloseTo(0);
    }
});


it("centres the Worx slatted tray on the chassis at zero display offsets in preview and map", () => {
    const robot = parseRobotUrdf(ROBOT_URDF)!;
    const geometry = dockVisualGeometry({dock_visual_x_offset: 0, dock_visual_y_offset: 0}, robot.wheelXOffset, "worx", robot.chassisCenterX);
    const top = dockBounds("top", "worx", geometry);
    const side = dockBounds("side", "worx", geometry);
    expect(top.y + top.height / 2).toBeCloseTo(-robot.chassisCenterX);
    expect(top.x + top.width * 308 / 721).toBeCloseTo(0);
    expect(side.x + side.width / 2).toBeCloseTo(-robot.chassisCenterX);
});
