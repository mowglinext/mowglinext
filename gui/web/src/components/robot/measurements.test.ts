import {DOCK_ARTWORK, type LayeredDockStyle} from "./dockAssets";
import {describe, expect, it} from "vitest";
import {MEASUREMENTS, measurementGeometry, validMeasurement, type MeasurementKey} from "./measurements";
import {parseRobotUrdf, previewRobotGeometry} from "../../utils/robotModel";
import {ROBOT_URDF} from "../../test/robotUrdf";
import {dockBounds, dockVisualGeometry} from "./dockGeometry";
const live = parseRobotUrdf(ROBOT_URDF)!;
describe("physical measurement endpoints", () => {
    it("uses wheel centres rather than tyre edges for track", () => {
        const values={wheel_track:.32,wheel_width:.08,wheel_x_offset:.06};
        const robot=previewRobotGeometry(live,values);
        const m=measurementGeometry("wheel_track",robot,values,"standard")!;
        expect(m.from).toEqual([-.16,-.06]); expect(m.to).toEqual([.16,-.06]);
        expect(m.to[0]-m.from[0]).toBeCloseTo(.32);
    });
    it("keeps signed chassis offsets relative to the robot frame with shifted wheels", () => {
        const values={chassis_z_offset:-.07,chassis_height:.14,wheel_x_offset:.08,caster_x_offset:.39};
        const robot=previewRobotGeometry(live,values);
        const m=measurementGeometry("chassis_z_offset",robot,values,"standard")!;
        expect(m.from[1]).toBe(0); expect(m.to[1]).toBeCloseTo(.07); expect(m.value).toBe(-.07);
        const caster=measurementGeometry("caster_x_offset",robot,values,"standard")!;
        expect(caster.from[1]).toBe(0); expect(caster.to[1]).toBe(-.39);
    });
    it("shows tool width independently of the blade radius", () => {
        const values={tool_width:.27,blade_radius:.09}, r=previewRobotGeometry(live,values);
        const m=measurementGeometry("tool_width",r,values,"standard")!;
        expect(m.to[0]-m.from[0]).toBe(.27);
        expect(measurementGeometry("blade_radius",r,values,"standard")!.value).toBe(.09);
    });
    for(const style of Object.keys(DOCK_ARTWORK) as LayeredDockStyle[]) it("measures "+style+" dock adjustment from its built-in alignment", () => {
        const values={dock_visual_x_offset:0,dock_visual_y_offset:0,dock_visual_width:.55,dock_visual_length:.8};
        const x=measurementGeometry("dock_visual_x_offset",live,values,style)!;
        expect(x.from).toEqual(x.to); expect(x.value).toBe(0);
        const shifted=measurementGeometry("dock_visual_x_offset",live,{...values,dock_visual_x_offset:-.12},style)!;
        expect(shifted.to[1]-shifted.from[1]).toBeCloseTo(.12);
        const width=measurementGeometry("dock_visual_width",live,values,style)!;
        const b=dockBounds("top",style,dockVisualGeometry(values,live.wheelXOffset,style,live.chassisCenterX));
        expect(width.from[0]).toBe(b.x); expect(width.to[0]).toBe(b.x+.55);
    });
    it("matches every overlay length to its field without bitmap scaling", () => {
        const values={wheel_radius:.11,wheel_track:.32,blade_radius:.09,blade_x_offset:.02,blade_y_offset:-.0575,tool_width:.21,
            chassis_length:.54,chassis_width:.36,chassis_height:.14,chassis_z_offset:-.07,chassis_center_x:.16,
            wheel_width:.04,wheel_x_offset:.02,caster_radius:.025,caster_track:.22,caster_x_offset:.34,
            dock_visual_length:.67,dock_visual_width:.46,dock_visual_x_offset:.1,dock_visual_y_offset:-.08};
        const robot=previewRobotGeometry(live,values);
        for(const key of Object.keys(MEASUREMENTS) as MeasurementKey[]) {
            const m=measurementGeometry(key,robot,values,"worx")!;
            expect(Math.hypot(m.to[0]-m.from[0],m.to[1]-m.from[1]),key).toBeCloseTo(Math.abs(values[key]));
        }
    });
    it("does not label a fallback preview as an empty or invalid value", () => {
        for(const value of [null,"",NaN,Infinity,-.2])
            expect(measurementGeometry("wheel_radius",live,{wheel_radius:value},"standard")).toBeNull();
        expect(validMeasurement("chassis_z_offset",-.07)).toBe(true);
        expect(validMeasurement("caster_track",0)).toBe(true);
        expect(validMeasurement("dock_visual_width",4)).toBe(false);
    });
});
