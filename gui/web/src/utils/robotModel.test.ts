import {describe,expect,it} from "vitest";
import {parseRobotUrdf,previewRobotGeometry,robotBounds,sensorCorners} from "./robotModel";
import {ROBOT_URDF} from "../test/robotUrdf";
describe("shared robot geometry",()=>{
    it("reads visual geometry and mounts without doubling the GPS/IMU footprint",()=>{
        const r=parseRobotUrdf(ROBOT_URDF)!;
        expect(r.fromUrdf).toBe(true);
        expect(r.baseLength).toBe(.6);
        expect(r.wheelRadius).toBe(.1);
        expect(r.casterZ).toBe(-.07);
        expect(r.sensors?.find(s=>s.id==="gps")).toMatchObject({length:.05,width:.05,x:.3,z:.2});
        expect(r.sensors?.find(s=>s.id==="imu")?.width).toBe(.03);
        expect(r.sensors?.find(s=>s.id==="lidar")?.yaw).toBe(3.1408);
    });
    it("overlays edited geometry without resizing wheels/sensors or mutating the live model",()=>{
        const live=parseRobotUrdf(ROBOT_URDF)!;
        const r=previewRobotGeometry(live,{chassis_length:.9,chassis_width:.6,chassis_center_x:.3,gps_x:.4});
        expect([r.baseLength,r.baseWidth,r.casterXOffset]).toEqual([.9,.6,.72]);
        expect(r.wheelRadius).toBe(live.wheelRadius);
        expect(r.sensors?.find(s=>s.id==="gps")).toMatchObject({x:.4,width:.05});
        expect(live.sensors?.find(s=>s.id==="gps")?.x).toBe(.3);
        expect(live.baseLength).toBe(.6);
    });
    it("ignores empty, negative and nonfinite dimension edits",()=>{
        const r=parseRobotUrdf(ROBOT_URDF)!;
        expect(previewRobotGeometry(r,{chassis_length:"",chassis_width:-1,wheel_radius:"Infinity",gps_x:null})).toEqual(r);
    });
    it.each(["<broken", "<robot/>", ROBOT_URDF.replace('0.60 0.45 0.19','NaN 0.45 0.19'),
        ROBOT_URDF.replace('radius="0.1"','radius="-1"')])("rejects malformed geometry",xml=>{
        expect(parseRobotUrdf(xml)).toBeNull();
    });
    it("includes an outboard sensor in viewport bounds and projects all mount rotations",()=>{
        const live=parseRobotUrdf(ROBOT_URDF)!;
        const r=previewRobotGeometry(live,{gps_y:1,gps_z:.8,imu_roll:Math.PI/2});
        expect(robotBounds(r,"top",0).x).toBeCloseTo(-1.025);
        expect(robotBounds(r,"side",0).y).toBeCloseTo(-.81);
        const corners=sensorCorners(r.sensors!.find(s=>s.id==="imu")!,"top");
        const width=Math.max(...corners.map(p=>p[0]))-Math.min(...corners.map(p=>p[0]));
        expect(width).toBeCloseTo(.01);
    });
});

it("uses explicit caster axle placement and retains automatic geometry when reset",()=>{
    const live=parseRobotUrdf(ROBOT_URDF)!;
    expect(previewRobotGeometry(live,{caster_x_offset:.32,chassis_length:.8}).casterXOffset).toBe(.32);
    expect(previewRobotGeometry(live,{caster_x_offset:0}).casterXOffset).toBe(0);
    expect(previewRobotGeometry(live,{caster_x_offset:-1,chassis_length:.8}).casterXOffset).toBeCloseTo(.55);
});

it("moves only the shell when editing vertical offset and preserves that offset on height edits",()=>{
    const live=parseRobotUrdf(ROBOT_URDF)!;
    const next=previewRobotGeometry(live,{chassis_z_offset:-.06});
    expect(next.chassisCenterZ).toBeCloseTo(.035);
    expect(next.baseHeight).toBe(.19);
    expect(next.wheelZ).toBe(live.wheelZ);
    expect(next.casterZ).toBe(live.casterZ);
    expect(next.bladeZ).toBe(live.bladeZ);
    expect(next.sensors).toEqual(live.sensors);
    const taller=previewRobotGeometry(live,{chassis_height:.24});
    expect(taller.chassisCenterZ!-.24/2).toBeCloseTo(-.05);
    expect(previewRobotGeometry(live,{chassis_z_offset:0}).chassisCenterZ).toBeCloseTo(.095);
});
