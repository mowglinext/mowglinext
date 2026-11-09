import {describe,expect,it} from "vitest";
import {parseRobotUrdf,previewRobotGeometry,visibleRobotGeometry,robotBounds,sensorCorners} from "./robotModel";
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
        expect([r.baseLength,r.baseWidth,r.casterXOffset]).toEqual([.9,.6,live.casterXOffset]);
        expect(r.wheelRadius).toBe(live.wheelRadius);
        expect(r.sensors?.find(s=>s.id==="gps")).toMatchObject({x:.4,width:.05});
        expect(live.sensors?.find(s=>s.id==="gps")?.x).toBe(.3);
        expect(live.baseLength).toBe(.6);
    });
    it("ignores empty, negative and nonfinite dimension edits",()=>{
        const r=parseRobotUrdf(ROBOT_URDF)!;
        expect(previewRobotGeometry(r,{lidar_enabled:true,chassis_length:"",chassis_width:-1,wheel_radius:"Infinity",gps_x:null})).toEqual(r);
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

it("uses fixed caster placement and reads legacy automatic configurations",()=>{
    const live=parseRobotUrdf(ROBOT_URDF)!;
    expect(previewRobotGeometry(live,{caster_x_offset:.32,chassis_length:.8}).casterXOffset).toBe(.32);
    expect(previewRobotGeometry(live,{caster_x_offset:0}).casterXOffset).toBe(0);
    expect(previewRobotGeometry(live,{caster_x_offset:-1,chassis_length:.8}).casterXOffset).toBeCloseTo(.55);
});

it("moves only the shell when editing vertical offset and preserves that offset on height edits",()=>{
    const live=parseRobotUrdf(ROBOT_URDF)!;
    const next=previewRobotGeometry(live,{lidar_enabled:true,chassis_z_offset:-.06});
    expect(next.chassisCenterZ).toBeCloseTo(.035);
    expect(next.baseHeight).toBe(.19);
    expect(next.wheelZ).toBe(live.wheelZ);
    expect(next.casterZ).toBe(live.casterZ);
    expect(next.bladeZ).toBe(live.bladeZ);
    expect(next.sensors).toEqual(live.sensors);
    const taller=previewRobotGeometry(live,{chassis_height:.24});
    expect(taller.chassisCenterZ!-.24/2).toBeCloseTo(-.05);
    expect(previewRobotGeometry(live,{lidar_enabled:true,chassis_z_offset:0}).chassisCenterZ).toBeCloseTo(.095);
});


it("preserves sensor visual offsets when unchanged joint settings are previewed",()=>{
    const xml=ROBOT_URDF.replace('<link name="gps_link"><visual>',
        '<link name="gps_link"><visual><origin xyz="0.02 0 0.03"/>');
    const live=parseRobotUrdf(xml)!;
    expect(live.sensors!.find(s=>s.id==="gps")).toMatchObject({x:.32,z:.23});
    const preview=previewRobotGeometry(live,{gps_x:.3,gps_y:0,gps_z:.2});
    expect(preview.sensors!.find(s=>s.id==="gps")).toEqual(live.sensors!.find(s=>s.id==="gps"));
});

it("rotates visual offsets about the edited joint and matches the resulting URDF",()=>{
    const xml=ROBOT_URDF.replace('<link name="imu_link"><visual>',
        '<link name="imu_link"><visual><origin xyz="0.02 0.01 0.03"/>');
    const live=parseRobotUrdf(xml)!;
    const edited={imu_x:.25,imu_y:.04,imu_z:.12,imu_roll:.4,imu_pitch:-.3,imu_yaw:1.2};
    const expected=parseRobotUrdf(xml.replace('0.18 -0.195 0.095','0.25 0.04 0.12')
        .replace('rpy="0 0 0"/></joint>','rpy="0.4 -0.3 1.2"/></joint>'))!;
    const preview=previewRobotGeometry(live,edited);
    expect(preview.sensors!.find(s=>s.id==="imu")).toEqual(expected.sensors!.find(s=>s.id==="imu"));
    // Feeding a preview through a second edit must not accumulate the offset.
    expect(previewRobotGeometry(preview,edited)).toEqual(preview);
    expect(live.sensors!.find(s=>s.id==="imu")!.mount).toEqual({x:.18,y:-.195,z:.095});
});

// The URDF contains all mounting frames even when no scanner is installed.
describe("sensor artwork visibility", () => {
    const live = parseRobotUrdf(ROBOT_URDF)!;
    for (const disabled of [false, "false", 0, "0", undefined]) it(`hides disabled LiDAR (${disabled}) without losing its saved mount`, () => {
        const values = {lidar_enabled: disabled, lidar_x: .25};
        expect(visibleRobotGeometry(live, values).sensors?.map(s => s.id)).toEqual(["gps", "imu"]);
        expect(previewRobotGeometry(live, values).sensors?.some(s => s.id === "lidar")).toBe(false);
        expect(live.sensors?.some(s => s.id === "lidar")).toBe(true);
        expect(previewRobotGeometry(live, {...values, lidar_enabled: true}).sensors?.find(s => s.id === "lidar")?.mount?.x).toBe(.25);
    });
    for (const enabled of [true, "true", 1, "1"]) it(`preserves the enabled scanner (${enabled})`, () => {
        expect(visibleRobotGeometry(live, {lidar_enabled: enabled})).toBe(live);
    });
});


describe("offset cutting tool previews", () => {
    it("places a right-offset blade independently of the wheel axle and retains single-caster layout", () => {
        const live = parseRobotUrdf(ROBOT_URDF)!;
        const r = previewRobotGeometry(live, {chassis_center_x:.195,wheel_x_offset:-.005,
            blade_x_offset:0,blade_y_offset:-.0575,caster_track:0});
        expect(r.bladeX).toBe(.195);
        expect(r.bladeY).toBe(-.0575);
        expect(r.casterTrack).toBe(0);
        expect(r.wheelXOffset).toBe(-.005);
        expect(live.bladeY).toBe(0);
    });
    it("retains the running blade offset if omitted and follows a moved chassis centre", () => {
        const live = {...parseRobotUrdf(ROBOT_URDF)!,bladeX:.21,bladeY:-.0575};
        const r = previewRobotGeometry(live,{chassis_center_x:.3});
        expect(r.bladeX).toBeCloseTo(.33);
        expect(r.bladeY).toBe(-.0575);
        expect(previewRobotGeometry(live,{blade_x_offset:0,blade_y_offset:0})).toMatchObject({bladeX:.18,bladeY:0});
    });
});
