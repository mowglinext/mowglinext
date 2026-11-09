import {expect,it} from "vitest";
import {sensorArtworkProjection} from "./sensorArtworkProjection";
import {parseRobotUrdf} from "../../utils/robotModel";
import {ROBOT_URDF} from "../../test/robotUrdf";
const sensors=parseRobotUrdf(ROBOT_URDF)!.sensors!;
it("uses a mounted IMU's side art when it is rolled onto its edge",()=>{
    const imu={...sensors.find(s=>s.id==="imu")!,roll:Math.PI/2};
    const top=sensorArtworkProjection(imu,"top");
    expect(top.view).toBe("side");
    expect(top.height).toBe(.01);
    const side=sensorArtworkProjection(imu,"side");
    expect(side.view).toBe("top");
    expect(side.height).toBe(.03);
});
it("keeps a cylindrical LiDAR's side diameter independent of yaw",()=>{
    const lidar={...sensors.find(s=>s.id==="lidar")!,yaw:Math.PI/2};
    expect(sensorArtworkProjection(lidar,"side")).toMatchObject({view:"side",width:.08,height:.06});
});
