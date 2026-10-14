import {projectPoint,rotatePoint,type RobotView,type SensorGeometry} from "../../utils/robotModel";

/** Choose the visible local face, so an IMU turned on its side uses its side
 * artwork rather than a flat top icon. Basis vectors retain yaw/roll/pitch. */
export function sensorArtworkProjection(sensor:SensorGeometry, view:RobotView) {
    const rpy=[sensor.roll,sensor.pitch,sensor.yaw];
    const planes=[
        {view:"top" as const,u:[0,-1,0],v:[-1,0,0],width:sensor.width,height:sensor.length},
        {view:"side" as const,u:[-1,0,0],v:[0,0,-1],width:sensor.length,height:sensor.height},
        {view:"side" as const,u:[0,-1,0],v:[0,0,-1],width:sensor.width,height:sensor.height},
    ];
    const candidates=planes.map(plane=>{
        const u=rotatePoint(plane.u,rpy),v=rotatePoint(plane.v,rpy);
        const a=projectPoint(u[0],u[1],u[2],view),b=projectPoint(v[0],v[1],v[2],view);
        return {...plane,a,b,area:Math.abs(a[0]*b[1]-a[1]*b[0])};
    }).sort((a,b)=>b.area-a.area);
    const best=candidates[0], [x,y]=projectPoint(sensor.x,sensor.y,sensor.z,view);
    // A vertical cylindrical LiDAR has rotational symmetry in side elevation.
    if(sensor.id==="lidar" && view==="side" && sensor.roll===0 && sensor.pitch===0)
        return {view:"side" as const,width:sensor.width,height:sensor.height,transform:`translate(${x} ${y})`};
    return {view:best.view,width:best.width,height:best.height,
        transform:`matrix(${best.a[0]} ${best.a[1]} ${best.b[0]} ${best.b[1]} ${x} ${y})`};
}
