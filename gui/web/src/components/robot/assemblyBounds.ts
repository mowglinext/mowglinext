import type {RobotGeometry} from "../../hooks/useRobotDescription";
import {robotBounds,type RobotView,type Bounds} from "../../utils/robotModel";
import {partBounds} from "./partAssets";

/** Viewport extents also cover decorative caster forks outside the URDF tyre.
 * This changes framing only, never component dimensions or the map pose. */
export function assemblyBounds(robot:RobotGeometry,view:RobotView,padding=.04):Bounds {
    const physical=robotBounds(robot,view,0);
    const source=partBounds("caster",view,view==="top" ? (robot.casterWidth ?? robot.casterRadius) : robot.casterRadius*2,robot.casterRadius*2);
    // Top caster is turned about its tyre axle so the swivel leads the wheel.
    const part=view === "top" ? {...source,x:-source.x-source.width,y:-source.y-source.height} : source;
    const positions=view==="top" ? [-1,1].map(sign=>[sign*robot.casterTrack/2,-robot.casterXOffset])
        : [[-robot.casterXOffset,-(robot.casterZ ?? (-robot.wheelRadius+robot.casterRadius))]];
    const x=Math.min(physical.x,...positions.map(p=>p[0]+part.x))-padding;
    const y=Math.min(physical.y,...positions.map(p=>p[1]+part.y))-padding;
    return {x,y,
        width:Math.max(physical.x+physical.width,...positions.map(p=>p[0]+part.x+part.width))+padding-x,
        height:Math.max(physical.y+physical.height,...positions.map(p=>p[1]+part.y+part.height))+padding-y};
}
