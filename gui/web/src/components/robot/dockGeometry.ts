import type {Bounds, RobotView} from "../../utils/robotModel";

// Nominal generic dock illustration, not measured robot/collision geometry.
// Dock pose is the parked robot's base_link. Head is at +X, entry at -X.
export const DOCK_LENGTH = .67, DOCK_WIDTH = .46, DOCK_HEIGHT = .15;
export function dockBounds(view:RobotView):Bounds {
    return view === "top" ? {x:-DOCK_WIDTH/2,y:-.57,width:DOCK_WIDTH,height:DOCK_LENGTH}
        : {x:-.57,y:-DOCK_HEIGHT,width:DOCK_LENGTH,height:DOCK_HEIGHT};
}
