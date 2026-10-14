import {DEFAULT_DOCK_STYLE} from "./dockAssets";
import {DOCK_ARTWORK} from "./dockAssets";
import {PartSprite} from "./PartSprite";
import type {RobotView} from "../../utils/robotModel";

import {dockBounds, type DockVisualGeometry, type LayeredDockStyle} from "./dockGeometry";

export function DockGraphic({view="top",artwork="full",offsetY=0,style=DEFAULT_DOCK_STYLE,geometry}:{view?:RobotView;artwork?:"full"|"map";offsetY?:number;style?:LayeredDockStyle;geometry?:DockVisualGeometry}) {
    const b=dockBounds(view,style,geometry);
    return <g data-layer="styled-dock" data-dock-style={style} transform={`translate(${b.x+b.width/2} ${b.y+b.height/2+offsetY})`}>
        <PartSprite artwork={artwork} part={DOCK_ARTWORK[style].part} view={view} width={b.width} height={b.height}
            fallback={<rect x={-b.width/2} y={-b.height/2} width={b.width} height={b.height} rx={.03} fill="#344740" stroke="#86bfaa" strokeWidth={.002}/>}/>
    </g>;
}
